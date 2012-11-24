/*
 * linux/kernel/workqueue.c
 *
 * Generic mechanism for defining kernel helper threads for running
 * arbitrary tasks in process context.
 *
 * Started by Ingo Molnar, Copyright (C) 2002
 *
 * Derived from the taskqueue/keventd code by:
 *
 *   David Woodhouse <dwmw2@redhat.com>
 *   Andrew Morton <andrewm@uow.edu.au>
 *   Kai Petzke <wpp@marie.physik.tu-berlin.de>
 *   Theodore Ts'o <tytso@mit.edu>
 */

#define __KERNEL_SYSCALLS__

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/init.h>
#include <linux/unistd.h>
#include <linux/signal.h>
#include <linux/completion.h>
#include <linux/workqueue.h>
#include <linux/slab.h>
#include <linux/resource.h>
#include <asm/uaccess.h>

/*
 * smp_num_cpus is "#defined" on UP machines, an extern on SMP
 *
 * We will have to live with the max number of cpus in the arrays
 * but we will created just enough threads.
 */
#ifndef smp_num_cpus
extern int smp_num_cpus;
#endif
/*
 * The per-CPU workqueue:
 */
struct cpu_workqueue_struct {

	spinlock_t lock;

	atomic_t nr_queued;
	struct list_head worklist;
	wait_queue_head_t more_work;
	wait_queue_head_t work_done;

	struct workqueue_struct *wq;
	struct task_struct *thread;
	struct completion exit;

} ____cacheline_aligned;

/*
 * The externally visible workqueue abstraction is an array of
 * per-CPU workqueues:
 */
struct workqueue_struct {
	struct workqueue_struct * next;
	struct cpu_workqueue_struct cpu_wq[NR_CPUS];
};
struct workqueue_struct * workqueue_head = NULL;
struct {
	struct workqueue_struct * next;
} workqueue_head_ptr = { .next =(struct workqueue_struct *)& workqueue_head };
	
spinlock_t workqueue_head_lock = SPIN_LOCK_UNLOCKED;
/*
 * Queue work on a workqueue. Return non-zero if it was successfully
 * added.
 *
 * We queue the work to the CPU it was submitted, but there is no
 * guarantee that it will be processed by that CPU.
 */
int queue_work(struct workqueue_struct *wq, struct work_struct *work)
{
	unsigned long flags;
	int ret = 0, cpu = smp_processor_id();
	struct cpu_workqueue_struct *cwq = wq->cpu_wq + cpu;

	if (!test_and_set_bit(0, &work->pending)) {
		BUG_ON(!list_empty(&work->entry));
		work->wq_data = cwq;

		spin_lock_irqsave(&cwq->lock, flags);
		list_add_tail(&work->entry, &cwq->worklist);
		atomic_inc(&cwq->nr_queued);
		spin_unlock_irqrestore(&cwq->lock, flags);

		wake_up(&cwq->more_work);
		ret = 1;
	}
	return ret;
}

static void delayed_work_timer_fn(unsigned long __data)
{
	struct work_struct *work = (struct work_struct *)__data;
	struct cpu_workqueue_struct *cwq = work->wq_data;
	unsigned long flags;

	/*
	 * Do the wakeup within the spinlock, so that flushing
	 * can be done in a guaranteed way.
	 */
	spin_lock_irqsave(&cwq->lock, flags);
	list_add_tail(&work->entry, &cwq->worklist);
	wake_up(&cwq->more_work);
	spin_unlock_irqrestore(&cwq->lock, flags);
}

static void dummy_delayed_work_timer_fn(unsigned long __data)
{
	struct work_struct *work = (struct work_struct *)__data;
	struct timer_list *timer = &work->timer;

	timer->function = delayed_work_timer_fn;
	return;
}

int queue_delayed_work(struct workqueue_struct *wq, struct work_struct *work, unsigned long delay)
{
	int ret = 0, cpu = smp_processor_id();
	struct timer_list *timer = &work->timer;
	struct cpu_workqueue_struct *cwq = wq->cpu_wq + cpu;

	if (!test_and_set_bit(0, &work->pending)) {
		BUG_ON(timer_pending(timer));
		BUG_ON(!list_empty(&work->entry));

		/*
		 * Increase nr_queued so that the flush function
		 * knows that there's something pending.
		 */
		atomic_inc(&cwq->nr_queued);
		work->wq_data = cwq;

		timer->expires = jiffies + delay;
		timer->data = (unsigned long)work;
		timer->function = delayed_work_timer_fn;
		add_timer(timer);

		ret = 1;
	}
	return ret;
}
/*
 * Cancel work, intended mostly as an analog for del_timer()...  Returns
 * true if the work element was found in either the timer or the work
 * pending queue.  It is possible for us to just miss some work which
 * has already been dequeued but has not yet executed (or is mid
 * execution) its designated function.
 *
 * The same is true of del_timer, but there one could use del_timer_sync
 * which is not practicable here.  Which is not to say that we don't use
 * del_timer_sync(), but that we don't guarantee that the work element
 * has either executed or will not execute as a result of the work being
 * deleted.
 */
int cancel_work(struct work_struct *work)
{
	struct timer_list *timer = &work->timer;
	struct cpu_workqueue_struct *cwq = work->wq_data;
	unsigned long flags;
	int ret = 0;
	
	if (!test_bit(0,&work->pending))
		return 0;

	timer->function = dummy_delayed_work_timer_fn;
	if (!(ret = del_timer_sync(timer))){
		spin_lock_irqsave(&cwq->lock, flags);
		if (!list_empty(&work->entry)){
			list_del_init(&work->entry);
			ret = 1;
		}
		spin_unlock_irqrestore(&cwq->lock, flags);
	}
	clear_bit(0, &work->pending);
	ret += (timer->function == delayed_work_timer_fn) ? 1 : 0;
	if (ret)
		atomic_dec(&cwq->nr_queued);
	return ret;	    
}
/*
 * Analog of mod_timer().  We don't require work to have been queued in
 * any way.  Return is 1 if work was queued prior to the call (same as
 * mod_timer()) and NOT if this request queued work (it will always suceed).
 */
int mod_delayed_work(struct workqueue_struct *wq, struct work_struct *work, 
		     unsigned long expires)
{
	int ret = cancel_work(work);

	queue_delayed_work(wq, work, expires - jiffies);
	return ret;
}
	


static inline void run_workqueue(struct cpu_workqueue_struct *cwq)
{
	unsigned long flags;

	/*
	 * Keep taking off work from the queue until
	 * done.
	 */
	spin_lock_irqsave(&cwq->lock, flags);
	while (!list_empty(&cwq->worklist)) {
		struct work_struct *work = list_entry(cwq->worklist.next, struct work_struct, entry);
		void (*f) (void *) = work->func;
		void *data = work->data;

		list_del_init(cwq->worklist.next);
		spin_unlock_irqrestore(&cwq->lock, flags);

		BUG_ON(work->wq_data != cwq);
		clear_bit(0, &work->pending);
		f(data);

		/*
		 * We only wake up 'work done' waiters (flush) when
		 * the last function has been fully processed.
		 */
		if (atomic_dec_and_test(&cwq->nr_queued))
			wake_up(&cwq->work_done);

		spin_lock_irqsave(&cwq->lock, flags);
	}
	spin_unlock_irqrestore(&cwq->lock, flags);
}

typedef struct startup_s {
	struct cpu_workqueue_struct *cwq;
	struct completion done;
	const char *name;
} startup_t;

static int worker_thread(void *__startup)
{
	startup_t *startup = __startup;
	struct cpu_workqueue_struct *cwq = startup->cwq;
	int cpu = cwq - cwq->wq->cpu_wq;
	DECLARE_WAITQUEUE(wait, current);
  	struct k_sigaction sa;

	daemonize();
	sprintf(current->comm, "%s/%d", startup->name, cpu);
	cwq->thread = current;

	set_cpus_allowed(current, 1UL << cpu);

	spin_lock_irq(&current->sig->siglock);
	siginitsetinv(&current->blocked, sigmask(SIGCHLD));
	recalc_sigpending(current);
	spin_unlock_irq(&current->sig->siglock);

	complete(&startup->done);

	/* Install a handler so SIGCLD is delivered */
	sa.sa.sa_handler = SIG_IGN;
	sa.sa.sa_flags = 0;
	siginitset(&sa.sa.sa_mask, sigmask(SIGCHLD));
	do_sigaction(SIGCHLD, &sa, (struct k_sigaction *)0);

	for (;;) {
		set_task_state(current, TASK_INTERRUPTIBLE);

		add_wait_queue(&cwq->more_work, &wait);
		if (!cwq->thread)
			break;
		if (list_empty(&cwq->worklist))
			schedule();
		else
			set_task_state(current, TASK_RUNNING);
		remove_wait_queue(&cwq->more_work, &wait);

		if (!list_empty(&cwq->worklist))
			run_workqueue(cwq);

		if (signal_pending(current)) {
			while (waitpid(-1, NULL, __WALL|WNOHANG) > 0)
				/* SIGCHLD - auto-reaping */ ;

			/* zap all other signals */
			spin_lock_irq(&current->sig->siglock);
			flush_signals(current);
			recalc_sigpending(current);
			spin_unlock_irq(&current->sig->siglock);
		}
	}
	remove_wait_queue(&cwq->more_work, &wait);
	complete(&cwq->exit);

	return 0;
}

/*
 * flush_workqueue - ensure that any scheduled work has run to completion.
 *
 * Forces execution of the workqueue and blocks until its completion.
 * This is typically used in driver shutdown handlers.
 *
 * NOTE: if work is being added to the queue constantly by some other
 * context then this function might block indefinitely.
 */
void flush_workqueue(struct workqueue_struct *wq)
{
	struct cpu_workqueue_struct *cwq;
	int cpu;

	for (cpu = 0; cpu < smp_num_cpus; cpu++) {
		cwq = wq->cpu_wq + cpu_logical_map(cpu);

		if (atomic_read(&cwq->nr_queued)) {
			DECLARE_WAITQUEUE(wait, current);

			if (!list_empty(&cwq->worklist))
				run_workqueue(cwq);

			/*
			 * Wait for helper thread(s) to finish up
			 * the queue:
			 */
			set_task_state(current, TASK_INTERRUPTIBLE);
			add_wait_queue(&cwq->work_done, &wait);
			if (atomic_read(&cwq->nr_queued))
				schedule();
			else
				set_task_state(current, TASK_RUNNING);
			remove_wait_queue(&cwq->work_done, &wait);
		}
	}
}

struct workqueue_struct *create_workqueue(const char *name)
{
	int ret, cpu, destroy = 0;
	struct cpu_workqueue_struct *cwq;
	startup_t startup;
	struct workqueue_struct *wq;
	unsigned long flags;

	BUG_ON(strlen(name) > 10);
	startup.name = name;

	wq = kmalloc(sizeof(*wq), GFP_KERNEL);
	if (!wq)
		return NULL;

	spin_lock_irqsave(&workqueue_head_lock, flags);
	wq->next = workqueue_head;
	workqueue_head = wq;
	spin_unlock_irqrestore(&workqueue_head_lock, flags);
	

	for (cpu = 0; cpu < smp_num_cpus; cpu++) {
		cwq = wq->cpu_wq + cpu_logical_map(cpu);

		spin_lock_init(&cwq->lock);
		cwq->wq = wq;
		cwq->thread = NULL;
		atomic_set(&cwq->nr_queued, 0);
		INIT_LIST_HEAD(&cwq->worklist);
		init_waitqueue_head(&cwq->more_work);
		init_waitqueue_head(&cwq->work_done);

		init_completion(&startup.done);
		startup.cwq = cwq;
		ret = kernel_thread(worker_thread, &startup,
						CLONE_FS | CLONE_FILES);
		if (ret < 0)
			destroy = 1;
		else {
			wait_for_completion(&startup.done);
			BUG_ON(!cwq->thread);
		}
	}
	/*
	 * Was there any error during startup? If yes then clean up:
	 */
	if (destroy) {
		destroy_workqueue(wq);
		wq = NULL;
	}
	return wq;
}

void destroy_workqueue(struct workqueue_struct *wq)
{
	struct cpu_workqueue_struct *cwq;
	struct workqueue_struct * wqp = workqueue_head_ptr.next;
	unsigned long flags;
	int cpu;

	flush_workqueue(wq);

	for (cpu = 0; cpu < smp_num_cpus; cpu++) {
		cwq = wq->cpu_wq + cpu_logical_map(cpu);
		if (!cwq->thread)
			continue;
		/*
		 * Initiate an exit and wait for it:
		 */
		init_completion(&cwq->exit);
		cwq->thread = NULL;
		wake_up(&cwq->more_work);

		wait_for_completion(&cwq->exit);
	}
	spin_lock_irqsave(&workqueue_head_lock, flags);

	while (wqp && wqp->next != wq)
		wqp = wqp->next;
	if(! wqp)
		BUG();
	wqp->next = wq->next;
	spin_unlock_irqrestore(&workqueue_head_lock, flags);
	
	kfree(wq);
}

static struct workqueue_struct *keventd_wq;

int schedule_work(struct work_struct *work)
{
	return queue_work(keventd_wq, work);
}

int schedule_delayed_work(struct work_struct *work, unsigned long delay)
{
	return queue_delayed_work(keventd_wq, work, delay);
}

void flush_scheduled_work(void)
{
	flush_workqueue(keventd_wq);
}
extern asmlinkage long sys_setpriority(int which, int who, int niceval);
extern asmlinkage long sys_sched_setscheduler(pid_t pid, int policy, 
					      struct sched_param *param);

int set_workqueue_priority(struct workqueue_struct *wq,  int policy, 
			   struct sched_param *priority)
{
	struct cpu_workqueue_struct *cwq;
	struct sched_param other_pry = {.sched_priority = 0};
	mm_segment_t old_fs = get_fs();
	int rtn = 0, cpu;

	set_fs(KERNEL_DS);
	for(cpu = 0; cpu < smp_num_cpus; cpu++) {
		cwq = wq->cpu_wq + cpu_logical_map(cpu);
		if(policy != SCHED_OTHER){
			if( (rtn = sys_sched_setscheduler(cwq->thread->pid, 
						policy, priority)))
				break;
		}else{
			if( (rtn = sys_sched_setscheduler(cwq->thread->pid, 
						policy, &other_pry)))
				break;
			if( (rtn = sys_setpriority(PRIO_PROCESS, 
						  cwq->thread->pid, 
						   priority->sched_priority)))
				break;
		}
	}
	set_fs(old_fs);
	return rtn;
}

#ifdef CONFIG_PROC_FS

#include <linux/proc_fs.h>
#include <linux/string.h>

#define HEAD_LINES 1
#define HEAD1 (1 - HEAD_LINES)
#define HEAD2 (2 - HEAD_LINES)
#define DONE -10
static int g_read_completed = 0;
static int g_cpu = 0;
static char g_buff[40 + 7 * NR_CPUS];
static int g_buffidx = 0;
static int g_buf_end = 0;

static int workqueue_read_proc(char *page_buffer, char **my_first_byte,
	off_t virtual_start, int length, int *eof, void *data)
{
	struct workqueue_struct * wqp = workqueue_head_ptr.next;
	char * const my_base = page_buffer;
	struct task_struct *t;
	char * slash_ptr;
	unsigned long flags;
	int cpu, i = 0;
	int policy, nice, rt_priority, pid[NR_CPUS];
	char comm[11];

	*my_first_byte = page_buffer;
	if (virtual_start == 0){
	/* Just been opened */
		g_read_completed = HEAD1;
		g_cpu = 0;
		g_buffidx = 0;
		g_buf_end = 0;
	} else if ((i = g_buf_end - g_buffidx) > 0){
		if (i > length)
			i = length;
		memcpy(my_base, &g_buff[g_buffidx], i);
		g_buffidx += i;
		return i;
	} else if (g_read_completed == DONE) {
		return 0;

	}

	g_buf_end = g_buffidx = 0;
	switch (g_read_completed) {

	case HEAD1:
		g_buf_end = sprintf(&g_buff[0],
				    "workqueue policy priority/nice pid(s).\n");
		/*                   1234567890 12345    123        */
		/*                             1     1234   12345678  */
		break;
	default:
		/*
		 * Since workqueues come and go rather slowly, we will assume
		 * that the list is static, but only in that we keep the lock
		 * any longer than is needed to look up one entry.  If it
		 * changes, we could produce the same entry twice.  The
		 * convers of missing an entry could also happen if a new
		 * workqueue is entered while we work.  But that should be
		 * rather rare...
		 */
		spin_lock_irqsave(&workqueue_head_lock, flags);
		i = 1;
		while ( i <= g_read_completed && wqp){
			wqp = wqp->next;
			i++;
		}
		if( ! wqp ){
			g_read_completed = DONE;
			spin_unlock_irqrestore(&workqueue_head_lock, flags);
			return 0;
		}
		t = wqp->cpu_wq[0].thread;
		policy = t->policy;
		nice = task_nice(t);
		rt_priority = t->rt_priority;
		strncpy(&comm[0],t->comm,10);
		comm[10] = 0;
		slash_ptr = strchr(comm,'/');
		if (slash_ptr)
			*slash_ptr = 0;
		for (cpu = 0; cpu < smp_num_cpus; cpu++)
			pid[cpu] = 
				wqp->cpu_wq[cpu_logical_map(cpu)].thread->pid;
			
		spin_unlock_irqrestore(&workqueue_head_lock, flags);
		g_buf_end = sprintf(&g_buff[0],"%10s %5s    %3d        ",
				    comm,
				    policy == SCHED_FIFO ? "FIFO " :  
				         policy == SCHED_RR ? " RR  " :
				                      "OTHER",
				    policy == SCHED_OTHER ?  nice :
				    rt_priority);
		
		for (cpu = 0; cpu < smp_num_cpus; cpu++)
			g_buf_end += sprintf(&g_buff[g_buf_end]," %d", 
					     pid[cpu]);

		g_buff[g_buf_end++] = '\n';
	}
	g_read_completed++;
	if ((i = g_buf_end) > length)
		i = length;
	memcpy(my_base, &g_buff[0], i);
	g_buffidx += i;
	return i;

}
static int wq_simple_strtol(char **rtn)
{
	char *tmp = *rtn;

	while (*tmp == ' ')
		tmp++;
	return simple_strtol(tmp,rtn,10);
}
	
#define MAX_INBUF 50
static int workqueue_write_proc(struct file *file, const char *buf, 
					 unsigned long count, void *data)
{
	long pid;
	long policy;
	unsigned long flags;
	int rtn;
	char kbuf[MAX_INBUF + 1];
	char *tmp;
	struct workqueue_struct * wqp = workqueue_head_ptr.next;
	struct sched_param priority = {.sched_priority = 0};
	
	if (!count || count > MAX_INBUF)
		return -EINVAL;
	if (copy_from_user(&kbuf, buf, count))
		return -EFAULT;
	kbuf[count] = '\0';
	tmp = kbuf;
	pid = wq_simple_strtol(&tmp);
	policy = wq_simple_strtol(&tmp);
	priority.sched_priority = wq_simple_strtol(&tmp);

	spin_lock_irqsave(&workqueue_head_lock, flags);
	while (wqp) {
		wqp = wqp->next;
		if (wqp && wqp->cpu_wq[0].thread->pid == pid)
			break;
	}
	if (!wqp) {
		spin_unlock_irqrestore(&workqueue_head_lock, flags);
		return -EINVAL;
	}
	rtn = set_workqueue_priority(wqp, policy, &priority);
	spin_unlock_irqrestore(&workqueue_head_lock, flags);
	if (rtn >= 0)
		rtn = count;
	return rtn;
}
#endif

int __init  init_workqueues(void)
{
#ifdef CONFIG_PROC_FS
	struct proc_dir_entry *prc;
	prc = create_proc_read_entry("workqueue", S_IRUGO | S_IWUSR, 0,
				     workqueue_read_proc, 0);
	if (prc)
		prc-> write_proc = workqueue_write_proc;
#endif
	keventd_wq = create_workqueue("events");
	BUG_ON(!keventd_wq);
	return 0;
}
__initcall(init_workqueues);

EXPORT_SYMBOL_GPL(create_workqueue);
EXPORT_SYMBOL_GPL(queue_work);
EXPORT_SYMBOL_GPL(queue_delayed_work);
EXPORT_SYMBOL_GPL(cancel_work);
EXPORT_SYMBOL_GPL(mod_delayed_work);
EXPORT_SYMBOL_GPL(flush_workqueue);
EXPORT_SYMBOL_GPL(destroy_workqueue);
EXPORT_SYMBOL_GPL(set_workqueue_priority);

EXPORT_SYMBOL(schedule_work);
EXPORT_SYMBOL(schedule_delayed_work);
EXPORT_SYMBOL(flush_scheduled_work);

