#ifndef EMULAT_SOFTIRQ
#define EMULAT_SOFTIRQ

/*
 * 
 * 2003-4-25  Emulate_softirq.c  by George Anzinger
 *	        Copyright (C) 2003 MontaVista Software.
 *		Copyright 2003 Sony Corporation.
 *		Copyright 2003 Matsushita Electric Industrial Co., Ltd.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 *
 * This code is designed to move softirq and timer processing from
 * softirq to workqueues.  It defines, or rather, redefines several
 * softirq functions to do workqueue things.  It also traps bh locks and
 * turns them into simple spin locks as there is no longer any bh code
 * once this is done.
 */

#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <asm/semaphore.h>
#include <asm/current.h>
#include <linux/module.h>
/*
 * This shoud be set to the name of the workqueue we are to use
 * for this sub system.
 */ 
#ifndef WQ_NAME
#define WQ_NAME UNAMED	 /* default workqueue index */
#endif
#ifdef THIS_IS_EMULATE_SOFTIRQ	/* if included by emulate_softirq.c  */
#define EXTERN
#define EMULATE_C(...) __VA_ARGS__
#else
#define EXTERN extern
#define EMULATE_C(...)
#endif

	
	
#define EM_DEBUGx	    /* remove 'x' to enable */
struct softirq_to_wq {
	struct workqueue_struct *wq;
	struct softirq_action sa;
	struct work_struct ws;
}; 
struct tmr_to_fun {
	union {
		struct timer_list *timr;
		struct tmr_to_fun *next;
	} ptr;
	struct work_struct work;
};
#define nextptr ptr.next
#define tmr ptr.timr
/*
 * multi level in the following allows the embedded macros to be
 * resolved before they are protected by quotes or conconcat.
 */
#define BUILD(a,b) a ## b
#define BUILD0(a,b) BUILD(a,b)
#define STRING0(m) STRINGX(m)
#define STRINGX(m)# m
#define NUM_TIMERS 1000
#ifndef THIS_IS_EMULATE_SOFTIRQ
#define THIS_WQ BUILD0(wq_ , WQ_NAME)
#else
#define THIS_WQ ILLEGAL.
#endif 
struct wq_this{
	struct workqueue_struct *wq;
	char name[10];
	int tasklet_count;
	spinlock_t free_wk_ptrs_lock;
	struct tmr_to_fun * free_wk_ptrs;
	struct tmr_to_fun  tmr_to_fun_ay[NUM_TIMERS];
};
#ifndef THIS_IS_EMULATE_SOFTIRQ
extern struct wq_this THIS_WQ;
#ifdef WQ_DATA_HERE
struct wq_this THIS_WQ	= {.name = STRING0(WQ_NAME)};
#endif 
#endif 
extern struct softirq_to_wq softirq_wq[TASKLET_SOFTIRQ];
extern void softirq_wrap(void *data_in);
extern void init_wq_this(struct wq_this *wq);

#define open_softirq(nr, action_in, data_in) do { \
		 char name[10] = {"sir_wq_xx"}; \
		 name[7] = (nr/10) + '0'; \
		 name[8] = (nr%10) + '0'; \
		 init_wq_this(&THIS_WQ);\
		 softirq_wq[nr].sa.action = action_in; \
		 softirq_wq[nr].sa.data = data_in;     \
		 INIT_WORK(&softirq_wq[nr].ws, softirq_wrap, (void *)nr); \
	    } while (0)
/*
 *		   softirq_wq[nr].ws.data = (void *)nr; 
 *		   softirq_wq[nr].ws.func = softirq_wrap; 
 */
#define raise_softirq(nr)  queue_work(THIS_WQ.wq, &softirq_wq[nr].ws)
#define cpu_raise_softirq(cpu,nr) raise_softirq(nr)
#undef __cpu_raise_softirq
#define __cpu_raise_softirq(cpu, nr) cpu_raise_softirq(cpu,nr)
extern int del_timer_sync_to_wq(struct wq_this *wq, struct timer_list *timer);
extern void add_timer_to_wq(struct wq_this *wq, struct timer_list *timer);
extern int mod_timer_to_wq(struct wq_this *wq, struct timer_list *timer, 
			   unsigned long exp);

#ifndef THIS_IS_EMULATE_SOFTIRQ
#undef del_timer_sync
#define add_timer(timer)      add_timer_to_wq(&THIS_WQ, timer)
#define del_timer_sync(timer) del_timer_sync_to_wq(&THIS_WQ, timer)
#define del_timer(timer)      del_timer_sync_to_wq(&THIS_WQ, timer)
#define mod_timer(timer, exp) mod_timer_to_wq(&THIS_WQ, timer, exp)
#define init_timer(timer)
#define timer_pending(timer)  \
((((struct tmr_to_fun *)((timer)->list.prev)) == THIS_WQ.tmr_to_fun_ay) ? \
((struct tmr_to_fun*)((timer)->list.next))->work.pending : 0)
#endif
#ifndef preempt_disable
#define preempt_disable()
#define preempt_enable()
#endif
struct recur_mutex {
	struct semaphore sem;
	struct task_struct *who;
	int    count;
};
#ifdef CONFIG_SMP
#define MUTEX_INIT_ARRAY				\
		{__MUTEX_INITIALIZER(bh_mtx[0].sem)},	\
		{__MUTEX_INITIALIZER(bh_mtx[1].sem)},	\
		{__MUTEX_INITIALIZER(bh_mtx[2].sem)},	\
		{__MUTEX_INITIALIZER(bh_mtx[3].sem)},	\
		{__MUTEX_INITIALIZER(bh_mtx[4].sem)},	\
		{__MUTEX_INITIALIZER(bh_mtx[5].sem)},	\
		{__MUTEX_INITIALIZER(bh_mtx[6].sem)},	\
		{__MUTEX_INITIALIZER(bh_mtx[7].sem)},	\
		{__MUTEX_INITIALIZER(bh_mtx[8].sem)},	\
		{__MUTEX_INITIALIZER(bh_mtx[9].sem)},	\
		{__MUTEX_INITIALIZER(bh_mtx[10].sem)},	\
		{__MUTEX_INITIALIZER(bh_mtx[11].sem)},	\
		{__MUTEX_INITIALIZER(bh_mtx[12].sem)},	\
		{__MUTEX_INITIALIZER(bh_mtx[13].sem)},	\
		{__MUTEX_INITIALIZER(bh_mtx[14].sem)},	\
		{__MUTEX_INITIALIZER(bh_mtx[15].sem)},	\
		{__MUTEX_INITIALIZER(bh_mtx[16].sem)},	\
		{__MUTEX_INITIALIZER(bh_mtx[17].sem)},	\
		{__MUTEX_INITIALIZER(bh_mtx[18].sem)},	\
		{__MUTEX_INITIALIZER(bh_mtx[19].sem)},	\
		{__MUTEX_INITIALIZER(bh_mtx[20].sem)},	\
		{__MUTEX_INITIALIZER(bh_mtx[21].sem)},	\
		{__MUTEX_INITIALIZER(bh_mtx[22].sem)},	\
		{__MUTEX_INITIALIZER(bh_mtx[23].sem)},	\
		{__MUTEX_INITIALIZER(bh_mtx[24].sem)},	\
		{__MUTEX_INITIALIZER(bh_mtx[25].sem)},	\
		{__MUTEX_INITIALIZER(bh_mtx[26].sem)},	\
		{__MUTEX_INITIALIZER(bh_mtx[27].sem)},	\
		{__MUTEX_INITIALIZER(bh_mtx[28].sem)},	\
		{__MUTEX_INITIALIZER(bh_mtx[29].sem)},	\
		{__MUTEX_INITIALIZER(bh_mtx[30].sem)},	\
		{__MUTEX_INITIALIZER(bh_mtx[31].sem)}
#else
#define MUTEX_INIT_ARRAY {__MUTEX_INITIALIZER(bh_mtx[0].sem)}
#endif

EXTERN struct recur_mutex bh_mtx[NR_CPUS] EMULATE_C( = { MUTEX_INIT_ARRAY });

#define mutex_enter(x) do {						\
	preempt_disable();						\
	{								\
		struct recur_mutex *rcm = x[smp_processor_id()];	\
		if (rcm->count && rcm->who == current) {		\
			rcm->count++;					\
			preempt_enable();				\
		} else {						\
		       down(&rcm->sem);					\
		       rcm->who = current;				\
		       rcm->count = 1;					\
		}							\
	}} while (0)
#define mutex_exit(x) do {						\
		struct recur_mutex *rcm = x[smp_processor_id()];	\
		if (!(--rcm->count)){					\
		      up(&rcm->sem);					\
		      preempt_enable();					\
		}							\
	} while (0)
#if 1
#undef local_bh_disable
#undef local_bh_enable
#undef __local_bh_enable
#define local_bh_disable() mutex_enter(&bh_mtx)
#define local_bh_enable()  mutex_exit(&bh_mtx)
#define __local_bh_enable() local_bh_enable()
#endif
/*
 * Tasklet emulation stuff
 *
Tasklets:
   Properties:
   * If tasklet_schedule() is called, then tasklet is guaranteed
     to be executed on some cpu at least once after this.
   * If the tasklet is already scheduled, but its excecution is still not
     started, it will be executed only once.
   * If this tasklet is already running on another CPU (or schedule is called
     from tasklet itself), it is rescheduled for later.
   * Tasklet is strictly serialized wrt itself, but not
     wrt another tasklets. If client needs some intertask synchronization,
     he makes it with spinlocks.

  *
  * So here is what we do:  We will define one workqueue for all tasklets 
  * by this intercepted occurance.
  * We intercept the tasklet_init, kill, schedule, hi_schedule, enable,
  * and disable calls.	We will borrow a lot of code from the intercepted
  * functions so the symanatics and functions work mostly the same.  The
  * only difference we want to see is that the code is run from a workqueue
  * and not softirq.
  *
*/
struct wq_tasklet{
	struct wq_this *wq;
	struct work_struct work;
};

extern void wq_tasklet_init(struct wq_this * wq, struct tasklet_struct *t,
			    void (*func)(unsigned long), unsigned long data);
extern void wq_tasklet_kill(struct tasklet_struct *t);
#define TASKLET_NEEDS_INIT 777
#ifndef THIS_IS_EMULATE_SOFTIRQ
#undef DECLARE_TASKLET
#undef DECLARE_TASKLET_DISABLED
#define DECLARE_TASKLET(name, func, data) \
struct tasklet_struct name = { 	(struct tasklet_struct *)&THIS_WQ,  \
                              0, ATOMIC_INIT(TASKLET_NEEDS_INIT), func, data }

#define DECLARE_TASKLET_DISABLED(name, func, data) \
struct tasklet_struct name = { (struct tasklet_struct *)&THIS_WQ, \
                                0, ATOMIC_INIT(TASKLET_NEEDS_INIT + 1), \
                                 func, data }

#define  tasklet_init(a, b, c) wq_tasklet_init(&THIS_WQ, a, b, c)
#define  tasklet_kill(a) wq_tasklet_kill(a)
#define  tasklet_hi_schedule(a) wq_tasklet_hi_schedule(a)
#define  tasklet_schedule(a) wq_tasklet_schedule(a)
#define  tasklet_enable(a) wq_tasklet_enable(a)
#define  tasklet_hi_enable(a) wq_tasklet_hi_enable(a)
#endif
extern void FASTCALL(__wq_tasklet_schedule(struct tasklet_struct *t));
static inline void wq_tasklet_schedule(struct tasklet_struct *t)
{
	if (!test_and_set_bit(TASKLET_STATE_SCHED, &t->state))
		__wq_tasklet_schedule(t);
}

extern void FASTCALL(__wq_tasklet_hi_schedule(struct tasklet_struct *t));

static inline void wq_tasklet_hi_schedule(struct tasklet_struct *t)
{
	BUG_ON(!t);
	if (!test_and_set_bit(TASKLET_STATE_SCHED, &t->state))
		__wq_tasklet_hi_schedule(t);
}

static inline void wq_tasklet_enable(struct tasklet_struct *t)
{
	BUG_ON(!t);
	smp_mb__before_atomic_dec();
	if(atomic_dec_and_test(&t->count) && 
	   test_bit(TASKLET_STATE_SCHED, &t->state)){
		__wq_tasklet_schedule(t);
	}
		
}

static inline void wq_tasklet_hi_enable(struct tasklet_struct *t)
{
	BUG_ON(!t);
	smp_mb__before_atomic_dec();
	if(atomic_dec_and_test(&t->count) && 
	   test_bit(TASKLET_STATE_SCHED, &t->state)){
		__wq_tasklet_schedule(t);
	}
}

/*
 * And finally the old bh stuff...
 */
EXTERN struct tasklet_struct wq_bh_task_vec[32];

#ifndef THIS_IS_EMULATE_SOFTIRQ  /* if not included by emulate_softirq.c  */

#define mark_bh(nr)  wq_tasklet_hi_schedule(wq_bh_task_vec+nr)
#define init_bh(nr, routine) \
	 wq_tasklet_init(&THIS_WQ, wq_bh_task_vec + nr, routine, 0)
#define remove_bh(nr)  wq_tasklet_kill(wq_bh_task_vec + nr)
/*
static inline void mark_bh(int nr)
{
	tasklet_hi_schedule(bh_task_vec+nr);
}
extern struct tasklet_struct bh_task_vec[];
extern void init_bh(int nr, void (*routine)(void));
extern void remove_bh(int nr);
*/

#endif
#endif

