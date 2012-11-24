/*
 * linux/lib/emulate_softirq.c
 *
 * 
 * 2003-4-25  Emulate_softirq.c  by George Anzinger
 *	        Copyright (C) 2003 MontaVista Software.
 *		Copyright 2003 Sony Corporation
 *		Copyright 2003 Matsushita Electric Industrial Co., Ltd.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 *
 * This code should NOT be considered a final solution to anything.  It
 * is intended to make it easy to move a driver or package of drivers
 * and kernel services from 'softirq' services to workqueues.  The only
 * required modification of the affected soft ware is to include the
 * header file as follows:
 *
 * #define TASKLET_WQ_INDEX ##
 * #include <linux/emulate_softirq.h>
 *
 * where '##' should be a small unique number (less than 10 at this time).
 * This is used as an index into an array of tasklet workqueues and allows 
 * different subsystems to use different workqueues.
 *
 * The header file must be included in with all sources that reference
 * in any way, any part of the softirq system, i.e. timers, tasklets,
 * and bh code.  The main way the code works is to trap, with macros,
 * the functions that interface to these services and send the request
 * to workqueue services that do the same thing.  This, of course,
 * entails some inefficacies, array lookups and translations.  The
 * solution for this is to directly use the workqueue services and, if
 * desired, to provide switches in the code to switch back to the
 * softirq services.
 *
 * Note that by making the include conditional on a config macro one can
 * move a subsystem back and forth between the conventional softirq
 * services and the workqueue way of doing things.
 *
 * As a demonstration of the ability of this package, the network code
 * was moved to workqueue services by adding the above include in the
 * <linux/skbuf.h> header file.  This file seems to be included,
 * directly or indirectly, in all network code so the one, conditional,
 * include was the only change to the network code.
 *
 * Note that once softirq services are out of the picture, the bh_lock
 * is no longer needed, so this package eliminates that as part of what
 * it does.
 */

#define THIS_IS_EMULATE_SOFTIRQ
#include <linux/slab.h>
#include <linux/emulate_softirq.h>
#include <linux/workqueue.h>
#include <linux/timer.h>
#include <linux/init.h>

struct softirq_to_wq softirq_wq[TASKLET_SOFTIRQ];

void softirq_wrap(void *data_in)
{
	int nr = (int)data_in;
	void (*f) (struct softirq_action *) = softirq_wq[nr].sa.action;
	void *data =  softirq_wq[nr].sa.data;

	/*  do softirq locking here */
	mutex_enter(&bh_mtx);

	f(data);

	/*  do softirq unlocking here */
	mutex_exit(&bh_mtx);
}
EXPORT_SYMBOL(softirq_wrap);
/*
 *
 * We need a work structure per timer, defined here.  Note, we do
 * not modify the users timer as he may then pass it to us again and
 * we would be lost.  So we use the timer in the 'work_struct'...
 * This means we also have to trap the mod_timer calls.
 */
void init_wq_this(struct wq_this *wq)
{
	int i;

	if (wq->wq)
		return;
	wq->wq = create_workqueue(wq->name);
	spin_lock_init(&wq->free_wk_ptrs_lock);
	for (i = 0; i < NUM_TIMERS; i++){
		wq->tmr_to_fun_ay[i].work.func = (void(*)(void*))10;
	}
}
EXPORT_SYMBOL(init_wq_this);


static inline void free_tmr_wq(struct wq_this *wq, 
			       struct tmr_to_fun *tmr_to_fun)
{
	tmr_to_fun->nextptr = wq->free_wk_ptrs;
	tmr_to_fun->work.func = NULL;
	wq->free_wk_ptrs = tmr_to_fun;
		
}
static inline void bld_free_list(struct wq_this *wq)
{
	int i;

	for (i = NUM_TIMERS - 1; i >= 0; --i) {
		if (!wq->tmr_to_fun_ay[i].work.pending)
			free_tmr_wq(wq, &wq->tmr_to_fun_ay[i]);
	}
}
static inline struct tmr_to_fun * get_tmr_to_fun(struct wq_this *wq)
{
	struct tmr_to_fun *ret;
	if (!wq->free_wk_ptrs)
		bld_free_list(wq);

	ret = wq->free_wk_ptrs;
	if (ret)
		wq->free_wk_ptrs = wq->free_wk_ptrs->nextptr;
	return ret;
}

static void run_wq_timer(void * data)
{
	struct timer_list * timer = (struct timer_list *)data;

	mutex_enter(&bh_mtx);
	timer->function(timer->data);
	mutex_exit(&bh_mtx);
}

static struct tmr_to_fun *find_timer(struct wq_this *wq, 
				     struct timer_list *timer )
{
	struct tmr_to_fun *ret = (struct tmr_to_fun*)timer->list.next;
	if (!wq->wq)
		init_wq_this(wq);
		
	spin_lock(&wq->free_wk_ptrs_lock);
	if (timer->list.prev == (struct list_head *)wq->tmr_to_fun_ay) {
		if (ret->tmr == timer)
			return (struct tmr_to_fun*)timer->list.next;
	}
	ret = get_tmr_to_fun(wq);
	if (ret) {
		timer->list.prev = (struct list_head *)wq->tmr_to_fun_ay;
		timer->list.next = (struct list_head *)ret;
		ret->tmr = timer;
		INIT_WORK(&ret->work, run_wq_timer, (void *)timer);
	}
	BUG_ON(!ret);/* too many active timers */
	return ret;
}

void add_timer_to_wq(struct wq_this *wq, struct timer_list *timer)
{
	struct tmr_to_fun * tmr_to_fun = find_timer(wq, timer);

	BUG_ON(!queue_delayed_work(wq->wq, &tmr_to_fun->work, 
			   timer->expires - jiffies));
	spin_unlock(&wq->free_wk_ptrs_lock);
}
EXPORT_SYMBOL(add_timer_to_wq);

int del_timer_sync_to_wq(struct wq_this *wq, struct timer_list *timer)
{
	struct tmr_to_fun * tmr_to_fun = find_timer(wq, timer);
	int ret;

	ret = cancel_work(&tmr_to_fun->work);
	spin_unlock(&wq->free_wk_ptrs_lock);
	return ret;
}
EXPORT_SYMBOL(del_timer_sync_to_wq);

int mod_timer_to_wq(struct wq_this *wq, struct timer_list *timer, 
		    unsigned long expire)
{
	struct tmr_to_fun * tmr_to_fun = find_timer(wq, timer);
	int ret;

	ret = mod_delayed_work(wq->wq, &tmr_to_fun->work, expire);
	timer->expires = expire;
	spin_unlock(&wq->free_wk_ptrs_lock);
	return ret;
}
EXPORT_SYMBOL(mod_timer_to_wq);

/*
 * tasklet emulation code
 */


static kmem_cache_t *wq_tasklet_cache;
/*
 * We don't need to requeue disabled tasklets as that will be done
 * by the enable code.	Keeps us from looping.	We do need to test
 * for disabled tasklets as that can happen while in the queue.
 */
void wq_tasklet_action(void * data)
{
	struct tasklet_struct *t = data;

	mutex_enter(&bh_mtx);
	if (tasklet_trylock(t)) {
		if (!atomic_read(&t->count)) {
			BUG_ON(!test_and_clear_bit(TASKLET_STATE_SCHED, 
						   &t->state));
			t->func(t->data);
		}
		tasklet_unlock(t);
	} else
		BUG();
	mutex_exit(&bh_mtx);
	
}
EXPORT_SYMBOL(wq_tasklet_action);

static struct tasklet_struct *wq_do_setup(struct wq_this * wq, 
					  struct tasklet_struct *t)
{
	struct wq_tasklet * tlwork;
	if (!wq_tasklet_cache)
		wq_tasklet_cache = kmem_cache_create("wq_tasklet_cache", 
						     sizeof(struct wq_tasklet), 
						     0, 0, NULL, 0);
	if(!wq->wq )
		init_wq_this(wq);
	wq->tasklet_count++;
	tlwork = kmem_cache_alloc(wq_tasklet_cache, GFP_KERNEL | GFP_ATOMIC);
	BUG_ON(!tlwork);

	INIT_WORK(&tlwork->work, wq_tasklet_action, (void *)t);
	tlwork->wq = wq;
	return (struct tasklet_struct*)tlwork;
}

void wq_tasklet_init(struct wq_this * wq, struct tasklet_struct *t,
		     void (*func)(unsigned long), unsigned long data)
{
	t->next = wq_do_setup(wq, t);
	t->state = 0;
	atomic_set(&t->count, 0);
	t->data = data;
	t->func = func;
}
#define MAGIC_TASKLET_NO 1000
EXPORT_SYMBOL(wq_tasklet_init);

void wq_tasklet_kill(struct tasklet_struct *t)
{
	struct wq_tasklet * work = (struct  wq_tasklet*)(t->next);
	struct wq_this *wq = work->wq;

	tasklet_kill(t);
	--wq->tasklet_count;
	kmem_cache_free(wq_tasklet_cache, work);
}
EXPORT_SYMBOL(wq_tasklet_kill);

void __wq_tasklet_hi_schedule(struct tasklet_struct *t)
{
	if (unlikely(atomic_read(&t->count) >= TASKLET_NEEDS_INIT)) {
		t->next = wq_do_setup((struct wq_this *)(t->next), t);
		atomic_sub(TASKLET_NEEDS_INIT, &t->count);
	}
	if (!atomic_read(&t->count)){
		BUG_ON(!queue_work(((struct wq_tasklet*)(t->next))->wq->wq,
			   &((struct  wq_tasklet*)(t->next))->work));
	}
}
EXPORT_SYMBOL(__wq_tasklet_hi_schedule);

void __wq_tasklet_schedule(struct tasklet_struct *t)
{
	if (unlikely(atomic_read(&t->count) >= TASKLET_NEEDS_INIT)) {
		t->next = wq_do_setup((struct wq_this *)(t->next), t);
		atomic_sub(TASKLET_NEEDS_INIT, &t->count);
	}

	if( !atomic_read(&t->count)){
		BUG_ON(!queue_work(((struct wq_tasklet*)(t->next))->wq->wq,
				   &((struct  wq_tasklet*)(t->next))->work));
	}
}
EXPORT_SYMBOL(__wq_tasklet_schedule);

int __init init_emulate_softirq(void)
{
	int i;
	for (i = 1; i < NR_CPUS; i++) {
		sema_init(&bh_mtx[i].sem, 1);
	}
	return 0;
}
__initcall(init_emulate_softirq);
