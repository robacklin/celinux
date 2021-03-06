/*
 * Idle daemon for PowerPC.  Idle daemon will handle any action
 * that needs to be taken when the system becomes idle.
 *
 * Written by Cort Dougan (cort@cs.nmt.edu)
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
 */
#include <linux/config.h>
#include <linux/errno.h>
#include <linux/sched.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/smp.h>
#include <linux/smp_lock.h>
#include <linux/stddef.h>
#include <linux/unistd.h>
#include <linux/ptrace.h>
#include <linux/slab.h>

#include <asm/pgtable.h>
#include <asm/uaccess.h>
#include <asm/system.h>
#include <asm/io.h>
#include <asm/processor.h>
#include <asm/mmu.h>
#include <asm/cache.h>

#include <asm/time.h>
#include <asm/iSeries/LparData.h>
#include <asm/iSeries/HvCall.h>
#include <asm/iSeries/ItLpQueue.h>

unsigned long maxYieldTime = 0;
unsigned long minYieldTime = 0xffffffffffffffffUL;

#ifdef CONFIG_PPC_ISERIES
static void yield_shared_processor(void)
{
	struct paca_struct *lpaca = get_paca();
	unsigned long tb;
	unsigned long yieldTime;

	HvCall_setEnabledInterrupts( HvCall_MaskIPI |
				     HvCall_MaskLpEvent |
				     HvCall_MaskLpProd |
				     HvCall_MaskTimeout );

	tb = get_tb();
	/* Compute future tb value when yield should expire */
	HvCall_yieldProcessor( HvCall_YieldTimed, tb+tb_ticks_per_jiffy );

	yieldTime = get_tb() - tb;
	if ( yieldTime > maxYieldTime )
		maxYieldTime = yieldTime;

	if ( yieldTime < minYieldTime )
		minYieldTime = yieldTime;
	
	/* The decrementer stops during the yield.  Force a fake decrementer
	 * here and let the timer_interrupt code sort out the actual time.
	 */
	lpaca->xLpPaca.xIntDword.xFields.xDecrInt = 1;
	process_iSeries_events();
}
#endif /* CONFIG_PPC_ISERIES */

int idled(void)
{
	struct paca_struct *lpaca;
	long oldval;
#ifdef CONFIG_PPC_ISERIES
	unsigned long CTRL;
#endif

	/* endless loop with no priority at all */
	current->nice = 20;
	current->counter = -100;
#ifdef CONFIG_PPC_ISERIES
	/* ensure iSeries run light will be out when idle */
	current->thread.flags &= ~PPC_FLAG_RUN_LIGHT;
	CTRL = mfspr(CTRLF);
	CTRL &= ~RUNLATCH;
	mtspr(CTRLT, CTRL);
#endif
	init_idle();	

	lpaca = get_paca();

	for (;;) {
#ifdef CONFIG_PPC_ISERIES
		if ( lpaca->xLpPaca.xSharedProc ) {
			if ( ItLpQueue_isLpIntPending( lpaca->lpQueuePtr ) )
				process_iSeries_events();
			if ( !current->need_resched )
				yield_shared_processor();
		}
		else 
#endif
		{
			/* Avoid an IPI by setting need_resched */
			oldval = xchg(&current->need_resched, -1);
			if (!oldval) {
				while(current->need_resched == -1) {
#ifdef CONFIG_PPC_ISERIES
					HMT_medium();
					if ( ItLpQueue_isLpIntPending( lpaca->lpQueuePtr ) )
						process_iSeries_events();
#endif
					HMT_low();
				}
			}
		}
		HMT_medium();
		if (current->need_resched) {
			schedule();
			check_pgt_cache();
		}
	}
	return 0;
}

/*
 * SMP entry into the idle task - calls the same thing as the
 * non-smp versions. -- Cort
 */
int cpu_idle(void)
{
	idled();
	return 0; 
}
