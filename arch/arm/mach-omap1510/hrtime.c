/*
 * arch/arm/mach-omap1510/hrtime.c
 *
 * High-Res Timer Implementation for TI OMAP1510 boards
 *
 * Author: George G. Davis <gdavis@mvista.com>
 *
 * Copyright (C) 2003 MontaVista, Software, Inc. 
 * Copyright 2003 Sony Corporation.
 * Copyright 2003 Matsushita Electric Industrial Co., Ltd.
 * 
 * This file is licensed under  the terms of the GNU General Public 
 * License version 2. This program is licensed "as is" without any 
 * warranty of any kind, whether express or implied.
 */

#include <linux/config.h>
#include <linux/init.h>
#include <linux/sched.h>
#include <linux/time.h>
#include <linux/hrtime.h>
#include <linux/timex.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#ifdef CONFIG_VST
#ifndef CONFIG_HIGH_RES_TIMERS
#include <asm/hrtime.h>
#endif
#endif
#include <asm/errno.h>
#include <asm/hardware.h>
#include <asm/preem_latency.h>
#include <asm/mach/irq.h>
#include <asm/irq.h>

/*
 * It seems like the coding style for the OMAP 1510 is for everyone to
 * define their own constant that gives the timer frequency.  I found
 * LATCH, LATCH_RATE, cycles_per_jiffie, MPUTICKS_PER_USEC, CK_CLKIN,
 * TICKS_PER_USEC, DPM_MD_HZ and tb_ticks_per_second.  Most of these
 * aren't setup to be used by other code.  So... here's my version.
 */
#define TIMER_TICKS_PER_JIFFIE ((u32)12000000 / 2 / HZ)
/*
 * MAX_TIMER_TICKS has some leeway to make sure the timer doesn't wrap
 * before we update jiffies.
 */
#define MAX_TIMER_TICKS (~(u32)0 - 4*TIMER_TICKS_PER_JIFFIE)

int
schedule_hr_timer_int(unsigned ref_jiffies, int ref_cycles)
{
	int temp_cycles;
	volatile mputimer_regs_t * subhz_timer = mputimer_base(2);

	BUG_ON(ref_cycles < 0);

	/*
	 * Get offset from last jiffy
	 */
	temp_cycles = (ref_jiffies - jiffies) * arch_cycles_per_jiffy +
		ref_cycles - get_arch_cycles(jiffies);

	if(unlikely(temp_cycles <= 0))
		return -ETIME;

	subhz_timer->cntl = MPUTIM_CLOCK_ENABLE;
	subhz_timer->load_tim = temp_cycles;
	subhz_timer->cntl = MPUTIM_CLOCK_ENABLE | MPUTIM_ST;

	return 0;
}

int
get_arch_cycles(unsigned ref_jiffies)
{
	extern unsigned long systimer_mark;
	extern unsigned long do_getmachinecycles(void);
	int ret;
	unsigned temp_jiffies;
	unsigned diff_jiffies;

	do {
                /* snapshot jiffies */
		temp_jiffies = jiffies;
		barrier();

                /* calculate cycles since the current jiffy */
		ret = do_getmachinecycles() - systimer_mark;

                /* compensate for ref_jiffies in the past */
                if (unlikely(diff_jiffies = jiffies - ref_jiffies))
                        ret += diff_jiffies * arch_cycles_per_jiffy;

		barrier();	
                /* repeat if we didn't have a consistent view of the world */
	} while(unlikely(temp_jiffies != jiffies));

	return ret;
}

int schedule_nonperiodic_timer_int(unsigned count)
{
	unsigned temp_cycles = count * arch_cycles_per_jiffy;
	volatile mputimer_regs_t * subhz_timer = mputimer_base(2);

	/*
  	 * We need to check to see if next_timer happens so far
  	 * out that we would overflow our interval timer.
 	 */
 	if (temp_cycles > MAX_TIMER_TICKS)
         	temp_cycles = MAX_TIMER_TICKS;

	subhz_timer->cntl = MPUTIM_CLOCK_ENABLE;
	subhz_timer->load_tim = temp_cycles;
	subhz_timer->cntl = MPUTIM_CLOCK_ENABLE | MPUTIM_ST;
	return 0;
}

#ifndef do_hr_timer_int()
#define do_hr_timer_int() mark_bh(TIMER_BH)
#endif

extern void update_jiffies_vst();
static void
hr_timer_interrupt(int irq, void *dev_id, struct pt_regs *regs)
{
	volatile mputimer_regs_t * subhz_timer = mputimer_base(2);
	subhz_timer->cntl = 0;
#ifdef CONFIG_VST
	update_jiffies_vst();
#endif
	do_hr_timer_int();
}

static struct irqaction hr_timer_irq = {
	.name		= "high-res timer",
	.handler	= hr_timer_interrupt,
	.flags		= SA_INTERRUPT
};

static int 
hr_timer_init(void)
{
	int ret;
	volatile mputimer_regs_t * subhz_timer = mputimer_base(2);

	subhz_timer->cntl = 0;
	ret = setup_arm_irq(INT_TIMER3, &hr_timer_irq);

	return ret;
}

__initcall(hr_timer_init);
