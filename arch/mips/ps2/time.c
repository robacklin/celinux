/* $Id: time.c,v 1.1 2001/10/28 09:19:07 lethal Exp $
 *
 *  Copyright (C) 1991, 1992, 1995  Linus Torvalds
 *  Copyright (C) 1996, 1997, 1998  Ralf Baechle
 *  Copyright (C) 2000  Sony Computer Entertainment Inc.
 *  Copyright (C) 2001  Paul Mundt <lethal@chaoticdreams.org>
 *
 * This file contains the time handling details for PC-style clocks as
 * found in some MIPS systems.
 */
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/kernel_stat.h>
#include <linux/sched.h>
#include <linux/kernel.h>
#include <linux/param.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/interrupt.h>

#include <asm/bootinfo.h>
#include <asm/mipsregs.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/ps2/irq.h>

#include <linux/mc146818rtc.h>
#include <linux/timex.h>

#define CPU_FREQ  294912000		/* CPU clock frequency (Hz) */
#define BUS_CLOCK (CPU_FREQ/2)		/* bus clock frequency (Hz) */
#define TM0_COMP  (BUS_CLOCK/256/HZ)	/* to generate 100Hz */

static volatile int *tm0_count = (volatile int *)0xb0000000;
static volatile int *tm0_mode  = (volatile int *)0xb0000010;
static volatile int *tm0_comp  = (volatile int *)0xb0000020;

static unsigned int last_cycle_count;
static int timer_intr_delay;

struct ps2_rtc {
	u_char padding_1;
	u_char sec;
	u_char min;
	u_char hour;
	u_char padding_2;
	u_char day;
	u_char mon;
	u_char year;
} *prtc = (struct ps2_rtc *)0x81fff010;

/**
 * 	ps2_do_gettimeoffset - Get Time Offset
 *
 * 	Returns the time duration since the last timer
 * 	interrupt in usecs.
 */
static unsigned long ps2_do_gettimeoffset(void)
{
	unsigned int count;
	int delay;

	count = read_32bit_cp0_register(CP0_COUNT);
	count -= last_cycle_count;
	count = (count * 1000 + (CPU_FREQ / 1000 / 2)) / (CPU_FREQ / 1000);
	delay = (timer_intr_delay * 10000 + (TM0_COMP / 2)) / TM0_COMP;

	return delay + count;
}

/**
 *	ps2_rtc_set_time - Set RTC Time
 *
 *	@nowtime: Time to set
 *
 *	Sets the RTC to the given time @nowtime.
 *
 *	In order to set the CMOS clock precisely, this routine has
 *	to be called 500 ms after the second @nowtime has started,
 *	because when @nowtime is written into the registers of
 *	the CMOS clock, it will jump to the next second precisely
 *	500 ms later. Check the Motorola MC146818A or Dallas
 *	DS12887 data sheet for details.
 *
 *	BUG: This routine does not handle hour overflow properly;
 *	     it just sets the minutes. Usually you won't notice
 *	     until after reboot!
 */
static int ps2_rtc_set_time(unsigned long nowtime)
{
	int retval = 0;
	int real_seconds, real_minutes, cmos_minutes;
	unsigned char save_control, save_freq_select;

	/* tell the clock it's being set */
	save_control = CMOS_READ(RTC_CONTROL);
	CMOS_WRITE((save_control | RTC_SET), RTC_CONTROL);

	/* stop and reset prescaler */
	save_freq_select = CMOS_READ(RTC_FREQ_SELECT);
	CMOS_WRITE((save_freq_select | RTC_DIV_RESET2), RTC_FREQ_SELECT);

	cmos_minutes = CMOS_READ(RTC_MINUTES);

	if (!(save_control & RTC_DM_BINARY) || RTC_ALWAYS_BCD)
		BCD_TO_BIN(cmos_minutes);

	/*
	 * Since we're only adjusting minutes and seconds,
	 * don't interfere with hour overflow. This avoids
	 * messing with unknown time zones but requires your
	 * RTC not to be off by more than 15 minutes
	 */
	real_seconds = nowtime % 60;
	real_minutes = nowtime / 60;

	if (((abs(real_minutes - cmos_minutes) + 15)/30) & 1)
		/* correct for half hour time zone */
		real_minutes += 30;

	real_minutes %= 60;

	if (abs(real_minutes - cmos_minutes) < 30) {
		if (!(save_control & RTC_DM_BINARY) || RTC_ALWAYS_BCD) {
			BIN_TO_BCD(real_seconds);
			BIN_TO_BCD(real_minutes);
		}
		CMOS_WRITE(real_seconds,RTC_SECONDS);
		CMOS_WRITE(real_minutes,RTC_MINUTES);
	} else {
		printk(KERN_WARNING
		       "ps2_rtc_set_time: can't update from %d to %d\n",
		       cmos_minutes, real_minutes);
 		retval = -1;
	}

	/* 
	 * The following flags have to be released exactly in this order,
	 * otherwise the DS12887 (popular MC146818A clone with integrated
	 * battery and quartz) will not reset the oscillator and will not
	 * update precisely 500 ms later. You won't find this mentioned in
	 * the Dallas Semiconductor data sheets, but who believes data
	 * sheets anyway ...                           -- Markus Kuhn
	 */
	CMOS_WRITE(save_control, RTC_CONTROL);
	CMOS_WRITE(save_freq_select, RTC_FREQ_SELECT);

	return retval;
}

/**
 * 	ps2_timer_interrupt - Timer Interrupt Routine
 *
 * 	@irq:    interrupt
 * 	@dev_id: device ID
 * 	@regs:   registers as they appear on the stack
 *	         during a syscall/exception.
 * 	
 * 	Timer interrupt routine, wraps the generic timer_interrupt() but
 * 	sets the timer interrupt delay and clears interrupts first.
 */
static void ps2_timer_interrupt(int irq, void *dev_id, struct pt_regs *regs)
{
	int cpu = smp_processor_id();

        irq_enter(cpu, IRQ_INTC_TIMER0);
        kstat.irqs[0][IRQ_INTC_TIMER0]++;

	/* Set the timer interrupt delay */
	timer_intr_delay = *tm0_count;

	/* Clear the interrupt */
	*tm0_mode = *tm0_mode;

	timer_interrupt(irq, dev_id, regs);
	irq_exit(cpu, IRQ_INTC_TIMER0);
}

/**
 * 	ps2_timer_setup - Timer Setup Routine
 *
 * 	@irq: Structure defining interrupt.
 *
 * 	Registers timer interrupt routine with appropriate
 * 	interrupt.
 */
static void ps2_timer_setup(struct irqaction *irq)
{
	/* Setup our handler */
	irq->handler = ps2_timer_interrupt;

	/* Setup interrupt */
	setup_irq(IRQ_INTC_TIMER0, irq);
}

/**
 * 	ps2_init_time - Initialize Clock
 *
 * 	Loads up the RTC with an almost useful default value
 * 	and registers our callbacks.
 */
void __init ps2_init_time(void)
{
	rtc_set_time      = ps2_rtc_set_time;
	do_gettimeoffset  = ps2_do_gettimeoffset;
	board_timer_setup = ps2_timer_setup;

	/*
	 * FIXME: This is really dependant on what the timezone is set to,
	 * contrary to popular belief, not everyone hacking PS2 code is in a
	 * JST timezone .. Anyone have any clues on how to probe the timezone
	 * information from the thing so this can be done dynamically? Static
	 * timezone configuration needs to be drug out and shot. -Lethal
	 */

	/* convert JST(UTC-9) to UTC */
	ps2_rtc_set_time(mktime(BCD_TO_BIN(prtc->year) + 2000,
				BCD_TO_BIN(prtc->mon),
				BCD_TO_BIN(prtc->day),
				BCD_TO_BIN(prtc->hour),
				BCD_TO_BIN(prtc->min),
				BCD_TO_BIN(prtc->sec) - 60 * 60 * 9));

	/* setup 100Hz interval timer */
	*tm0_count = 0;
	*tm0_comp = TM0_COMP;

	/* busclk / 256, zret, cue, cmpe, equf */
	*tm0_mode = 2 | (1 << 6) | (1 << 7) | (1 << 8) | (1 << 10);

	clear_cp0_status(ST0_IM);
	set_cp0_status(IE_IRQ0 | IE_IRQ1);
}

