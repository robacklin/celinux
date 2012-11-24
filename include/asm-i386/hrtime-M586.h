/*
 * include/linux/asm-i386/hrtime-M586.h
 *
 *
 * 2003-7-7  Posix Clocks & timers 
 *                           by George Anzinger george@mvista.com
 *
 *			     Copyright (C) 2003 by MontaVista Software.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or (at
 * your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.

 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 * MontaVista Software | 1237 East Arques Avenue | Sunnyvale | CA 94085 | USA 
 */
#include <asm/msr.h>
#ifndef _ASM_HRTIME_M586_H
#define _ASM_HRTIME_M586_H

#ifdef __KERNEL__

#ifdef _INCLUDED_FROM_TIME_C
/*
 * This gets redefined when we calibrate the TSC
 */
struct timer_conversion_bits timer_conversion_bits = {
	_arch_cycles_per_jiffy:LATCH
};
#endif

extern inline unsigned long
quick_get_cpuctr(void)
{
	unsigned long value;
	rdtscl(value);
	return value - last_update;
}
/*
 * This function moves the last_update value to the closest to the
 * current 1/HZ boundry.  This MUST be called under the write xtime_lock.
 */
extern inline unsigned long
stake_cpuctr(void)
{
	if ((unsigned)quick_get_cpuctr() > arch_cycles_per_jiffy) {
		last_update += arch_cycles_per_jiffy;
	}
}
#define arch_hrtime_init(x) (x)

/* 
 * We use various scaling.  The sc32 scales by 2**32, sc_n by the first
 * parm.  When working with constants, choose a scale such that
 * x/n->(32-scale)< 1/2.  So for 1/3 <1/2 so scale of 32, where as 3/1
 * must be shifted 3 times (3/8) to be less than 1/2 so scale should be
 * 29
 *
 * The principle high end is when we can no longer keep 1/HZ worth of
 * arch time (TSC counts) in an integer.  This will happen somewhere
 * between 40GHz and 50GHz with HZ set to 100.  For now we are cool and
 * the scale of 24 works for the nano second to arch from 2MHz to
 * 40+GHz.
 */
#define HR_TIME_SCALE_NSEC 22
#define HR_TIME_SCALE_USEC 14
extern inline int
arch_cycle_to_usec(unsigned long update)
{
	return (mpy_sc32(update, arch_to_usec));
}
/*
 * We use the same scale for both the pit and the APIC
 */
extern inline int
arch_cycle_to_latch(unsigned long update)
{
	return (mpy_sc32(update, arch_to_latch));
}
#define compute_latch(APIC_clocks_jiffie) arch_to_latch = \
                                             div_sc32(APIC_clocks_jiffie, \
				                      arch_cycle_per_jiffy);

extern inline int
arch_cycle_to_nsec(long update)
{
	return mpy_sc_n(HR_TIME_SCALE_NSEC, update, arch_to_nsec);
}
/* 
 * And the other way...
 */
extern inline int
usec_to_arch_cycle(unsigned long usec)
{
	return mpy_sc_n(HR_TIME_SCALE_USEC, usec, usec_to_arch);
}
extern inline int
nsec_to_arch_cycle(unsigned long nsec)
{
	return mpy_sc_n(HR_TIME_SCALE_NSEC, nsec, nsec_to_arch);
}

EXTERN int pit_pgm_correction;

#ifdef _INCLUDED_FROM_TIME_C

#include <asm/io.h>

#ifndef USEC_PER_SEC
#define USEC_PER_SEC 1000000
#endif
	/*
	 * Code for runtime calibration of high res timers.  Watch out,
	 * cycles_per_sec will overflow when we get a ~ 2.14 GHz
	 * machine...  We are starting with tsc_cycles_per_5_jiffies set
	 * to 5 times the actual value (as set by calibrate_tsc() ).
	 */
#define init_hrtimers()							\
        arch_to_usec = fast_gettimeoffset_quotient;			\
 									\
        arch_to_latch = div_ll_X_l(					\
		mpy_l_X_l_ll(fast_gettimeoffset_quotient, 		\
			     CLOCK_TICK_RATE),           		\
		             (USEC_PER_SEC));				\
									\
        arch_to_nsec = div_sc_n(HR_TIME_SCALE_NSEC,			\
                               CALIBRATE_TIME * NSEC_PER_USEC,		\
                               tsc_cycles_per_5_jiffies);		\
									\
        nsec_to_arch = div_sc_n(HR_TIME_SCALE_NSEC,			\
                                tsc_cycles_per_5_jiffies,		\
                                CALIBRATE_TIME * NSEC_PER_USEC);	\
        usec_to_arch = div_sc_n(HR_TIME_SCALE_USEC,			\
                                tsc_cycles_per_5_jiffies,		\
                                CALIBRATE_TIME );			\
        arch_cycles_per_jiffy = tsc_cycles_per_5_jiffies / CAL_JIFS;

#endif				/* _INCLUDED_FROM_HRTIME_C */
#endif				/* __KERNEL__ */
#endif				/* _ASM_HRTIME-M586_H */
