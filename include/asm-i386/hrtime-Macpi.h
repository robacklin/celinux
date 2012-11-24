/*
 * include/linux/asm-i386/hrtime-Macpi.h
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
#include <asm/io.h>
#ifndef _ASM_HRTIME_Macpi_H
#define _ASM_HRTIME_Macpi_H

#ifdef __KERNEL__

/*
 * These are specific to the ACPI pm counter
 * The spec says the counter can be either 32 or 24 bits wide.  We treat them
 * both as 24 bits.  Its faster than doing the test.
 */
#define SIZE_MASK 0xffffff

extern int acpi_pm_tmr_address;

extern inline unsigned long
quick_get_cpuctr(void)
{
	return (inl(acpi_pm_tmr_address) - last_update) & SIZE_MASK;
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
 */
#define HR_SCALE_ARCH_NSEC 22
#define HR_SCALE_ARCH_USEC 32
#define HR_SCALE_NSEC_ARCH 32
#define HR_SCALE_USEC_ARCH 29

#ifndef  PM_TIMER_FREQUENCY
#define PM_TIMER_FREQUENCY  3579545	/*45   counts per second */
#endif
#define PM_TIMER_FREQUENCY_x_100  357954545	/* counts per second * 100 */

#define cf_arch_to_usec (SC_32(100000000)/(long long)PM_TIMER_FREQUENCY_x_100)

extern inline int
arch_cycle_to_usec(unsigned long update)
{
	return (mpy_sc32(update, arch_to_usec));
}
/* 
 * Note: In the SMP case this value will be overwritten when the APIC
 * clocks are figured out using the "compute_latch function below.  If
 * the system is not SMP, the PIT is the ticker and this is the
 * conversion for that.
 */
#define cf_arch_to_latch SC_32(CLOCK_TICK_RATE)/(long long)(CLOCK_TICK_RATE * 3)

#ifndef USE_APIC_TIMERS
/*
 * We need to take 1/3 of the presented value (or more exactly)
 * CLOCK_TICK_RATE /PM_TIMER_FREQUENCY.  Note that these two timers are
 * on the same cyrstal so will be EXACTLY 1/3.
 */
extern inline int
arch_cycle_to_latch(unsigned long update)
{
	return (mpy_sc32(update, arch_to_latch));
}
#else
/*
 * APIC clocks run from a low of 33MH to say 200MH.  The PM timer runs
 * about 3.5 MH.  We want to scale so that ( APIC << scale )/PM is less
 * 2 ^ 32.  Lets use 2 ^ 19, leaves plenty of room.
 */
#define HR_SCALE_ARCH_LATCH 19

#define compute_latch(APIC_clocks_jiffie) arch_to_latch = div_sc_n(   \
                                                    HR_SCALE_ARCH_LATCH,   \
				                    APIC_clocks_jiffie,   \
				                    arch_cycles_per_jiffy);
extern inline int
arch_cycle_to_latch(unsigned long update)
{
	return (mpy_sc_n(HR_SCALE_ARCH_LATCH, update, arch_to_latch));
}

#endif

#define cf_arch_to_nsec (SC_n(HR_SCALE_ARCH_NSEC,100000000000LL)/ \
                           (long long)PM_TIMER_FREQUENCY_x_100)

extern inline int
arch_cycle_to_nsec(long update)
{
	return mpy_sc_n(HR_SCALE_ARCH_NSEC, update, arch_to_nsec);
}
/* 
 * And the other way...
 */
#define cf_usec_to_arch (SC_n( HR_SCALE_USEC_ARCH,PM_TIMER_FREQUENCY_x_100)/ \
                                            (long long)100000000)
extern inline int
usec_to_arch_cycle(unsigned long usec)
{
	return mpy_sc_n(HR_SCALE_USEC_ARCH, usec, usec_to_arch);
}
#define cf_nsec_to_arch (SC_n( HR_SCALE_NSEC_ARCH,PM_TIMER_FREQUENCY)/ \
                                            (long long)1000000000)
extern inline int
nsec_to_arch_cycle(unsigned long nsec)
{
	return mpy_sc32(nsec, nsec_to_arch);
}

extern int hrt_get_acpi_pm_ptr(void);

#ifdef _INCLUDED_FROM_TIME_C

#include <asm/io.h>
struct timer_conversion_bits timer_conversion_bits = {
	_arch_cycles_per_jiffy:((PM_TIMER_FREQUENCY + HZ / 2) / HZ),
	_nsec_to_arch:cf_nsec_to_arch,
	_usec_to_arch:cf_usec_to_arch,
	_arch_to_nsec:cf_arch_to_nsec,
	_arch_to_usec:cf_arch_to_usec,
	_arch_to_latch:cf_arch_to_latch
};
int acpi_pm_tmr_address;


/*
 * No run time conversion factors need to be set up as the pm timer has a fixed
 * speed.
 *
 * Here we have a local udelay for our init use only.  The system delay has
 * has not yet been calibrated when we use this, however, we do know
 * tsc_cycles_per_5_jiffies...
 */
extern unsigned long tsc_cycles_per_5_jiffies;

static inline __init void hrt_udelay(int usec)
{
        long now,end;
        rdtscl(end);
        end += (usec * tsc_cycles_per_5_jiffies) / (USEC_PER_JIFFIES * 5);
        do {rdtscl(now);} while((end - now) > 0);

}

#if defined( CONFIG_HIGH_RES_TIMER_ACPI_PM_ADD) && CONFIG_HIGH_RES_TIMER_ACPI_PM_ADD > 0
#define default_pm_add CONFIG_HIGH_RES_TIMER_ACPI_PM_ADD
#define message "High-res-timers: ACPI pm timer not found.  Trying specified address %d\n"
#else
#define default_pm_add 0
#define message \
        "High-res-timers: ACPI pm timer not found(%d) and no backup."\
        "\nCheck BIOS settings or supply a backup.  See configure documentation.\n"
#endif
#define fail_message \
"High-res-timers: >-<--><-->-<-->-<-->-<--><-->-<-->-<-->-<-->-<-->-<-->-<-->-<\n"\
"High-res-timers: >Failed to find the ACPI pm timer                           <\n"\
"High-res-timers: >-<--><-->-<-->-<-->-<-->Boot will fail in Calibrate Delay  <\n"\
"High-res-timers: >Supply a valid default pm timer address                    <\n"\
"High-res-timers: >or get your BIOS to turn on ACPI support.                  <\n"\
"High-res-timers: >See CONFIGURE help for more information.                   <\n"\
"High-res-timers: >-<--><-->-<-->-<-->-<--><-->-<-->-<-->-<-->-<-->-<-->-<-->-<\n"
/*
 * After we get the address, we set last_update to the current timer value
 */
static inline __init void  init_hrtimers(void)
{
        acpi_pm_tmr_address = hrt_get_acpi_pm_ptr(); 
        if (!acpi_pm_tmr_address){                    
                printk(message,default_pm_add);
                if ( (acpi_pm_tmr_address = default_pm_add)){
                        last_update +=  quick_get_cpuctr();
                        hrt_udelay(4);
                       if (!quick_get_cpuctr()){
                                printk("High-res-timers: No ACPI pm timer found at %d.\n",
                                       acpi_pm_tmr_address);
                                acpi_pm_tmr_address = 0;
                        } 
                } 
        }else{
                if (default_pm_add != acpi_pm_tmr_address) {
                        printk("High-res-timers: Ignoring supplied default ACPI pm timer address.\n"); 
                }
                last_update +=  quick_get_cpuctr();
        }
        if (!acpi_pm_tmr_address){
                printk(fail_message);
        }else{
                printk("High-res-timers: Found ACPI pm timer at %d\n",
                       acpi_pm_tmr_address);
        }
}

#endif   /* _INCLUDED_FROM_TIME_C_ */
#endif				/* __KERNEL__ */
#endif				/* _ASM_HRTIME-Mapic_H */
