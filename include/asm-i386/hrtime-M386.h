/*
 *
 * File: include/asm-i386/hrtime-M386.h
 * Copyright (C) 1999 by the University of Kansas Center for Research, Inc.
 * Copyright (C) 2001 by MontaVista Software.
 *
 * This software was developed by the Information and
 * Telecommunication Technology Center (ITTC) at the University of
 * Kansas.  Partial funding for this project was provided by Sprint. This
 * software may be used and distributed according to the terms of the GNU
 * Public License, incorporated herein by reference.  Neither ITTC nor
 * Sprint accept any liability whatsoever for this product.
 *
 * This project was developed under the direction of Dr. Douglas Niehaus.
 * 
 * Authors: Balaji S., Raghavan Menon
 *	    Furquan Ansari, Jason Keimig, Apurva Sheth
 *
 * Thanx to Michael Barabanov for helping me with the non-pentium code.
 *
 * Please send bug-reports/suggestions/comments to utime@ittc.ukans.edu
 * 
 * Further details about this project can be obtained at
 *    http://hegel.ittc.ukans.edu/projects/utime/ 
 *    or in the file Documentation/utime.txt
 */
/* This is in case its not a pentuim or a ppro.
 * we dont have access to the cycle counters
 */
/* 
 * This code swiped from the utime project to support high res timers
 * Principle thief George Anzinger george@mvista.com
 */
#ifndef _ASM_HRTIME_M386_H
#define _ASM_HRTIME_M386_H

#ifdef __KERNEL__


extern int base_c0,base_c0_offset;
#define timer_latch_reset(x) _timer_latch_reset = x
extern int _timer_latch_reset;

/*
 * Never call this routine with local ints on.
 * update_jiffies_sub()
 */

extern inline unsigned int read_timer_chip(void)
{
	unsigned int next_intr;

	LATCH_CNT0();
	READ_CNT0(next_intr);
	return next_intr;
}

/* This routine would keep track of the lost jiffies_u
 * for architectures that have decimal places in cycles_per_usec
 * ie for the 386/486 cycles_per_usec = 1.1932
 */
#define HR_SCALE_ARCH_NSEC 20
#define HR_SCALE_ARCH_USEC 30
#define HR_SCALE_NSEC_ARCH 32
#define HR_SCALE_USEC_ARCH 29

#define cf_arch_to_usec (SC_n(HR_SCALE_ARCH_USEC,1000000)/ \
                           (long long)CLOCK_TICK_RATE)

extern inline int arch_cycles_to_usec(long update)
{
	return (mpy_sc_n(HR_SCALE_ARCH_USEC, update ,arch_to_usec));
}
#define cf_arch_to_nsec (SC_n(HR_SCALE_ARCH_NSEC,1000000000)/ \
                           (long long)CLOCK_TICK_RATE)

extern inline int arch_cycles_to_nsec(long update)
{
        return mpy_sc_n(HR_SCALE_ARCH_NSEC,  update, arch_to_nsec);
}
/* 
 * And the other way...
 */
#define cf_usec_to_arch (SC_n( HR_SCALE_USEC_ARCH,CLOCK_TICK_RATE)/ \
                                            (long long)1000000)
extern inline int usec_to_arch_cycles(unsigned long usec)
{
        return mpy_sc_n(HR_SCALE_USEC_ARCH,usec,usec_to_arch);
}
#define cf_nsec_to_arch (SC_n( HR_SCALE_NSEC_ARCH,CLOCK_TICK_RATE)/ \
                                            (long long)1000000000)
extern inline int nsec_to_arch_cycles(long nsec)
{
        return (mpy_ex32(nsec,nsec_to_arch));
}
/*
 * If this is defined otherwise to allow NTP adjusting, it should
 * be scaled by about 16 bits (or so) to allow small percentage
 * changes
 */
#define arch_cycles_to_latch(x) x
/*
 * This function updates base_c0
 * This function is always called under the write_lock_irq(&xtime_lock)
 * It returns the number of "clocks" since the last call to it.
 *
 * There is a problem having a counter that has a period the same as it is
 * interagated.  I.e. did it just roll over or has a very short time really
 * elapsed.  (One of the reasons one should not use the PIT for both ints
 * and time.)  We will take tha occurance of an interrupt since last time
 * to indicate that the counter has reset.  This will work will for the 
 * get_cpuctr() code but is flawed for the quick_get_cpuctr() as it is
 * called when ever time is requested.  For that code, we make sure that
 * we never move backward in time.
 */
extern inline  unsigned long get_cpuctr(void)
{
	int c0;
	long rtn;

	spin_lock(&i8253_lock);
	c0 = read_timer_chip();

        rtn = base_c0 - c0 + _timer_latch_reset;

/*	if (rtn < 0) { */
/*                rtn += _timer_latch_reset; */
/*        } */
	base_c0 = c0;
        base_c0_offset = 0;
	spin_unlock(&i8253_lock);

	return rtn;
}
/*
 * In an SMP system this is called under the read_lock_irq(xtime_lock)
 * In a UP system it is also called with this lock (PIT case only)
 * It returns the number of "clocks" since the last call to get_cpuctr (above).
 */
extern inline unsigned long quick_get_cpuctr(void)
{
	register  int c0;
        long rtn;

	spin_lock(&i8253_lock);
	c0 = read_timer_chip();
        /*
         * If the new count is greater than 
         * the last one (base_c0) the chip has just rolled and an 
         * interrupt is pending.  To get the time right. We need to add
         * _timer_latch_reset to the answer.  All this is true if only
         * one roll is involved, but base_co should be updated at least
         * every 1/HZ.
         */
        rtn = base_c0 - c0;
	if (rtn < base_c0_offset) {
                rtn += _timer_latch_reset;
        }
        base_c0_offset = rtn;
	spin_unlock(&i8253_lock);
        return rtn;
}

#ifdef _INCLUDED_FROM_TIME_C
int base_c0 = 0;
int base_c0_offset = 0;
struct timer_conversion_bits timer_conversion_bits = {
        _cycles_per_jiffies: (LATCH),
        _nsec_to_arch:       cf_nsec_to_arch,
        _usec_to_arch:       cf_usec_to_arch,
        _arch_to_nsec:       cf_arch_to_nsec,
        _arch_to_usec:       cf_arch_to_usec,
        _arch_to_latch:      1
};
int _timer_latch_reset;

#define set_last_timer_cc() (void)(1)

/* This returns the correct cycles_per_sec from a calibrated one
 */
#define arch_hrtime_init(x) (CLOCK_TICK_RATE)

/*
 * The reload_timer_chip routine is called under the timerlist lock (irq off)
 * and, in SMP, the xtime_lock.  We also take the i8253_lock for the chip access
 */

extern inline void reload_timer_chip( int new_latch_value)
{
	int c1, c1new, delta;
        unsigned char pit_status;
	/*
         * In put value is in timer units for the 386 platform.
         * We must be called with irq disabled.
	 */
	spin_lock(&i8253_lock);
	/*
         * we need to get this last value of the timer chip
	 */
	LATCH_CNT0_AND_CNT1();
	READ_CNT0(delta);
	READ_CNT1(c1);
	base_c0 -= delta;

	new_latch_value = arch_cycles_to_latch( new_latch_value );
        if (new_latch_value < TIMER_DELTA){
                new_latch_value = TIMER_DELTA;
        }
        IF_ALL_PERIODIC( put_timer_in_periodic_mode());
        outb_p(new_latch_value & 0xff, PIT0);	/* LSB */
	outb(new_latch_value >> 8, PIT0);	/* MSB */
        do {
                outb_p(PIT0_LATCH_STATUS,PIT_COMMAND);
                pit_status = inb(PIT0);
        }while (pit_status & PIT_NULL_COUNT);
        do {
                LATCH_CNT0_AND_CNT1();
                READ_CNT0(delta);
                READ_CNT1(c1new);
        } while (!(((new_latch_value-delta)&0xffff) < 15));

        IF_ALL_PERIODIC(
                outb_p(LATCH & 0xff, PIT0);	/* LSB */
                outb(LATCH >> 8, PIT0);	        /* MSB */
                )

	/*
         * this is assuming that the counter one is latched on with
	 * 18 as the value
	 * Most BIOSes do this i guess....
	 */
        /*IF_DEBUG(if (delta > 50000) BREAKPOINT); */
        c1 -= c1new;
	base_c0 += ((c1 < 0) ? (c1 + 18) : (c1)) + delta;
        if ( base_c0 < 0 ){
                base_c0 += _timer_latch_reset;
        }
	spin_unlock(&i8253_lock);
	return;
}
/*
 * No run time conversion factors need to be set up as the PIT has a fixed
 * speed.
 */
#define init_hrtimers()

#endif /* _INCLUDED_FROM_HRTIME_C_ */
#define final_clock_init()
#endif /* __KERNEL__ */
#endif /* _ASM_HRTIME_M386_H */

