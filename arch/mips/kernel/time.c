/*
 * Copyright 2001, 2002, 2003 MontaVista Software Inc.
 * Author: Jun Sun, jsun@mvista.com or jsun@junsun.net
 *
 * Common time service routines for MIPS machines. See
 * Documents/MIPS/README.txt.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */
#include <linux/config.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/sched.h>
#include <linux/param.h>
#include <linux/time.h>
#include <linux/timer.h>
#include <linux/smp.h>
#include <linux/kernel_stat.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/hrtime.h>

#include <asm/bootinfo.h>
#include <asm/cpu.h>
#include <asm/time.h>
#include <asm/hardirq.h>
#include <asm/div64.h>
#include <asm/debug.h>

/* This is for machines which generate the exact clock. */
#define USECS_PER_JIFFY (1000000/HZ)
#define USECS_PER_JIFFY_FRAC ((u32)((1000000ULL << 32) / HZ))

/*
 * forward reference
 */
extern rwlock_t xtime_lock;
extern volatile unsigned long wall_jiffies;

spinlock_t rtc_lock = SPIN_LOCK_UNLOCKED;

/*
 * whether we emulate local_timer_interrupts for SMP machines.
 */
int emulate_local_timer_interrupt;

/*
 * By default we provide the null RTC ops
 */
static unsigned long null_rtc_get_time(void)
{
	return mktime(2000, 1, 1, 0, 0, 0);
}

static int null_rtc_set_time(unsigned long sec)
{
	return 0;
}

unsigned long (*rtc_get_time)(void) = null_rtc_get_time;
int (*rtc_set_time)(unsigned long) = null_rtc_set_time;


/*
 * timeofday services, for syscalls.
 */
void do_gettimeofday(struct timeval *tv)
{
	unsigned long flags, lost;

	read_lock_irqsave (&xtime_lock, flags);

	*tv = xtime;
	tv->tv_usec += do_gettimeoffset();

	/*
	 * xtime is atomically updated in timer_bh.  jiffies - wall_jiffies
	 * is nonzero if the timer bottom half hasn't executed yet.
	 */
	lost = jiffies - wall_jiffies;
	if (lost)
		tv->tv_usec += lost * USECS_PER_JIFFY;

	read_unlock_irqrestore (&xtime_lock, flags);

	if (tv->tv_usec >= 1000000) {
		tv->tv_usec -= 1000000;
		tv->tv_sec++;
	}
}

void do_settimeofday(struct timeval *tv)
{
	write_lock_irq (&xtime_lock);

	/* This is revolting. We need to set the xtime.tv_usec
	 * correctly. However, the value in this location is
	 * is value at the last tick.
	 * Discover what correction gettimeofday
	 * would have done, and then undo it!
	 */
	tv->tv_usec -= do_gettimeoffset();

	if (tv->tv_usec < 0) {
		tv->tv_usec += 1000000;
		tv->tv_sec--;
	}

	wall_to_monotonic.tv_sec += xtime.tv_sec - tv->tv_sec;
	wall_to_monotonic.tv_nsec += (xtime.tv_usec - tv->tv_usec) * 
		NSEC_PER_USEC;

	if (wall_to_monotonic.tv_nsec > NSEC_PER_SEC) {
		wall_to_monotonic.tv_nsec -= NSEC_PER_SEC;
		wall_to_monotonic.tv_sec++;
	}
	if (wall_to_monotonic.tv_nsec < 0) {
		wall_to_monotonic.tv_nsec += NSEC_PER_SEC;
		wall_to_monotonic.tv_sec--;
	}

	xtime = *tv;
	time_adjust = 0;			/* stop active adjtime() */
	time_status |= STA_UNSYNC;
	time_maxerror = NTP_PHASE_LIMIT;
	time_esterror = NTP_PHASE_LIMIT;

	write_unlock_irq (&xtime_lock);

	clock_was_set();
}


/*
 * Gettimeoffset routines.  These routines returns the time duration
 * since last timer interrupt in usecs.
 *
 * If the exact CPU counter frequency is known, use fixed_rate_gettimeoffset.
 * Otherwise use calibrate_gettimeoffset()
 *
 * If the CPU does not have counter register all, you can either supply
 * your own gettimeoffset() routine, or use null_gettimeoffset() routines,
 * which gives the same resolution as HZ.
 */


/* This is for machines which generate the exact clock. */
#define USECS_PER_JIFFY (1000000/HZ)

/* usecs per counter cycle, shifted to left by 32 bits */
static unsigned int sll32_usecs_per_cycle=0;

/* how many counter cycles in a jiffy */
unsigned long cycles_per_jiffy=0;

/* Cycle counter value at the previous timer interrupt.. */
static unsigned int timerhi, timerlo;

/* expirelo is the count value for next CPU timer interrupt */
static unsigned int expirelo;

/* last time when xtime and rtc are sync'ed up */
static long last_rtc_update;

/* the function pointer to one of the gettimeoffset funcs*/
unsigned long (*do_gettimeoffset)(void) = null_gettimeoffset;

inline unsigned long __noinstrument clock_to_usecs(unsigned long cycles)
{
	unsigned long usecs;

	if (sll32_usecs_per_cycle == 0)
		return 0UL;

	__asm__("multu\t%1,%2\n\t"
	        "mfhi\t%0"
	        :"=r" (usecs)
	        :"r" (cycles),
	         "r" (sll32_usecs_per_cycle));

	return usecs;
}

unsigned long null_gettimeoffset(void)
{
	return 0;
}

unsigned long fixed_rate_gettimeoffset(void)
{
	u32 count;
	unsigned long res;

	/* Get last timer tick in absolute kernel time */
	count = read_c0_count();

	/* .. relative to previous jiffy (32 bits is enough) */
	count -= timerlo;

	__asm__("multu\t%1,%2\n\t"
	        "mfhi\t%0"
	        :"=r" (res)
	        :"r" (count),
	         "r" (sll32_usecs_per_cycle));

	/*
	 * Due to possible jiffies inconsistencies, we need to check
	 * the result so that we'll get a timer that is monotonic.
	 */
	if (res >= USECS_PER_JIFFY)
		res = USECS_PER_JIFFY-1;

	return res;
}

/*
 * Cached "1/(clocks per usec)*2^32" value.
 * It has to be recalculated once each jiffy.
 */
static unsigned long cached_quotient;

/* Last jiffy when calibrate_divXX_gettimeoffset() was called. */
static unsigned long last_jiffies = 0;


/*
 * This is copied from dec/time.c:do_ioasic_gettimeoffset() by Mercij.
 */
unsigned long calibrate_div32_gettimeoffset(void)
{
	u32 count;
	unsigned long res, tmp;
	unsigned long quotient;

	tmp = jiffies;

	quotient = cached_quotient;

	if (last_jiffies != tmp) {
		last_jiffies = tmp;
		if (last_jiffies != 0) {
			unsigned long r0;
			do_div64_32(r0, timerhi, timerlo, tmp);
			do_div64_32(quotient, USECS_PER_JIFFY,
			            USECS_PER_JIFFY_FRAC, r0);
			cached_quotient = quotient;
		}
	}

	/* Get last timer tick in absolute kernel time */
	count = read_c0_count();

	/* .. relative to previous jiffy (32 bits is enough) */
	count -= timerlo;

	__asm__("multu  %2,%3"
	        : "=l" (tmp), "=h" (res)
	        : "r" (count), "r" (quotient));

	/*
	 * Due to possible jiffies inconsistencies, we need to check
	 * the result so that we'll get a timer that is monotonic.
	 */
	if (res >= USECS_PER_JIFFY)
		res = USECS_PER_JIFFY - 1;

	return res;
}

unsigned long calibrate_div64_gettimeoffset(void)
{
	u32 count;
	unsigned long res, tmp;
	unsigned long quotient;

	tmp = jiffies;

	quotient = cached_quotient;

	if (tmp && last_jiffies != tmp) {
		last_jiffies = tmp;
		__asm__(".set\tnoreorder\n\t"
	        ".set\tnoat\n\t"
	        ".set\tmips3\n\t"
	        "lwu\t%0,%2\n\t"
	        "dsll32\t$1,%1,0\n\t"
	        "or\t$1,$1,%0\n\t"
	        "ddivu\t$0,$1,%3\n\t"
	        "mflo\t$1\n\t"
	        "dsll32\t%0,%4,0\n\t"
	        "nop\n\t"
	        "ddivu\t$0,%0,$1\n\t"
	        "mflo\t%0\n\t"
	        ".set\tmips0\n\t"
	        ".set\tat\n\t"
	        ".set\treorder"
	        :"=&r" (quotient)
	        :"r" (timerhi),
	         "m" (timerlo),
	         "r" (tmp),
	         "r" (USECS_PER_JIFFY));
	        cached_quotient = quotient;
	}

	/* Get last timer tick in absolute kernel time */
	count = read_c0_count();

	/* .. relative to previous jiffy (32 bits is enough) */
	count -= timerlo;

	__asm__("multu\t%1,%2\n\t"
	        "mfhi\t%0"
	        :"=r" (res)
	        :"r" (count),
	         "r" (quotient));

	/*
	 * Due to possible jiffies inconsistencies, we need to check
	 * the result so that we'll get a timer that is monotonic.
	 */
	if (res >= USECS_PER_JIFFY)
		res = USECS_PER_JIFFY-1;

	return res;
}

#if defined(CONFIG_MIPS_COMMON_HRT)
int hr_time_resolution = CONFIG_HIGH_RES_RESOLUTION;

int get_arch_cycles(unsigned long ref_jiffies)
{
	int ret;
	int temp;
	unsigned long old_jiffies;
	unsigned long jiffy_expire;

	/* loop until we get an consistent reading of both vars */
	do {
		old_jiffies = jiffies;
		jiffy_expire = expirelo;
	} while (old_jiffies != jiffies);

	db_assert(time_before_eq(ref_jiffies, old_jiffies));
	db_assert(time_after_eq(ref_jiffies + 10, old_jiffies));

	ret = read_c0_count() + arch_cycles_per_jiffy - jiffy_expire;
	db_assert(ret > 0);
	if (unlikely(temp=old_jiffies - ref_jiffies)) 
		ret += temp * arch_cycles_per_jiffy;

	return ret;
}

static inline unsigned 
calc_next_expire(unsigned ref_jiffies, unsigned cycles)
{
	unsigned ret;
	int temp;
	unsigned long old_jiffies;
	unsigned long jiffy_expire;

	/* loop until we get an consistent reading of both vars */
	do {
		old_jiffies = jiffies;
		jiffy_expire = expirelo;
	} while (old_jiffies != jiffies);

	/* they should be off too much */
	db_assert(time_before_eq(ref_jiffies, old_jiffies+10));
	db_assert(time_after_eq(ref_jiffies + 10, old_jiffies));

	ret = jiffy_expire - arch_cycles_per_jiffy + cycles;
	if (unlikely(temp=ref_jiffies - old_jiffies)) 
		ret += temp * arch_cycles_per_jiffy;

	return ret;
}

int schedule_hr_timer_int(unsigned long ref_jiffies, int cycles)
{
	unsigned count;

	db_assert(intr_off());
	db_assert(time_before_eq(ref_jiffies, jiffies+10));
	db_assert(time_after_eq(ref_jiffies + 10, jiffies));
	db_assert(cycles <= arch_cycles_per_jiffy);

	count = (calc_next_expire(ref_jiffies, (unsigned)cycles));
	write_c0_compare(count);
	return time_after(read_c0_count(), count); 
}

static inline void schedule_timer_int_asap(void)
{
	unsigned inc = 0;
	unsigned inc_base=2;
	unsigned compare;

	db_assert(intr_off()); 

	do {
		inc += inc_base;
		compare = read_c0_count() + inc;
		write_c0_compare(compare);
	} while (time_after(read_c0_count(), compare));
}

int schedule_jiffies_int(unsigned long ref_jiffies)
{
	int ret = schedule_hr_timer_int(ref_jiffies, arch_cycles_per_jiffy);
	if (ret)
		schedule_timer_int_asap();
	return ret;
}

/*
 * return (u32)( ((u64)x * (u64)y) >> shift ), where 0 < shift <= 32
 */
static inline int scaled_mult(int x, int y, int shift)
{
	int hi;
	unsigned lo;

	db_assert(shift <= 32);
	db_assert(shift > 0);

	__asm__("mult\t%2,%3\n\t"
	        "mfhi\t%0\n\t"
	        "mflo\t%1"
	        :"=r" (hi), "=r" (lo)
	        :"r" (x), "r" (y));

	if (shift == 32)
		return hi;
	else
		return (hi << (32 - shift)) | (lo >> shift);
}

#define nsec2cycle_shift	32
#define cycle2nsec_shift	24
unsigned scaled_cycles_per_nsec, scaled_nsecs_per_cycle;

int nsec_to_arch_cycle(int nsec)
{
	return scaled_mult(nsec, scaled_cycles_per_nsec, nsec2cycle_shift);
}

int arch_cycle_to_nsec(int cycles)
{
	return scaled_mult(cycles, scaled_nsecs_per_cycle, cycle2nsec_shift);
}

static void hr_time_init(void)
{
	u64 temp;

	/* make sure no overflow for the scale factors */
	db_assert((mips_counter_frequency >> (32- nsec2cycle_shift)) < 
			1000000000);
	db_assert((1000000000 >> (32 - cycle2nsec_shift)) <
			mips_counter_frequency);

	/* 
	 * the current setting allow counter freq to range from
	 * a few MHz to 1GHz.
	 */

	temp = (u64)mips_counter_frequency << nsec2cycle_shift;
	do_div64_32(scaled_cycles_per_nsec, temp >> 32, temp, 1000000000);

	temp = (u64) 1000000000 << cycle2nsec_shift;
	do_div64_32(scaled_nsecs_per_cycle, temp >> 32, temp, mips_counter_frequency);
}

static unsigned long hrt_gettimeoffset(void)
{
	int ret;

	/* xtime_lock should be held if we are called */
	ret = get_arch_cycles(jiffies);
	ret = arch_cycle_to_nsec(ret) / 1000;

	/*
	if (ret >= USECS_PER_JIFFY)
		ret = USECS_PER_JIFFY-1;
	*/
	return ret;
}
#endif /* defined(CONFIG_MIPS_COMMON_HRT) */

/*
 * local_timer_interrupt() does profiling and process accounting
 * on a per-CPU basis.
 *
 * In UP mode, it is invoked from the (global) timer_interrupt.
 *
 * In SMP mode, it might invoked by per-CPU timer interrupt, or
 * a broadcasted inter-processor interrupt which itself is triggered
 * by the global timer interrupt.
 */
void local_timer_interrupt(int irq, void *dev_id, struct pt_regs *regs)
{
	if (!user_mode(regs)) {
		if (prof_buffer && current->pid) {
			extern int _stext;
			unsigned long pc = regs->cp0_epc;

			pc -= (unsigned long) &_stext;
			pc >>= prof_shift;
			/*
			 * Dont ignore out-of-bounds pc values silently,
			 * put them into the last histogram slot, so if
			 * present, they will show up as a sharp peak.
			 */
			if (pc > prof_len-1)
			pc = prof_len-1;
			atomic_inc((atomic_t *)&prof_buffer[pc]);
		}
	}

#ifdef CONFIG_SMP
	/* in UP mode, update_process_times() is invoked by do_timer() */
	update_process_times(user_mode(regs));
#endif
}

/*
 * high-level timer interrupt service routines.  This function
 * is set as irqaction->handler and is invoked through do_IRQ.
 */
static inline void do_timer_interrupt(struct pt_regs *regs)
{
	do_timer(regs);
	local_timer_interrupt(0, NULL, regs);
}

void timer_interrupt(int irq, void *dev_id, struct pt_regs *regs)
{
	write_lock (&xtime_lock);

	if (cpu_has_counter) {
		unsigned count;

#ifdef CONFIG_VST
		update_jiffies_vst();
#endif
#if defined(CONFIG_MIPS_COMMON_HRT)

		int do_timer_flag=0;
		for(;;) {
			write_c0_compare(expirelo);
			count = read_c0_count();
			if (time_before_eq(count, expirelo))
				break;

			do_timer_interrupt(regs);
			expirelo += cycles_per_jiffy;
			do_timer_flag++;
		}

		/* if we are not doing jiffy timer, we must be doing hr timer */
		if (!do_timer_flag) {
			do_hr_timer_int();
		}

#else /* defined(CONFIG_MIPS_COMMON_HRT) */

		/* ack timer interrupt, and try to set next interrupt */
		expirelo += cycles_per_jiffy;
		write_c0_compare(expirelo);
		count = read_c0_count();

		/* check to see if we have missed any timer interrupts */
		if (time_after(count, expirelo)) {
			/* missed_timer_count ++; */
			expirelo = count + cycles_per_jiffy;
			write_c0_compare(expirelo);
		}

		do_timer_interrupt(regs);

#endif /* defined(CONFIG_MIPS_COMMON_HRT) */

		/* Update timerhi/timerlo for intra-jiffy calibration. */
		timerhi += count < timerlo;	/* Wrap around */
		timerlo = count;

	} else {
		/*
		 * call the generic timer interrupt handling, we don't know if 
		 * we have missed any timer interrupt.
		 */
		do_timer_interrupt(regs);
	}

	/*
	 * If we have an externally synchronized Linux clock, then update
	 * CMOS clock accordingly every ~11 minutes. rtc_set_time() has to be
	 * called as close as possible to 500 ms before the new second starts.
	 */
	if ((time_status & STA_UNSYNC) == 0 &&
	    xtime.tv_sec > last_rtc_update + 660 &&
	    xtime.tv_usec >= 500000 - ((unsigned) tick) / 2 &&
	    xtime.tv_usec <= 500000 + ((unsigned) tick) / 2) {
		if (rtc_set_time(xtime.tv_sec) == 0) {
			last_rtc_update = xtime.tv_sec;
		} else {
			last_rtc_update = xtime.tv_sec - 600;
			/* do it again in 60 s */
		}
	}
	write_unlock (&xtime_lock);

	/*
	 * If jiffies has overflowed in this timer_interrupt we must
	 * update the timer[hi]/[lo] to make fast gettimeoffset funcs
	 * quotient calc still valid. -arca
	 */
	if (!jiffies) {
		timerhi = timerlo = 0;
	}
}

asmlinkage void ll_timer_interrupt(int irq, struct pt_regs *regs)
{
	int cpu = smp_processor_id();

	preempt_disable();

	irq_enter(cpu, irq);
	kstat.irqs[cpu][irq]++;

	/* we keep interrupt disabled all the time */
	timer_interrupt(irq, NULL, regs);

	irq_exit(cpu, irq);

	if (softirq_pending(cpu))
		do_softirq();

#if defined(CONFIG_PREEMPT) 
	for(;;) {
		preempt_enable_no_resched();
		if (preempt_is_disabled() || !need_resched())
			break;
		db_assert(intr_off());
		db_assert(!in_interrupt());

		preempt_disable();
		__sti();
		preempt_schedule();
		__cli();
	}
#endif
#ifdef CONFIG_ILATENCY
	intr_ret_from_exception();
#endif /* CONFIG_ILATENCY */
}

asmlinkage void ll_local_timer_interrupt(int irq, struct pt_regs *regs)
{
	int cpu = smp_processor_id();

	preempt_disable();

	irq_enter(cpu, irq);
	kstat.irqs[cpu][irq]++;

	/* we keep interrupt disabled all the time */
	local_timer_interrupt(irq, NULL, regs);

	irq_exit(cpu, irq);

	if (softirq_pending(cpu))
		do_softirq();

#if defined(CONFIG_PREEMPT)
	for(;;) {
		preempt_enable_no_resched();
		if (preempt_is_disabled() || !need_resched())
			break;

		db_assert(intr_off());
		db_assert(!in_interrupt());

		preempt_disable();
		__sti();
		preempt_schedule();
		__cli();
	}
#endif
#ifdef CONFIG_ILATENCY
	intr_ret_from_exception();
#endif /* CONFIG_ILATENCY */
}

/*
 * time_init() - it does the following things.
 *
 * 1) board_time_init() -
 * 	a) (optional) set up RTC routines,
 *      b) (optional) calibrate and set the mips_counter_frequency
 *	    (only needed if you intended to use fixed_rate_gettimeoffset
 *	     or use cpu counter as timer interrupt source)
 * 2) setup xtime based on rtc_get_time().
 * 3) choose a appropriate gettimeoffset routine.
 * 4) calculate a couple of cached variables for later usage
 * 5) board_timer_setup() -
 *	a) (optional) over-write any choices made above by time_init().
 *	b) machine specific code should setup the timer irqaction.
 *	c) enable the timer interrupt
 */

void (*board_time_init)(void) = NULL;
void (*board_timer_setup)(struct irqaction *irq) = NULL;

unsigned int mips_counter_frequency = 0;

static struct irqaction timer_irqaction = {
	timer_interrupt,
	SA_INTERRUPT,
	0,
	"timer",
	NULL,
	NULL
};

void __init time_init(void)
{
	if (board_time_init)
		board_time_init();

	xtime.tv_sec = rtc_get_time();
	xtime.tv_usec = 0;

	wall_to_monotonic.tv_sec = -xtime.tv_sec;
	wall_to_monotonic.tv_nsec = 0;

	/* choose appropriate gettimeoffset routine */
	if (!cpu_has_counter) {
		/* no cpu counter - sorry */
		do_gettimeoffset = null_gettimeoffset;
	} else if (mips_counter_frequency != 0) {
		/* we have cpu counter and know counter frequency! */
		do_gettimeoffset = fixed_rate_gettimeoffset;
	} else if ((current_cpu_data.isa_level == MIPS_CPU_ISA_M32) ||
		   (current_cpu_data.isa_level == MIPS_CPU_ISA_I) ||
		   (current_cpu_data.isa_level == MIPS_CPU_ISA_II) ) {
		/* we need to calibrate the counter but we don't have
		 * 64-bit division. */
		do_gettimeoffset = calibrate_div32_gettimeoffset;
	} else {
		/* we need to calibrate the counter but we *do* have
		 * 64-bit division. */
		do_gettimeoffset = calibrate_div64_gettimeoffset;
	}

#if defined(CONFIG_MIPS_COMMON_HRT)
	do_gettimeoffset = hrt_gettimeoffset;
#endif

	/* caclulate cache parameters */
	if (mips_counter_frequency) {
		cycles_per_jiffy = mips_counter_frequency / HZ;

		/* sll32_usecs_per_cycle = 10^6 * 2^32 / mips_counter_freq */
		/* any better way to do this? */
		sll32_usecs_per_cycle = mips_counter_frequency / 100000;
		sll32_usecs_per_cycle = 0xffffffff / sll32_usecs_per_cycle;
		sll32_usecs_per_cycle *= 10;

		/*
		 * For those using cpu counter as timer,  this sets up the
		 * first interrupt
		 */
		write_c0_compare(cycles_per_jiffy);
		write_c0_count(0);
		expirelo = cycles_per_jiffy;
	}

	/*
	 * Call board specific timer interrupt setup.
	 *
	 * this pointer must be setup in machine setup routine.
	 *
	 * Even if the machine choose to use low-level timer interrupt,
	 * it still needs to setup the timer_irqaction.
	 * In that case, it might be better to set timer_irqaction.handler
	 * to be NULL function so that we are sure the high-level code
	 * is not invoked accidentally.
	 */
	board_timer_setup(&timer_irqaction);

#if defined(CONFIG_MIPS_COMMON_HRT)
	if ( (read_c0_status() & CAUSEF_IP7) == 0) {
		/* we enable high res timers, but we are using CPU counter! */
		panic("Enabling high res timers without using CPU counter!");
	}

	hr_time_init();
#endif /* defined(CONFIG_MIPS_COMMON_HRT) */
}

#define FEBRUARY		2
#define STARTOFTIME		1970
#define SECDAY			86400L
#define SECYR			(SECDAY * 365)
#define leapyear(year)		((year) % 4 == 0)
#define days_in_year(a)		(leapyear(a) ? 366 : 365)
#define days_in_month(a)	(month_days[(a) - 1])

static int month_days[12] = {
	31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31
};

void to_tm(unsigned long tim, struct rtc_time * tm)
{
	long hms, day, gday;
	int i;

	gday = day = tim / SECDAY;
	hms = tim % SECDAY;

	/* Hours, minutes, seconds are easy */
	tm->tm_hour = hms / 3600;
	tm->tm_min = (hms % 3600) / 60;
	tm->tm_sec = (hms % 3600) % 60;

	/* Number of years in days */
	for (i = STARTOFTIME; day >= days_in_year(i); i++)
	day -= days_in_year(i);
	tm->tm_year = i;

	/* Number of months in days left */
	if (leapyear(tm->tm_year))
	days_in_month(FEBRUARY) = 29;
	for (i = 1; day >= days_in_month(i); i++)
	day -= days_in_month(i);
	days_in_month(FEBRUARY) = 28;
	tm->tm_mon = i-1;	/* tm_mon starts from 0 to 11 */

	/* Days are what is left over (+1) from all that. */
	tm->tm_mday = day + 1;

	/*
	 * Determine the day of week
	 */
	tm->tm_wday = (gday + 4) % 7; /* 1970/1/1 was Thursday */
}

/*
 * calibrate the mips_counter_frequency
 */
#define		CALIBRATION_CYCLES		20
void calibrate_mips_counter(void)
{
	volatile unsigned long ticks;
	volatile unsigned long countStart, countEnd;

	if (mips_counter_frequency) {
		/* it is already specified by the board */
		printk("MIPS CPU counter frequency is fixed at %d Hz\n",
			mips_counter_frequency);
		return;
	}

	if (!cpu_has_fpu) {
		/* we don't have cpu counter */
		return;
	}

	printk("calibrating MIPS CPU counter frequency ...");

	/* wait for the change of jiffies */
	ticks = jiffies;
	while (ticks == jiffies);

	/* read cpu counter */
	countStart = read_c0_count();

	/* loop for another n jiffies */
	ticks += CALIBRATION_CYCLES + 1;
	while (ticks != jiffies);

	/* read counter again */
	countEnd = read_c0_count();

	/* assuming HZ is 10's multiple */
	db_assert((HZ % CALIBRATION_CYCLES) == 0);

	mips_counter_frequency = 
		(countEnd - countStart) * (HZ / CALIBRATION_CYCLES);
	printk(" %d Hz\n", mips_counter_frequency);
}

EXPORT_SYMBOL(rtc_lock);
EXPORT_SYMBOL(cycles_per_jiffy);
#if defined(CONFIG_MIPS_COMMON_HRT)
EXPORT_SYMBOL(nsec_to_arch_cycle);
EXPORT_SYMBOL(arch_cycle_to_nsec);
#endif
