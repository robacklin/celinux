/*
 * include/asm-arm/arch-adifcc/preem_latency.h
 *
 * Timing support for preemptible kernel latency measurement times
 *
 * The readclock() macro must return the contents of a monotonic incremental
 * counter. The Intel XScale Performance Monitor clock counter register is
 * ideal for this purpose as it is a 32-bit up counter which is optionally
 * clocked directly from the processor core clock or the core clock divided
 * by 64.
 *
 *
 * NOTES:
 *
 * 1. This implementation requires the XScale PMU exclusively for preemptible
 *    latency measurements. The PMU cannot be used for other performance
 *    measurements concurrently with preemption latency measurement support.
 *
 * 2. We manually start the PMU rather than starting it via pmu_start() to
 *    to avoid enabling PMU overflow interrupts.
 * 
 */

#include <linux/config.h>
#include <asm/xscale-pmu.h>

#define readclock_init()	do {					\
		unsigned int pmnc = 3; /* PMU_ENABLE | PMU_RESET */	\
		__asm__ __volatile__("mcr    p14, 0, %0, c0, c0, 0" 	\
			: : "r" (pmnc));                		\
	} while (0)

#define readclock(x)	do {						\
		__asm__ __volatile__("mrc	p14, 0, %0, c1, c0, 0"	\
			: "=r" (x));					\
	} while (0)


#define clock_to_usecs(x)	((x) / 400) 
#define TICKS_PER_USEC		400


