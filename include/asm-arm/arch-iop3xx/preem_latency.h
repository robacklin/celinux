/*
 * include/asm-arm/arch-iop3xx/preem_latency.h
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
		__asm__ __volatile__("mcr    p14, 0, %0, c0, c0, 0"	\
			: : "r" (pmnc));                		\
	} while (0)

#define readclock(x)	do {						\
		__asm__ __volatile__("mrc	p14, 0, %0, c1, c0, 0"	\
			: "=r" (x));					\
	} while (0)


#ifdef CONFIG_ARCH_IOP310

#define clock_to_usecs(x)	((x) / 733) /* Assumes 733Mhz CCLK */
#define TICKS_PER_USEC		733

#elif defined(CONFIG_ARCH_IOP321)

extern unsigned int processor_id;

static inline unsigned clock_to_usecs(unsigned x)
{
	if(processor_id & 0x00000030)
		return ((x) / 600);
	else
		return ((x) / 400);
}

static inline unsigned __ticks_per_usec(void)
{
	if(processor_id & 0x00000030)
		return 600;
	else
		return 400;
}

#define TICKS_PER_USEC	__ticks_per_usec()
#endif

