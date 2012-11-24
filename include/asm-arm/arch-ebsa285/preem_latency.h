/*
 * include/asm-arm/arch-ebsa285/preem_latency.h
 *
 * Support for preemptible kernel latency measurement times
 *
 * The readclock() macro must return the contents of a monotonic incremental
 * counter.
 *
 * NOTES:
 *
 * 1. This implementation uses the DEC21285 TIMER3 exclusively for preemptible
 *    latency measurements. It must not be used for any other purpose during
 *    preemptible kernel latency measurement.
 *
 * 2. This implementation assumes that DEC21285 fclk_in is 50MHz.
 *
 * 3. As currently configured, the maximum latency which can be measured is
 *    approx 5.4 seconds. If latency times exceed approx 5.4 seconds, overflow
 *    will occur.
 *
 */

#include <linux/config.h>
#include <linux/types.h>
#include <linux/sc_math.h>
#include <linux/time.h>
#include <asm/hardware/dec21285.h>

#define	PREK_CLOCK_TICK_RATE	(50000000 / 16)
#define prek_cycles_per_jiffy	(PREK_CLOCK_TICK_RATE / HZ)

#define prek_usec_per_cycle	\
	(SC_n(24, USEC_PER_SEC) / PREK_CLOCK_TICK_RATE)

#define prek_cycles_per_usec	\
	(SC_n(24, PREK_CLOCK_TICK_RATE) / USEC_PER_SEC)

#define prek_cycle_to_usec(cycles)	\
	mpy_sc_n(24, (cycles), prek_usec_per_cycle)

#define prek_usec_to_arch_cycle(nsec)	\
	mpy_sc_n(24, (nsec), prek_cycles_per_usec)

#define PREK_TIMER_RELOAD	(prek_cycles_per_jiffy - 1)

#define clock_to_usecs(x)	prek_cycle_to_usec((x >> 8))

#define TICKS_PER_USEC          (PREK_CLOCK_TICK_RATE / USEC_PER_SEC)

#define readclock_init()						\
	do {								\
		*CSR_TIMER3_CLR = 0;					\
		*CSR_TIMER3_LOAD = 0;					\
		*CSR_TIMER3_CNTL = TIMER_CNTL_ENABLE | TIMER_CNTL_DIV16;\
	} while (0)

#define readclock(x)							\
	do {								\
	        x = ~(*CSR_TIMER3_VALUE) << 8;				\
	} while (0)
