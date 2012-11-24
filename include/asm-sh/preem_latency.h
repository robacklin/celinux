/*
 * include/asm-sh/preem_latency.h
 * timing support for preempt-stats patch
 */

#ifndef _ASM_PREEM_LATENCY_H
#define _ASM_PREEM_LATENCY_H

#include <asm/io.h>
#include <asm/timex.h>

#if defined(__sh3__)
#define TMU1_TCNT     0xfffffea4      /* Long access */
#elif defined(__SH4__)
#define TMU1_TCNT     0xffd80018      /* Long access */
#endif

#define readclock_init()
#define readclock(v) v = (unsigned long)ctrl_inl(TMU1_TCNT)
#define KFI_MODULE_CLOCK	40000000 //P0 = 40MHz
#define clock_to_usecs(clocks)	((clocks)/((KFI_MODULE_CLOCK/4)/1000000))
#define TICKS_PER_USEC CLOCK_TICK_RATE/1000000

#define CLOCK_COUNTS_DOWN

#define INTERRUPTS_ENABLED(x) ((x & 0x000000f0) == 0)

#define TMU_TSTR2       ((volatile unsigned char *)0xFE100004)
#define TMU3_TCOR       ((volatile unsigned long *)0xFE100008)
#define TMU3_TCNT       ((volatile unsigned long *)0xFE10000C)
#define TMU3_TCR        ((volatile unsigned short *)0xFE100010)
#define TMU4_TCOR       ((volatile unsigned long *)0xFE100014)
#define TMU4_TCNT       ((volatile unsigned long *)0xFE100018)
#define TMU4_TCR        ((volatile unsigned short *)0xFE10001C)
#define readclock3(v) v = (unsigned long)ctrl_inl(TMU3_TCNT)
#define readclock4(v) v = -(unsigned long)ctrl_inl(TMU4_TCNT)
static int init_kfi_readclock3 = 1;
static unsigned long __kenel_boot_time = 0;
static inline unsigned long __noinstrument kfi_readclock3(void)
{
	unsigned long x;
	if (init_kfi_readclock3)
	{
		*TMU3_TCR = 0x0002;	// P0/64 Count
		//*TMU3_TCR = 0x0004;	// P0/1024 Count
		*TMU3_TCOR = 0xFFFFFFFF;
		*TMU3_TCNT = 0xFFFFFFFF;
		*TMU_TSTR2 = (*TMU_TSTR2|0x01);
		init_kfi_readclock3--;
		x = 0;
	}
	else readclock3(x);
	return x;
}
#ifdef CONFIG_KFI
static int init_kfi_readclock = 1;
static inline unsigned long __noinstrument kfi_readclock(void)
{
	unsigned long x;
	if (init_kfi_readclock)
	{
		// P0 is 40MHz
		*TMU4_TCR = 0x0000;	// P0/4 Count
		//*TMU4_TCR = 0x0001;	// P0/16 Count
		//*TMU4_TCR = 0x0002;	// 1.6usec P0/64 Count
		//*TMU4_TCR = 0x0003;	// P0/256 Count
		//*TMU4_TCR = 0x0004;	// P0/1024 Count
		//*TMU4_TCR = 0x0006;	// RTC Count 128Hz
		*TMU4_TCOR = 0xFFFFFFFF;
		*TMU4_TCNT = 0xFFFFFFFF;
		*TMU_TSTR2 = (*TMU_TSTR2|0x02);
		init_kfi_readclock--;
	}
	readclock4(x);
	return x;
}
#define kfi_clock_to_usecs clock_to_usecs
#endif

#endif /* _ASM_PREEM_LATENCY_H */
