/*
 * include/asm-arm/arch-omap/preem_latency.h
 * timing support for preempt-stats patch
 */

#include <asm/hardware.h>

extern unsigned int read_mputimer1(void);

#define readclock_init()
#define readclock(x)		x = ~(read_mputimer1())
#define clock_to_usecs(x)	((x) / 6)

#define PSR_F_BIT               0x40    /* FIQ unmask/mask (0/1) */
#define PSR_I_BIT               0x80    /* IRQ unmask/mask (0/1) */
#define INTERRUPTS_ENABLED(x)   (!(x & PSR_I_BIT))
/* mputimer 1 runs @ 6Mhz  6 ticks = 1 microsecond */
#define TICKS_PER_USEC		6

#ifdef CONFIG_KFI
extern unsigned long do_getmachinecycles(void);
extern unsigned long machinecycles_to_usecs(unsigned long mputicks);
#define kfi_readclock do_getmachinecycles
#define kfi_clock_to_usecs(cycles) machinecycles_to_usecs(cycles)
#endif

