#ifndef _LINUX_SMP_BALANCE_H
#define _LINUX_SMP_BALANCE_H

/*
 * per-architecture load balancing logic, e.g. for hyperthreading
 */

#ifdef ARCH_HAS_SMP_BALANCE
#include <asm/smp_balance.h>
#else
#define arch_load_balance(x, y)		(0)
#endif

#endif /* _LINUX_SMP_BALANCE_H */
