/*
 * High-resolution timer header file for Xtensa.
 * 
 * Author: source@mvista.com
 *
 * Based on include/asm-mips/hrtime.h
 *
 * 2001-2003 (c) MontaVista Software, Inc. This file is licensed under the
 * terms of the GNU General Public License version 2. This program is
 * licensed "as is" without any warranty of any kind, whether express
 * or implied.
 */
#ifndef _ASM_HRTIMER_H
#define _ASM_HRTIMER_H

#include <linux/sc_math.h>

#define db_assert(x)  if (!(x)) { \
	panic("assertion failed at %s:%d: %s\n", __FILE__, __LINE__, #x); }

extern int schedule_hr_timer_int(unsigned long ref_jiffies, int cycles); 
extern int get_arch_cycles(unsigned long ref_jiffies);

extern int arch_cycles_per_jiffy;
extern int nsec_to_arch_cycle(int nsecs);
extern int arch_cycle_to_nsec(int cycles);

extern int hr_time_resolution;

extern int schedule_jiffies_int(unsigned long ref_jiffies);

extern int hr_time_resolution;

#endif /* _ASM_HRTIMER_H */
