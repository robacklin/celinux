/*
 * include/asm-arm/arch-omap1510/hrtime.h
 *
 * HRT hooks for the TI OMAP1510
 *
 * Author: George G. Davis <gdavis@mvista.com>
 *
 * Copyright 2003 (c) MontaVista, Software, Inc. 
 * Copyright 2003 Sony Corporation.
 * Copyright 2003 Matsushita Electric Industrial Co., Ltd.
 * 
 * This file is licensed under  the terms of the GNU General Public 
 * License version 2. This program is licensed "as is" without any 
 * warranty of any kind, whether express or implied.
 */


#ifndef __ASM_ARCH_HRTIME_H_
#define __ASM_ARCH_HRTIME_H_

#include <linux/timex.h>

int schedule_hr_timer_int(unsigned, int);
int get_arch_cycles(unsigned);

#define hr_time_resolution	1000
#define arch_cycles_per_jiffy	(CLOCK_TICK_RATE / HZ)
#define schedule_jiffies_int(x)	(get_arch_cycles(x) >= arch_cycles_per_jiffy)

#define SC_ARCH2NSEC		23
#define SC_NSEC2ARCH		32

#define scaled_nsec_per_arch_cycle	\
	(SC_n(SC_ARCH2NSEC, NSEC_PER_SEC) / CLOCK_TICK_RATE)

#define scaled_arch_cycles_per_nsec	\
	(SC_n(SC_NSEC2ARCH, CLOCK_TICK_RATE) / NSEC_PER_SEC)

#define arch_cycle_to_nsec(cycles)	\
	mpy_sc_n(SC_ARCH2NSEC, (cycles), scaled_nsec_per_arch_cycle)

#define nsec_to_arch_cycle(nsec)	\
	mpy_sc_n(SC_NSEC2ARCH, (nsec), scaled_arch_cycles_per_nsec)

#endif	// __ASM_ARCH_HRTIME_H_
