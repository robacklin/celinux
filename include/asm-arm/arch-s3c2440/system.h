/*
 * linux/include/asm-arm/arch-s3c2440/system.h
 *
 * Copyright (C) 2003 Samsung Electronics
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file COPYING in the main directory of this archive
 * for more details.
 */

#include <linux/config.h>
#include <asm/arch/hardware.h>

static inline void arch_idle(void)
{
	cpu_do_idle();
}

static inline void arch_reset(char mode)
{
	if (mode == 's') {
		/* Jump into ROM at address 0 */
		cpu_reset(0);
	} else {
		/* Use on-chip reset capability */
		WTCNT = 0x100;
		WTDAT = 0x100;
		WTCON = 0x8021;
	}
}
