/*
 * linux/include/asm-arm/arch-s3c2440/hardware.h
 *
 * Copyright (C) 2003 Samsung Electronics.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#ifndef __ASM_ARCH_HARDWARE_H
#define __ASM_ARCH_HARDWARE_H

#include <linux/config.h>
#include <asm/mach-types.h>

#define MEM_SIZE		0x04000000
#define PCIO_BASE		0

#define VIO_BASE		0xe8000000	/* virtual start of IO space */
#define PIO_START		0x48000000	/* physical start of IO space */

#define io_p2v(x) ((x) | 0xa0000000)
#define io_v2p(x) ((x) & ~0xa0000000)

#ifndef __ASSEMBLY__
#include <asm/types.h>

/*
 * This __REG() version gives the same results as the one above, except
 * that we are fooling gcc some how so it generates far better and smaller
 * assembly code for access to contigous registers. It's a shame that gcc
 * doesn't guess this by itself
 */
typedef struct { volatile u32 offset[4096]; } __regbase;
#define __REGP(x)	((__regbase *)((x)&~4095))->offset[((x)&4095)>>2]
#define __REG(x)	__REGP(io_p2v(x))

/* Let's kick gcc's ass again... */
# define __REG2(x,y)	\
	( __builtin_constant_p(y) ? (__REG((x) + (y))) \
				  : (*(volatile u32 *)((u32)&__REG(x) + (y))) )

#define __PREG(x)	(io_v2p((u32)&(x)))

#else	/* __ASSEMBLY__ */

# define __REG(x)	io_p2v(x)
# define __PREG(x)	io_v2p(x)

#endif	/* __ASSEMBLY__ */

#include "S3C2440.h"

/*
 * S3C2440 GPIO edge detection for IRQs:
 * IRQs are generated on Falling-Edge, Rising-Edge, both, low level or higg level.
 * This must be called *before* the corresponding IRQ is registered.
 */
#define EXT_LOWLEVEL		0
#define EXT_HIGHLEVEL		1
#define EXT_FALLING_EDGE	2
#define EXT_RISING_EDGE		4
#define EXT_BOTH_EDGES		6

#define FULLUP_EN		0x0
#define FULLUP_DIS		0x1

#ifndef __ASSEMBLY__
	extern int set_external_irq(int irq, int edge, int pullup);
#endif
	
/* Externl clock frequency used by CPU */
#define FIN	12000000

#ifdef CONFIG_S3C2440_SMDK2440
#include "smdk2440.h"
#endif

#endif /* __ASM_ARCH_HARDWARE_H */
