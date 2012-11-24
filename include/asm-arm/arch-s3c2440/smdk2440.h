/*
 * include/asm-arm/arch-s3c2440/smdk2440.h
 *
 * s3c2440-SMDK2440 specific definiton
 *
 * Copyright (C) 2003 Samsung Electronics
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file COPYING in the main directory of this archive
 * for more details.
 */

#ifndef __ASM_ARCH_HARDWARE_H
#error "include <asm/hardware.h> instead"
#endif

/* definition of IRQ */
#define IRQ_CS8900              IRQ_EINT9
#define IRQ_KBD                 IRQ_EINT1

/* CS8900a, nGCS3 */
#define pCS8900_BASE		0x19000000
#define vCS8900_BASE		0xd0000000

