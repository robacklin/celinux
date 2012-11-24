#ifndef __ASM_XTENSA_DIV64_H
#define __ASM_XTENSA_DIV64_H

/* 
 * include/asm-xtensa/div64.h
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2001 Tensilica Inc.
 */

#define do_div(n,base) ({ \
	int __res = n % ((unsigned int) base); \
	n /= (unsigned int) base; \
	__res; })

#endif /* __ASM_XTENSA_DIV64_H */
