#ifndef __ASM_XTENSA_CACHE_H
#define __ASM_XTENSA_CACHE_H

/*
 * include/asm-xtensa/cache.h
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2001 Tensilica Inc.
 */

#include <xtensa/config/core.h>

#define L1_CACHE_BYTES		XCHAL_CACHE_LINESIZE_MAX

#define SMP_CACHE_BYTES		L1_CACHE_BYTES

#endif /* __ASM_XTENSA_CACHE_H */
