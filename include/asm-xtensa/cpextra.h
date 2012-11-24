#ifndef __ASM_XTENSA_CPEXTRA_H
#define __ASM_XTENSA_CPEXTRA_H

/*
 * include/asm-xtensa/cpextra.h
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2003 Tensilica Inc.
 */

#include <xtensa/config/core.h>

/*
 *  Maximum size in bytes of a contiguous collection of all
 *  custom state save areas (extra and coprocessor state for
 *  a given Xtensa processor configuration).
 *  Allows for all save areas to be minimally aligned
 *  for the CHAL save/restore sequences.
 */
#define TOTAL_CPEXTRA_SIZE	\
			( XCHAL_EXTRA_SA_SIZE + XCHAL_EXTRA_SA_ALIGN \
			+ XCHAL_CP0_SA_SIZE + XCHAL_CP0_SA_ALIGN \
			+ XCHAL_CP1_SA_SIZE + XCHAL_CP1_SA_ALIGN \
			+ XCHAL_CP2_SA_SIZE + XCHAL_CP2_SA_ALIGN \
			+ XCHAL_CP3_SA_SIZE + XCHAL_CP3_SA_ALIGN \
			+ XCHAL_CP4_SA_SIZE + XCHAL_CP4_SA_ALIGN \
			+ XCHAL_CP5_SA_SIZE + XCHAL_CP5_SA_ALIGN \
			+ XCHAL_CP6_SA_SIZE + XCHAL_CP6_SA_ALIGN \
			+ XCHAL_CP7_SA_SIZE + XCHAL_CP7_SA_ALIGN \
			+ 16		/* for possible end-alignment padding */ \
			)

#endif /* __ASM_XTENSA_CPEXTRA_H */
