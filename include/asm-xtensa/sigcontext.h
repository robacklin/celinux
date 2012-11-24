#ifndef __ASM_XTENSA_SIGCONTEXT_H
#define __ASM_XTENSA_SIGCONTEXT_H

/*
 * include/asm-xtensa/sigcontext.h
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2001 Tensilica Inc.
 *	Authors:	Kevin Chea, Chris Songer,
 *			Marc Gauthier <marc@tensilica.com> <marc@alumni.uwaterloo.ca>
 */

#include <xtensa/config/core.h>
#include <asm/cpextra.h>


struct _cpstate {
	unsigned char _cpstate[TOTAL_CPEXTRA_SIZE];
} __attribute__ ((aligned (XCHAL_CPEXTRA_SA_ALIGN)));


struct sigcontext {
	unsigned long	oldmask;

	/* CPU registers */
	unsigned long sc_aregs[16];
	unsigned long sc_pc;
	unsigned long sc_ps;
	unsigned long sc_wmask;
	unsigned long sc_wb;
	unsigned long sc_ws;
	unsigned long sc_lbeg;
	unsigned long sc_lend;
	unsigned long sc_lcount;
	unsigned long sc_sar;
	unsigned long sc_regfile[XCHAL_NUM_AREGS - 16];
	struct _cpstate *sc_cpstate;
};

#endif /* __ASM_XTENSA_SIGCONTEXT_H */
