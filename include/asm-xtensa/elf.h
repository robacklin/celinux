#ifndef __ASM_XTENSA_ELF_H
#define __ASM_XTENSA_ELF_H

/*
 * include/asm-xtensa/elf.h
 *
 * ELF register definitions
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2001 - 2003 Tensilica Inc.
 *	Authors:	Christian Zankel <zankel@tensilica.com> <chris@zankel.net>
 *			Kevin Chea
 *			Marc Gauthier <marc@tensilica.com> <marc@alumni.uwaterloo.ca>
 *			Joe Taylor <joe@tensilica.com, joetylr@yahoo.com>
 */

#include <asm/ptrace.h>
#include <asm/page.h>
#include <asm/cpextra.h>
#include <xtensa/config/core.h>

/* Xtensa processor ELF architecture-magic number */
#define EM_XTENSA	94
#define EM_XTENSA_OLD	0xABC7

/* ELF register definitions. This is needed for core dump support.  */

/* 
 * elf_gregset_t contains the application-level state in the following order:
 * Processor info: 	config_version, cpuxy
 * Processor state:	pc, ps, exccause, excvaddr, wb, ws,
 *			lbeg, lend, lcount, sar
 * GP regs:		ar0 - arXX
 */

typedef unsigned long elf_greg_t;

typedef struct {

	elf_greg_t xchal_config_id0;
	elf_greg_t xchal_config_id1;
	elf_greg_t cpux;
	elf_greg_t cpuy;
	elf_greg_t pc;
	elf_greg_t ps;
	elf_greg_t exccause;
	elf_greg_t excvaddr;
	elf_greg_t wb;
	elf_greg_t ws;
	elf_greg_t lbeg;
	elf_greg_t lend;
	elf_greg_t lcount;
	elf_greg_t sar;
	elf_greg_t syscall_nr;
	elf_greg_t ar[XCHAL_NUM_AREGS];

} xtensa_gregset_t;

#define ELF_NGREG (sizeof(xtensa_gregset_t) / sizeof(elf_greg_t))
typedef elf_greg_t elf_gregset_t[ELF_NGREG];


/*
 *  Compute the size of the coprocessor & extra state layout (register info) table
 *  (see arch/xtensa/kernel/cptable.S).
 *  (Actually this is the maximum size of the table, as opposed to the actual size
 *   which is available from the _xtensa_reginfo_table_size global variable.)
 */
#ifndef XCHAL_EXTRA_SA_CONTENTS_LIBDB_NUM
# define XTENSA_CPE_LTABLE_SIZE	0	/* interim until all core.h's are updated */
#else
# define XTENSA_CPE_SEGMENT(num)	(num ? (1+num) : 0)	/* number of entries for a CP or extra */
# define XTENSA_CPE_LTABLE_ENTRIES	\
		( XTENSA_CPE_SEGMENT(XCHAL_EXTRA_SA_CONTENTS_LIBDB_NUM) \
		+ XTENSA_CPE_SEGMENT(XCHAL_CP0_SA_CONTENTS_LIBDB_NUM) \
		+ XTENSA_CPE_SEGMENT(XCHAL_CP1_SA_CONTENTS_LIBDB_NUM) \
		+ XTENSA_CPE_SEGMENT(XCHAL_CP2_SA_CONTENTS_LIBDB_NUM) \
		+ XTENSA_CPE_SEGMENT(XCHAL_CP3_SA_CONTENTS_LIBDB_NUM) \
		+ XTENSA_CPE_SEGMENT(XCHAL_CP4_SA_CONTENTS_LIBDB_NUM) \
		+ XTENSA_CPE_SEGMENT(XCHAL_CP5_SA_CONTENTS_LIBDB_NUM) \
		+ XTENSA_CPE_SEGMENT(XCHAL_CP6_SA_CONTENTS_LIBDB_NUM) \
		+ XTENSA_CPE_SEGMENT(XCHAL_CP7_SA_CONTENTS_LIBDB_NUM) \
		+ 1		/* final entry */	\
		)
# define XTENSA_CPE_LTABLE_SIZE		(XTENSA_CPE_LTABLE_ENTRIES * 8)	/* in bytes */
#endif /* defined(XCHAL_EXTRA_SA_CONTENTS_LIBDB_NUM) */


/*
 * Instantiations of the elf_fpregset_t type contain, in most
 * architectures, the floating point (FPU) register set.
 * For Xtensa, this type is extended to contain all custom state,
 * ie. coprocessor and "extra" (non-coprocessor) state (including,
 * for example, TIE-defined states and register files; as well
 * as other optional processor state).
 * This includes FPU state if a floating-point coprocessor happens
 * to have been configured within the Xtensa processor.
 *
 * TOTAL_FPREGS_SIZE is the required size (without rounding)
 * of elf_fpregset_t.  It provides space for the following:
 *
 *  a)	32-bit mask of active coprocessors for this task (similar
 *	to CPENABLE in single-threaded Xtensa processor systems)
 *
 *  b)	table describing the layout of custom states (ie. of
 *      individual registers, etc) within the save areas
 *
 *  c)  save areas for each coprocessor and for non-coprocessor
 *      ("extra") state
 *
 * Note that save areas may require up to 16-byte alignment when
 * accessed by save/restore sequences.  We do not need to ensure
 * such alignment in an elf_fpregset_t structure because custom
 * state is not directly loaded/stored into it; rather, save area
 * contents are copied to elf_fpregset_t from the active save areas
 * (see 'struct task_struct' definition in processor.h for that)
 * using memcpy().  But we do allow space for such alignment,
 * to allow optimizations of layout and copying.
 */

#define TOTAL_FPREGS_SIZE	(4 + XTENSA_CPE_LTABLE_SIZE + TOTAL_CPEXTRA_SIZE)
#define ELF_NFPREG	((TOTAL_FPREGS_SIZE + sizeof(elf_fpreg_t) - 1) / sizeof(elf_fpreg_t))

typedef unsigned int elf_fpreg_t;
typedef elf_fpreg_t elf_fpregset_t[ELF_NFPREG];

#define ELF_CORE_COPY_REGS(_eregs, _pregs) xtensa_elf_core_copy_regs (&_eregs, _pregs);
extern void xtensa_elf_core_copy_regs (elf_gregset_t *, struct pt_regs *);


/*
 * This is used to ensure we don't load something for the wrong architecture.
 */
#define elf_check_arch(x) ( ( (x)->e_machine == EM_XTENSA )  || \
			    ( (x)->e_machine == EM_XTENSA_OLD ) )

/*
 * These are used to set parameters in the core dumps.
 */
#define ELF_CLASS	ELFCLASS32
#if XCHAL_HAVE_LE
#define ELF_DATA	ELFDATA2LSB
#elif XCHAL_HAVE_BE
#define ELF_DATA	ELFDATA2MSB
#else
#error endianess not defined
#endif
#define ELF_ARCH	EM_XTENSA

#define USE_ELF_CORE_DUMP
#define ELF_EXEC_PAGESIZE	PAGE_SIZE

/* This is the location that an ET_DYN program is loaded if exec'ed.  Typical
   use of this is to invoke "./ld.so someprog" to test out a new version of
   the loader.  We need to make sure that it is out of the way of the program
   that it will "exec", and that there is sufficient room for the brk.  */

#define ELF_ET_DYN_BASE         (2 * TASK_SIZE / 3)


/* This yields a mask that user programs can use to figure out what
   instruction set this CPU supports.  This could be done in user space,
   but it's not easy, and we've already done it here.  */

#define ELF_HWCAP	(0)

/* This yields a string that ld.so will use to load implementation
   specific libraries for optimization.  This is more specific in
   intent than poking at uname or /proc/cpuinfo.

   For the moment, we have only optimizations for the Intel generations,
   but that could change... */

#define ELF_PLATFORM  (NULL)

/* The Xtensa processor ABI says that when the program starts, a2
   contains a pointer to a function which might be registered using
   `atexit'.  This provides a mean for the dynamic linker to call
   DT_FINI functions for shared libraries that have been loaded before
   the code runs.

   A value of 0 tells we have no such handler. 

   We might as well make sure everything else is cleared too (except
   for the stack pointer in a1), just to make things more
   deterministic.  Also, clearing a0 terminates debugger backtraces.
 */

#define ELF_PLAT_INIT(_r) \
  do { _r->aregs[0]=0; /*_r->aregs[1]=0;*/ _r->aregs[2]=0;  _r->aregs[3]=0;  \
       _r->aregs[4]=0;  _r->aregs[5]=0;    _r->aregs[6]=0;  _r->aregs[7]=0;  \
       _r->aregs[8]=0;  _r->aregs[9]=0;    _r->aregs[10]=0; _r->aregs[11]=0; \
       _r->aregs[12]=0; _r->aregs[13]=0;   _r->aregs[14]=0; _r->aregs[15]=0; \
  } while (0)

#ifdef __KERNEL__
#define SET_PERSONALITY(ex, ibcs2) set_personality((ibcs2)?PER_SVR4:PER_LINUX)
#endif

#endif /* __ASM_XTENSA_ELF_H */
