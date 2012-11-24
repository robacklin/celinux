#ifndef __ASM_XTENSA_ASSUME_H
#define __ASM_XTENSA_ASSUME_H

/* 
 * include/asm-xtensa/assume.h
 *
 * This file enforces various assumptions about all Xtensa
 * configurations that the architecture-specific code for Linux
 * makes.
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2001 Tensilica Inc.
 */

#include <xtensa/config/core.h>
#include <xtensa/config/system.h>
#include <asm/pgtable.h>


#if (XCHAL_HAVE_TLBS != 1)
#error Linux must have an MMU!
#endif

#if ((XCHAL_ITLB_ARF_WAYS == 0) || (XCHAL_DTLB_ARF_WAYS == 0))
#error MMU must have auto-refill ways
#endif

/* Linux was ported to Xtensa assuming all auto-refill ways in set 0
   had the same properties (a very likely assumption).  Multiple sets
   of auto-refill ways will still work properly, but not as optimally
   as the Xtensa designer may have assumed.

   We make this case a hard #error, killing the kernel build, to alert
   the developer to this condition (which is more likely an error).
   You super-duper clever developers can change it to a warning or
   remove it altogether if you think you know what you're doing.  :)
*/

#if ((XCHAL_ITLB_ARF_SETS != 1) || (XCHAL_DTLB_ARF_SETS != 1))
#error Linux may not use all auto-refill ways as efficiently as you think
#endif

#if (XCHAL_MMU_MAX_PTE_PAGE_SIZE != XCHAL_MMU_MIN_PTE_PAGE_SIZE)
#error Only one page size allowed!
#endif

#if (XCHAL_HAVE_WINDOWED != 1)
#error Linux requires the Xtensa Windowed Registers Option.
#endif

#if ( ! XCHAL_HAVE_INTERRUPTS)
#error What is Linux without interrupts?
#endif

#if XCHAL_ICACHE_SIZE > 0
# if (XCHAL_ICACHE_SIZE % (XCHAL_ICACHE_LINESIZE*XCHAL_ICACHE_WAYS*4)) != 0
#  error cache configuration outside expected/supported range!
# endif
#endif

#if XCHAL_DCACHE_SIZE > 0
# if (XCHAL_DCACHE_SIZE % (XCHAL_DCACHE_LINESIZE*XCHAL_DCACHE_WAYS*4)) != 0
#  error cache configuration outside expected/supported range!
# endif
#endif 

/* 20feb2003 -- jn
 *   cache aliasing is now being supported, have fear.
 */
#if 0
#if XCHAL_ICACHE_SIZE > (XCHAL_ICACHE_WAYS * PAGE_SIZE)
#error Cache aliasing not supported.

#endif
#if XCHAL_DCACHE_SIZE > (XCHAL_DCACHE_WAYS * PAGE_SIZE)
#error Cache aliasing not supported.
#endif
#endif


/* Verify instruction/data ram/rom and xlmi don't overlay vmalloc space. */

#define __IN_VMALLOC(addr) ( ((addr) >= VMALLOC_START) && ((addr) < VMALLOC_END) )
#define __SPAN_VMALLOC(start,end) ( ((start) < VMALLOC_START) && ((end) >= VMALLOC_END) )
#define INSIDE_VMALLOC(start,end) \
	( __IN_VMALLOC((start)) || __IN_VMALLOC(end) || __SPAN_VMALLOC((start),(end)) )

#if XCHAL_NUM_INSTROM

# if XCHAL_NUM_INSTROM == 1
#  if INSIDE_VMALLOC(XCHAL_INSTROM0_VADDR,XCHAL_INSTROM0_VADDR+XCHAL_INSTROM0_SIZE)
#   error vmalloc range conflicts with instrom0
#  endif
# endif

# if XCHAL_NUM_INSTROM == 2
#  if INSIDE_VMALLOC(XCHAL_INSTROM1_VADDR,XCHAL_INSTROM1_VADDR+XCHAL_INSTROM1_SIZE)
#   error vmalloc range conflicts with instrom1
#  endif
# endif

#endif

#if XCHAL_NUM_INSTRAM

# if XCHAL_NUM_INSTRAM == 1
#  if INSIDE_VMALLOC(XCHAL_INSTRAM0_VADDR,XCHAL_INSTRAM0_VADDR+XCHAL_INSTRAM0_SIZE)
#   error vmalloc range conflicts with instram0
#  endif
# endif

# if XCHAL_NUM_INSTRAM == 2
#  if INSIDE_VMALLOC(XCHAL_INSTRAM1_VADDR,XCHAL_INSTRAM1_VADDR+XCHAL_INSTRAM1_SIZE)
#   error vmalloc range conflicts with instram1
#  endif
# endif

#endif

#if XCHAL_NUM_DATAROM

# if XCHAL_NUM_DATAROM == 1
#  if INSIDE_VMALLOC(XCHAL_DATAROM0_VADDR,XCHAL_DATAROM0_VADDR+XCHAL_DATAROM0_SIZE)
#   error vmalloc range conflicts with datarom0
#  endif
# endif

# if XCHAL_NUM_DATAROM == 2
#  if INSIDE_VMALLOC(XCHAL_DATAROM1_VADDR,XCHAL_DATAROM1_VADDR+XCHAL_DATAROM1_SIZE)
#   error vmalloc range conflicts with datarom1
#  endif
# endif

#endif

#if XCHAL_NUM_DATARAM

# if XCHAL_NUM_DATARAM == 1
#  if INSIDE_VMALLOC(XCHAL_DATARAM0_VADDR,XCHAL_DATARAM0_VADDR+XCHAL_DATARAM0_SIZE)
#   error vmalloc range conflicts with dataram0
#  endif
# endif

# if XCHAL_NUM_DATARAM == 2
#  if INSIDE_VMALLOC(XCHAL_DATARAM1_VADDR,XCHAL_DATARAM1_VADDR+XCHAL_DATARAM1_SIZE)
#   error vmalloc range conflicts with dataram1
#  endif
# endif

#endif

#if XCHAL_NUM_XLMI

# if XCHAL_NUM_XLMI == 1
#  if INSIDE_VMALLOC(XCHAL_XLMI0_VADDR,XCHAL_XLMI0_VADDR+XCHAL_XLMI0_SIZE)
#   error vmalloc range conflicts with xlmi0
#  endif
# endif

# if XCHAL_NUM_XLMI == 2
#  if INSIDE_VMALLOC(XCHAL_XLMI1_VADDR,XCHAL_XLMI1_VADDR+XCHAL_XLMI1_SIZE)
#   error vmalloc range conflicts with xlmi1
#  endif
# endif

#endif

#if (XCHAL_NUM_INSTROM > 2) || \
    (XCHAL_NUM_INSTRAM > 2) || \
    (XCHAL_NUM_DATARAM > 2) || \
    (XCHAL_NUM_DATAROM > 2) || \
    (XCHAL_NUM_XLMI    > 2)
#error Insufficient checks on vmalloc above for more than 2 devices
#endif
 
#endif /* __ASM_XTENSA_ASSUME_H */
