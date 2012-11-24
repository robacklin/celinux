#ifndef __ASM_XTENSA_PAGE_H
#define __ASM_XTENSA_PAGE_H

/*
 * include/asm-xtensa/page.h
 *
 * Definitions for page handling.  Derived from MIPS.
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 1994 - 1999 by Ralf Baechle
 *
 * Copyright (C) 2001 Tensilica Inc.
 *	Authors:	Joe Taylor <joe@tensilica.com, joetylr@yahoo.com>
 *			Marc Gauthier
 */

#include <xtensa/config/core.h>
#include <asm/xtutil.h>

/* PAGE_SHIFT determines the page size */
#define PAGE_SHIFT	XCHAL_MMU_MIN_PTE_PAGE_SIZE
#define PAGE_SIZE	(1 << PAGE_SHIFT)
#define PAGE_MASK	(~(PAGE_SIZE-1))



#ifdef __KERNEL__

#define XCDCACHE_WAY_SIZE (XCHAL_DCACHE_SIZE / XCHAL_DCACHE_WAYS)
#if ( XCDCACHE_WAY_SIZE > PAGE_SIZE)
  #define XTENSA_CACHE_ALIAS 1
#else
  #define XTENSA_CACHE_ALIAS 0
#endif

#ifndef _LANGUAGE_ASSEMBLY

extern void xt_panic(void);
#define BUG() do { printk("kernel BUG at %s:%d!\n", __FILE__, __LINE__); xt_panic(); } while (0)
#define PAGE_BUG(page) do {  BUG(); } while (0)

extern void clear_page_asm(void * page);
extern void copy_page_asm (void * to, void * from);

#define clear_page(page)	clear_page_asm (page)
#define copy_page(to, from)	copy_page_asm (to, from)


/* 19feb2003 --jn
 * Some Xtensa's can have cache alias problems, for those we have to
 * use the magical copy and clear user page functions.
 */

#if XTENSA_CACHE_ALIAS
  extern void clear_user_page(void *to, unsigned long addr);
  extern void copy_user_page(void *to, void *from, unsigned long addr);
#else
  #define clear_user_page(page, vaddr)	        clear_page(page)
  #define copy_user_page(to, from, vaddr)	copy_page(to, from)
#endif

/*
 * These are used to make use of C type-checking..
 */
typedef struct { unsigned long pte; } pte_t;		/* page table entry */
typedef struct { unsigned long pmd; } pmd_t;		/* PMD table entry */
typedef struct { unsigned long pgd; } pgd_t;		/* PGD table entry */
typedef struct { unsigned long pgprot; } pgprot_t;

#define pte_val(x)	((x).pte)
#define pmd_val(x)	((x).pmd)
#define pgd_val(x)	((x).pgd)
#define pgprot_val(x)	((x).pgprot)

#define __pte(x)	((pte_t) { (x) } )
#define __pmd(x)	((pmd_t) { (x) } )
#define __pgd(x)	((pgd_t) { (x) } )
#define __pgprot(x)	((pgprot_t) { (x) } )

/* Pure 2^n version of get_order */
extern __inline__ int get_order(unsigned long size)
{
	int order;

	size = (size-1) >> (PAGE_SHIFT-1);
	order = -1;
	do {
		size >>= 1;
		order++;
	} while (size);
	return order;
}

#endif /* _LANGUAGE_ASSEMBLY */

/* to align the pointer to the (next) page boundary */
#define PAGE_ALIGN(addr)	(((addr)+PAGE_SIZE-1)&PAGE_MASK)

/*
 * This handles the memory map.  We handle pages at
 * XCHAL_KSEG_CACHED_VADDR for kernels with 32 bit address space.
 * These macros are for conversion of kernel address, not user
 * addresses.
 */

#define PAGE_OFFSET	XCHAL_KSEG_CACHED_VADDR
#define __pa(x)		((unsigned long) (x) - PAGE_OFFSET)
#define __va(x)		((void *)((unsigned long) (x) + PAGE_OFFSET))
#define VALID_PAGE(page)	((page - mem_map) < max_mapnr)

/* WARNING: Assembly code in arch/xtensa/kernel/handlers.S mimics this
 * macro definition.  If you change this definition, you should review
 * the port in handlers.S.  Within that file, just search for
 * 'virt_to_page'.
 */
#define virt_to_page(kaddr)	(mem_map + (__pa(kaddr) >> PAGE_SHIFT))

#define VM_DATA_DEFAULT_FLAGS		(VM_READ | VM_WRITE | VM_EXEC | \
					 VM_MAYREAD | VM_MAYWRITE | VM_MAYEXEC)

#endif /* defined (__KERNEL__) */

#endif /* __ASM_XTENSA_PAGE_H */
