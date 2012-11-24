#ifndef __ASM_XTENSA_PGTABLE_H
#define __ASM_XTENSA_PGTABLE_H

/*
 * include/asm-xtensa/pgtable.h
 *
 * Derived from MIPS.
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2001 - 2003 Tensilica Inc.
 *	Authors:	Marc Gauthier <marc@tensilica.com> <marc@alumni.uwaterloo.ca>
 *			Joe Taylor <joe@tensilica.com, joetylr@yahoo.com>
 */

#include <asm/addrspace.h>
#include <asm/page.h>
#include <asm/xtutil.h>
#include <xtensa/config/core.h>

#ifndef _LANGUAGE_ASSEMBLY

#include <linux/sched.h>
#include <linux/linkage.h>
#include <linux/config.h>



/* If the xtensa configuration has a cache-way larger than the
 * page size, then we have to provide our own get_unmapped_area
 * function to attempt to prevent cache aliasing problems
 */

#if XTENSA_CACHE_ALIAS

/* We provide our own get_unmapped_area to avoid cache alias issues */
#define HAVE_ARCH_UNMAPPED_AREA
#define PG_mapped  PG_arch_1

/* This is a mask of the bits that can cause cache aliasing problems.
 * Because Xtensa has a configurable cache size, we use
 * configuration information to compute the mask.
 */
#define XT_CACHE_WAYSIZE (XCHAL_DCACHE_SIZE / XCHAL_DCACHE_WAYS)
#define XT_CACHE_ALIAS_BITS (( (-1) & ~(PAGE_SIZE-1)) & (XT_CACHE_WAYSIZE-1))

#endif



/* If the dcache is writeback or we have cache aliasing, may need to flush.
 */

#if ((XCHAL_DCACHE_IS_WRITEBACK > 0) || (XTENSA_CACHE_ALIAS > 0))

extern void flush_cache_all(void);
extern void flush_cache_mm(struct mm_struct *mm);
extern void flush_cache_range(struct mm_struct *mm, unsigned long start,
				 unsigned long end);
extern void flush_cache_page(struct vm_area_struct *vma, unsigned long page);
extern void flush_icache_range(unsigned long start, unsigned long end);
extern void flush_icache_user_range(struct vm_area_struct *vma, struct page *page, unsigned long start, unsigned long len);
extern void flush_icache_page(struct vm_area_struct *vma, struct page *page);
extern void flush_page_to_ram(struct page *page);


#else

//#define flush_dcache_page(page)				do { } while (0)
#define flush_cache_all()				do { } while (0)
#define flush_cache_mm(mm)				do { } while (0)
#define flush_cache_range(mm,start,end)			do { } while (0)
#define flush_cache_page(vma,page)			do { } while (0)
#define flush_cache_sigtramp(addr)			do { } while (0)
#define flush_icache_user_range(vma, page, addr, len)	do { } while (0)
#define flush_page_to_ram(page)				do { } while (0)
#define flush_icache_range(start, end)			do { } while (0)
#define flush_icache_page(vma, page)			do { } while (0)

#endif



/* 19feb2003 -- jn 
 *   enabling dcache aliasing
 */

#if XTENSA_CACHE_ALIAS
void flush_dcache_page(struct page *page);
//  void xtensa_cache_init(void);

#else
#define flush_dcache_page(page)			do { } while (0)
//  #define xtensa_cache_init()			do { } while (0)
#endif



#endif /* !defined (_LANGUAGE_ASSEMBLY) */


/* Use the first min-wired way for mapping page-table pages.
 * Page coloring requires a second min-wired way.
 */

#if (XCHAL_DTLB_MINWIRED_SETS == 0)
#error Need a min-wired way for mapping page-table pages
#endif

#define _FIRST_MIN_WIRED  XCHAL_DTLB_SET(XCHAL_DTLB_MINWIRED_SET0, WAY)
#define _MIN_WIRED_COUNT  XCHAL_DTLB_SET(XCHAL_DTLB_MINWIRED_SET0, WAYS)

#define WIRED_WAY_FOR_PAGE_TABLE  _FIRST_MIN_WIRED

#if XTENSA_CACHE_ALIAS
# if (_MIN_WIRED_COUNT >= 2)
#  define WIRED_WAY_FOR_COPY_USER_PAGE  (_FIRST_MIN_WIRED + 1)
# else
#  error Page coloring requires its own min-wired dtlb way
# endif
#endif



/*
 * The Xtensa architecture port of Linux uses essentially the same
 * two-level page tables as the i386, i.e. the logical three-level
 * Linux page table layout folded.  That is, for each task's memory space:
 *
 *	the PGD table (page directory), ie. 3rd-level page table,
 *		is one page (4 kB) of 1024 (PTRS_PER_PGD) pointers to PTE tables
 *		(are ptrs to PMD tables in arch's where PMDs are not folded).
 *
 *		For a given task, the pointer to the PGD table is:
 *			(t->mm ? t->mm : t->active_mm)->pgd
 *		where t is (struct task_struct *) (eg. current()).
 *
 *	the PMD tables (page middle-directory), ie. 2nd-level page tables,
 *		are absent (folded, PTRS_PER_PMD == 1).
 *
 *	the PTE tables (page table entry), ie. 1st-level page tables,
 *		are one page (4 kB) of 1024 (PTRS_PER_PTE) PTEs;
 *		special PTE tables include:
 *		- invalid_pte_table (pointed to by PMD table, for absent mappings)
 *
 *	individual pages are 4 kB;
 *		special pages include:
 *		- empty_zero_page
 */

/* PMD_SHIFT determines the v.m. size a PMD table *entry* can map */
#define PMD_SHIFT	22
#define PMD_SIZE	(1UL << PMD_SHIFT)
#define PMD_MASK	(~(PMD_SIZE-1))

/* PGDIR_SHIFT determines the v.m. size a PGD table *entry* can map */
#define PGDIR_SHIFT	22
#define PGDIR_SIZE	(1UL << PGDIR_SHIFT)
#define PGDIR_MASK	(~(PGDIR_SIZE-1))

/* Entries per page directory level: we use two-level, so
 * we don't really have any PMD directory physically.
 */
#define PTRS_PER_PTE	1024
#define PTRS_PER_PMD	1
#define PTRS_PER_PGD	1024
#define USER_PTRS_PER_PGD	(TASK_SIZE/PGDIR_SIZE)
#define FIRST_USER_PGD_NR	0	/* task VM starts at address zero */

/*  27feb2003 -- jn
 *
 *  0xC0000000-0xC8000000 reserved for kernel vmalloc.  See also
 *  assume.h.  When caches are aliasable, we need to reserve a small
 *  portion of virtual space for copy_user_page, and clear_user_page.
 *  This is used to reduce the amount of cache flushing required.
 */

#if (XTENSA_CACHE_ALIAS)

#define XTENSA_ALIAS_RESERVE_SIZE	XCDCACHE_WAY_SIZE
#define XTENSA_ALIAS_RESERVE_START	0xC0010000
#define VMALLOC_START	(XTENSA_ALIAS_RESERVE_START + XTENSA_ALIAS_RESERVE_SIZE)

#else
#define VMALLOC_START	0xC0010000
#endif

#define VMALLOC_VMADDR(x)	((unsigned long)(x))
#define VMALLOC_END		0xC7FF0000


/*  More assumptions:  */
#if (XCHAL_MMU_RINGS != 4)
#error Linux build assumes 4 ring levels.
#endif
#if (XCHAL_MMU_RING_BITS != 2)
#error We assume exactly two bits for RING.
#endif
#if (XCHAL_MMU_CA_BITS != 4)
#error We assume exactly four bits for CA.
#endif
#if (XCHAL_MMU_SR_BITS != 0)
#error We have no room for SR bits.
#endif


/* Xtensa Linux config PTE layout (when present):
 *	31-12:	PPN
 *	11-6:	software:
 *		11:	unused
 *		10:	page modified
 *		9:	page accessed
 *		8:	page writeable
 *		7:	page readable
 *		6:	page present
 *	5-4:	RING
 *	3-0:	CA
 *
 * Similar to the Alpha and MIPS ports, we need to keep track of the ref
 * and mod bits in software.  We have a software "yeah you can read
 * from this page" bit, and a hardware one which actually lets the
 * process read from the page.  On the same token we have a software
 * writable bit and the real hardware one which actually lets the
 * process write to the page, this keeps a mod bit via the hardware
 * dirty bit.
 *
 * See further below for PTE layout for swapped-out pages.
 */

#define GLOBAL_RING                  3		/* global ring level (user-level) */
#define USER_RING                    1		/* user ring level */
#define KERNEL_RING                  0		/* kernel ring level (also global) */

#define _PAGE_VALID                 (1<<0)	/* ca: page is accessible
						   (todo: replace with CA!=invalid) */
#define _PAGE_SILENT_READ           (1<<0)	/* ca: (synonym)*/
#define _PAGE_DIRTY                 (1<<1)	/* ca: page is writable
						   (because store attempted, so is "dirty") */
#define _PAGE_SILENT_WRITE          (1<<1)	/* ca: (synonym) */
/*  None of these cache modes include MP coherency:  */
#define _CACHE_BYPASS               (0<<2)	/* ca: bypass cache non-speculative */
#define _CACHE_CACHABLE_NONCOHERENT (1<<2)	/* ca: cachable speculative */
#define _CACHE_UNCACHED             (2<<2)	/* ca: uncached speculative */
#define _CACHE_CACHABLE_SPECIAL     (3<<2)	/* ca: special (isolate, no execute, ...) */
#define _CACHE_MASK                 (15<<0)	/* ca: all CA bits */

#define _PAGE_GLOBAL                (GLOBAL_RING<<4)	/* ring: global access (ring=3) */
/*define _PAGE_xxx                  (3<<4)*/		/* ring: ??? access (ring=2) */
#define _PAGE_USER                  (USER_RING<<4)	/* ring: user access (ring=1) */
#define _PAGE_KERNEL                (KERNEL_RING<<4)	/* ring: kernel access (ring=0) */
#define _PAGE_RING_MASK		    (3<<4)		/* ring bits */

#define _PAGE_PRESENT               (1<<6)	/* software: page not swapped out nor "none" */
#define _PAGE_READ                  (1<<7)	/* software: page readable */
#define _PAGE_WRITE                 (1<<8)	/* software: page writable */
#define _PAGE_ACCESSED              (1<<9)	/* software: page accessed (read) */
#define _PAGE_MODIFIED              (1<<10)	/* software: page modified (written/dirty) */

#define __READABLE	(_PAGE_READ | _PAGE_SILENT_READ | _PAGE_ACCESSED)
#define __WRITEABLE	(_PAGE_WRITE | _PAGE_SILENT_WRITE | _PAGE_MODIFIED)

#define _PAGE_CHG_MASK  (PAGE_MASK | _PAGE_ACCESSED | _PAGE_MODIFIED | _CACHE_MASK)

/* PAGE_COPY is historical and identical to PAGE_READONLY.  See Linux
 * Kernel Internals, second edition, by Beck, et al, pg. 76.
 *
 * PAGE_KERNEL_ASM is an assembler version of PAGE_KERNEL.  The
 * assembler doesn't understand __pgprot, so we need a version without
 * it.  PAGE_KERNEL and PAGE_KERNEL_ASM should always have identical
 * content.
 */
#define PAGE_NONE		__pgprot(_PAGE_PRESENT |			    _PAGE_USER   | _CACHE_CACHABLE_NONCOHERENT)
#define PAGE_SHARED		__pgprot(_PAGE_PRESENT | _PAGE_READ | _PAGE_WRITE | _PAGE_USER   | _CACHE_CACHABLE_NONCOHERENT)
#define PAGE_COPY		__pgprot(_PAGE_PRESENT | _PAGE_READ |		    _PAGE_USER   | _CACHE_CACHABLE_NONCOHERENT)
#define PAGE_READONLY		__pgprot(_PAGE_PRESENT | _PAGE_READ |		    _PAGE_USER   | _CACHE_CACHABLE_NONCOHERENT)
#define PAGE_USERIO		__pgprot(_PAGE_PRESENT | _PAGE_READ | _PAGE_WRITE | _PAGE_USER   | _CACHE_BYPASS)
#define PAGE_KERNEL		__pgprot(_PAGE_PRESENT | __READABLE | __WRITEABLE | _PAGE_KERNEL | _CACHE_CACHABLE_NONCOHERENT)
#define PAGE_KERNEL_ASM		        (_PAGE_PRESENT | __READABLE | __WRITEABLE | _PAGE_KERNEL | _CACHE_CACHABLE_NONCOHERENT)
#define PAGE_KERNEL_UNCACHED	__pgprot(_PAGE_PRESENT | __READABLE | __WRITEABLE | _PAGE_KERNEL |_CACHE_BYPASS)

/*
 * On certain configurations of Xtensa MMUs (eg. the initial Linux config),
 * the MMU can't do page protection for execute, and considers that the same as
 * read.  Also, write permissions may imply read permissions.
 * What follows is the closest we can get by reasonable means..
 * See linux/mm/mmap.c for protection_map[] array that uses these definitions.
 */
#define __P000	PAGE_NONE	/* private --- */
#define __P001	PAGE_READONLY	/* private --r */
#define __P010	PAGE_COPY	/* private -w- */
#define __P011	PAGE_COPY	/* private -wr */
#define __P100	PAGE_READONLY	/* private x-- */
#define __P101	PAGE_READONLY	/* private x-r */
#define __P110	PAGE_COPY	/* private xw- */
#define __P111	PAGE_COPY	/* private xwr */

#define __S000	PAGE_NONE	/* shared  --- */
#define __S001	PAGE_READONLY	/* shared  --r */
#define __S010	PAGE_SHARED	/* shared  -w- */
#define __S011	PAGE_SHARED	/* shared  -wr */
#define __S100	PAGE_READONLY	/* shared  x-- */
#define __S101	PAGE_READONLY	/* shared  x-r */
#define __S110	PAGE_SHARED	/* shared  xw- */
#define __S111	PAGE_SHARED	/* shared  xwr */

#if !defined (_LANGUAGE_ASSEMBLY)

#define pte_ERROR(e) \
	printk("%s:%d: bad pte %08lx.\n", __FILE__, __LINE__, pte_val(e))
#define pmd_ERROR(e) \
	printk("%s:%d: bad pmd entry %08lx.\n", __FILE__, __LINE__, pmd_val(e))
#define pgd_ERROR(e) \
	printk("%s:%d: bad pgd entry %08lx.\n", __FILE__, __LINE__, pgd_val(e))

/*
 * BAD_PAGETABLE is used when we need a bogus (PTE) page-table, while
 * BAD_PAGE is used for a bogus page.
 *
 * ZERO_PAGE is a global shared page that is always zero: used
 * for zero-mapped memory areas etc..
 */
extern pte_t __bad_page(void);
extern pte_t *__bad_pagetable(void);

extern unsigned long empty_zero_page;
/*extern unsigned long zero_page_mask;*/

#define BAD_PAGETABLE __bad_pagetable()
#define BAD_PAGE __bad_page()

/*  No need to do page-colouring, for now:  -Marc  */
#define ZERO_PAGE(vaddr) (virt_to_page(empty_zero_page))
/*#define ZERO_PAGE(vaddr) \
	(virt_to_page(empty_zero_page + (((unsigned long)(vaddr)) & zero_page_mask)))*/

/* number of bits that fit into a memory pointer */
#define BITS_PER_PTR			(8*sizeof(unsigned long))

/* to align the pointer to a pointer address */
#define PTR_MASK			(~(sizeof(void*)-1))

/*
 * sizeof(void*) == (1 << SIZEOF_PTR_LOG2)
 */
#define SIZEOF_PTR_LOG2			2

extern pgd_t swapper_pg_dir[PAGE_SIZE/sizeof(pgd_t)];
extern pte_t invalid_pte_table[PAGE_SIZE/sizeof(pgd_t)];

/* to find an entry in a page-table */
#define PAGE_PTR(address) \
((unsigned long)(address)>>(PAGE_SHIFT-SIZEOF_PTR_LOG2)&PTR_MASK&~PAGE_MASK)

extern void load_pgd(unsigned long pg_dir);

/*
 * Conversion functions: convert a page and protection to a page entry,
 * and a page entry and page directory to the page they refer to.
 */
extern inline unsigned long pmd_page(pmd_t pmd)
{
	return pmd_val(pmd);
}

extern inline void pmd_set(pmd_t * pmdp, pte_t * ptep)
{
	pmd_val(*pmdp) = (((unsigned long) ptep) & PAGE_MASK);
}

/*
 * Notes:
 * - Conditions pte_none() and pte_valid() are mutually exclusive.
 * - Conditions pte_none() and pte_present() are mutually exclusive.
 * - PTEs reported as neither pte_none() nor pte_present() contain information
 *   about swapped out pages; the format of these is described further below
 *   with the SWP_xxx() macro definitions.
 * - PTEs reported as pte_valid() are always pte_present(), but the reverse
 *   is not always true.
 * - pte_clear() causes a PTE entry to satisfy the pte_none() condition.
 *
 * Certain virtual addresses must have specific non-zero ring levels.
 * Which means that a PTE cannot be all zeroes (at least not for
 * any virtual address that might ever be accessible by a user task)
 * to indicate the pte_none() condition.  So we mask out the ring
 * in the pte_none() comparison.
 *
 * XTFIXME: we might get a slight optimization on this by assuming we
 * always use _PAGE_USER for pte_none() entries (which appears to be the
 * case currently), thus comparing the PTE with _PAGE_USER rather
 * than masking out ring bits then comparing to zero.
 */
extern inline void set_pte(pte_t *ptep, pte_t pteval);
extern inline int  pte_none(pte_t pte)    { return (pte_val(pte) & ~_PAGE_RING_MASK) == 0; }
extern inline int  pte_present(pte_t pte) { return pte_val(pte) & _PAGE_PRESENT; }
extern inline int  pte_valid(pte_t pte)   { return pte_val(pte) & _PAGE_VALID; }
extern inline void pte_clear(pte_t *ptep) { set_pte(ptep, __pte(_PAGE_USER)); }

/* Certain architectures need to do special things when pte's
 * within a page table are directly modified.  Thus, the following
 * hook is made available.
 */
extern inline void set_pte(pte_t *ptep, pte_t pteval)
{
	*ptep = pteval;
#if (XCHAL_DCACHE_IS_WRITEBACK > 0) && (XTENSA_CACHE_ALIAS > 0)
	{
	    unsigned long flags;
	    save_and_cli(flags);
	    xthal_dcache_line_writeback((void*)ptep);
	    restore_flags(flags);
	}
#endif
}


/*
 * Empty pgd/pmd entries point to the invalid_pte_table.
 */
extern inline int pmd_none(pmd_t pmd)
{
	return pmd_val(pmd) == (unsigned long) invalid_pte_table;
}

extern void * high_memory;

extern inline int pmd_bad(pmd_t pmd)
{
	return ((pmd_page(pmd) > (unsigned long) high_memory) ||
	        (pmd_page(pmd) < PAGE_OFFSET));
}

extern inline int pmd_present(pmd_t pmd)
{
	return (pmd_val(pmd) != ((unsigned long) invalid_pte_table));
}

extern inline void pmd_clear(pmd_t *pmdp)
{
	pmd_val(*pmdp) = ((unsigned long) invalid_pte_table);
}


/*
 * The "pgd_xxx()" functions here are trivial for a folded two-level
 * setup: the pgd is never bad, and a pmd always exists (as it's folded
 * into the pgd entry)
 */
extern inline int pgd_none(pgd_t pgd)		{ return 0; }
extern inline int pgd_bad(pgd_t pgd)		{ return 0; }
extern inline int pgd_present(pgd_t pgd)	{ return 1; }
extern inline void pgd_clear(pgd_t *pgdp)	{ }

#define pte_page(x)		(mem_map+(unsigned long)((pte_val(x) >> PAGE_SHIFT)))


/*
 * The following only work if pte_present() is true.
 * Undefined behaviour if not..
 */

extern inline int pte_read(pte_t pte)	{ return pte_val(pte) & _PAGE_READ; }
extern inline int pte_write(pte_t pte)	{ return pte_val(pte) & _PAGE_WRITE; }
extern inline int pte_dirty(pte_t pte)	{ return pte_val(pte) & _PAGE_MODIFIED; }
extern inline int pte_young(pte_t pte)	{ return pte_val(pte) & _PAGE_ACCESSED; }

extern inline pte_t pte_wrprotect(pte_t pte)
{
	pte_val(pte) &= ~(_PAGE_WRITE | _PAGE_SILENT_WRITE);
	return pte;
}

extern inline pte_t pte_rdprotect(pte_t pte)
{
	pte_val(pte) &= ~(_PAGE_READ | _PAGE_SILENT_READ);
	return pte;
}

extern inline pte_t pte_mkclean(pte_t pte)
{
	pte_val(pte) &= ~(_PAGE_MODIFIED|_PAGE_SILENT_WRITE);
	return pte;
}

extern inline pte_t pte_mkold(pte_t pte)
{
	pte_val(pte) &= ~(_PAGE_ACCESSED|_PAGE_SILENT_READ);
	return pte;
}

extern inline pte_t pte_mkwrite(pte_t pte)
{
	pte_val(pte) |= _PAGE_WRITE;
	if (pte_val(pte) & _PAGE_MODIFIED)
		pte_val(pte) |= _PAGE_SILENT_WRITE;
	return pte;
}

extern inline pte_t pte_mkread(pte_t pte)
{
	pte_val(pte) |= _PAGE_READ;
	if (pte_val(pte) & _PAGE_ACCESSED)
		pte_val(pte) |= _PAGE_SILENT_READ;
	return pte;
}

extern inline pte_t pte_mkdirty(pte_t pte)
{
	pte_val(pte) |= _PAGE_MODIFIED;
	if (pte_val(pte) & _PAGE_WRITE)
		pte_val(pte) |= _PAGE_SILENT_WRITE;
	return pte;
}

extern inline pte_t pte_mkyoung(pte_t pte)
{
	pte_val(pte) |= _PAGE_ACCESSED;
	if (pte_val(pte) & _PAGE_READ)
		pte_val(pte) |= _PAGE_SILENT_READ;
	return pte;
}


/*
 * Conversion functions: convert a page and protection to a page entry,
 * and a page entry and page directory to the page they refer to.
 *
 * WARNING: Assembly code in arch/xtensa/kernel/handlers.S mimics this
 * macro definition.  If you change this definition, you should review
 * the port in handlers.S.  Within that file, just search for
 * 'mk_pte'.
 */
#define mk_pte(page, pgprot)						\
({									\
	pte_t   __pte;							\
									\
	pte_val(__pte) = ((unsigned long)(page - mem_map) << PAGE_SHIFT) | \
	                 pgprot_val(pgprot);				\
									\
	__pte;								\
})

extern inline pte_t mk_pte_phys(unsigned long physpage, pgprot_t pgprot)
{
	return __pte(((physpage & PAGE_MASK) - PAGE_OFFSET) | pgprot_val(pgprot));
}

extern inline pte_t pte_modify(pte_t pte, pgprot_t newprot)
{
	return __pte((pte_val(pte) & _PAGE_CHG_MASK) | pgprot_val(newprot));
}

#define page_pte(page)		page_pte_prot(page, __pgprot(0))

/* to find an entry in a kernel page-table-directory */
#define pgd_offset_k(address)	pgd_offset(&init_mm, address)

#define pgd_index(address)	((address) >> PGDIR_SHIFT)

/* to find an entry in a page-table-directory */
extern inline pgd_t *pgd_offset(struct mm_struct *mm, unsigned long address)
{
	return mm->pgd + pgd_index(address);
}

/* Find an entry in the second-level page table.. */
extern inline pmd_t *pmd_offset(pgd_t *dir, unsigned long address)
{
	return (pmd_t *) dir;
}

/* Find an entry in the third-level page table.. */ 
extern inline pte_t *pte_offset(pmd_t * dir, unsigned long address)
{
	return (pte_t *) (pmd_page(*dir)) +
	       ((address >> PAGE_SHIFT) & (PTRS_PER_PTE - 1));
}

#endif /*  !defined (_LANGUAGE_ASSEMBLY) */


#ifdef _LANGUAGE_ASSEMBLY

/* Assembly macro _PGD_INDEX is the same as C version pgd_index().
 *
 * Entry Conditions:
 *	a3 = unsigned long address
 *
 * Exit Conditions:
 *	a3 = page directory index
 *	All other registers are preserved
 */

	.macro	_PGD_INDEX	reg
	extui	\reg, \reg, PGDIR_SHIFT, 32-PGDIR_SHIFT
	.endm


/* Assembly macro _PGD_OFFSET is the same as C version pgd_offset().
 *
 * Entry Conditions:
 *	a2 = struct mm_struct *mm
 *	a3 = unsigned long address
 *
 * Exit Conditions:
 *	a2 = pgd_offset(mm, addr)
 *	a3 = pgd_index(addr)
 */

	.macro  _PGD_OFFSET	mm, addr
	l32i	\mm, \mm, MM_PGD
	_PGD_INDEX	\addr
	addx4	\mm, \addr, \mm
	.endm


/* Assembly macro _PMD_OFFSET is the same as C version pmd_offset().
 * Currently, it's just a type case in C and a no-op in assembly.
 * There is no second-level page directory in this version.
 *
 * Entry Conditions:
 *	a2 = pgd_t *pgd
 *	a3 = unsigned long address
 *
 * Exit Conditions:
 *	None.  Macro does nothing.  */

	.macro  _PMD_OFFSET	pgd, addr
	.endm


#endif  /* _LANGUAGE_ASSEMBLY */


#if !defined (_LANGUAGE_ASSEMBLY)

/*
 * Initialize new page directory with pointers to invalid ptes
 */
extern void pgd_init(unsigned long page);

extern void __bad_pte(pmd_t *pmd);
extern void __bad_pte_kernel(pmd_t *pmd);

#define pte_free_kernel(pte)    free_pte_fast(pte)
#define pte_free(pte)           free_pte_fast(pte)
#define pgd_free(pgd)           free_pgd_fast(pgd)
#define pgd_alloc(mm)           get_pgd_fast()

extern int do_check_pgt_cache(int, int);
extern void paging_init(void);

/*
 * Define swap file entries, which are PTEs for swapped-out pages.
 *
 * Background:
 * Each PTE in a process VM's page table is either:
 *   "present" -- valid and not swapped out, protection bits are meaningful;
 *   "not present" -- which further subdivides in these two cases:
 *      "none" -- no mapping at all; identified by pte_none(), set by pte_clear();
 *      "swapped out" -- the page is swapped out, and the SWP macros below
 *      		are used to store swap file info in the PTE itself.
 *
 * In the Xtensa processor MMU, any PTE entries in user space (or anywhere
 * in virtual memory that can map differently across address spaces)
 * must have a correct ring value that represents the RASID field that
 * is changed when switching address spaces.  Eg. such PTE entries cannot
 * be set to ring zero, because that can cause a (global) kernel ASID
 * entry to be created in the TLBs (even with invalid cache attribute),
 * potentially causing a multihit exception when going back to another
 * address space that mapped the same virtual address at another ring.
 *
 * SO: we avoid using ring bits (_PAGE_RING_MASK) in "not present" PTEs.
 * We also avoid using the _PAGE_VALID and _PAGE_PRESENT bits which must
 * be zero for non-present pages.
 *
 * We end up with the following available bits:  1..3 and 7..31.
 * We don't bother with 1..3 for now (we can use them later if needed),
 * and chose to allocate 6 bits for SWP_TYPE and the remaining 19 bits
 * for SWP_OFFSET.  At least 5 bits are needed for SWP_TYPE, because it
 * is currently implemented as an index into swap_info[MAX_SWAPFILES]
 * and MAX_SWAPFILES is currently defined as 32 in <linux/swap.h>.
 * However, for some reason all other architectures in the 2.4 kernel
 * reserve either 6, 7, or 8 bits so I'll not detract from that for now.  :)
 * SWP_OFFSET is an offset into the swap file in page-size units, so
 * with 4 kB pages, 19 bits supports a maximum swap file size of 2 GB.
 *
 * XTFIXME:  2 GB isn't very big.  Other bits can be used to allow
 * larger swap sizes.  In the meantime, it appears relatively easy to get
 * around the 2 GB limitation by simply using multiple swap files.
 */
#define SWP_TYPE(x)		(((x).val >> 7) & 0x3f)
#define SWP_OFFSET(x)		((x).val >> 13)
#define SWP_ENTRY(type,offset)	((swp_entry_t) { ((type) << 7) | ((offset) << 13) })
#define pte_to_swp_entry(pte)	((swp_entry_t) { pte_val(pte) & ~_PAGE_RING_MASK })
#define swp_entry_to_pte(x)	__pte( (x).val | _PAGE_USER )


/* Needs to be defined here and not in linux/mm.h, as it is arch dependent */
#define PageSkip(page)		(0)
#define kern_addr_valid(addr)	(1)

/* TLB operations. */

#define ITLB_WAYS_LOG2      XCHAL_ITLB_WAY_BITS
#define DTLB_WAYS_LOG2      XCHAL_DTLB_WAY_BITS
#define ITLB_PROBE_SUCCESS  (1 << ITLB_WAYS_LOG2)
#define DTLB_PROBE_SUCCESS  (1 << DTLB_WAYS_LOG2)


extern inline void set_rasid_register (unsigned long val)
{
	__asm__ __volatile__ (" wsr   %0, "XTSTR(RASID)"\n\t"
			      " isync\n"
			      : : "a" (val));
}

extern inline unsigned long get_rasid_register (void)
{
	unsigned long tmp;
	__asm__ __volatile__ (" rsr   %0, "XTSTR(RASID)"\n\t"
			      : "=a" (tmp));
	return tmp;
}

extern inline unsigned long itlb_probe(unsigned long addr)
{
	unsigned long tmp;
	__asm__ __volatile__("pitlb  %0, %1\n\t"
			     : "=a" (tmp)
			     : "a" (addr));
	return tmp;
}

extern inline unsigned long dtlb_probe(unsigned long addr)
{
	unsigned long tmp;
	__asm__ __volatile__("pdtlb  %0, %1\n\t"
			     : "=a" (tmp)
			     : "a" (addr));
	return tmp;
}

extern inline void invalidate_itlb_entry (unsigned long probe)
{
	__asm__ __volatile__("iitlb  %0\n\t"
			     "isync\n\t"
			     : : "a" (probe));
}

extern inline void invalidate_dtlb_entry (unsigned long probe)
{
	__asm__ __volatile__("idtlb  %0\n\t"
			     "dsync\n\t"
			     : : "a" (probe));
}

/* Use the .._no_isync functions with caution.  Generally, these are
 * handy for bulk invalidates followed by a single 'isync'.  The
 * caller must follow up with an 'isync', which can be relatively
 * expensive on some Xtensa implementations. */

extern inline void invalidate_itlb_entry_no_isync (unsigned entry)
{
	/* Caller must follow up with 'isync'. */
	asm volatile ("iitlb  %0\n"
		      : : "a" (entry) );
}

extern inline void invalidate_dtlb_entry_no_isync (unsigned entry)
{
	/* Caller must follow up with 'isync'. */
	asm volatile ("idtlb  %0\n"
		      : : "a" (entry) );
}

extern inline void set_itlbcfg_register (unsigned long val)
{
	__asm__ __volatile__("wsr  %0, "XTSTR(ITLBCFG)"\n\t"
			     "isync\n\t"
			     : : "a" (val));
}

extern inline void set_dtlbcfg_register (unsigned long val)
{
	__asm__ __volatile__("wsr  %0, "XTSTR(DTLBCFG)"\n\t"
			     "dsync\n\t"
			     : : "a" (val));
}

extern inline void set_ptevaddr_register (unsigned long val)
{
	__asm__ __volatile__(" wsr  %0, "XTSTR(PTEVADDR)"\n\t"
			     " isync\n"
			     : : "a" (val));
}

extern inline unsigned long read_ptevaddr_register (void)
{
	unsigned long tmp;
	__asm__ __volatile__("rsr  %0, "XTSTR(PTEVADDR)"\n\t"
			     : "=a" (tmp));
	return tmp;
}

extern inline void write_dtlb_entry (pte_t entry, int way)
{
	__asm__ __volatile__("wdtlb  %1, %0\n\t"
			     "dsync\n\t"
			     : : "r" (way), "r" (entry) );
}

extern inline void write_itlb_entry (pte_t entry, int way)
{
	__asm__ __volatile__("witlb  %1, %0\n\t"
			     "isync\n\t"
			     : : "r" (way), "r" (entry) );
}

extern inline unsigned long read_dtlb_virtual (int way)
{
	/* DO NOT USE THIS FUNCTION.  This instruction isn't part of
	 * the Xtensa ISA and exists only for test purposes on T1040.
	 * You may find it helpful for MMU debugging, however.
	 *
	 * 'at' is the unmodified input register
	 * 'as' is the output register, as follows (specific to the Linux config):
	 *
	 *      as[31..12] contain the virtual address
	 *      as[11..08] are meaningless
	 *      as[07..00] contain the asid
	 */

	unsigned long tmp;
	__asm__ __volatile__("rdtlb0  %0, %1\n\t"
			     : "=a" (tmp), "+a" (way)   );
	return tmp;
}

extern inline unsigned long read_dtlb_translation (int way)
{
	/* DO NOT USE THIS FUNCTION.  This instruction isn't part of
	 * the Xtensa ISA and exists only for test purposes on T1040.
	 * You may find it helpful for MMU debugging, however.
	 *
	 * 'at' is the unmodified input register
	 * 'as' is the output register, as follows (specific to the Linux config):
	 *
	 *      as[31..12] contain the physical address
	 *      as[11..04] are meaningless
	 *      as[03..00] contain the cache attribute
	 */

	unsigned long tmp;
	__asm__ __volatile__("rdtlb1  %0, %1\n\t"
			     : "=a" (tmp), "+a" (way)   );
	return tmp;
}

extern inline unsigned long read_itlb_virtual (int way)
{
	/* DO NOT USE THIS FUNCTION.  This instruction isn't part of
	 * the Xtensa ISA and exists only for test purposes on T1040.
	 * You may find it helpful for MMU debugging, however.
	 *
	 * 'at' is the unmodified input register
	 * 'as' is the output register, as follows (specific to the Linux config):
	 *
	 *      as[31..12] contain the virtual address
	 *      as[11..08] are meaningless
	 *      as[07..00] contain the asid
	 */

	unsigned long tmp;
	__asm__ __volatile__("ritlb0  %0, %1\n\t"
			     : "=a" (tmp), "+a" (way)   );
	return tmp;
}

extern inline unsigned long read_itlb_translation (int way)
{
	/* DO NOT USE THIS FUNCTION.  This instruction isn't part of
	 * the Xtensa ISA and exists only for test purposes on T1040.
	 * You may find it helpful for MMU debugging, however.
	 *
	 * 'at' is the unmodified input register
	 * 'as' is the output register, as follows (specific to the Linux config):
	 *
	 *      as[31..12] contain the physical address
	 *      as[11..04] are meaningless
	 *      as[03..00] contain the cache attribute
	 */

	unsigned long tmp;
	__asm__ __volatile__("ritlb1  %0, %1\n\t"
			     : "=a" (tmp), "+a" (way)   );
	return tmp;
}


extern inline void invalidate_page_table (void)
{
	invalidate_dtlb_entry (WIRED_WAY_FOR_PAGE_TABLE);
}

extern inline void invalidate_itlb_mapping (unsigned address)
{
	unsigned long tlb_entry;
	if ((tlb_entry = itlb_probe (address)) & ITLB_PROBE_SUCCESS)
		invalidate_itlb_entry (tlb_entry);
}

extern inline void invalidate_dtlb_mapping (unsigned address)
{
	unsigned long tlb_entry;
	if ((tlb_entry = dtlb_probe (address)) & DTLB_PROBE_SUCCESS)
		invalidate_dtlb_entry (tlb_entry);
}


/* The kernel (in mm/memory.c) often invokes this macro when a page
 * previously wasn't mapped.  In handle_2nd_level_miss() when no page
 * is mapped, we map the invalid_pte_table into the Page Table to
 * generate another exception that exposes the type of access.  After
 * mapping in the appropriate page (often by calling the kernel's
 * handle_mm_fault()), we need to remove the mapping of
 * invalid_pte_table from the Page Table.  This macro does this
 * function.  The 2nd-level miss handler will later fill in the
 * correct mapping. */

extern  void update_mmu_cache(struct vm_area_struct * vma,
			      unsigned long address, 
			      pte_t pte);


/* XTFIXME:  Possible optimization opportunity here. */
//#include <asm-generic/pgtable.h>
/* 27feb2003 jn
 * copied these into pgalloc.h to remove a circular
 * dependency.
 */




#endif /* !defined (_LANGUAGE_ASSEMBLY) */

#define io_remap_page_range remap_page_range

/* No page table caches to init */
#define pgtable_cache_init()	do { } while (0)

#endif /* __ASM_XTENSA_PGTABLE_H */
