#ifndef __ASM_XTENSA_PGALLOC_H
#define __ASM_XTENSA_PGALLOC_H

/*
 * include/asm-xtensa/pgalloc.h
 *
 * Derived from MIPS.
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2001 Tensilica Inc.
 *	Authors:	Marc Gauthier
 *			Joe Taylor <joe@tensilica.com, joetylr@yahoo.com>
 */

#include <linux/config.h>
#include <linux/mm.h>

/* TLB flushing:
 *
 *  - flush_tlb_all() flushes all processes TLB entries
 *  - flush_tlb_mm(mm) flushes the specified mm context TLB entries
 *  - flush_tlb_page(mm, vmaddr) flushes a single page
 *  - flush_tlb_range(mm, start, end) flushes a range of pages
 */
extern void flush_tlb_all(void);
extern void flush_tlb_mm(struct mm_struct *mm);
extern void flush_tlb_range(struct mm_struct *mm, unsigned long start,
			       unsigned long end);
extern void flush_tlb_page(struct vm_area_struct *vma, unsigned long page);

extern inline void flush_tlb_pgtables(struct mm_struct *mm,
                                      unsigned long start, unsigned long end)
{
	/* Nothing to do on Xtensa Processors.  */
}


/*
 * Allocate and free page tables. The xxx_kernel() versions are
 * used to allocate a kernel page table - this turns on ASN bits
 * if any.
 */

#define pgd_quicklist (current_cpu_data.pgd_quick)
#define pmd_quicklist ((unsigned long *)0)
#define pte_quicklist (current_cpu_data.pte_quick)
#define pgtable_cache_size (current_cpu_data.pgtable_cache_sz)

extern __inline__ pgd_t *get_pgd_slow(void)
{
	pgd_t *ret = (pgd_t *)__get_free_page(GFP_KERNEL), *init;

	if (ret) {
		init = pgd_offset(&init_mm, 0);
		pgd_init((unsigned long)ret);
		memcpy (ret + USER_PTRS_PER_PGD, init + USER_PTRS_PER_PGD,
			(PTRS_PER_PGD - USER_PTRS_PER_PGD) * sizeof(pgd_t));
	}
	return ret;
}

extern __inline__ pgd_t *get_pgd_fast(void)
{
	unsigned long *ret;

	preempt_disable();
	if((ret = pgd_quicklist) != NULL) {
		pgd_quicklist = (unsigned long *)(*ret);
		ret[0] = ret[1];
		pgtable_cache_size--;
		preempt_enable();
	} else {
		preempt_enable();
		ret = (unsigned long *)get_pgd_slow();
	}
	return (pgd_t *)ret;
}

extern __inline__ void free_pgd_fast(pgd_t *pgd)
{
	preempt_disable();
	*(unsigned long *)pgd = (unsigned long) pgd_quicklist;
	pgd_quicklist = (unsigned long *) pgd;
	pgtable_cache_size++;
	preempt_enable();
}

extern __inline__ void free_pgd_slow(pgd_t *pgd)
{
	free_page((unsigned long)pgd);
}

extern pte_t *get_pte_slow(pmd_t *pmd, unsigned long address_preadjusted);
extern pte_t *get_pte_kernel_slow(pmd_t *pmd, unsigned long address_preadjusted);

extern __inline__ pte_t *get_pte_fast(void)
{
	unsigned long *ret;

	preempt_disable();
	if((ret = (unsigned long *)pte_quicklist) != NULL) {
		pte_quicklist = (unsigned long *)(*ret);
		ret[0] = ret[1];
		pgtable_cache_size--;
	}
	preempt_enable();
	return (pte_t *)ret;
}

extern __inline__ void free_pte_fast(pte_t *pte)
{
	preempt_disable();
	*(unsigned long *)pte = (unsigned long) pte_quicklist;
	pte_quicklist = (unsigned long *) pte;
	pgtable_cache_size++;
	preempt_enable();
}

extern __inline__ void free_pte_slow(pte_t *pte)
{
	free_page((unsigned long)pte);
}

/* We don't use pmd cache, so these are dummy routines */
extern __inline__ pmd_t *get_pmd_fast(void)
{
	return (pmd_t *)0;
}

extern __inline__ void free_pmd_fast(pmd_t *pmd)
{
}

extern __inline__ void free_pmd_slow(pmd_t *pmd)
{
}

extern void __bad_pte(pmd_t *pmd);
extern void __bad_pte_kernel(pmd_t *pmd);

#define pte_free_kernel(pte)    free_pte_fast(pte)
#define pte_free(pte)           free_pte_fast(pte)
#define pgd_free(pgd)           free_pgd_fast(pgd)
#define pgd_alloc(mm)           get_pgd_fast()

/*
 * allocating and freeing a pmd is trivial: the 1-entry pmd is
 * inside the pgd, so has no extra memory associated with it.
 * (In the PAE case we free the pmds as part of the pgd.)
 */

#define pmd_alloc_one_fast(mm, addr)	({ BUG(); ((pmd_t *)1); })
#define pmd_alloc_one(mm, addr)		({ BUG(); ((pmd_t *)2); })
#define pmd_populate(mm, pmd, pte)	pmd_set(pmd, pte)
#define pgd_populate(mm, pmd, pte)	BUG()


static inline pte_t *pte_alloc_one(struct mm_struct *mm, unsigned long address)
{
	pte_t *pte;

	pte = (pte_t *) __get_free_page(GFP_KERNEL);
	if (pte)
		clear_page(pte);
	return pte;
}

static inline pte_t *pte_alloc_one_fast(struct mm_struct *mm, unsigned long address)
{
	unsigned long *ret;

	preempt_disable();
	if ((ret = (unsigned long *)pte_quicklist) != NULL) {
		pte_quicklist = (unsigned long *)(*ret);
		ret[0] = ret[1];
		pgtable_cache_size--;
	}
	preempt_enable();
	return (pte_t *)ret;
}


/*
 * allocating and freeing a pmd is trivial: the 1-entry pmd is
 * inside the pgd, so has no extra memory associated with it.
 */
extern inline void pmd_free(pmd_t * pmd)
{
}

#define pmd_free_kernel		pmd_free


extern int do_check_pgt_cache(int, int);

extern inline void set_pgdir(unsigned long address, pgd_t entry)
{
	struct task_struct * p;
	pgd_t *pgd;
#ifdef CONFIG_SMP
	int i;
#endif	

	read_lock(&tasklist_lock);
	for_each_task(p) {
		if (!p->mm)
			continue;
		*pgd_offset(p->mm,address) = entry;
	}
	read_unlock(&tasklist_lock);
#ifndef CONFIG_SMP
	for (pgd = (pgd_t *)pgd_quicklist; pgd; pgd = (pgd_t *)*(unsigned long *)pgd)
		pgd[address >> PGDIR_SHIFT] = entry;
#else
	/* To pgd_alloc/pgd_free, one holds master kernel lock and so does our
	   callee, so we can modify pgd caches of other CPUs as well. -jj */
	for (i = 0; i < NR_CPUS; i++)
		for (pgd = (pgd_t *)cpu_data[i].pgd_quick; pgd; pgd = (pgd_t *)*(unsigned long *)pgd)
			pgd[address >> PGDIR_SHIFT] = entry;
#endif
}


static inline int ptep_test_and_clear_young(pte_t *ptep)
{
	pte_t pte = *ptep;
	if (!pte_young(pte))
		return 0;
	set_pte(ptep, pte_mkold(pte));
	return 1;
}

static inline int ptep_test_and_clear_dirty(pte_t *ptep)
{
	pte_t pte = *ptep;
	if (!pte_dirty(pte))
		return 0;
	set_pte(ptep, pte_mkclean(pte));
	return 1;
}

#if XTENSA_CACHE_ALIAS
static inline pte_t ptep_get_and_clear(pte_t *ptep)
{
	pte_t pte = *ptep;
	pte_clear(ptep);
	if (!pte_present(pte)) {
		struct page *page = pte_page(pte);
		if (VALID_PAGE(page) &&
		    (!page->mapping || !(page->mapping->i_mmap_shared))) {
			clear_bit(PG_mapped, &page->flags);
		}
	}
	return pte;
}
#else
static inline pte_t ptep_get_and_clear(pte_t *ptep)
{
	pte_t pte = *ptep;
	pte_clear(ptep);
	return pte;
}
#endif

static inline void ptep_set_wrprotect(pte_t *ptep)
{
	pte_t old_pte = *ptep;
	set_pte(ptep, pte_wrprotect(old_pte));
}

static inline void ptep_mkdirty(pte_t *ptep)
{
	pte_t old_pte = *ptep;
	set_pte(ptep, pte_mkdirty(old_pte));
}

#define pte_same(A,B)	(pte_val(A) == pte_val(B))


#endif /* __ASM_XTENSA_PGALLOC_H */
