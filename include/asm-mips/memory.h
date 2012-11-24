/*
 *  linux/include/asm-mips/memory.h
 *
 *  Copyright (C) 2003-2004 MontaVista Software
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 *  Note: this file should not be included by non-asm/.h files
 */
#ifndef __ASM_MIPS_MEMORY_H
#define __ASM_ARM_MEMORY_H

#include <linux/config.h>
/*
 * PFNs are used to describe any physical page; this means
 * PFN 0 == physical address 0.
 *
 * This is the PFN of the first RAM page in the kernel
 * direct-mapped view.  We assume this is the first page
 * of RAM in the mem_map as well.
 */
#define PHYS_OFFSET	UNCAC_BASE
#define PHYS_PFN_OFFSET (PHYS_OFFSET >> PAGE_SHIFT)

/*
 * Conversion between a struct page and a physical address.
 *
 * Note: when converting an unknown physical address to a
 * struct page, the resulting pointer must be validated
 * using VALID_PAGE().  It must return an invalid struct page
 * for any physical address not corresponding to a system
 * RAM address.
 *
*  page_to_pfn(page)   convert a struct page * to a PFN number
*  pfn_to_page(pfn)    convert a _valid_ PFN number to struct page *
*  pfn_valid(pfn)      indicates whether a PFN number is valid
*
*  virt_to_page(k)     convert a _valid_ virtual address to struct page *
*  virt_addr_valid(k)  indicates whether a virtual address is valid
*/
#ifndef CONFIG_DISCONTIGMEM

#define __phys_to_virt	phys_to_virt
#define page_to_pfn(page)       (((page) - mem_map) + PHYS_PFN_OFFSET)
#define pfn_to_page(pfn)        ((mem_map + (pfn)) - PHYS_PFN_OFFSET)
#define pfn_valid(pfn)          ((pfn) >= PHYS_PFN_OFFSET && (pfn) < (PHYS_PFN_OFFSET + max_mapnr))

#endif
#endif
