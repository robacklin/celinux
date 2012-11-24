#ifndef __ASM_MMU_H
#define __ASM_MMU_H

#include <asm/types.h>

typedef        phys_t  phys_addr_t;
extern phys_t (*fixup_bigphys_addr)(phys_t phys_addr, phys_t size);

typedef unsigned long mm_context_t[NR_CPUS];

#endif /* __ASM_MMU_H */
