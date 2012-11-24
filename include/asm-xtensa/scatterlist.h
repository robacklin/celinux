#ifndef __ASM_XTENSA_SCATTERLIST_H
#define __ASM_XTENSA_SCATTERLIST_H

/*
 * include/asm-xtensa/scatterlist.h
 *
 * This file is subject to the terms and conditions of the GNU General
 * Public License.  See the file "COPYING" in the main directory of
 * this archive for more details.
 */


struct scatterlist {
    char *  address;    /* Location data is to be transferred to */
    char * alt_address; /* Location of actual if address is a 
			 * dma indirect buffer.  NULL otherwise */
    unsigned int length;
    
    __u32 dvma_address;
};

struct mmu_sglist {
        char *addr;
        char *__dont_touch;
        unsigned int len;
        __u32 dvma_addr;
};

#define ISA_DMA_THRESHOLD (0x00ffffff)

#endif /* __ASM_XTENSA_SCATTERLIST_H */
