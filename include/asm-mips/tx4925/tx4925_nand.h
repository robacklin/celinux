#ifndef TX4925_TX4925_NAND_H
#define TX4925_TX4925_NAND_H

/*
 * linux/include/asm-mips/tx4925/tx4925_nand.h
 *
 * tx4925_nand defs
 *
 * Author: MontaVista Software, Inc.  source@mvista.com
 *
 * Copyright 2002 MontaVista Software Inc.
 *
 * 2003 (c) MontaVista Software, Inc. This file is licensed under
 * the terms of the GNU General Public License version 2. This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 */

#include <asm/tx4925/tx4925.h>

#define tx4925_read_nfmc(addr) (*(volatile unsigned int *)(addr))
#define tx4925_write_nfmc(b,addr) (*(volatile unsigned int *)(addr)) = (b)

/* TX4925 NAND Flash Memory Mode Control Register (NDFMCR) bits */
#define TX4925_NDFMCR_WE              0x80 /* Write Enable         */
#define TX4925_NDFMCR_ECC_CNTL_MASK   0x60 /* ECC Control          */
#define TX4925_NDFMCR_ECC_CNTL_ENAB   0x20 /* ECC Control          */
#define TX4925_NDFMCR_ECC_CNTL_READ   0x40 /* ECC Control          */
#define TX4925_NDFMCR_ECC_CNTL_DISAB  0x00 /* ECC Control          */
#define TX4925_NDFMCR_CE              0x10 /* Chip Enable          */
#define TX4925_NDFMCR_RESERVED        0x08 /* Reserved             */
#define TX4925_NDFMCR_BSPRT           0x04 /* Bus Seperate         */
#define TX4925_NDFMCR_ALE             0x02 /* Address Latch Enable */
#define TX4925_NDFMCR_CLE             0x01 /* Command Latch Enable */

/* TX4925 NAND Flash Memory Status Register (NDFSR) bits */
#define TX4925_NDSFR_BUSY             0x80 /* 0: Ready 1: Busy */

/* TX4925 NAND Flash Memory Interrupt Status Register (NDFISR) bits */
#define TX4925_NDFISR_RDY             0x01 /* Ready */

/* TX4925 NAND Flash Memory Interrupt Mask Register (NDFIMR) bits */
#define TX4925_NDFIMR_INTEN           0x80 /* Interrupt Enable */
#define TX4925_NDFIMR_MRDY            0x01 /* Mask Ready Interupt */

/* TX4925 NAND Flash Memory Reset Register (NDFRSTR) bits */
#define TX4925_NDFRSTR_RST            0x01 /* Reset */

struct tx4925_ndfmc_reg {
       volatile unsigned long dtr;
       volatile unsigned long mcr;
       volatile unsigned long sr;
       volatile unsigned long isr;
       volatile unsigned long imr;
       volatile unsigned long spr;
       volatile unsigned long rstr;
};                                       

#define TX4925_HOLD           2
#define TX4925_SPW            6
#define TX4925_BSPRT          1

#ifndef _LANGUAGE_ASSEMBLY

#define tx4925_ndfmcptr	  ((struct tx4925_ndfmc_reg *)TX4925_REG(TX4925_NDFMC_BASE))

#endif /* _LANGUAGE_ASSEMBLY */

#endif /*  TX4925_NAND_H */
