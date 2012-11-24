/***********************************************************************
 * include/asm-mips/jmr3927/pci.h
 *
 * Author: source@mvista.com
 *
 * Based on include/asm-mips/ddb5xxx/pci.h
 *
 * This file essentially defines the interface between board
 * specific PCI code and MIPS common PCI code.  Should potentially put
 * into include/asm/pci.h file.
 *
 * Copyright (C) 2000-2001 Toshiba Corporation 
 *
 * 2001-2003 (c) MontaVista Software, Inc. This file is licensed under the
 * terms of the GNU General Public License version 2. This program is
 * licensed "as is" without any warranty of any kind, whether express
 * or implied.
 */

#ifndef __ASM_TX3927_PCI_H
#define __ASM_TX3927__PCI_H

#include <linux/ioport.h>
#include <linux/pci.h>

/*
 * Each pci channel is a top-level PCI bus seem by CPU.  A machine  with
 * multiple PCI channels may have multiple PCI host controllers or a
 * single controller supporting multiple channels.
 */
struct pci_channel {
	struct pci_ops *pci_ops;
	struct resource *io_resource;
	struct resource *mem_resource;
};

/*
 * each board defines an array of pci_channels, that ends with all NULL entry
 */
extern struct pci_channel mips_pci_channels[];

/*
 * board supplied pci irq fixup routine
 */
extern void pcibios_fixup_irqs(void);

#endif  /* __ASM_TX3927_PCI_H */
