/*
 * linux/drivers/ide/tx4938ide.c 
 *
 * TX4938 internal IDE driver
 *
 * Author: source@mvista.com
 *
 * Copyright 2001-2002 MontaVista Software Inc.
 *
 * Copyright (C) 2000-2001 Toshiba Corporation 
 *
 * 2003 (c) MontaVista Software, Inc. This file is licensed under the
 * terms of the GNU General Public License version 2. This program is
 * licensed "as is" without any warranty of any kind, whether express
 * or implied.
 */

#include <linux/types.h>
#include <linux/mm.h>
#include <linux/interrupt.h>
#include <linux/blkdev.h>
#include <linux/hdreg.h>
#include <linux/ide.h>
#include <linux/init.h>
#include <asm/bootinfo.h>
#include <asm/tx4938/rbtx4938.h>

static int tx4938_ide_irq;

#ifdef __BIG_ENDIAN
/* TX4938 ATA(BigEndian) has a byte-swapped IDE interface */
static inline void insw_swap(unsigned int port, void *addr, unsigned long count)
{
	unsigned short *ptr = (unsigned short *)addr;
	printk("insw_swap\n");
	while (count--) {
		*ptr++ = inw(port);
	}
}
static inline void insl_swap(unsigned int port, void *addr, unsigned long count)
{
	unsigned int *ptr = (unsigned int *)addr;
	printk("insl_swap\n");
	while (count--) {
		*ptr++ = inl(port);
	}
}
static inline void outsw_swap(unsigned int port, const void *addr, unsigned long count)
{
	unsigned short *ptr = (unsigned short *)addr;
	printk("outsw_swap\n");
	while (count--) {
		outw((*ptr), port);
		ptr++;
	}
}
static inline void outsl_swap(unsigned int port, const void *addr, unsigned long count)
{
	unsigned int *ptr = (unsigned int *)addr;
	printk("outsl_swap\n");
	while (count--) {
		outl((*ptr), port);
		ptr++;
	}
}

static void tx4938_ideproc(ide_ide_action_t action, ide_drive_t *drive, void *buffer, unsigned int count)
{
	unsigned long port;
	unsigned int wcount = count;
	byte io_32bit = drive->io_32bit;
	port = IDE_DATA_REG;
#ifdef __BIG_ENDIAN
	port &= ~1;
#endif
	switch (action) {
	case ideproc_atapi_input_bytes:
	case ideproc_atapi_output_bytes:
		count++;
		wcount = count / 4;
		break;
	/* get rid of gcc 3.x warnings */
	case ideproc_ide_input_data:
	case ideproc_ide_output_data:
		break;
	}
	switch (action) {
	case ideproc_ide_input_data:
	case ideproc_atapi_input_bytes:
		if (io_32bit)
			insl_swap(port, buffer, wcount);
		else
			insw_swap(port, buffer, wcount<<1);
		break;
	case ideproc_ide_output_data:
	case ideproc_atapi_output_bytes:
		if (io_32bit)
			outsl_swap(port, buffer, wcount);
		else
			outsw_swap(port, buffer, wcount<<1);
		break;
	}
	switch (action) {
	case ideproc_atapi_input_bytes:
		if ((count & 0x03) >= 2)
			insw_swap(port, (u32 *)buffer + wcount, 1);
		break;
	case ideproc_atapi_output_bytes:
		if ((count & 0x03) >= 2)
			outsw_swap(port, (u32 *)buffer + wcount, 1);
		break;
	/* get rid of gcc 3.x warnings */
	case ideproc_ide_input_data:
	case ideproc_ide_output_data:
		break;
	}
}
#endif	/* __BIG_ENDIAN */

void __init tx4938_ide_init(void)
{
	hw_regs_t hw;
	int index;
	int offsets[IDE_NR_PORTS];
	int i;
	unsigned long port;

	for (i = 0; i < 8; i++)
		offsets[i] = i;
	offsets[IDE_CONTROL_OFFSET] = 6;
	offsets[IDE_CONTROL_OFFSET] += 0x10000;

	tx4938_ide_irq = RBTX4938_IRQ_IOC_ATA;

	if ((tx4938_ccfgptr->pcfg & (TX4938_PCFG_ATA_SEL | TX4938_PCFG_NDF_SEL)) !=
	    TX4938_PCFG_ATA_SEL)
		return;
	for (i = 0; i < 8; i++) {
		/* check EBCCRn.ISA, EBCCRn.BSZ, EBCCRn.ME */
		if ((tx4938_ebuscptr->cr[i] & 0x00f00008) == 0x00e00008)
			break;
	}
	if (i == 8) {
		printk(KERN_DEBUG "TX4938 ATA channel not found.\n");
		return;
	}

	port = KSEG1ADDR((tx4938_ebuscptr->cr[i] >> 48) << 20) + 0x10000 -
		mips_io_port_base;

	memset(&hw, 0, sizeof(hw));
	ide_setup_ports(&hw, port, offsets, 0, 0, 0, tx4938_ide_irq);
	index = ide_register_hw(&hw, NULL);
	printk("index returned is %d\n",index);
	if (index != -1) {
		ide_hwif_t *hwif = &ide_hwifs[index];
#ifdef __BIG_ENDIAN
		hwif->ideproc = tx4938_ideproc;
#endif	/* __BIG_ENDIAN */
		printk("%s: TX4938 IDE interface\n", hwif->name);
	}
}
