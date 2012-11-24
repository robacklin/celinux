/*
 * linux/arch/sh/kernel/pci-kzp01.c
 *
 * Copyright (C) 2003 Mitsubishi Electric Corporation
 * Based on pci-kzp01.c written by Ian DaSilva.
 *
 *
 * Author:  Ian DaSilva (idasilva@mvista.com)
 *
 * Highly leveraged from pci-bigsur.c, written by Dustin McIntire.
 *
 * May be copied or modified under the terms of the GNU General Public
 * License.  See linux/COPYING for more information.
 *
 * PCI initialization for the Hitachi SH7751 Solution Engine board (MS7751SE01)
 */

#include <linux/config.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pci.h>

#include <asm/io.h>
#include <asm/pci-sh7751.h>


/* PCI INT->IRQ transfer table*/
extern const int irq_list[];


/*
 * Description:  This function sets up and initializes the pcic, sets
 * up the BARS, maps the DRAM into the address space etc, etc.
 */
int __init pcibios_init_platform(void)
{
   /* All done, may as well say so... */
   printk("SH7751 PCI: Finished initialization of the PCI controller\n");

   return 1;
}

int __init pcibios_map_platform_irq(u8 slot, u8 pin)
{
	int			irq, irq_offset;

	switch( slot ){
	case 9:		/* LAN */
		irq_offset = 2;
		break;
	case 10:	/* VGA */
		irq_offset = 3;
		break;
#if 0
	case 15:	/* SuperI/O USB */
		irq_offset = 0;
		break;
#endif
	/* expansion slot */
	case 5:
	case 6:
	case 7:
		irq_offset = slot+3;
		break;

        default:
		return( -1 );
        }
	irq = irq_list[(irq_offset+pin-1)%4];

	//printk( "PCI: IRQ=%d slot=%d pin=%d\n", irq, (int)slot, (int)pin );
	return( irq );
}
