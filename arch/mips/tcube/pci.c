/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Based on:
 *   SNI specific PCI support for RM200/RM300.
 *
 *   Copyright (C) 1997 - 2000 Ralf Baechle
 */
#include <linux/config.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/pci.h>
#include <linux/types.h>
#include <linux/delay.h>

#include <asm/byteorder.h>
#include <asm/pci_channel.h>

#include <asm/tcube.h>

#include <linux/hdreg.h>
#include <asm/io.h>

static struct resource extpci_io_resource = {
    "ext pci IO space", 
    0,
    0x007FFFFF,
    IORESOURCE_IO
};

static struct resource extpci_mem_resource = {
    "ext pci memory space", 
    0x10000000,
    0x17FFFFFF,
    IORESOURCE_MEM
};

static struct resource iopci_io_resource = {
    "io pci IO space", 
    0x00800000,
    0x00FFFFFF,
    IORESOURCE_IO
};

static struct resource iopci_mem_resource = {
    "io pci memory space", 
    0x19000000,
    0x197FFFFF,
    IORESOURCE_MEM
};

extern struct pci_ops ddb5477_ext_pci_ops;
extern struct pci_ops ddb5477_io_pci_ops;

struct pci_channel mips_pci_channels[] = {
    { &ddb5477_ext_pci_ops, &extpci_io_resource, &extpci_mem_resource },
    { &ddb5477_io_pci_ops, &iopci_io_resource, &iopci_mem_resource },
    { NULL, NULL, NULL}
};

void __init pcibios_fixup(void)
{
    // --------------------------------------------
    // Okamura module  >>> move to board init()
    // --------------------------------------------
}


#define DATA_REG(x)		(x)
#define ERROR_REG(x)		((x) + 0x01)
#define FEATURE_REG(x)		((x) + 0x01)
#define SECTOR_CNT_REG(x)	((x) + 0x02)
#define SECTOR_NUM_REG(x)	((x) + 0x03)
#define CYLINDER_LOW_REG(x)	((x) + 0x04)
#define CYLINDER_HIGH_REG(x)	((x) + 0x05)
#define DRIVE_HEAD_REG(x)	((x) + 0x06)
#define STATUS_REG(x)		((x) + 0x07)
#define CMD_REG(x)		((x) + 0x07)
#define CTRL_REG(x)		((x) + 0x0e)

#define NUM_5701_IRQS 	32
#define NUM_5701_EPCI_IRQ 	4

extern int vr5701_irq_to_irq(int irq);
void __init pcibios_fixup_irqs(void)
{
    struct pci_dev *dev;
    int slot_num;

    pci_for_each_dev(dev) {
	int k;

	slot_num = PCI_SLOT(dev->devfn);
	switch( slot_num ){	// slot_num == AD[num]
	    /* EPCI */
	case 25-11:		// USB    INTC#
	    k = NUM_5701_IRQS + 2 ; break ;
	case 26-11:		// LAN    INTB#
	    k = NUM_5701_IRQS + 1 ; break ;
	case 27-11:		// VIDEO  INTA#
	    k = NUM_5701_IRQS + 0 ; break ;

	    /* IPCI */
	case 29-11:		// AC97   INTC#
	    k = NUM_5701_IRQS + NUM_5701_EPCI_IRQ + 2 ; break ;
	case 30-11:		// IDE    INTB#
	    k = NUM_5701_IRQS + NUM_5701_EPCI_IRQ + 1 ; break ;
	case 31-11:		// USBH   INTA#
	    k = NUM_5701_IRQS + NUM_5701_EPCI_IRQ + 0 ; break ;
	default:
	    printk(KERN_ERR "%s(): unknown slot_num=%d\n", __FUNCTION__, slot_num);
	    return;
	}

	pci_write_config_byte(dev, PCI_INTERRUPT_LINE, k);
	dev->irq = vr5701_irq_to_irq(k);
#ifdef DEBUG
	printk("%s %x slotnum = %d", dev->name, dev->class, slot_num);
	printk(" -> PCI IRQ to %d\n",k);
#endif
    }
}

#define IO_MEM_LOGICAL_START   0x3e000000
#define IO_MEM_LOGICAL_END     0x3fefffff

#define IO_PORT_LOGICAL_START  0x3ff00000
#define IO_PORT_LOGICAL_END    0x3fffffff

#define IO_MEM_VIRTUAL_OFFSET  0xb0000000
#define IO_PORT_VIRTUAL_OFFSET 0xb0000000

#define ONE_MEG   (1024 * 1024)

void __init pci_setup(void)
{
}

void __init ddb_pci_reset_bus(void)
{
    u32 temp;
    volatile unsigned long i;
    /*
     * I am not sure about the "official" procedure, the following
     * steps work as far as I know:
     * We first set PCI cold reset bit (bit 31) in PCICTRL-H.
     * Then we clear the PCI warm reset bit (bit 30) to 0 in PCICTRL-H.
     * The same is true for both PCI channels.
     */

#define FULL_RESET
#ifdef FULL_RESET
    temp = 0x08040000;
    ddb_out32(EPCI_CTRLL, temp);
    ddb_out32(IPCI_CTRLL, temp);
    temp = 0x42840018; //assert warm reset
    ddb_out32(EPCI_CTRLH, temp);
    ddb_out32(IPCI_CTRLH, temp);
    temp = 0x0;  // clear err & init intmask
    ddb_out32(EPCI_INTM, temp);
    ddb_out32(EPCI_ERR, temp);
    ddb_out32(IPCI_INTM, temp);
    ddb_out32(IPCI_ERR, temp);
#else
    temp = ddb_in32(EPCI_CTRLH);
    temp |= 0x40000000;
    ddb_out32(EPCI_CTRLH, temp);

    temp = ddb_in32(IPCI_CTRLH);
    temp |= 0x40000000;
    ddb_out32(IPCI_CTRLH, temp);
#endif
    for(i=0;i<1000;i++);

    temp = ddb_in32(EPCI_CTRLH);
    temp &= ~0xc0000000;
    ddb_out32(EPCI_CTRLH, temp);

    temp = ddb_in32(IPCI_CTRLH);
    temp &= ~0xc0000000;
    ddb_out32(IPCI_CTRLH, temp);
}

unsigned __init pcibios_assign_all_busses(void)
{
    /* we hope pci_auto has assigned the bus numbers to all buses */
    return 1;
}

int __init ide_wait(int ioport)
{
	int i;
	
	i=0;
	udelay(20);
	while ( inb(STATUS_REG(ioport)) & BUSY_STAT &&   /* busy & ~ready */
		~(inb(STATUS_REG(ioport)) &READY_STAT) ){
		i++;
		udelay(20);
		if (i>5000) {
			printk(" status = %x, error = %x\n",
			       inb(STATUS_REG(ioport)), inb(ERROR_REG(ioport)));
			return i;
		}
	}
	return 0;
}

void __init pcibios_fixup_resources(struct pci_dev *dev)
{
    int bases;

#if 1
    int pos;

    printk("adjusting pci device: %s\n", dev->name);
 
    switch (dev->hdr_type) {
    case PCI_HEADER_TYPE_NORMAL: bases = 6; break;
    case PCI_HEADER_TYPE_BRIDGE: bases = 2; break;
    case PCI_HEADER_TYPE_CARDBUS: bases = 1; break;
    default: bases = 0; break;
    }
    for (pos=0; pos < bases; pos++) {
	struct resource* res = &dev->resource[pos];
	if (res->start >= IO_MEM_LOGICAL_START && res->end <= IO_MEM_LOGICAL_END) {
	    res->start += IO_MEM_VIRTUAL_OFFSET;
	    res->end += IO_MEM_VIRTUAL_OFFSET;
	    printk("  MEM : %lx - %lx\n",res->start, res->end);
	}
	if (res->start >= IO_PORT_LOGICAL_START && res->end <= IO_PORT_LOGICAL_END) {
	    res->start += IO_PORT_VIRTUAL_OFFSET;
	    res->end += IO_PORT_VIRTUAL_OFFSET;
	    printk("  IO  : %lx - %lx\n",res->start, res->end);
	}
    }
#else
    int ipci_ctrlh;
    int ioport;
    int data;
    int ID_54_58_valid=0, ID_64_70_valid=0, ID_88_valid=0;
    int dma_mode=0, pio_mode=2;
    int i;
    unsigned short pci_cmd;

    /* IDE */
    if (dev->vendor == 0x1033 && dev->class == 0x010185){
	/* see ide-std.c for address */

	/* enble bus error */
	ipci_ctrlh = ddb_in32(IPCI_CTRLH);
	ddb_out32(IPCI_CTRLH, ipci_ctrlh | 0x00020000);

	printk(" Vr5701 IDE .. ");

	pci_write_config_byte(dev,PCI_LATENCY_TIMER, 0x40);

	/* enable IO */
	pci_read_config_word(dev, PCI_COMMAND, &pci_cmd);
	pci_cmd |= PCI_COMMAND_IO + PCI_COMMAND_MASTER;
	pci_write_config_word(dev, PCI_COMMAND, pci_cmd);

	pci_read_config_dword(dev, PCI_BASE_ADDRESS_0, &ioport);
	ioport &= ~0x01;

//	pci_write_config_word(dev, 0x48, 0x00CC); /* PIOCTRL: enable prefetch */

	/* hard reset */
	pci_read_config_dword(dev, PCI_BASE_ADDRESS_5, &bases);
	bases = bases & ~0x01;

	outb(0x04, bases);
	set_current_state(TASK_UNINTERRUPTIBLE);
	udelay(50);

	outb(0x06, bases);

	ide_wait(ioport);
	printk(" sector count=%x, number=%x, cylinder low=%x, hi=%x, dev/head=%x\n",
	       inb(SECTOR_CNT_REG(ioport)), inb(SECTOR_NUM_REG(ioport)),
	       inb(CYLINDER_LOW_REG(ioport)), inb(CYLINDER_HIGH_REG(ioport)),
	       inb(DRIVE_HEAD_REG(ioport)) );

	if (inb(SECTOR_CNT_REG(ioport)) == 0x01)
	    printk(" hard reset O.K.\n");
	else
	    printk(" hard reset error!\n");
		    
//	outb(0xe0, DRIVE_HEAD_REG(ioport));   /* select drive */

//	ide_wait(ioport);

#if 1
	/* soft reset */
	data = inb(CTRL_REG(ioport));
	outb(0x00, CTRL_REG(ioport));
	udelay(10);  

	outb(data | 0x04, CTRL_REG(ioport));
	udelay(10);  /* SRST to one at least 5us */

	outb(data & ~0x04, CTRL_REG(ioport));
	schedule_timeout(50);  /* clear for 2ms */

	if (ide_wait(ioport))
	    printk(" soft reset N.G.\n");
	else
	    printk(" soft reset O.K.\n");
	    
#endif
//	outb(data | 0x02, CTRL_REG(ioport));  /* nIEN */

	ide_wait(ioport);

	/* identify device */
	outb(0xec, CMD_REG(ioport));
 
	ide_wait(ioport);
	if (inb(STATUS_REG(ioport))&(0x01)){ /* err */
	    printk("identy returns %x",inb(STATUS_REG(ioport)));
	    goto PACKET_DEV;
	}

	for (i=0; i<256; i++){
	    data=inw(DATA_REG(ioport));
	    if (i==0){
		printk(" General Config %x\n",data);
	    }
	    if (i==53){
		ID_54_58_valid = data & 1<<0;
		ID_64_70_valid = data & 1<<1;
		ID_88_valid = data & 1<<2;
	    }
	    if (i==63 && data&0x7){
		printk(" Multiword DMA support flag %x\n",data&0x07);
	    }
	    if (i==64 && ID_64_70_valid && data&0x3){
		printk(" Advanced PIO mode support flag %x \n",data&0x03);
		pio_mode = data&0x02 ? 4: 3;
	    }
	    if (i==80){
		printk(" ATAPI support flag %x\n",data);
	    }
	    if (i==88 && ID_88_valid){
		dma_mode=data & 0x1f;
		printk(" UDMA mode support flag %x\n",data);
	    }
	}
	    

#if 0
//	data = inb(CTRL_REG(ioport));
//	outb(data & ~0x02, CTRL_REG(ioport));  /* nIEN = 0 */
//	outb(data | 0x02, CTRL_REG(ioport));  /* nIEN = 1 */
	/* PIO mode */
	if (pio_mode){
	    if (ide_wait(ioport))
		goto INIT_ERROR;

	    outb(0x08 | pio_mode, SECTOR_CNT_REG(ioport)); /* transfer mode */ 
	    ide_wait(ioport);
	    outb(0x03, FEATURE_REG(ioport));  /* select set transfer mode */
	    ide_wait(ioport);
	    outb(0xef, CMD_REG(ioport));		/* set feature */
	    udelay(2000);
	    ide_wait(ioport);
	    printk("Set PIO mode to %d\n",pio_mode);

	    pci_read_config_dword(dev, 0x40, &data);
	    data &= 0x000000ff;
	    if (pio_mode == 1){
		data |= 0x00004700;
	    }
	    if (pio_mode == 2){
		data |= 0x00003300;
	    }
	    if (pio_mode == 3){
		data |= 0x00002200;
	    }
	    if (pio_mode == 4){
		data |= 0x00002000;
	    }
	    pci_write_config_dword(dev, 0x40, data); /* PIOTIM */
	}
#endif

#if 1
	/* set feature */
	/* UDMA mode */
	if (dma_mode){
	    if (ide_wait(ioport))
		goto INIT_ERROR;

	    data=0;
	    for(i=0; i<5; i++){
		if (!(dma_mode & 0x01))
		    break;
		data++;
		dma_mode>>=1;
	    }
	    data--;
#if 1
	    outb(0x40 | (data&0x7), SECTOR_CNT_REG(ioport)); /* transfer mode */ 
	    udelay(10);
	    outb(0x03, FEATURE_REG(ioport));  /* select set transfer mode */
	    udelay(10);
	    outb(0xef, CMD_REG(ioport));		/* set feature */
#endif
		    
	    pci_read_config_dword(dev, PCI_BASE_ADDRESS_4, &bases); /* BAR_BMAS */
	    bases = bases & ~0x01;

	    outb(0x20, bases+0x02); /* IDE_STAT: enable DMA */
	    printk(" ioport %x, reg=%x\n",bases+0x02, inb(bases+0x02));
	    pci_write_config_byte(dev, 0x4b, 0x01); /* UDMACTRL: enable master */
	    pci_write_config_word(dev, 0x4c, dma_mode<<4); /* UDMATIM: set mode */
	    printk("Set UDMA mode to %d\n",data);
	}
#endif

      PACKET_DEV:

      INIT_ERROR:

#ifdef DEBUG
	printk("cmd base = %x", ioport);
	printk(" status=%x\n", inb(STATUS_REG(ioport)));

	pci_read_config_dword(dev,PCI_BASE_ADDRESS_1, &bases) ;   /* CTRL */
	bases = bases & ~0x01;
	printk("ctrl base = %x", bases);
	printk(" altstatus=%x\n",inb(bases+0x06));
#endif
	/* disable buserr */
	ddb_out32(IPCI_CTRLH, ipci_ctrlh );
    }
#endif
}
