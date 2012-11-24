/* 
 *
 * linux/arch/sh/kernel/setup_kzp01.c
 *
 * Copyright (C) 2003 Mitsubishi Electric Corporation
 * Copyright (C) 2001 Nobuhiro Sakawa
 *
 * from
 *
 * linux/arch/sh/kernel/setup_se.c
 *
 * Copyright (C) 2000  Kazumoto Kojima
 *
 * Hitachi SolutionEngine Support.
 *
 */

#include <linux/config.h>
#include <linux/init.h>
#include <linux/irq.h>

#include <linux/hdreg.h>
#include <linux/ide.h>
#include <linux/delay.h>
#include <linux/pci.h>
#include <asm/io.h>
#include <asm/pci-sh7751.h>
#include <asm/alim1543.h>
#include <asm/rtc.h>

#define PCIMCR_MRSET_OFF	0xBFFFFFFF
#define PCIMCR_RFSH_OFF		0xFFFFFFFB


/*
 * Only long word accesses of the PCIC's internal local registers and the
 * configuration registers from the CPU is supported.
 */
#define PCIC_WRITE(x,v) (*(volatile unsigned long*)PCI_REG(x)=(v))
#define PCIC_READ(x) (*(volatile unsigned long*)PCI_REG(x))

/*
 * Configure the Super I/O chip PCI
 */
/* pci config area write 32bit */
static void 
pci_cfgwr(unsigned char bus_no,		/* PCI BUS No. */
          unsigned char dev_no,		/* Device No.  */
          unsigned char func_no,	/* Function No.*/
          unsigned char adr,		/* config adr  */
          unsigned long data    )	/* write data  */
{
	unsigned long x;
	dev_no  &= 0x1f;
	func_no &= 0x07;
	adr     &= 0xfc;

	x = 0x80000000  | ((unsigned long)bus_no<<16)
	 		| ((unsigned long)dev_no<<11) 
			| ((unsigned long)func_no<<8) | adr;
	*(unsigned long *)0xfe2001c0 = x;
	*(unsigned long *)0xfe200220 = data;
}
/* pci config area read 32bit */
static unsigned long			/* read data   */
pci_cfgrd(unsigned char bus_no,	/* PCI BUS No. */
	   unsigned char dev_no,	/* Device No.  */
	   unsigned char func_no,	/* Function No.*/
	   unsigned char adr)		/* config adr  */
{
	unsigned long x;
	dev_no  &= 0x1f;
	func_no &= 0x07;
	adr     &= 0xfc;

	x = 0x80000000  | ((unsigned long)bus_no<<16)
	 		| ((unsigned long)dev_no<<11) 
			| ((unsigned long)func_no<<8) | adr;
	*(unsigned long *)0xfe2001c0 = x;
	return *(unsigned long *)0xfe200220;
}

static void __init pci_init_pcic()
{
   unsigned long bcr1, wcr1, wcr2, wcr3, mcr;
   unsigned short bcr2;

   /*
    * Initialize the slave bus controller on the pcic.  The values used
    * here should not be hardcoded, but they should be taken from the bsc
    * on the processor, to make this function as generic as possible.
    * (i.e. Another sbc may usr different SDRAM timing settings -- in order
    * for the pcic to work, its settings need to be exactly the same.)
    */
   bcr1 = (*(volatile unsigned long*)(SH7751_BCR1));
   bcr2 = (*(volatile unsigned short*)(SH7751_BCR2));
   wcr1 = (*(volatile unsigned long*)(SH7751_WCR1));
   wcr2 = (*(volatile unsigned long*)(SH7751_WCR2));
   wcr3 = (*(volatile unsigned long*)(SH7751_WCR3));
   mcr = (*(volatile unsigned long*)(SH7751_MCR));

	bcr1 |= 0x00080000;  /* Enable Bit 19, BREQEN */
   (*(volatile unsigned long*)(SH7751_BCR1)) = bcr1;   

	bcr1 |= 0x40080000;  /* Enable Bit 19 BREQEN, set PCIC to slave */
	PCIC_WRITE( SH7751_PCIBCR1, bcr1 );		/* PCIC BCR1 */
	PCIC_WRITE( SH7751_PCIBCR2, bcr2 );		/* PCIC BCR2 */
	PCIC_WRITE( SH7751_PCIWCR1, wcr1 );		/* PCIC WCR1 */
	PCIC_WRITE( SH7751_PCIWCR2, wcr2 );		/* PCIC WCR2 */
	PCIC_WRITE( SH7751_PCIWCR3, wcr3 );		/* PCIC WCR3 */
	mcr &= PCIMCR_MRSET_OFF&PCIMCR_RFSH_OFF;
	PCIC_WRITE( SH7751_PCIMCR, mcr );		/* PCIC MCR */

   /* Enable all interrupts, so we know what to fix */
	PCIC_WRITE( SH7751_PCIINTM, 0x00000000 );
	PCIC_WRITE( SH7751_PCIAINTM, 0x0000380f );

   /* Set up standard PCI config registers */
	PCIC_WRITE( SH7751_PCICONF1, 0x02900007 );	/* Bus Master, Mem & I/O access */
	PCIC_WRITE( SH7751_PCICONF2, 0x06000000 );	/* PCI Class code & Revision ID */
	PCIC_WRITE( SH7751_PCICONF4, 0xFFF00001 );	/* PCI I/O address (local regs) */
	//PCIC_WRITE( SH7751_PCICONF11, 0x350E1054 );	/* PCI Subsystem ID & Vendor ID */
	
	/* SDRAM(Area3) */
	PCIC_WRITE( SH7751_PCICONF5, 0x0C000000 );	/* PCI MEM address (local RAM 64M) */
	PCIC_WRITE( SH7751_PCILSR0, 0x03F00000 );	/* MEM (full 64M exposed) */
	PCIC_WRITE( SH7751_PCILAR0, 0x0C000000 );	/* MEM (direct map from PCI) */

#if 1
	/* SRAM(Area4) */
	PCIC_WRITE( SH7751_PCICONF6, 0x10000000 );	/* PCI MEM address (SRAM) */
 	PCIC_WRITE( SH7751_PCILSR1, 0x00F00000 );	/* MEM (SRAM) */
 	PCIC_WRITE( SH7751_PCILAR1, 0x10000000 );	/* MEM (SRAM) */
#endif

   /* Now turn it on... */
	PCIC_WRITE( SH7751_PCICR, 0xA5000041 );

   /*
    * Set PCIMBR and PCIIOBR here, assuming a single window
    * (16M MEM, 256K IO) is enough.  If a larger space is
    * needed, the readx/writex and inx/outx functions will
    * have to do more (e.g. setting registers for each call).
    */

   /*
    * Set the MBR so PCI address is one-to-one with window,
    * meaning all calls go straight through... use ifdef to
    * catch erroneous assumption.
 */
	PCIC_WRITE( SH7751_PCIMBR, 0 );
	//PCIC_WRITE( SH7751_PCIMBR, PCIBIOS_MIN_MEM );

   /* Set IOBR for window containing area specified in pci.h */
   PCIC_WRITE(SH7751_PCIIOBR, 0 );
   //PCIC_WRITE(SH7751_PCIIOBR, (PCIBIOS_MIN_IO & SH7751_PCIIOBR_MASK));
}

static void __init pci_init_device()
{
	int			val;

#if 1
	/* PCI Bridge */
	pci_cfgwr(0,0,0,PCI_PRIMARY_BUS,0x40010100);		/* bus number reg Pri=0 , Sec=1 lat.timer=0x40 */
	/*  I/O Address 0x00000000 - 0x7fffffff  is SecBus */
	pci_cfgwr(0,0,0,PCI_IO_BASE,0x0000f101);		/* io base limit addr reg */
	pci_cfgwr(0,0,0,PCI_IO_BASE_UPPER16,0x7fff0000);		/* io upper base limit addr reg */
	pci_cfgwr(0,0,0,PCI_MEMORY_BASE,0xBFF0B000);	/* memory limit:base addr reg */
	pci_cfgwr(0,0,0,PCI_PREF_MEMORY_BASE,0xc001c001);	/* pre memory limit:base addr reg */
	pci_cfgwr(0,0,0,PCI_INTERRUPT_LINE,0x00080000);		/* VGA Enable */
	pci_cfgwr(0,0,0,PCI_COMMAND,0x02900007);		/* primary command reg */
#endif   

#if 1
	/* Super I/O */
	pci_cfgwr(1,2,0,PCI_COMMAND,0x0280000f);	/* SIO CMD_reg init */
	pci_cfgwr(1,2,0,0x50,0x20000000);	/* RTC enable */
	pci_cfgwr(1,2,0,0x40,0x0040c000);	/* Key & mouse PS2 , DMA_32bit */
	//pci_cfgwr(1,2,0,0x58,0x01000000);	/* GPIOWR use */
	pci_cfgwr(1,2,0,0x58,0x01000044);	/* GPIOWR use */

	/* Power Management Unit */
	/* pci_bus_no=1 pci_dev_no=0x0c */
#define ACPI_IO_BASE 0xa100
#define SMB_IO_BASE  0xa180 
	pci_cfgwr(1,0x0c,0,PCI_BASE_ADDRESS_0, ACPI_IO_BASE|1 );  /* IO_Base */
	pci_cfgwr(1,0x0c,0,PCI_BASE_ADDRESS_1, SMB_IO_BASE|1 );  /* IO_Base */
	pci_cfgwr(1,0x0c,0,PCI_COMMAND,0x02800001);  /* DEVSEL=mid&IO*/
	pci_cfgwr(1,0x0c,0,0xb4,0x200);  		    /* SPLED 1Hz*/
	outw( 0x100, ACPI_IO_BASE );				    /* clear */
#endif // if 0

#if 1
	/* IDE (SuperI/O) */
	pci_cfgwr(1,2,0,0x44,0x19);			/* ISA bridge /INT_enable , IDE_IRQ11 */
	//pci_cfgwr(1,2,0,0x44,0x0);		/* ISA bridge /INT_Disable , IDE_Disable */
	pci_cfgwr(1,0xB,0,PCI_COMMAND,0x02800007);	/* IDE_ctrl com register */
	//pci_cfgwr(1,0xB,0,0x8,0x0101fac1);	/* IDE_ctrl prim second IDE enable */
	//pci_cfgwr(1,0xB,0,0x58,0x00484802);
	/* IDE */
#endif

#if 1
	/* CF (TrueIDE) */
#define CF_CTRL		0x300
#define CF_BASE		0x320
#define CF_CD2		0x02		/* CD2 */
#define CF_CD1		0x01		/* CD1 */
#define CF_CTRL_MD	0x4			/* IO MODE(1) TureIDE(0) */
#define CF_CTRL_RST	0x2			/* RESET */
#define CF_CTRL_PW	0x1			/* CF POWER ON */
	val = inb( CF_CTRL );
	if( val&(CF_CD1|CF_CD2)==(CF_CD1|CF_CD2) ){
		int			i;
		/* TrueIDE */
		outb( CF_CTRL_RST|CF_CTRL_PW, CF_CTRL );
		/* Power ON */
		outb( CF_CTRL_PW, CF_CTRL );
		for( i=0; i<5000; i++ ){
			mdelay( 1 );
			outb( 0x55, CF_BASE+2 );
			if( inb(CF_BASE+2)!=0x55 )
				continue;
			outb( 0xAA, CF_BASE+2 );
			if( inb(CF_BASE+2)!=0xAA )
				continue;
			break;
		}
		mdelay( 10 );
		/* Reset OFF */
		outb( CF_CTRL_PW, CF_CTRL );
	}
#endif

#if 1
	/* VGA Init */
	{
		unsigned long id;

		id =  pci_cfgrd(1, 0xa, 0, 0);
		printk("VGA ID is %x\n", id);

		pci_cfgwr(1, 0xa, 0, 0x10, 0xbf000000); /* Upper MEM Base 0xbf000000 */
		pci_cfgwr(1, 0xa, 0, 0x4, 0x02800083);

		outb(0x3c2, 3);
	}
#endif
}

/* PCI INT->IRQ transfer table */
const int irq_list[] =
	{ 3, 10, 7, 9 };

static void __init pci_init_irq_routing_table()
{
	unsigned long id;
	int         irq;
	unsigned long		val, i, flag[2];
	const int irq_route[] = {
		0, 8, 0, 2, 4, 5, 7, 6, 0, 1, 3, 9, 11, 0, 13, 15
	};
#if defined(CONFIG_PCNET32) /*** Lan chip AM79C793 ***/
	id = pci_cfgrd(1,9,0,PCI_VENDOR_ID);
	outb(0x03,0x3c2);                           /* ram & i/o disable */
#endif /* } */

	/* PCI Interrupt to ISA IRQ Routing Table (PIRT) */
	val = pci_cfgrd( 1, 2, 0, 0x48 ) & 0xffff0000;
	pci_cfgwr( 1, 2, 0, 0x48, val|
		(irq_route[irq_list[0]]+irq_route[irq_list[1]]*0x10+
		irq_route[irq_list[2]]*0x100+irq_route[irq_list[3]]*0x1000) );

	/* Interrupt Unit Edge/Level Control Register (ELCR) */
	flag[0] = flag[1] = 0;
	for( i=0; i<4; i++ ){
		int irq = irq_list[i];
		flag[irq/8] |= 1<<(irq%8);
	}
	val = inb( 0x4d0 );
	outb( val|flag[0], 0x4d0 );
	val = inb( 0x4d1 );
	outb( val|flag[1], 0x4d1 );

	//printk("ELCR 1=%x 2=%x\n",inb(0x4d0), inb(0x4d1));
}

#ifndef BCD_TO_BIN
#define BCD_TO_BIN(val) ((val)=((val)&15) + ((val)>>4)*10)
#endif

#ifndef BIN_TO_BCD
#define BIN_TO_BCD(val) ((val)=(((val)/10)<<4) + (val)%10)
#endif

static void __init init_rtc()
{
	unsigned int		sec, min, hr, day, mon, year, year100;

	ctrl_outb( RCR2_RESET, RCR2 );	/* Reset pre-scaler & stop RTC */

	outb( 0, 0x70 );
	sec = inb( 0x71 );
	BIN_TO_BCD( sec );
	ctrl_outb( sec, RSECCNT );

	outb( 2, 0x70 );
	min = inb( 0x71 );
	BIN_TO_BCD( min );
	ctrl_outb( min, RMINCNT );

	outb( 4, 0x70 );
	hr = inb( 0x71 );
	BIN_TO_BCD( hr );
	ctrl_outb( hr, RHRCNT );

	outb( 7, 0x70 );
	day = inb( 0x71 );
	BIN_TO_BCD( day );
	ctrl_outb( day, RDAYCNT );

	outb( 8, 0x70 );
	mon = inb( 0x71 );
	BIN_TO_BCD( mon );
	ctrl_outb( mon, RMONCNT );

	outb( 9, 0x70 );
	year = inb( 0x71 );
	if( year<70 )
		year += 2000;
	else
		year += 1900;
	year100 = year/100;
	year %= 100;
	BIN_TO_BCD( year100 );
	BIN_TO_BCD( year );
	ctrl_outw( year100*0x100+year, RYRCNT );

	ctrl_outb( RCR2_RTCEN|RCR2_START, RCR2 );	/* Start RTC */
}

/*
 * Initialize the board
 */
void __init setup_kzp(void)
{
	pci_init_pcic();
	pci_init_device();
	pci_init_irq_routing_table();
	init_rtc();
}



/*
 * Initialize IRQ setting
 */

static unsigned char m_irq_mask = 0xfb;
static unsigned char s_irq_mask = 0xff;

static void disable_kzp_irq(unsigned int irq)
{
	unsigned long flags;

	save_and_cli(flags);
	if( irq < 8) {
		m_irq_mask |= (1 << irq);
		outb(m_irq_mask,I8259_M_MR);
	} else {
		s_irq_mask |= (1 << (irq - 8));
		outb(s_irq_mask,I8259_S_MR);
	}
	restore_flags(flags);

}

static void enable_kzp_irq(unsigned int irq)
{
	unsigned long flags;

	save_and_cli(flags);

	if( irq < 8) {
		m_irq_mask &= ~(1 << irq);
		outb(m_irq_mask,I8259_M_MR);
	} else {
		s_irq_mask &= ~(1 << (irq - 8));
		outb(s_irq_mask,I8259_S_MR);
	}
	restore_flags(flags);
}

static inline int kzp_irq_real(unsigned int irq)
{
	int value;
	int irqmask;

	if ( irq < 8) {
		irqmask = 1<<irq;
		outb(0x0b,I8259_M_CR);		/* ISR register */
		value = inb(I8259_M_CR) & irqmask;
		outb(0x0a,I8259_M_CR);		/* back ro the IPR reg */
		return value;
	}
	irqmask = 1<<(irq - 8);
	outb(0x0b,I8259_S_CR);		/* ISR register */
	value = inb(I8259_S_CR) & irqmask;
	outb(0x0a,I8259_S_CR);		/* back ro the IPR reg */
	return value;
}

static void mask_and_ack_kzp(unsigned int irq)
{
	unsigned long flags;
/*
printk("ack irq=%lx\n",irq);
*/
	save_and_cli(flags);

	if(irq < 8) {
		if(m_irq_mask & (1<<irq)){
		  if(!kzp_irq_real(irq)){
//		    irq_err_count++;
		    printk("spurious 8259A interrupt: IRQ %x\n",irq);
		   }
		} else {
			m_irq_mask |= (1<<irq);
		}
		inb(I8259_M_MR);		/* DUMMY */
		outb(m_irq_mask,I8259_M_MR);	/* disable */
		outb(0x60+irq,I8259_M_CR);	/* EOI */
		
	} else {
		if(s_irq_mask & (1<<(irq - 8))){
		  if(!kzp_irq_real(irq)){
//		    irq_err_count++;
		    printk("spurious 8259A interrupt: IRQ %x\n",irq);
		  }
		} else {
			s_irq_mask |= (1<<(irq - 8));
		}
		inb(I8259_S_MR);		/* DUMMY */
		outb(s_irq_mask,I8259_S_MR);	/* disable */
		outb(0x60+(irq-8),I8259_S_CR); 	/* EOI */
		outb(0x60+2,I8259_M_CR);
	}
	restore_flags(flags);
}

static void end_kzp_irq(unsigned int irq)
{
/*
printk("end irq=%lx\n",irq);
*/
	if (!(irq_desc[irq].status & (IRQ_DISABLED|IRQ_INPROGRESS)))
		enable_kzp_irq(irq);
}

static unsigned int startup_kzp_irq(unsigned int irq)
{
	enable_kzp_irq(irq);
	return 0;
}

static void shutdown_kzp_irq(unsigned int irq)
{
	disable_kzp_irq(irq);
}

static struct hw_interrupt_type kzp_irq_type = {
	"KZP01-IRQ",
	startup_kzp_irq,
	shutdown_kzp_irq,
	enable_kzp_irq,
	disable_kzp_irq,
	mask_and_ack_kzp,
	end_kzp_irq
};

static void make_kzp_irq(unsigned int irq)
{
	irq_desc[irq].handler = &kzp_irq_type;
	irq_desc[irq].status  = IRQ_DISABLED;
	irq_desc[irq].action  = 0;
	irq_desc[irq].depth   = 1;
	disable_kzp_irq(irq);
}

void __init init_kzp_IRQ(void)
{
	int i;
	/*
	 * Super I/O (Just mimic PC):
	 *  1: keyboard
	 *  3: serial 0
	 *  4: serial 1
	 *  5: printer
	 *  6: floppy
	 *  7: lan
	 *  8: rtc
	 *  9: serial 3
	 * 10: @@@@KMCKMC@@@@ usb irq 
	 * 11: @@@@KMC_USE_CF@@@@@ cf irq (real & dummy)
	 * 12: mouse
	 * 14: ide0
	 * 15: ide1
	 */

	*(unsigned short *)(INTC_ICR) =		/* IRL interrrupt on */
		*(unsigned short *)(INTC_ICR) | 0x80; 
	*(unsigned short *)(INTC_IPRD) =	/* IRL0 pri 15 <-- 8259 */
		*(unsigned short *)(INTC_IPRD) | 0xf000;
	*(unsigned long *)(INTC_INTMSK00) = 0x3fff; 

	outb(0x11, I8259_M_CR); 	/* mater icw1 edge trigger  */
	outb(0x11, I8259_S_CR);		/* slave icw1 edge trigger  */
	outb(0x20, I8259_M_MR); 	/* m icw2 base vec 0x08	    */
	outb(0x28, I8259_S_MR);		/* s icw2 base vec 0x70	    */
	outb(0x04, I8259_M_MR);		/* m icw3 slave irq2	    */
	outb(0x02, I8259_S_MR);		/* s icw3 slave id	    */
	outb(0x01, I8259_M_MR);		/* m icw4 non buf normal eoi*/
	outb(0x01, I8259_S_MR);		/* s icw4 non buf normal eo1*/
	outb(0xfb, I8259_M_MR);		/* disable irq0--irq7  */
	outb(0xff, I8259_S_MR);		/* disable irq8--irq15 */

	for ( i=1; i < 16; i++) {
		if(i != 2) 
			make_kzp_irq(i);
	}

}

int kzp01_irq_demux(int irq)
{
	if(irq == 2){
	    unsigned int poll;

	    outb(0x0c,I8259_M_CR);
	    poll = inb(I8259_M_CR);
	    if(poll & 0x80) irq = poll & 0x07;
	    if(irq == 2){
		outb(0x0c,I8259_S_CR);
		poll = inb(I8259_S_CR);
		irq = (poll & 0x07) + 8;
	    }
	}
	if(irq==16){
	    static kmckmc;
	    kmckmc++;
	}

	return irq;
}

