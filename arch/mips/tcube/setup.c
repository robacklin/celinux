/***********************************************************************
 *
 * Based on:
 *   arch/mips/yboard/setup.c
 *       Setup file for NEC YBOARD.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/config.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/kdev_t.h>
#include <linux/types.h>
#include <linux/console.h>
#include <linux/sched.h>
#include <linux/pci.h>
#include <linux/ide.h>
#include <linux/fs.h>		/* for ROOT_DEV */
#include <linux/ioport.h>
#include <linux/param.h>	/* for HZ */

#include <asm/bootinfo.h>
#include <asm/addrspace.h>
#include <asm/time.h>
#include <asm/bcache.h>
#include <asm/irq.h>
#include <asm/reboot.h>
#include <asm/gdb-stub.h>
#include <asm/debug.h>
#include <asm/tcube.h>

int prom_get_macadr(const char *name,u_char *mac_addr);
int prom_get_bool(const char *name, int *pVal);
int prom_get_int(const char *name, int *pVal);

void pci_setup(void);		// KOSE.ADD

// #define	USE_CPU_COUNTER_TIMER	/* whether we use cpu counter */

#define	CPU_COUNTER_FREQUENCY		150000000

static void tcube_machine_restart(char *command)
{
        static void (*back_to_prom) (void) = (void (*)(void)) 0xbfc00000;
	back_to_prom();
}

static void tcube_machine_halt(void)
{
	printk("T-Cube halted.\n");
	while (1);
}

static void tcube_machine_power_off(void)
{
	printk("T-Cube halted. Please turn off the power.\n");
	while (1);
}


static void __init tcube_time_init(void)
{
	mips_counter_frequency = CPU_COUNTER_FREQUENCY;
//	rtc_ricoh_rx5c348_init();
}

extern int setup_irq(unsigned int irq, struct irqaction *irqaction);
static void __init tcube_timer_setup(struct irqaction *irq)
{
	unsigned int count;

	/* printk("tcube_timer_setup:\n"); */
	/* we are using the cpu counter for timer interrupts */
	setup_irq(7, irq);

	/* to generate the first timer interrupt */
	count = read_c0_count();
	write_c0_compare(count + 10000);
}

extern void tcube_irq_setup(void);

#if defined(CONFIG_BLK_DEV_INITRD)
extern unsigned long __rd_start, __rd_end, initrd_start, initrd_end;
#endif 

static void chk_init_5701_reg(unsigned long addr, unsigned long data)
{
	unsigned long a = ddb_in32(addr);

	if( a != data ){
		printk("Unexpected 5701 reg : addr = %08lX, expected = %08lX, read = %08lX\n",
			addr + VR5701_IO_BASE, data, a );
	}
}

static void __init tcube_board_init(void)
{
/*
	0000_0000 ¡Á 03FF_FFFF  64M  SDRAM
	1000_0000 ¡Á 17FF_FFFF 128M  PCIW0(MEM)
	1800_0000 ¡Á 187F_FFFF   8M  PCIW1(IO)
	1880_0000 ¡Á 18FF_FFFF   8M  IOPCIW0(IO)
	1900_0000 ¡Á 197F_FFFF   8M  IOPCIW1(MEM)
	1980_0000 ¡Á 19FF_FFFF   8M  PCI-CFG
	1A00_0000 ¡Á 1A7F_FFFF   8M  IOPCI-CFG
	1F00_0000 ¡Á 1FFF_FFFF  16M  FROM
*/
	chk_init_5701_reg(0,0x1e00008f);
	chk_init_5701_reg(PADR_SDRAM01,0x000000aa);     // 64M
	//	chk_init_5701_reg(PADR_SDRAM01,0x000000a9);     // 128M
	chk_init_5701_reg(PADR_LOCALCS0,0x1f00004c);	// 16M
	chk_init_5701_reg(LOCAL_CST0,0x00088622);
	chk_init_5701_reg(LOCAL_CFG,0x000f0000);

	/* ------------ reset PCI bus and BARs ----------------- */
	ddb_pci_reset_bus();

	/* setup PCI windows - window0 for MEM/config, window1 for IO */
	ddb_set_pdar(PADR_PCIW0,   0x10000000, 0x08000000 , 32, 0, 1);
	ddb_set_pdar(PADR_PCIW1,   0x18000000, 0x00800000 , 32, 0, 1);
	ddb_set_pdar(PADR_IOPCIW0, 0x18800000, 0x00800000 , 32, 0, 1);
	ddb_set_pdar(PADR_IOPCIW1, 0x19000000, 0x00800000 , 32, 0, 1);

	/* 
	 * We use pci master register 0  for memory space / config space
	 * And we use register 1 for IO space.
	 * Note that for memory space, we bump up the pci base address
	 * so that we have 1:1 mapping between PCI memory and cpu physical.
	 * For PCI IO space, it starts from 0 in PCI IO space but with
	 * DDB_xx_IO_BASE in CPU physical address space.
	 */
	ddb_set_pmr(EPCI_INIT0, DDB_PCICMD_MEM, 0x10000000, DDB_PCI_ACCESS_32);
	ddb_set_pmr(EPCI_INIT1, DDB_PCICMD_IO,  0x00000000, DDB_PCI_ACCESS_32);
	ddb_set_pmr(IPCI_INIT0, DDB_PCICMD_IO,  0x00800000, DDB_PCI_ACCESS_32);
	ddb_set_pmr(IPCI_INIT1, DDB_PCICMD_MEM, 0x19000000, DDB_PCI_ACCESS_32);

	/* PCI cross window should be set properly */
	ddb_set_bar(IPCI_BAR_IPCIW0, 0x10000000, 0);
	ddb_set_bar(IPCI_BAR_IPCIW1, 0x18000000, 0);
	ddb_set_bar(PCI_BAR_IPCIW0, 0x18800000, 0);
	ddb_set_bar(PCI_BAR_IPCIW1, 0x19000000, 0);

	/* Ext. PCI memory space */
	ddb_out32(PCI_BAR_MEM01, 0x00000008);
	ddb_out32(PCI_BAR_MEM23, 0x00000008);

	ddb_out8 (PCI_MLTIM,     0x40);
//	ddb_out8 (PCI_MLTIM,     0x4);

	ddb_out32(PCI_BAR_LCS0, 0xffffffff);
	ddb_out32(PCI_BAR_LCS1, 0xffffffff);
	ddb_out32(PCI_BAR_LCS2, 0xffffffff);
	ddb_out32(PCI_BAR_LCS3, 0xffffffff);

	/* Int. PCI memory space */
	//	ddb_out32(IPCI_BAR_MEM01, 0x00000008);
	ddb_out8 (IPCI_MLTIM,    0x40);

	ddb_out32(IPCI_BAR_LCS0, 0xffffffff);
	ddb_out32(IPCI_BAR_LCS1, 0xffffffff);
	ddb_out32(IPCI_BAR_LCS2, 0xffffffff);
	ddb_out32(IPCI_BAR_LCS3, 0xffffffff);

	/* setup GPIO */
	ddb_out32(GIU_DIR0, 0xf7ebffdf);
	ddb_out32(GIU_DIR1, 0x000007fa);
	ddb_out32(GIU_FUNCSEL0, 0xf1c1ffff);
	ddb_out32(GIU_FUNCSEL1, 0x000007f0);
	chk_init_5701_reg(GIU_DIR0, 0xf7ebffdf);
	chk_init_5701_reg(GIU_DIR1, 0x000007fa);
	chk_init_5701_reg(GIU_FUNCSEL0, 0xf1c1ffff);
	chk_init_5701_reg(GIU_FUNCSEL1, 0x000007f0);

//	ddb_out32(DDB_BARC0, 0xffffffff);
//	ddb_out32(DDB_BARM230, 0xffffffff);
//	ddb_out32(DDB_BAR00, 0xffffffff);
//	ddb_out32(DDB_BAR10, 0xffffffff);
//	ddb_out32(DDB_BAR20, 0xffffffff);
//	ddb_out32(DDB_BAR30, 0xffffffff);
//	ddb_out32(DDB_BAR40, 0xffffffff);
//	ddb_out32(DDB_BAR50, 0xffffffff);
//	ddb_out32(DDB_BARB0, 0xffffffff);

//	ddb_out32(DDB_BARC1, 0xffffffff);
//	ddb_out32(DDB_BARM231, 0xffffffff);
//	ddb_out32(DDB_BAR01, 0xffffffff);
//	ddb_out32(DDB_BAR11, 0xffffffff);
//	ddb_out32(DDB_BAR21, 0xffffffff);
//	ddb_out32(DDB_BAR31, 0xffffffff);
//	ddb_out32(DDB_BAR41, 0xffffffff);
//	ddb_out32(DDB_BAR51, 0xffffffff);
//	ddb_out32(DDB_BARB1, 0xffffffff);


#if 0
#if defined(CONFIG_VGA_CONSOLE)
        conswitchp = &vga_con;

	screen_info = (struct screen_info) {
		0, 25,			/* orig-x, orig-y */
		0,			/* unused */
		0,			/* orig-video-page */
		0,			/* orig-video-mode */
		80,			/* orig-video-cols */
		0,0,0,			/* ega_ax, ega_bx, ega_cx */
		25,			/* orig-video-lines */
		1,			/* orig-video-isVGA */
		16			/* orig-video-points */
	};
#elif defined(CONFIG_DUMMY_CONSOLE)
        conswitchp = &dummy_con;
#endif
#endif

}

void __init shima_tcube_setup(void)
{
	extern int panic_timeout;

	/* printk("tcube_setup %s:%d\n",__FILE__,__LINE__); */
	irq_setup = tcube_irq_setup;
	set_io_port_base(0xB8000000);

	board_time_init    = tcube_time_init;
	board_timer_setup  = tcube_timer_setup;

	_machine_restart   = tcube_machine_restart;
	_machine_halt      = tcube_machine_halt;
	_machine_power_off = tcube_machine_power_off;

	/* setup resource limits */
	ioport_resource.end = 0x01000000;
	iomem_resource.end  = 0xffffffff;
	
	/* Reboot on panic */
	panic_timeout = 180;

#ifdef CONFIG_FB
	conswitchp = &dummy_con;
#endif

	tcube_board_init();

#if defined(CONFIG_BLK_DEV_INITRD)
	ROOT_DEV = MKDEV(RAMDISK_MAJOR, 0);
	initrd_start = (unsigned long)&__rd_start;
	initrd_end =   (unsigned long)&__rd_end;
#endif
}

void __init bus_error_init(void){
  /* do nothing */
}

/*
 * Local Variables:
 *   c-basic-offset: 8
 * End:
 */
