/*
 * S3C2410 C-level Power-OFF/Wakeup Management Routines
 *
 * Initial SA1100 code:
 * Copyright (c) 2001 Cliff Brake <cbrake@accelent.com>
 *
 * Adapted for S3C2410 by Seongil Na:
 * Copyright (c) 2003 Samsung Electronics Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 *
 * History:
 * 2003-10-20:	Seongil Na <seongil@samsung.com>
 * 	- System suspend
 * 	- System resume
 * 	
 * 2003-11-05:	Seongil Na <seongil@samsung.com>
 * 	- CPU idle
 *
 * 2003-12-11:	Seongil Na <seongil@samsung.com>
 * 				Jaehoon Jeong <hooni_jeong@samsung.com>
 *  - Modified from swl patch
 * 	- Complete System suspend/resume
 * 	- Remove CPU idle to s3c2410_idle.c
 *
 */

#if defined(CONFIG_MODVERSIONS)
#define MODVERSIONS
#include <linux/modversions.h>
#endif

#include <linux/module.h>			/* Kernel module definitions */
#include <linux/init.h>				/* Some macros of _init, _exit, Etc */
#include <linux/kernel.h>			/* We will be in the kernel mode of execution */
#include <linux/proc_fs.h>
#include <linux/interrupt.h>		/* Interrupt handler, Tasklet */
#include <linux/sched.h>			/* xtime structure */
#include <linux/delay.h>			/* udelay() */
#include <linux/pm.h>				/* pm_idle() */
#include <asm/hardware.h>			/* Definitions for S3C2440 */
#include <asm/arch/irqs.h>			/* Interrupt mapping for S3C2440 */
#include <asm/arch/cpu_s3c2440.h>	/* s3c2440_get_pclk() */

MODULE_AUTHOR("Samsung Electronics, Inc.");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("");

//#define _DEBUG_
#ifdef _DEBUG_
	#define DPRINTK(x, args...) printk("DEBUG>>%s:%d: "x, __FUNCTION__, __LINE__, ##args)
#else
	#define DPRINTK(x, args...) do {} while (0)
#endif

static int pm_event_value = 0xA;

extern void s3c2440_pm_cpu_poweroff(void);
extern void s3c2440_pm_cpu_resume(void);

unsigned long s3c2440_pm_sleep_phys_sp(void *sp);
static void s3c2440_pm_save_regs(unsigned int* p);
static void s3c2440_pm_restore_regs(unsigned int* p);
static void s3c2440_pm_stop_lcd(void);
static void s3c2440_pm_stop_dma(void);
static void s3c2440_pm_stop_iisbus(void);
static void s3c2440_pm_stop_gpio(void);
static void s3c2440_pm_stop_misc(void);
static void s3c2440_pm_wakeup_prepare(void);
static void s3c2440_pm_clear_interrupt(void);
static void s3c2440_pm_pwm_timer_update(void);
void s3c24440_pm_suspend(void);
static void s3c2440_pm_banking_softirq(int softirq[], int flag);
static void s3c2440_pm_tasklet_suspend(unsigned long dummy);
static void s3c2440_pm_handler(int irq, void* dev_id, struct pt_regs* regs);
static void  s3c2440_pm_wakeup_setup(void);
static int __init s3c2440_pm_init(void);
static void __exit s3c2440_pm_exit(void);

/* 
 * Convert a virtual address to physical address in assemble code
 */
unsigned long s3c2440_pm_sleep_phys_sp(void *sp)
{
	return virt_to_phys(sp);
}

/*
 * Save registers for wakeup
 */
// FIXME: We have to fix a vitual address of VA_IO_PORT_BASE
#define VA_IO_PORT_BASE		0xF6000000
#define OFFSET_EINTPEND		0xAC
static void s3c2440_pm_save_regs(unsigned int* p)
{
	/* Save GPIO port */
	unsigned int i = VA_IO_PORT_BASE;
	for (i; i < VA_IO_PORT_BASE + OFFSET_EINTPEND; i += 4)
		*p++ = *((unsigned int*)i);
		
	/* Save 2440 Extended Register */
	*p++ = DSC0;
	*p++ = DSC1;
	*p++ = MSLCON;
	*p++ = GPJCON;
	*p++ = GPJDAT;
	*p++ = GPJUP;
		
	/* Save Interrupt */	
	*p++ = INTMOD;
	*p++ = INTMSK;
	*p++ = INTSUBMSK;
	//printk("debug suspend isr INTMSK : %lx\n", INTMSK);

	/* Save LCD  	*/
	*p++ = LCDLPCSEL;
	*p++ = LCDINTMSK;
	*p++ = TPAL;
	*p++ = DITHMODE;
	*p++ = BLUELUT;
	*p++ = GREENLUT;
	*p++ = REDLUT;
	*p++ = LCDADDR3;
	*p++ = LCDADDR2;
	*p++ = LCDADDR1;
	*p++ = LCDCON5;
	*p++ = LCDCON4;
	*p++ = LCDCON3;
	*p++ = LCDCON2;
	*p++ = LCDCON1;
	
	/* Save Uart */
	*p++ = UBRDIV0;
	*p++ = ULCON0; 
	*p++ = UFCON0;
	*p++ = UMCON0;
	*p++ = UCON0;

	/* Save TS */
	*p++ = ADCCON; 
	*p++ = ADCTSC; 
	*p++ = ADCDLY; 

	/* Save NAND Controller */
	*p++ = NFCONF; 
	*p++ = NFCONT;

	/* Save DMA Registers */	
	/* DMA channel 0 */
	*p++ = DISRC0;
	*p++ = DISRCC0;
	*p++ = DIDST0;
	*p++ = DIDSTC0;
	*p++ = DCON0;
	*p++ = DMTRIG0;

	/* DMA channel 1 */
	*p++ = DISRC1;
	*p++ = DISRCC1;
	*p++ = DIDST1;
	*p++ = DIDSTC1;
	*p++ = DCON1;
	*p++ = DMTRIG1;

	/* DMA channel 2 */
	*p++ = DISRC2;
	*p++ = DISRCC2;
	*p++ = DIDST2;
	*p++ = DIDSTC2;
	*p++ = DCON2;
	*p++ = DMTRIG2;

	/* DMA channel 3 */
	*p++ = DISRC3;
	*p++ = DISRCC3;
	*p++ = DIDST3;
	*p++ = DIDSTC3;
	*p++ = DCON3;
	*p++ = DMTRIG3;

	/* Save IIS-BUS */
	*p++ = IISFIFOC; 
	*p++ = IISPSR; 
	*p++ = IISMOD; 
	*p++ = IISCON; 

	return;
}

/*
 * Restore registers
 */
static void s3c2440_pm_restore_regs(unsigned int* p)
{
	/* Restore GPIO Port */
	unsigned int i = VA_IO_PORT_BASE;
	for (i; i < VA_IO_PORT_BASE + OFFSET_EINTPEND; i += 4)
		*((unsigned int*)i) = *p++;
		
	/* Restore 2440 Extended Register */
	DSC0 = *p++;
	DSC1 = *p++;
	MSLCON = *p++;
	GPJCON = *p++;
	GPJDAT = *p++;
	GPJUP = *p++;
		
	/* Restore Interrupt */	
	INTMOD    =  *p++; 
	INTMSK    =  *p++; 
	INTSUBMSK =  *p++;

	//printk("debug isr resume INTMSK : %lx\n", INTMSK);

	/* Restore  LCD */
	LCDLPCSEL =  *p++; 
	LCDINTMSK =  *p++; 
	TPAL      =  *p++; 
	DITHMODE  =  *p++; 
	BLUELUT   =  *p++; 
	GREENLUT  =  *p++; 
	REDLUT    =  *p++; 
	LCDADDR3  =  *p++; 
	LCDADDR2  =  *p++; 
	LCDADDR1  =  *p++; 
	LCDCON5   =  *p++; 
	LCDCON4   =  *p++; 
	LCDCON3   =  *p++; 
	LCDCON2   =  *p++; 
	LCDCON1   =  *p++;

	/* Restore USB */
	UBRDIV0  = *p++;
	ULCON0   = *p++;
	UFCON0   = *p++;
	UMCON0   = *p++;
	UCON0    = *p++;

	/* Restore TS */
	ADCCON   = *p++; 
	ADCTSC   = *p++; 
	ADCDLY   = *p++; 

	/* Restore Nand Controller */
	NFCONF	 = *p++; 
	NFCONT	 = *p++; 

	/* Restore DMA Registers */	
	/* DMA channel 0 */
	DISRC0	 = *p++;
	DISRCC0	 = *p++;
	DIDST0	 = *p++;
	DIDSTC0  = *p++;
	DCON0	 = *p++;
	DMTRIG0  = *p++;

	/* DMA channel 1 */
	DISRC1	 = *p++;
	DISRCC1	 = *p++;
	DIDST1	 = *p++;
	DIDSTC1	 = *p++;
	DCON1	 = *p++;
	DMTRIG1	 = *p++;

	/* DMA channel 2 */
	DISRC2	 = *p++;
	DISRCC2	 = *p++;
	DIDST2	 = *p++;
	DIDSTC2	 = *p++;
	DCON2	 = *p++;
	DMTRIG2	 = *p++;

	/* DMA channel 3 */
	DISRC3	 = *p++;
	DISRCC3	 = *p++;
	DIDST3	 = *p++;
	DIDSTC3	 = *p++;
	DCON3	 = *p++;
	DMTRIG3	 = *p++;

	/* Restore IIS-BUS */
	IISFIFOC = *p++; 
	IISPSR	 = *p++; 
	IISMOD	 = *p++; 
	IISCON	 = *p++; 
	return;
}

/*
 * Stop a LCD controller
 */
static void s3c2440_pm_stop_lcd(void)
{
	GPGDAT &= ~GPGCON_LCD_PWREN;
	LCDCON1		= 0x0;
	LCDCON2		= 0x0;
	LCDCON3		= 0x0;
	LCDCON4 	= 0x0;
	LCDCON5		= 0x0;
	LCDADDR1	= 0x0;
	LCDADDR2	= 0x0;
	LCDADDR3	= 0x0;
	LCDLPCSEL	= 0x0;
	TPAL		= 0x0;

	return;
}

/*
 * Stop a DMA 
 */
static void s3c2440_pm_stop_dma(void)
{
	DMTRIG0	 	= 0x4;
	DMTRIG1	 	= 0x4;
	DMTRIG2	 	= 0x4;
	DMTRIG3	 	= 0x4;

	DMTRIG0	 	= 0x2;
	DMTRIG1	 	= 0x2;
	DMTRIG2	 	= 0x2;
	DMTRIG3	 	= 0x2;
	return;
}

/*
 * Stop a IIS-BUS controller
 */
static void s3c2440_pm_stop_iisbus(void)
{
	IISFIFOC	= 0x0;
	IISPSR		= 0x0;
	IISMOD		= 0x0;
	IISCON	 	= 0x0;

	return;
}

/*
 * Stop GPIO pins
 * 	This code derives from WinCE PowerOff
 * 	
 * 	- Check point
 * 		1) NC pin: Input pull-up on
 * 		2) If input is driver externally: Input pull-up off
 * 		3) If a connected component draws some current: Output low
 * 		4) If a connected component draws no current: Output high
 *
 * 	- The configuration order for setting the ports
 * 		1) Setting value (GPnDAT)
 * 		2) Setting control register (GPnCON)
 * 		3) Setting pull-up register (GPnUP)
 */
static void s3c2440_pm_stop_gpio(void)
{
    // Check point
    // 1) NC pin: input pull-up on 
    // 2) If input is driver externally: input pull-up off
    // 3) If a connected component draws some current: output low.
    // 4) If a connected component draws no current: output high.

    // Follow the configuration order for setting the ports. 
    // 1) setting value(GPnDAT) 
    // 2) setting control register  (GPnCON)
    // 3) configure pull-up resistor(GPnUP)  

    //CAUTION: Below configurations are only for SMDK2440 eva. board
    //*** PORT A GROUP
    // [22:nFCE] [21:nRSTOUT] [20:nFRE] [19:nFWE] [18:ALE] [17:CLE] [16:nGCS5]
    // [15:nGCS4] [14:nGCS3] [13:nGCS2] [12:nGCS1] [11:ADR26] [10:ADR25] [9:ADR24] [8:ADR23]  
    // [7:ADR22] [6:ADR21] [5:ADR20] [4:ADR19] [3:ADR18] [2:ADR17] [1:ADR16] [0:ADR0]
    GPACON = 0x7fffff; 

    //**** PORT B GROUP
    // [10:nXDREQ0] [9:nXDACK0] [8:nXDREQ1] 
    // [7:nXDACK1] [6:nSS_KBD] [5:nDIS_OFF] [4:L3CLOCK] [3:L3DATA] [2:L3MODE] [1:nIrDATXDEN] [0:Keyboard]
    // * nXDREQ0/1:ext.pD, nSS_KBD:ext.pU, nDIS_OFF:ext.pD, nIrDATXDEN:ext.pU, Keyboard:ext.pU  
    GPBDAT = 0x0;
    GPBCON = 0x0;  // all input  
    GPBUP  = (1<<10)+(1<<8)+(1<<6)+(1<<5)+(1<<1)+(1<<0);//0x563;

    //*** PORT C GROUP
    // [15:VD7] [14:VD6] [13:VD5] [12:VD4] [11:VD3] [10:VD2] [9:VD1] [8:VD0] 
    // [7:LCD_LPCREVB] [6:LCD_LPCREV] [5:LCD_LPCOE] [4:VM] [3:VFRAME] [2:VLINE] [1:VCLK] [0:LEND]
    // * LCD_LPCREV and LCD_LPCREVB are connected the analog circuit in LCD or board. So, this signal should be output L.
    GPCDAT = 0x0;
    GPCCON = (1<<14)+(1<<12);//0x5000;
    GPCUP  = (1<<7)+(1<<6);//0xc0;  

    //*** PORT D GROUP
    // [15:VD23] [14:VD22] [13:VD21] [12:VD20] [11:VD19] [10:VD18] [9:VD17] [8:VD16] 
    // [7:VD15] [6:VD14] [5:VD13] [4:VD12] [3:VD11] [2:VD10] [1:VD9] [0:VD8]
    // * 5V tol. pad(GPD[10:8]) should be output L 
    // * The input of Maxim IC may have current sink, so the port should have configured output 'H'(GPD0)
    // * The output of Maxim IC drives 'H', so disable the pull-up(GPD1)
    GPDDAT = (1<<0);//0x1;
    GPDCON = (1<<20)+(1<<18)+(1<<16)+(1<<0);//0x150001;	
    GPDUP  = (1<<10)+(1<<9)+(1<<8)+(1<<1);//0x702;

    //*** PORT E GROUP
    // [15:IICSDA] [14:IICSCL] [13:SPICLK] [12:SPIMOSI] [11:SPIMISO] [10:SDATA3] [9:SDDATA2] [8:SDDATA1] 
    // [7:SDDATA0] [6:SDCMD] [5:SDCLK] [4:I2SSDO] [3:I2SSDI] [2:CDCLK] [1:I2SSCLK] [0:I2SLRCK]
    // * 5V tol. pad(GPE[13:11]) should be output L 
    // * The output of CODEC IC drives 'H', so disable the pull-up
    // * IICSDA/IICSCL:ext pU, SDDAT[3:0]/SDCMD:ext pU 
    GPEDAT = 0x0;
    GPECON = (1<<26)+(1<<24)+(1<<22);//0x5400000;	
    GPEUP  = (0xf<<12)+(0xf<<8)+(0x3<<6)+(1<<3);//0xffc8;

    //*** PORT F GROUP
    // [7:nLED_8] [7:nLED_4] [7:nLED_2] [7:nLED_1] [7:nIRQ_PCMCIA] [7:EINT2] [7:KBDINT] [7:EINT0]
    // * GPF[7:4] are connected to 'H' through LED, so make output 'H'
    // * nIRQ_PCMCIA=Hi-Z Before initialize
    // * EINT0 is the wake-up source
    // * EINT2/0:ext pU, KBDINT:Keyboard controller drives high 
    GPFDAT = (0xf<<4);//0xf0;
    GPFCON = (0x5<<12)+(0x5<<8)+2;//0x5502;
    GPFUP  = (0xf<<4)+7;//0xf7;

    //*** PORT G GROUP
    // [15:EINT23] [14:EINT22] [13:EINT21] [12:EINT20] [11:EINT19] [10:nCD_SD] [9:DMAMODE] [8:DMASTART]
    // [7:KBDSPICLK] [6:KBDSPIMOSI] [5:KBDSPIMISO] [4:LCD_PWREN] [3:EINT11] [2:nSS_SPI] [1:IRQ_LAN] [0:IRQ_PCMCIA]
    // * 5V tol. pad(GPG[13:11]) should be output L
    // * GPG[15:13]:NAND boot config, EINT19:ext pU, nCD_SD:ext pU, EINT11:ext pU
    // * The input of Maxim IC may have current sink, so the port should have configured output 'H'(GPG9)
    // * The output of Maxim IC drives 'H', so disable the pull-up(GPG10)
    GPGDAT = (1<<9);//0x200;
    GPGCON = (1<<18)+(1<<14)+(1<<12)+(1<<10);//0x45400;
    GPGUP  = (0xe<<12)+(0xe<<8)+(0xe<<4)+(1<<3);//0xeee8;

    //*** PORT H GROUP
    // [10:CLKOUT1] [9:CLKOUT0] [8:UCLK]
    // [7:nCTS1] [6:nRTS1] [5:RXD1] [4:TXD1] [3:RXD0] [2:TXD0] [1:nRTS0] [0:nCTS0] 
    // * WP_SP:ext pU
    // * The input of Maxim IC may have current sink, so the port should have configured output 'H'(GPH4/2/1)
    // * The output of Maxim IC drives 'H', so disable the pull-up(GPH5/3/0)
    GPHDAT = (1<<4)+(1<<2)+(1<<1);  
    GPHCON = (1<<8)+(1<<4)+(1<<2);//0x114; 
    GPHUP  = (1<<8)+(1<<5)+(1<<3)+1;//0x129;

    //*** PORT J GROUP
    // [12:CAMRESET] [11:CAMCLKOUT] [10:CAMHREF] [9:CAMVS] [8:CAMPCLKIN] 
    // [7:CAMD7] [6:CAMD6] [5:CAMD5] [4:CAMD4] [3:CAMD3] [2:CAMD2] [1:CAMD1] [0:CAMD0]
    // * The output of buffer IC drives 'H', so disable the pull-up(GPJ[10:0])
    GPJDAT = 0x0;  	
    GPJCON = 0x0;  // all input
    GPJUP  = 0x7ff;	

    //External interrupt will be falling edge triggered. 
    EXTINT0 = 0x22222222;	// EINT[7:0]
    EXTINT1 = 0x22222222;	// EINT[15:8]
    EXTINT2 = 0x22222222;	// EINT[23:16]

    return;
}

/*
 * Configure a miscellaneous registers
 */
static void s3c2440_pm_stop_misc(void)
{
	/* Disable RTC controller */
	RTCCON = 0x0;	// Disable, XTAL 1/2^15, Merge BCD, No reset
	
	/* Configure a ACD controller as standby mode */
	ADCCON |= ADCCON_STDBM;
	
	/* Suspend USB port 0,1 */
	MISCCR |= MISCCR_USB1_SUSPEND | MISCCR_USB0_SUSPEND;
	
	/* Previous state */
	MISCCR |= MISCCR_HZSTOP;
	
	/* Configure MISCCR[1:0] for the pull-up resisters on data bus */
	/* D[31:0] is pull-up off */
	MISCCR |= (MISCCR_SPUCR1 | MISCCR_SPUCR0);
	
	/* Memory Sleep Control Regisger */
	// In the evaluation board, Even though in sleep mode, the devices are all supplied the power.
	MSLCON = (0<<11)|(0<<10)|(0<<9)|(0<<8)|(0<<7)|(0<<6)|(0<<5)|(0<<4)|(0<<3)|(0<<2)|(0<<1)|(0<<0);
	
	/* Driver strength control */
	DSC0 = (1<<31)|(3<<8)|(3<<0);
	DSC1 = (3<<28)|(3<<26)|(3<24)|(3<<22)|(3<<20)|(3<<18);

	return;
}

/*
 * Prepare for wakeup
 */
static void s3c2440_pm_wakeup_prepare(void)
{
	/* Interrupt disable */
	INTMSK = 0xFFFFFFFF;
	SRCPND = 0xFFFFFFFF;
	INTPND = 0xFFFFFFFF;
	GPFCON = 0x550A;
	GPGCON = 0x55550100;
	
	return;
}

/*
 * Clear interrupt
 */
static void s3c2440_pm_clear_interrupt(void)
{
	EINTPEND  = EINTPEND;
	LCDSRCPND = LCDSRCPND;
	LCDINTPND = LCDINTPND;
	SUBSRCPND = SUBSRCPND;
	SRCPND    = SRCPND;
	INTPND    = INTPND;
	
	return;
}

#define SYS_TIMER234_PRESCALER  202
#define SYS_TIMER4_MUX          1	/* 1/4  */
#define SYS_TIMER4_DIVIDER      4 
#define RESCHED_PERIOD          10      /* 10 ms */
static unsigned long  NowTCNTB4;
static void s3c2440_pm_pwm_timer_update(void)
{
	TCFG0 = (TCFG0 & ~(0xFF << 8)) | (SYS_TIMER234_PRESCALER << 8); 
	TCFG1 = (TCFG1 & ~(0xF << 16)) | (SYS_TIMER4_MUX << 16);
	NowTCNTB4 = (RESCHED_PERIOD * s3c2440_get_pclk()) / 
				((SYS_TIMER234_PRESCALER + 1) * (SYS_TIMER4_DIVIDER) * 1000);
	TCNTB4 = NowTCNTB4;
	TCON = (TCON & ~(7 << 20)) | (6 << 20);
	/* interval mode(auto reload), update TCNTB4, stop */
	TCON = (TCON & ~(7 << 20)) | (5 << 20);
	/* interval mode, no operation, start for Timer 4 */

	return;
}
/*
 * System suspend algorithm
 */
#define CPU_SAVE_SIZE		150
static unsigned int cpu_saved_reg[CPU_SAVE_SIZE];
void s3c2440_pm_suspend(void)
{
	/* Turn off interrupts */
	cli();
	
	/* Save some meaning values into GSTATUS3,4 */
	GSTATUS3 = virt_to_phys(s3c2440_pm_cpu_resume);
	GSTATUS4 = 0xAA;	// Test
	DPRINTK("GSTATUS3[0x%X], GSTATUS4[0x%X]\n", GSTATUS3, GSTATUS4);
	
	/* Save registers */
	cpu_saved_reg[CPU_SAVE_SIZE] = xtime.tv_sec;
	s3c2440_pm_save_regs(cpu_saved_reg);
	
	/* Stop a LCD controller */
	s3c2440_pm_stop_lcd();

	/* Stop a DMA Operation */
	s3c2440_pm_stop_dma();

	/* Stop a IIS-BUS controller */
	s3c2440_pm_stop_iisbus();

	/* Configure a miscellaneous registers */
	s3c2440_pm_stop_misc();

	/* Prepare for wakeup */
	s3c2440_pm_wakeup_prepare();

	/* Configure a GPIOs during power_off mode */
	s3c2440_pm_stop_gpio();

	/* CPU power off now.. */
	udelay(10);
	s3c2440_pm_cpu_poweroff();

	/* zzz */

	xtime.tv_sec = cpu_saved_reg[CPU_SAVE_SIZE];

	// FIXME
	/* Check a wakeup */
	//s3c2440_pm_check_wakeup();
	
	/* Restore registers */
	s3c2440_pm_restore_regs(cpu_saved_reg);

	/* Clear interrupt */
	s3c2440_pm_clear_interrupt();

	/* Update PWM timer */
	s3c2440_pm_pwm_timer_update();

	/* Flush all caches */
	cpu_cache_clean_invalidate_all();

	/* Turn on interrupts */
	sti();
	
	return;
}

static void s3c2440_pm_tasklet_suspend(unsigned long);
static DECLARE_TASKLET_DISABLED(tasklet_pm_suspend,
				s3c2440_pm_tasklet_suspend,
				(unsigned long)0);
enum
{
	local_irq_cnt = 0,
	local_bh_cnt,
	local_save,
	local_restore,
	local_clear,
};

static void s3c2440_pm_banking_softirq(int softirq[], int flag)
{
	switch (flag)
	{
	case local_save:
		 softirq[local_irq_cnt] = local_irq_count(0);
		 softirq[local_bh_cnt]  = local_bh_count(0);
	case local_clear:
		 local_irq_count(0) = 0;
		 local_bh_count(0)  = 0;
		 break;
	case local_restore:
		 local_irq_count(0) = softirq[local_irq_cnt];
		 local_bh_count(0)  = softirq[local_bh_cnt];
		 break;
	default:
		 break;
	}

	return;			
}

static void s3c2440_pm_tasklet_suspend(unsigned long dummy)
{
	int bank[local_bh_cnt+1];

	s3c2440_pm_banking_softirq(bank, local_save);
	
	//FIXME: suspend all devices
	//ret = pm_send_all(PM_SUSPEND, (void*)2);
	
	DPRINTK("Entring power_off mode\n");
	
	udelay(100);
	s3c2440_pm_suspend();

	//FIXME: resume all devices
	//ret = pm_send_all(PM_RESUME, (void *)0);
	
	s3c2440_pm_banking_softirq(bank, local_restore);
	
	DPRINTK("Resumed from power_off mode\n");
	return;
}

static void s3c2440_pm_handler(int irq, void* dev_id, struct pt_regs* regs)
{
	pm_event_value = 0xF;

	DPRINTK("Power button interrupt handler\n");
	return;
}

/*
 * Setup a EINT0(Power button) for wakeup source from Power_OFF mode
 */
static void  s3c2440_pm_wakeup_setup(void)
{
	GPFCON  &= ~(0x3<<0);	// EINT0(GPF0) as EINT0)
	GPFCON  |=  (0x2<<0);
	EXTINT0 &= ~(0x7<<0);	// Configure EINT0 as Falling Edge Mode
	EXTINT0 |=  (0x2<<0);
	
	DPRINTK("EXTINT0(0x%08X) GPFCON (0x%08X)\n", EXTINT0,GPFCON);
	return;
}

/* Addition PM Proc Interface */
#ifdef CONFIG_PROC_FS
#define PM_ENTRY_NAME			"pm"
#define PM_SUSPEND_ENTRY_NAME	"suspend"
#define PM_EVENT_ENTRY_NAME	"event"
static struct proc_dir_entry* s3c2440_pm_proc = NULL;

/*
 * Writing to /proc/pm/suspend puts the System suspend
 */
static int s3c2440_pm_suspend_write_proc(struct file* file, const char* buffer,
										 unsigned long count, void* data)
{
	const char *event;

	DPRINTK("Writing a /proc/pm/suspend\n");

	event = buffer;

	if(*event == '1')
	{
		//DPRINTK("Entring power_off mode\n");
		tasklet_schedule(&tasklet_pm_suspend);

		//DPRINTK("Resumed from power_off mode\n");
	}
	else
		DPRINTK("Invalid Power_off Command %c\n", event[0]);
 	
	return count;
}
/*
 * reading to /proc/pm/event gets a System suspend request
 */
static int s3c2440_pm_event_read_proc(char *page, char **start, off_t off, int count, int *eof, void *data)
{
	int len;
	DPRINTK("Reading a /proc/pm/event\n");
	if(pm_event_value == 0xF)
	{
		DPRINTK("1. pm_event_value %x\n", pm_event_value);
		len = sprintf(page, "f\n");
		pm_event_value = 0xA;
	}
	else
	{
		DPRINTK("2. pm_event_value %x\n", pm_event_value);
		len = sprintf(page, "a\n");
	}
	return len;
}
#endif /* CONFIG_PROC_FS */

/*
 * Power Management initial moduel
 * 	- Support a system suspend/resume
 */
static int __init s3c2440_pm_init(void)
{
	int ret = 0;

/* Addition PM Proc Interface */
#ifdef CONFIG_PROC_FS
	struct proc_dir_entry* entry_suspend = NULL;
	struct proc_dir_entry* entry_event = NULL;

	s3c2440_pm_proc = proc_mkdir(PM_ENTRY_NAME, NULL);
	if (s3c2440_pm_proc != NULL)
	{
		entry_suspend = create_proc_entry(PM_SUSPEND_ENTRY_NAME, 0, s3c2440_pm_proc);
		if (entry_suspend)
			entry_suspend->write_proc = s3c2440_pm_suspend_write_proc;
		entry_event = create_proc_entry(PM_EVENT_ENTRY_NAME, 0, s3c2440_pm_proc);
		if (entry_event)
			entry_event->read_proc = s3c2440_pm_event_read_proc;
	}
	else
	{
		DPRINTK("Unable to create /proc/pm entry\n");
		return -ENOMEM;
	}
#endif /* CONFIG_PROC_FS */

	/* Enable a tasklet for power button */
	//register_sysctl_table(pm_dir_table, 1);
	tasklet_enable(&tasklet_pm_suspend);
	
	/* Setup a power button and install a interrupt handler */
	s3c2440_pm_wakeup_setup();
	ret = request_irq(IRQ_EINT0, s3c2440_pm_handler, 
			SA_INTERRUPT, "Power button", NULL);
	if (ret != 0)
	{
		printk(KERN_INFO "Can't setup a power button\n");
		return -ENODEV;
	}
	
	printk(KERN_INFO "S3C2440 Power management module installed\n");
	return 0;
}

static void __exit s3c2440_pm_exit(void)
{
	/* Free a power button interrupt handler */
	free_irq(IRQ_EINT0, NULL);

	printk(KERN_INFO "S3C2440 Power management module removed\n");
}

module_init(s3c2440_pm_init);
module_exit(s3c2440_pm_exit);
