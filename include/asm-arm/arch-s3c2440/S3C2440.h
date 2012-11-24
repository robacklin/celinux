/*
 * linux/include/asm-arm/arch-s3c2440/S3C2440.h
 *
 * Based On linux/include/asm-arm/arch-sa1100/SA1100.h
 *
 * Copyright (C) 2003 Samsung Electronics
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file COPYING in the main directory of this archive
 * for more details.
 */

/* Be sure that virtual mapping is defined right */
#ifndef __ASM_ARCH_HARDWARE_H
#error You must include hardware.h not S3C2440.h
#endif

#include "bitfield.h"

#define MEM_CTL_BASE	0x48000000
#define BWSCON		__REG(MEM_CTL_BASE+0x00)
#define BANKCON0	__REG(MEM_CTL_BASE+0x04)
#define BANKCON1	__REG(MEM_CTL_BASE+0x08)
#define BANKCON2	__REG(MEM_CTL_BASE+0x0c)
#define BANKCON3	__REG(MEM_CTL_BASE+0x10)
#define BANKCON4	__REG(MEM_CTL_BASE+0x14)
#define BANKCON5	__REG(MEM_CTL_BASE+0x18)
#define BANKCON6	__REG(MEM_CTL_BASE+0x1c)
#define BANKCON7	__REG(MEM_CTL_BASE+0x20)
#define REFRESH		__REG(MEM_CTL_BASE+0x24)
#define BANKSIZE	__REG(MEM_CTL_BASE+0x28)
#define MRSRB6		__REG(MEM_CTL_BASE+0x2c)
#define MRSRB7		__REG(MEM_CTL_BASE+0x30)

/* Bus width & wait status control register */
#define fBWSCON_ST(x)	Fld(1,(x)*4+3)	/* Determine SRAM for using UB/LB for bank(x) */
#define fBWSCON_WS(x)	Fld(1,(x)*4+2)	/* Determine WAIT status for bank (x) */
#define fBWSCON_DW(x)	Fld(2,(x)*4)	/* Determine data bus width for bank (x) */
#define fBWSCON_DW0	Fld(2,1)	/* Determine SRAM for using UB/LB for bank 0 (read only). */
#define BWSCON_ST7	FMsk(fBWSCON_ST(7))
#define BWSCON_WS7	FMsk(fBWSCON_WS(7))
#define BWSCON_DW7	FMsk(fBWSCON_DW(7))
#define BWSCON_ST6	FMsk(fBWSCON_ST(6))
#define BWSCON_WS6	FMsk(fBWSCON_WS(6))
#define BWSCON_DW6	FMsk(fBWSCON_DW(6))
#define BWSCON_ST5	FMsk(fBWSCON_ST(5))
#define BWSCON_WS5	FMsk(fBWSCON_WS(5))
#define BWSCON_DW5	FMsk(fBWSCON_DW(5))
#define BWSCON_ST4	FMsk(fBWSCON_ST(4))
#define BWSCON_WS4	FMsk(fBWSCON_WS(4))
#define BWSCON_DW4	FMsk(fBWSCON_DW(4))
#define BWSCON_ST3	FMsk(fBWSCON_ST(3))
#define BWSCON_WS3	FMsk(fBWSCON_WS(3))
#define BWSCON_DW3	FMsk(fBWSCON_DW(3))
#define BWSCON_ST2	FMsk(fBWSCON_ST(2))
#define BWSCON_WS2	FMsk(fBWSCON_WS(2))
#define BWSCON_DW2	FMsk(fBWSCON_DW(2))
#define BWSCON_ST1	FMsk(fBWSCON_ST(1))
#define BWSCON_WS1	FMsk(fBWSCON_WS(1))
#define BWSCON_DW1	FMsk(fBWSCON_DW(1))
#define BWSCON_DW0	FMsk(fBWSCON_DW0)
#define BWSCON_DW7_8	FInsrt(0x00,fBWSCON_DW(7))	/* 8bit */
#define BWSCON_DW7_16	FInsrt(0x01,fBWSCON_DW(7))	/* 16bit */
#define BWSCON_DW7_32	FInsrt(0x02,fBWSCON_DW(7))	/* 32bit */
#define BWSCON_DW6_8	FInsrt(0x00,fBWSCON_DW(6))	/* 8bit */
#define BWSCON_DW6_16	FInsrt(0x01,fBWSCON_DW(6))	/* 16bit */
#define BWSCON_DW6_32	FInsrt(0x02,fBWSCON_DW(6))	/* 32bit */
#define BWSCON_DW5_8	FInsrt(0x00,fBWSCON_DW(5))	/* 8bit */
#define BWSCON_DW5_16	FInsrt(0x01,fBWSCON_DW(5))	/* 16bit */
#define BWSCON_DW5_32	FInsrt(0x02,fBWSCON_DW(5))	/* 32bit */
#define BWSCON_DW4_8	FInsrt(0x00,fBWSCON_DW(4))	/* 8bit */
#define BWSCON_DW4_16	FInsrt(0x01,fBWSCON_DW(4))	/* 16bit */
#define BWSCON_DW4_32	FInsrt(0x02,fBWSCON_DW(4))	/* 32bit */
#define BWSCON_DW3_8	FInsrt(0x00,fBWSCON_DW(3))	/* 8bit */
#define BWSCON_DW3_16	FInsrt(0x01,fBWSCON_DW(3))	/* 16bit */
#define BWSCON_DW3_32	FInsrt(0x02,fBWSCON_DW(3))	/* 32bit */
#define BWSCON_DW2_8	FInsrt(0x00,fBWSCON_DW(2))	/* 8bit */
#define BWSCON_DW2_16	FInsrt(0x01,fBWSCON_DW(2))	/* 16bit */
#define BWSCON_DW2_32	FInsrt(0x02,fBWSCON_DW(2))	/* 32bit */
#define BWSCON_DW1_8	FInsrt(0x00,fBWSCON_DW(1))	/* 8bit */
#define BWSCON_DW1_16	FInsrt(0x01,fBWSCON_DW(1))	/* 16bit */
#define BWSCON_DW1_32	FInsrt(0x02,fBWSCON_DW(1))	/* 32bit */
#define BWSCON_DW0_8	FInsrt(0x00,fBWSCON_DW0)	/* 8bit */
#define BWSCON_DW0_16	FInsrt(0x01,fBWSCON_DW0)	/* 16bit */
#define BWSCON_DW0_32	FInsrt(0x02,fBWSCON_DW0)	/* 32bit */

/* Bank 0~5 control register */
#define	fBANKCON_Tacs	Fld(2,13)	/* Address set-up time before nGCSn		init:00	*/
#define	fBANKCON_Tcos	Fld(2,11)	/* Chip selection set-up time before nOE	init:00 */
#define fBANKCON_Tacc	Fld(3,8)	/* Access cycle					init:111 */
#define	fBANKCON_Toch	Fld(2,6)	/* Chip selection hold time after nOE		init:00 */
#define	fBANKCON_Tcah	Fld(2,4)	/* Address hold time after nGCSn		init:00 */
#define	fBANKCON_Tacp	Fld(2,2)	/* Page mode access cycle @ Page mode		init:00 */
#define	fBANKCON_PMC	Fld(2,0)	/* Page mode configuration			init:00 */
#define	BANKCON_Tacs	FMsk(fBANKCON_Tacs)
#define	BANKCON_Tcos	FMsk(fBANKCON_Tcos)
#define BANKCON_Tacc	FMsk(fBANKCON_Tacc)
#define	BANKCON_Toch	FMsk(fBANKCON_Toch)
#define	BANKCON_Tcah	FMsk(fBANKCON_Tcah)
#define	BANKCON_Tacp	FMsk(fBANKCON_Tacp)
#define	BANKCON_PMC	FMsk(fBANKCON_PMC)
#define	BANKCON_Tacs_0	FInsrt(0x00,fBANKCON_Tacs)	/* 0 clock */
#define	BANKCON_Tacs_1	FInsrt(0x01,fBANKCON_Tacs)	/* 1 clock */
#define	BANKCON_Tacs_2	FInsrt(0x02,fBANKCON_Tacs)	/* 2 clock */
#define	BANKCON_Tacs_4	FInsrt(0x03,fBANKCON_Tacs)	/* 4 clock */
#define	BANKCON_Tcos_0	FInsrt(0x00,fBANKCON_Tcos)	/* 0 clock */
#define	BANKCON_Tcos_1	FInsrt(0x01,fBANKCON_Tcos)	/* 1 clock */
#define	BANKCON_Tcos_2	FInsrt(0x02,fBANKCON_Tcos)	/* 2 clock */
#define	BANKCON_Tcos_4	FInsrt(0x03,fBANKCON_Tcos)	/* 4 clock */
#define	BANKCON_Tacc_1	FInsrt(0x00,fBANKCON_Tacc)	/* 1 clock */
#define	BANKCON_Tacc_2	FInsrt(0x01,fBANKCON_Tacc)	/* 2 clock */
#define	BANKCON_Tacc_3	FInsrt(0x02,fBANKCON_Tacc)	/* 3 clock */
#define	BANKCON_Tacc_4	FInsrt(0x03,fBANKCON_Tacc)	/* 4 clock */
#define	BANKCON_Tacc_6	FInsrt(0x04,fBANKCON_Tacc)	/* 6 clock */
#define	BANKCON_Tacc_8	FInsrt(0x05,fBANKCON_Tacc)	/* 8 clock */
#define	BANKCON_Tacc_10	FInsrt(0x06,fBANKCON_Tacc)	/* 10 clock */
#define	BANKCON_Tacc_14	FInsrt(0x07,fBANKCON_Tacc)	/* 14 clock */
#define	BANKCON_Toch_0	FInsrt(0x00,fBANKCON_Toch)	/* 0 clock */
#define	BANKCON_Toch_1	FInsrt(0x01,fBANKCON_Toch)	/* 1 clock */
#define	BANKCON_Toch_2	FInsrt(0x02,fBANKCON_Toch)	/* 2 clock */
#define	BANKCON_Toch_4	FInsrt(0x03,fBANKCON_Toch)	/* 4 clock */
#define	BANKCON_Tcah_0	FInsrt(0x00,fBANKCON_Tcah)	/* 0 clock */
#define	BANKCON_Tcah_1	FInsrt(0x01,fBANKCON_Tcah)	/* 1 clock */
#define	BANKCON_Tcah_2	FInsrt(0x02,fBANKCON_Tcah)	/* 2 clock */
#define	BANKCON_Tcah_4	FInsrt(0x03,fBANKCON_Tcah)	/* 4 clock */
#define	BANKCON_Tacp_2	FInsrt(0x00,fBANKCON_Tacp)	/* 2 clock */
#define	BANKCON_Tacp_3	FInsrt(0x01,fBANKCON_Tacp)	/* 3 clock */
#define	BANKCON_Tacp_4	FInsrt(0x02,fBANKCON_Tacp)	/* 4 clock */
#define	BANKCON_Tacp_6	FInsrt(0x03,fBANKCON_Tacp)	/* 6 clock */
#define	BANKCON_PMC_1	FInsrt(0x00,fBANKCON_PMC)	/* normal (1 data) */
#define	BANKCON_PMC_4	FInsrt(0x01,fBANKCON_PMC)	/* 4 data */
#define	BANKCON_PMC_8	FInsrt(0x02,fBANKCON_PMC)	/* 8 data */
#define	BANKCON_PMC_16	FInsrt(0x03,fBANKCON_PMC)	/* 16 data */

/* Bank 6,7 control register */
#define fBANKCON_MT	Fld(2,15)	/* Determine the memory type for bank6 and bank7.	init:11 */
#define fBANKCON_Trcd	Fld(2,2)	/* RAS to CAS delay					init:10 */
#define fBANKCON_SCAN	Fld(2,0)	/* Column address number				init:00 */
#define BANKCON_MT	FMsk(fBANKCON_MT)
#define BANKCON_Trcd	FMsk(fBANKCON_Trcd)	
#define BANKCON_SCAN	FMsk(fBANKCON_SCAN)
#define BANKCON_MT_ROM	FInsrt(0x00, fBANKCON_MT)	/* ROM or SRAM */
#define BANKCON_MT_SDRAM	FInsrt(0x01, fBANKCON_MT)	/* Sync. DRAM */
#define	BANKCON_Trcd_2	FInsrt(0x00,fBANKCON_Trcd)	/* 2 clock */
#define	BANKCON_Trcd_3	FInsrt(0x01,fBANKCON_Trcd)	/* 3 clock */
#define	BANKCON_Trcd_4	FInsrt(0x02,fBANKCON_Trcd)	/* 4 clock */
#define	BANKCON_SCAN_8	FInsrt(0x00,fBANKCON_SCAN)	/* normal (1 data) */
#define	BANKCON_SCAN_9	FInsrt(0x01,fBANKCON_SCAN)	/* 4 data */
#define	BANKCON_SCAN_10	FInsrt(0x02,fBANKCON_SCAN)	/* 8 data */

/* SDRAM refresh control register */
#define fREFRESH_REFEN	Fld(1,23)	/* SDRAM Refresh Enable		init:1 */
#define fREFRESH_TREFMD	Fld(1,22)	/* SDRAM Refresh Mode		init:0 */
#define fREFRESH_Trp	Fld(2,20)	/* SDRAM RAS pre-charge Time	init:10 */
#define fREFRESH_Trc	Fld(2,18)	/* SDRAM RC minimum Time	init:11 */
#define fREFRESH_RC	Fld(11,0)	/* DRAM/SDRAM Refresh Counter	init:0 */
#define REFRESH_Trp_2	FInsrt(0x00,fREFRESH_Trp)		/* 2 clock */
#define REFRESH_Trp_3	FInsrt(0x01,fREFRESH_Trp)		/* 3 clock */
#define REFRESH_Trp_4	FInsrt(0x02,fREFRESH_Trp)		/* 4 clock */
#define REFRESH_Trc_4	FInsrt(0x00,fREFRESH_Trc)		/* 4 clock */
#define REFRESH_Trc_5	FInsrt(0x01,fREFRESH_Trc)		/* 5 clock */
#define REFRESH_Trc_6	FInsrt(0x02,fREFRESH_Trc)		/* 6 clock */
#define REFRESH_Trc_7	FInsrt(0x03,fREFRESH_Trc)		/* 7 clock */
#define REFRESH_REFEN   (1 << 23)
#define REFRESH_TREFMD  (1 << 22)

/* Flexible bank size register */
#define fBANKSIZE_BURST_EN	Fld(1,7)	/* ARM core burst operation enable.		init:0 */
#define fBANKSIZE_SCKE_EN	Fld(1,5)	/* SDRAM power down mode enable control by SCKE	init:0 */
#define fBANKSIZE_SCLK_EN	Fld(1,4)	/* SCLK is enabled only during SDRAM access cycle for
						 * reducing power consumption. When SDRAM is not accessed,
						 * SCLK becomes 'L' level.			init:0 */
#define fBANKSIZE_BK76MAP	Fld(2,0)	/* BANK6/7 memory map, 				init:0 */
#define BANKSIZE_BURST_EN	FMsk(fBANKSIZE_BURST_EN)
#define BANKSIZE_SCKE_EN	FMsk(fBANKSIZE_SCKE_EN)
#define BANKSIZE_SCLK_EN	FMsk(fBANKSIZE_SCLK_EN)
#define BANKSIZE_BK76MAP	FMsk(fBANKSIZE_BK76MAP)
#define BANKSIZE_BK76MAP_128128	FInsrt(0x02,fBANKSIZE_BK76MAP)	/* 128MB/128MB */
#define BANKSIZE_BK76MAP_6464	FInsrt(0x01,fBANKSIZE_BK76MAP)	/* 64MB/64MB */
#define BANKSIZE_BK76MAP_3232	FInsrt(0x00,fBANKSIZE_BK76MAP)	/* 32MB/32MB */
#define BANKSIZE_BK76MAP_1616	FInsrt(0x07,fBANKSIZE_BK76MAP)	/* 16MB/16MB */
#define BANKSIZE_BK76MAP_88	FInsrt(0x06,fBANKSIZE_BK76MAP)	/* 8MB/8MB */
#define BANKSIZE_BK76MAP_44	FInsrt(0x05,fBANKSIZE_BK76MAP)	/* 4MB/4MB */
#define BANKSIZE_BK76MAP_22	FInsrt(0x04,fBANKSIZE_BK76MAP)	/* 2MB/2MB */

/* Mode register set register */
#define fMRSR_WBL	Fld(1,9)	/* Write burst length */
#define fMRSR_TM	Fld(2,7)	/* Test Mode */
#define fMRSR_CL	Fld(3,4)	/* CAS Latency */
#define fMRSR_BT	Fld(1,3)	/* Burst Type */
#define fMRSR_BL	Fld(3,0)	/* Burst Length */
#define MRSR_WBL	FMsk(fMRSR_WBL)
#define MRSR_TM		FMsk(fMRSR_TM)
#define MRSR_CL		FMsk(fMRSR_CL)
#define MRSR_BT		FMsk(fMRSR_BT)
#define MRSR_BL		FMsk(fMRSR_BL)
#define MRSR_CL_1	FInsrt(0x00, fMRSR_CL)	/* 1 clock */
#define MRSR_CL_2	FInsrt(0x01, fMRSR_CL)	/* 2 clock */
#define MRSR_CL_3	FInsrt(0x02, fMRSR_CL)	/* 3 clock */

/* Clock and Power Management */
#define CLK_PM_BASE	0x4c000000
#define LOCKTIME	__REG(CLK_PM_BASE+0x00)
#define MPLLCON		__REG(CLK_PM_BASE+0x04)
#define UPLLCON		__REG(CLK_PM_BASE+0x08)
#define CLKCON		__REG(CLK_PM_BASE+0x0c)
#define CLKSLOW		__REG(CLK_PM_BASE+0x10)
#define CLKDIVN		__REG(CLK_PM_BASE+0x14)

/* PLL lock time count register */
#define fULTIME	Fld(12,12)	/* UPLL lock time count value for UCLK.	init:0xfff */
#define fMLTIME	Fld(12,0)	/* MPLL lock time count value for MCLK.	init:0xfff */
#define ULTIME	FMsk(fULTIME)
#define MLTIME	FMsk(fMLTIME)

/* MPLL/UPLL configuration register */
#define fPLL_MDIV	Fld(8,12)	/* Main divider control	init:0x5c 0x28 */
#define fPLL_PDIV	Fld(6,4)	/* Pre-divider control	init:0x08 0x08 */
#define fPLL_SDIV	Fld(2,0)	/* Post divider control	init:0x0 0x0 */
#define PLL_MDIV	FMsk(fPLL_MDIV)
#define PLL_PDIV	FMsk(fPLL_PDIV))
#define PLL_SDIV	FMsk(fPLL_SDIV)

/* Clock generator control register */
#define CLKCON_SPI	(1<<18)
#define CLKCON_IIS	(1<<17)
#define CLKCON_IIC	(1<<16)
#define CLKCON_ADC	(1<<15)
#define CLKCON_RTC	(1<<14)
#define CLKCON_GPIO	(1<<13)
#define CLKCON_UART2	(1<<12)
#define CLKCON_UART1	(1<<11)
#define CLKCON_UART0	(1<<10)
#define CLKCON_SDI	(1<<9)
#define CLKCON_PWM	(1<<8)
#define CLKCON_USBD	(1<<7)
#define CLKCON_USBH	(1<<6)
#define CLKCON_LCDC	(1<<5)
#define CLKCON_NAND	(1<<4)
#define CLKCON_POWEROFF	(1<<3)
#define CLKCON_IDLE	(1<<2)

/* Slow clock control register */
#define CLKSLOW_UCLK_ON		(1<<7)
#define CLKSLOW_MPLL_OFF	(1<<5)
#define CLKSLOW_SLOW_BIT	(1<<4)
#define CLKSLOW_SLOW_VAL	FMsk(Fld(3,0))

/* Clock divider control register */
#define CLKDIVN_HDIVN		(3<<0)
#define CLKDIVN_PDIVN		(1<<0)

/* I/O port*/
#define GPACON		__REG(0x56000000)
#define GPADAT		__REG(0x56000004)
#define GPBCON		__REG(0x56000010)
#define GPBDAT		__REG(0x56000014)
#define GPBUP		__REG(0x56000018)
#define GPCCON		__REG(0x56000020)
#define GPCDAT		__REG(0x56000024)
#define GPCUP		__REG(0x56000028)
#define GPDCON		__REG(0x56000030)
#define GPDDAT		__REG(0x56000034)
#define GPDUP		__REG(0x56000038)
#define GPECON		__REG(0x56000040)
#define GPEDAT		__REG(0x56000044)
#define GPEUP		__REG(0x56000048)
#define GPFCON		__REG(0x56000050)
#define GPFDAT		__REG(0x56000054)
#define GPFUP		__REG(0x56000058)
#define GPGCON		__REG(0x56000060)
#define GPGDAT		__REG(0x56000064)
#define GPGUP		__REG(0x56000068)
#define GPHCON		__REG(0x56000070)
#define GPHDAT		__REG(0x56000074)
#define GPHUP		__REG(0x56000078)
#define MISCCR		__REG(0x56000080)
#define DCLKCON		__REG(0x56000084)
#define EXTINT0		__REG(0x56000088)
#define EXTINT1		__REG(0x5600008c)
#define EXTINT2		__REG(0x56000090)
#define EINTFLT0	__REG(0x56000094)
#define EINTFLT1	__REG(0x56000098)
#define EINTFLT2	__REG(0x5600009c)
#define EINTFLT3	__REG(0x560000a0)
#define EINTMASK	__REG(0x560000a4)
#define EINTPEND	__REG(0x560000a8)
#define GSTATUS0	__REG(0x560000ac)
#define GSTATUS1	__REG(0x560000b0)
#define GSTATUS2	__REG(0x560000b4)
#define GSTATUS3	__REG(0x560000b8)
#define GSTATUS4	__REG(0x560000bc)
#define DSC0		__REG(0x560000c4)
#define DSC1		__REG(0x560000c8)
#define MSLCON		__REG(0x560000cc)
#define GPJCON		__REG(0x560000d0)
#define GPJDAT		__REG(0x560000d4)
#define GPJUP		__REG(0x560000d8)

/* Modified from here by Samsung */
/* I/O Port control registers */
#define GPGCON_LCD_PWREN	(1 << 4)
/* Modified to here by Samsung */

#define GSTATUS0_nWAIT      (1 << 3)
#define GSTATUS0_NCON       (1 << 2)
#define GSTATUS0_RnB        (1 << 1)
#define GSTATUS0_nBATT_FLT  (1 << 0)

#define GSTATUS2_PWRST      (1 << 0)
#define GSTATUS2_OFFRST     (1 << 1)
#define GSTATUS2_WDTRST     (1 << 2)


/* Miscellaneous control register */
#define MISCCR_nEN_SCKE		(1<<19)	
#define MISCCR_nEN_SCLK1	(1<<18)	
#define MISCCR_nEN_SCLK0	(1<<17)	
#define MISCCR_nRSTCON		(1<<16)	/* nRSTOUT software control */
#define MISCCR_USB0_SUSPEND	(1<<12)	/* set USB port 0 to Sleep */
#define MISCCR_USB1_SUSPEND	(1<<13)	/* set USB port 1 to Sleep */
#define fMISCCR_CLKSEL(x)	Fld(3, 4*((x)+1))
#define MISCCR_CLKSEL(x)	FMsk(fMISCCR_CLKSEL(x))	/* select ? CLK with CLKOUTx pad */
#define MISCCR_CLKSEL_MPLL(x)	FInsrt(0x0, fMISCCR_CLKSEL(x))
#define MISCCR_CLKSEL_UPLL(x)	FInsrt(0x1, fMISCCR_CLKSEL(x))
#define MISCCR_CLKSEL_FCLK(x)	FInsrt(0x2, fMISCCR_CLKSEL(x))
#define MISCCR_CLKSEL_HCLK(x)	FInsrt(0x3, fMISCCR_CLKSEL(x))
#define MISCCR_CLKSEL_PCLK(x)	FInsrt(0x4, fMISCCR_CLKSEL(x))
#define MISCCR_CLKSEL_DCLK(x)	FInsrt(0x5, fMISCCR_CLKSEL(x))
#define MISCCR_USBPAD		(1<<3)
#define MISCCR_HZSTOP		(1<<2)
#define MISCCR_SPUCR1		(1<<1)
#define MISCCR_SPUCR0		(1<<0)

/* DCLK0/1 control register */
#define fDCLKCON_DCLK1CMP		Fld(4,24)	/* DCLK Compare value clock */
#define fDCLKCON_DCLK1DIV		Fld(4,20))	/* DCLK divide value */
#define fDCLKCON_DCLK0CMP		Fld(4,8)	/* DCLK Compare value clock */
#define fDCLKCON_DCLK0DIV		Fld(4,4))	/* DCLK divide value */
#define DCLKCON_DCLK1CMP		FMsk(fDCLKCON_DCLK1CMP)
#define DCLKCON_DCLK1DIV		FMsk(fDCLKCON_DCLK1DIV)
#define DCLKCON_DCLK1SelCK		(1<<17)
#define DCLKCON_DCLK1EN			(1<<16)
#define DCLKCON_DCLK0CMP		FMsk(fDCLKCON_DCLK0CMP)
#define DCLKCON_DCLK0DIV		FMsk(fDCLKCON_DCLK0DIV)
#define DCLKCON_DCLK0SelCK		(1<<1)
#define DCLKCON_DCLK0EN			(1<<0)

/* UART */
#define UART0_CTL_BASE		0x50000000
#define UART1_CTL_BASE		0x50004000
#define UART2_CTL_BASE		0x50008000
/* Offset */
#define oULCON			0x00	/* R/W, UART line control register */
#define oUCON			0x04	/* R/W, UART control register */
#define oUFCON			0x08	/* R/W, UART FIFO control register */
#define oUMCON			0x0C	/* R/W, UART modem control register */
#define oUTRSTAT		0x10	/* R  , UART Tx/Rx status register */
#define oUERSTAT		0x14	/* R  , UART Rx error status register */
#define oUFSTAT			0x18	/* R  , UART FIFO status register */
#define oUMSTAT			0x1C	/* R  , UART Modem status register */
#define oUTXH			0x20	/*   W, UART transmit(little-end) buffer */
#define oURXH			0x24	/* R  , UART receive(little-end) buffer */
#define oUBRDIV			0x28	/* R/W, Baud rate divisor register */
/* UART line control register */
#define fULCON_IR		Fld(1,6)	/* use Infra-Red mode */
#define fULCON_PAR		Fld(3,3)	/* what parity mode? */
#define fULCON_STOP		Fld(1,2)	/* The number of stop bits */
#define fULCON_WL		Fld(2,0)	/* word length */
#define ULCON_IR		FMsk(fULCON_IR)
#define ULCON_PAR		FMsk(fULCON_PAR)
#define ULCON_STOP		FMsk(fULCON_STOP)
#define ULCON_WL		FMsk(fULCON_WL)
#define ULCON_PAR_NONE		FInsrt(0x0, fULCON_PAR) /* No Parity */
#define ULCON_PAR_ODD		FInsrt(0x4, fULCON_PAR) /* Odd Parity */
#define ULCON_PAR_EVEN		FInsrt(0x5, fULCON_PAR) /* Even Parity */
#define ULCON_PAR_1		FInsrt(0x6, fULCON_PAR) /* Parity force/checked as 1 */
#define ULCON_PAR_0		FInsrt(0x7, fULCON_PAR) /* Parity force/checked as 0 */
#define ULCON_WL5		FInsrt(0x0, fULCON_WL)	/* 5 bits */
#define ULCON_WL6		FInsrt(0x1, fULCON_WL)	/* 6 bits */
#define ULCON_WL7		FInsrt(0x2, fULCON_WL)	/* 7 bits */
#define ULCON_WL8		FInsrt(0x3, fULCON_WL)	/* 8 bits */
/* UART line control register */
#define UCON_CLK_SEL		(1 << 10)	/* select clock for UART */
#define UCON_CLK_PCLK		(0 << 10)	/* PCLK for UART baud rate */
#define UCON_CLK_UCLK		(1 << 10)	/* UCLK for UART baud rate */
#define UCON_TX_INT_TYPE	(1 << 9)	/* TX Interrupt request type */
#define UCON_TX_INT_PLS		(0 << 9)	/* Pulse */
#define UCON_TX_INT_LVL		(1 << 9)	/* Level */
#define UCON_RX_INT_TYPE	(1 << 8)	/* RX Interrupt request type */
#define UCON_RX_INT_PLS		(0 << 8)	/* Pulse */
#define UCON_RX_INT_LVL		(1 << 8)	/* Level */
#define UCON_RX_TIMEOUT		(1 << 7)	/* RX timeout enable */
#define UCON_RX_ERR_INT		(1 << 6)	/* RX error status interrupt enable */
#define UCON_LOOPBACK		(1 << 5)	/* to enter the loop-back mode */
#define UCON_BRK_SIG		(1 << 4)	/* to send a break during 1 frame time */
#define fUCON_TX	Fld(2,2)		/* function to write Tx data
						   to the UART Tx buffer */
#define UCON_TX		FMsk(fUCON_TX)
#define UCON_TX_DIS	FInsrt(0x0, fUCON_TX)	/* Disable */
#define UCON_TX_INT	FInsrt(0x1, fUCON_TX)	/* Interrupt or polling */
#define UCON_TX_DMA0	FInsrt(0x2, fUCON_TX)	/* DMA0 request */
#define UCON_TX_DMA1	FInsrt(0x3, fUCON_TX)	/* DMA1 request */

#define fUCON_RX	Fld(2,0)		/* function to read data
						   from UART Rx buffer */
#define UCON_RX		FMsk(fUCON_RX)
#define UCON_RX_DIS	FInsrt(0x0, fUCON_RX)	/* Disable */
#define UCON_RX_INT	FInsrt(0x1, fUCON_RX)	/* Interrupt or polling */
#define UCON_RX_DMA0	FInsrt(0x2, fUCON_RX)	/* DMA0 request */
#define UCON_RX_DMA1	FInsrt(0x3, fUCON_RX)	/* DMA1 request */
/* UART FIFO control register */
#define fUFCON_TX_TR	Fld(2,6)	/* trigger level of transmit FIFO */
#define UFCON_TX_TR	FMsk(fUFCON_TX_TR)
#define UFCON_TX_TR0	FInsrt(0x0, fUFCON_TX_TR)	/* Empty */
#define UFCON_TX_TR4	FInsrt(0x1, fUFCON_TX_TR)	/* 4-byte */
#define UFCON_TX_TR8	FInsrt(0x2, fUFCON_TX_TR)	/* 8-byte */
#define UFCON_TX_TR12	FInsrt(0x3, fUFCON_TX_TR)	/* 12-byte */
#define fUFCON_RX_TR	Fld(2,4)	/* trigger level of receive FIFO */
#define UFCON_RX_TR	FMsk(fUFCON_RX_TR)
#define UFCON_RX_TR0	FInsrt(0x0, fUFCON_RX_TR)	/* Empty */
#define UFCON_RX_TR4	FInsrt(0x1, fUFCON_RX_TR)	/* 4-byte */
#define UFCON_RX_TR8	FInsrt(0x2, fUFCON_RX_TR)	/* 8-byte */
#define UFCON_RX_TR12	FInsrt(0x3, fUFCON_RX_TR)	/* 12-byte */
#define UFCON_TX_REQ	(1 << 2)	/* auto-cleared after resetting FIFO */
#define UFCON_RX_REQ	(1 << 1)	/* auto-cleared after resetting FIFO */
#define UFCON_FIFO_EN	(1 << 0)	/* FIFO Enable */
/* UART Modem control register */
#define UMCON_AFC	(1 << 4)	/* Enable Auto Flow Control */
#define UMCON_SEND	(1 << 0)	/* when not-AFC, set nRTS 1:'L' 0:'H' level */
/* UART Tx/Rx status register */
#define UTRSTAT_TR_EMP	(1 << 2)	/* 1: Transmitter buffer & shifter register empty */
#define UTRSTAT_TX_EMP	(1 << 1)	/* Transmit buffer reg. is empty */
#define UTRSTAT_RX_RDY	(1 << 0)	/* Receive buffer reg. has data */
/* UART Rx error status register */
#define UERSTAT_BRK	(1 << 3)	/* Break receive */
#define UERSTAT_FRAME	(1 << 2)	/* Frame Error */
#define UERSTAT_PARITY	(1 << 1)	/* Parity Error */
#define UERSTAT_OVERRUN	(1 << 0)	/* Overrun Error */
/* UART FIFO status register */
#define UFSTAT_TX_FULL	(1 << 9)	/* Transmit FIFO is full */
#define UFSTAT_RX_FULL	(1 << 8)	/* Receive FIFO is full */
#define fUFSTAT_TX_CNT	Fld(4,4)	/* Number of data in Tx FIFO */
#define UFSTAT_TX_CNT	FMsk(fUFSTAT_TX_CNT)
#define fUFSTAT_RX_CNT	Fld(4,0)	/* Number of data in Rx FIFO */
#define UFSTAT_RX_CNT	FMsk(fUFSTAT_RX_CNT)
/* UART Modem status register */
#define UMSTAT_dCTS	(1 << 3)	/* see Page 11-16 */
#define UMSTAT_CTS	(1 << 0)	/* CTS(Clear to Send) signal */
/* UART TRANSMIT BUFFER REGISTER */
#define fUTXH_DATA	Fld(8,0)	/* Transmit data for UARTn */
#define UTXH_DATA	FMsk (fUTXH_DATA)
/* UART RECEIVE BUFFER REGISTER */
#define fURXH_DATA	Fld(8,0)	/* Receive data for UARTn */
#define URXH_DATA	FMsk (fURXH_DATA)
/* Baud rate divisior register */
#define fUBRDIVn	Fld(16,0)	/* Baud rate division value (> 0) */
#define UBRDIVn		FMsk(fUBRDIVn)

/* Modified from here by Samsung */
/* FIXME: These definitions duplicate, because of power management code.
 * So I want to refine these definition by somebody :-)
 */	
#define ULCON0		__REG(0x50000000)	/* R/W, UART line control register */
#define UCON0		__REG(0x50000004)	/* R/W, UART control register */
#define UFCON0		__REG(0x50000008)	/* R/W, UART FIFO control register */
#define UMCON0		__REG(0x5000000C)	/* R/W, UART modem control register */
#define UTRSTAT0	__REG(0x50000010)	/* R  , UART Tx/Rx status register */
#define UERSTAT0	__REG(0x50000014)	/* R  , UART Rx error status register */
#define UFSTAT0		__REG(0x50000018)	/* R  , UART FIFO status register */
#define UMSTAT0		__REG(0x5000001C)	/* R  , UART Modem status register */
#define UTXH0		__REG(0x50000020)	/*   W, UART transmit(little-end) buffer */
#define URXH0		__REG(0x50000024)	/* R  , UART receive(little-end) buffer */
#define UBRDIV0		__REG(0x50000028)	/* R/W, Baud rate divisor register */
/* Modified to here by Samsung */

/* Interrupts */


#define INT_CTL_BASE		0x4a000000
/* Registers */
#define SRCPND			__REG(INT_CTL_BASE+0x00)
#define INTMOD			__REG(INT_CTL_BASE+0x04)
#define INTMSK			__REG(INT_CTL_BASE+0x08)
#define PRIORITY		__REG(INT_CTL_BASE+0x0c)
#define INTPND			__REG(INT_CTL_BASE+0x10)
#define INTOFFSET		__REG(INT_CTL_BASE+0x14)
#define SUBSRCPND		__REG(INT_CTL_BASE+0x18)
#define INTSUBMSK		__REG(INT_CTL_BASE+0x1c)

#define INT_ADCTC		(1 << 31)	/* ADC EOC and Touch interrupt (INT_ADC/INT_TC) */
#define INT_RTC			(1 << 30)	/* RTC alarm interrupt */
#define INT_SPI1		(1 << 29)	/* SPI1 interrupt */
#define INT_UART0		(1 << 28)	/* UART0 Interrupt (ERR, RXD, and TXD) */
#define INT_IIC			(1 << 27)	/* IIC interrupt */
#define INT_USBH		(1 << 26)	/* USB host interrupt */
#define INT_USBD		(1 << 25)	/* USB device interrupt */
#define INT_RESERVED24		(1 << 24)
#define INT_UART1		(1 << 23)	/* UART1 Interrupt (ERR, RXD, and TXD) */
#define INT_SPI0		(1 << 22)	/* SPI0 interrupt */
#define INT_SDI			(1 << 21)	/* SDI interrupt */
#define INT_DMA3		(1 << 20)	/* DMA channel 3 interrupt */
#define INT_DMA2		(1 << 19)	/* DMA channel 2 interrupt */
#define INT_DMA1		(1 << 18)	/* DMA channel 1 interrupt */
#define INT_DMA0		(1 << 17)	/* DMA channel 0 interrupt */
#define INT_LCD			(1 << 16)	/* LCD interrupt (INT_FrSyn and INT_FiCnt) */
#define INT_UART2		(1 << 15)	/* UART2 Interrupt (ERR, RXD, and TXD)  */
#define INT_TIMER4		(1 << 14)	/* Timer 4 interrupt */
#define INT_TIMER3		(1 << 13)	/* Timer 3 interrupt */
#define INT_TIMER2		(1 << 12)	/* Timer 2 interrupt */
#define INT_TIMER1		(1 << 11)	/* Timer 1 interrupt */
#define INT_TIMER0		(1 << 10)	/* Timer 0 interrupt */
#define INT_WDT			(1 << 9)	/* Watch-Dog timer interrupt */
#define INT_TICK		(1 << 8)	/* RTC time tick interrupt  */
#define INT_nBAT_FLT		(1 << 7)	/* Battery Fault interrupt */
#define INT_RESERVED6		(1 << 6)	/* Reserved for future use */
#define INT_EINT8_23		(1 << 5)	/* External interrupt 8 ~ 23 */
#define INT_EINT4_7		(1 << 4)	/* External interrupt 4 ~ 7 */
#define INT_EINT3		(1 << 3)	/* External interrupt 3 */
#define INT_EINT2		(1 << 2)	/* External interrupt 2 */
#define INT_EINT1		(1 << 1)	/* External interrupt 1 */
#define INT_EINT0		(1 << 0)	/* External interrupt 0 */

#define INT_ADC			(1 << 10)
#define INT_TC			(1 << 9)
#define INT_ERR2		(1 << 8)
#define INT_TXD2		(1 << 7)
#define INT_RXD2		(1 << 6)
#define INT_ERR1		(1 << 5)
#define INT_TXD1		(1 << 4)
#define INT_RXD1		(1 << 3)
#define INT_ERR0		(1 << 2)
#define INT_TXD0		(1 << 1)
#define INT_RXD0		(1 << 0)

/* Real Time Clock */
/* Registers */
#define RTC_CTL_BASE            0x57000000
#define RTCCON                  __REG(RTC_CTL_BASE+0x40)
#define TICNT                   __REG(RTC_CTL_BASE+0x44)
#define RTCALM                  __REG(RTC_CTL_BASE+0x50)
#define ALMSEC                  __REG(RTC_CTL_BASE+0x54)
#define ALMMIN                  __REG(RTC_CTL_BASE+0x58)
#define ALMHOUR                 __REG(RTC_CTL_BASE+0x5c)
#define ALMDATE                 __REG(RTC_CTL_BASE+0x60)
#define ALMMON                  __REG(RTC_CTL_BASE+0x64)
#define ALMYEAR                 __REG(RTC_CTL_BASE+0x68)
#define RTCRST                  __REG(RTC_CTL_BASE+0x6c)
#define BCDSEC                  __REG(RTC_CTL_BASE+0x70)
#define BCDMIN                  __REG(RTC_CTL_BASE+0x74)
#define BCDHOUR                 __REG(RTC_CTL_BASE+0x78)
#define BCDDATE                 __REG(RTC_CTL_BASE+0x7c)
#define BCDDAY                  __REG(RTC_CTL_BASE+0x80)
#define BCDMON                  __REG(RTC_CTL_BASE+0x84)
#define BCDYEAR                 __REG(RTC_CTL_BASE+0x88)
/* Fields */
#define fRTC_SEC                Fld(7,0)
#define fRTC_MIN                Fld(7,0)
#define fRTC_HOUR               Fld(6,0)
#define fRTC_DATE               Fld(6,0)
#define fRTC_DAY                Fld(2,0)
#define fRTC_MON                Fld(5,0)
#define fRTC_YEAR               Fld(8,0)
/* Mask */
#define Msk_RTCSEC              FMsk(fRTC_SEC)
#define Msk_RTCMIN              FMsk(fRTC_MIN)
#define Msk_RTCHOUR             FMsk(fRTC_HOUR)
#define Msk_RTCDATE             FMsk(fRTC_DATE)
#define Msk_RTCDAY              FMsk(fRTC_DAY)
#define Msk_RTCMON              FMsk(fRTC_MON)
#define Msk_RTCYEAR             FMsk(fRTC_YEAR)
/* bits */
#define RTCCON_EN               (1 << 0) /* RTC Control Enable */
#define RTCCON_CLKSEL           (1 << 1) /* BCD clock as XTAL 1/2^25 clock */
#define RTCCON_CNTSEL           (1 << 2) /* 0: Merge BCD counters */
#define RTCCON_CLKRST           (1 << 3) /* RTC clock count reset */
/* RTC Alarm */
#define RTCALM_GLOBAL           (1 << 6) /* Global alarm enable */
#define RTCALM_YEAR             (1 << 5) /* Year alarm enable */
#define RTCALM_MON              (1 << 4) /* Month alarm enable */
#define RTCALM_DAY              (1 << 3) /* Day alarm enable */
#define RTCALM_HOUR             (1 << 2) /* Hour alarm enable */
#define RTCALM_MIN              (1 << 1) /* Minute alarm enable */
#define RTCALM_SEC              (1 << 0) /* Second alarm enable */
#define RTCALM_EN               (RTCALM_GLOBAL | RTCALM_YEAR | RTCALM_MON |\
                                RTCALM_DAY | RTCALM_HOUR | RTCALM_MIN |\
                                RTCALM_SEC)
#define RTCALM_DIS              (~RTCALM_EN)

#define PWM_CTL_BASE            0x51000000
/* Registers */
#define TCFG0                   __REG(PWM_CTL_BASE+0x00)
#define TCFG1                   __REG(PWM_CTL_BASE+0x04)
#define TCON                    __REG(PWM_CTL_BASE+0x08)
#define TCNTB0                  __REG(PWM_CTL_BASE+0x0c)        
#define TCMPB0                  __REG(PWM_CTL_BASE+0x10)        
#define TCNTO0                  __REG(PWM_CTL_BASE+0x14)
#define TCNTB1                  __REG(PWM_CTL_BASE+0x18)
#define TCMPB1                  __REG(PWM_CTL_BASE+0x1c)
#define TCNTO1                  __REG(PWM_CTL_BASE+0x20)
#define TCNTB2                  __REG(PWM_CTL_BASE+0x24)
#define TCMPB2                  __REG(PWM_CTL_BASE+0x28)
#define TCNTO2                  __REG(PWM_CTL_BASE+0x2c)
#define TCNTB3                  __REG(PWM_CTL_BASE+0x30)
#define TCMPB3                  __REG(PWM_CTL_BASE+0x34)
#define TCNTO3                  __REG(PWM_CTL_BASE+0x38)
#define TCNTB4                  __REG(PWM_CTL_BASE+0x3c)
#define TCNTO4                  __REG(PWM_CTL_BASE+0x40)
/* Fields */
#define fTCFG0_DZONE            Fld(8,16)       /* the dead zone length (= timer 0) */
#define fTCFG0_PRE1             Fld(8,8)        /* prescaler value for time 2,3,4 */
#define fTCFG0_PRE0             Fld(8,0)        /* prescaler value for time 0,1 */
#define fTCON_TIMER0    Fld(5,0)
#define fTCON_TIMER1    Fld(4,8)
#define fTCON_TIMER2    Fld(4,12)
#define fTCON_TIMER3    Fld(4,16)
#define fTCON_TIMER4    Fld(3,20)

/* bits */
#define TCFG0_DZONE(x)          FInsrt((x), fTCFG0_DZONE)
#define TCFG0_PRE1(x)           FInsrt((x), fTCFG0_PRE1)
#define TCFG0_PRE0(x)           FInsrt((x), fTCFG0_PRE0)

#define TCON_4_AUTO             (1 << 22)       /* auto reload on/off for Timer 4 */
#define TCON_4_UPDATE           (1 << 21)       /* manual Update TCNTB4 */
#define TCON_4_ONOFF            (1 << 20)       /* 0: Stop, 1: start Timer 4 */
#define COUNT_4_ON              (TCON_4_ONOFF*1)
#define COUNT_4_OFF             (TCON_4_ONOFF*0)
#define TCON_3_AUTO     (1 << 19)       /* auto reload on/off for Timer 3 */
#define TCON_3_INVERT   (1 << 18)       /* 1: Inverter on for TOUT3 */
#define TCON_3_MAN      (1 << 17)       /* manual Update TCNTB3,TCMPB3 */
#define TCON_3_ONOFF    (1 << 16)       /* 0: Stop, 1: start Timer 3 */
#define TCON_2_AUTO     (1 << 15)       /* auto reload on/off for Timer 3 */
#define TCON_2_INVERT   (1 << 14)       /* 1: Inverter on for TOUT3 */
#define TCON_2_MAN      (1 << 13)       /* manual Update TCNTB3,TCMPB3 */
#define TCON_2_ONOFF    (1 << 12)       /* 0: Stop, 1: start Timer 3 */
#define TCON_1_AUTO     (1 << 11)       /* auto reload on/off for Timer 3 */
#define TCON_1_INVERT   (1 << 10)       /* 1: Inverter on for TOUT3 */
#define TCON_1_MAN      (1 << 9)       /* manual Update TCNTB3,TCMPB3 */
#define TCON_1_ONOFF    (1 << 8)       /* 0: Stop, 1: start Timer 3 */
#define TCON_0_AUTO     (1 << 3)       /* auto reload on/off for Timer 3 */
#define TCON_0_INVERT   (1 << 2)       /* 1: Inverter on for TOUT3 */
#define TCON_0_MAN      (1 << 1)       /* manual Update TCNTB3,TCMPB3 */
#define TCON_0_ONOFF    (1 << 0)       /* 0: Stop, 1: start Timer 3 */

#define TIMER3_ATLOAD_ON        (TCON_3_AUTO*1)
#define TIMER3_ATLAOD_OFF       FClrBit(TCON, TCON_3_AUTO)
#define TIMER3_IVT_ON   (TCON_3_INVERT*1)
#define TIMER3_IVT_OFF  (FClrBit(TCON, TCON_3_INVERT))
#define TIMER3_MANUP    (TCON_3_MAN*1)
#define TIMER3_NOP      (FClrBit(TCON, TCON_3_MAN))
#define TIMER3_ON       (TCON_3_ONOFF*1)
#define TIMER3_OFF      (FClrBit(TCON, TCON_3_ONOFF))
#define TIMER2_ATLOAD_ON        (TCON_2_AUTO*1)
#define TIMER2_ATLAOD_OFF       FClrBit(TCON, TCON_2_AUTO)
#define TIMER2_IVT_ON   (TCON_2_INVERT*1)
#define TIMER2_IVT_OFF  (FClrBit(TCON, TCON_2_INVERT))
#define TIMER2_MANUP    (TCON_2_MAN*1)
#define TIMER2_NOP      (FClrBit(TCON, TCON_2_MAN))
#define TIMER2_ON       (TCON_2_ONOFF*1)
#define TIMER2_OFF      (FClrBit(TCON, TCON_2_ONOFF))
#define TIMER1_ATLOAD_ON        (TCON_1_AUTO*1)
#define TIMER1_ATLAOD_OFF       FClrBit(TCON, TCON_1_AUTO)
#define TIMER1_IVT_ON   (TCON_1_INVERT*1)
#define TIMER1_IVT_OFF  (FClrBit(TCON, TCON_1_INVERT))
#define TIMER1_MANUP    (TCON_1_MAN*1)
#define TIMER1_NOP      (FClrBit(TCON, TCON_1_MAN))
#define TIMER1_ON       (TCON_1_ONOFF*1)
#define TIMER1_OFF      (FClrBit(TCON, TCON_1_ONOFF))
#define TIMER0_ATLOAD_ON        (TCON_0_AUTO*1)
#define TIMER0_ATLAOD_OFF       FClrBit(TCON, TCON_0_AUTO)
#define TIMER0_IVT_ON   (TCON_0_INVERT*1)
#define TIMER0_IVT_OFF  (FClrBit(TCON, TCON_0_INVERT))

#define TCON_TIMER1_CLR   FClrFld(TCON, fTCON_TIMER1);
#define TCON_TIMER2_CLR   FClrFld(TCON, fTCON_TIMER2);
#define TCON_TIMER3_CLR   FClrFld(TCON, fTCON_TIMER3);

#define LCD_CTL_BASE     0x4d000000
#define LCDCON1		__REG(LCD_CTL_BASE + 0x00)	/* LCD CTL1 */
#define LCDCON2		__REG(LCD_CTL_BASE + 0x04) 	/* LCD CTL2 */
#define LCDCON3		__REG(LCD_CTL_BASE + 0x08) 	/* LCD CTL3 */
#define LCDCON4		__REG(LCD_CTL_BASE + 0x0c) 	/* LCD CTL4 */
#define LCDCON5		__REG(LCD_CTL_BASE + 0x10) 	/* LCD CTL5 */
#define LCDADDR1	__REG(LCD_CTL_BASE + 0x14) 	/* (ST) Frame Buffer Start Addr 1 */
#define LCDADDR2	__REG(LCD_CTL_BASE + 0x18) 	/* (ST) Frame Buffer Start Addr 2 */
#define LCDADDR3	__REG(LCD_CTL_BASE + 0x1c) 	/* (ST) Visual Screen Address Set */
#define REDLUT		__REG(LCD_CTL_BASE + 0x20) 	/* (S) Red Lookup Table */
#define GREENLUT	__REG(LCD_CTL_BASE + 0x24) 	/* (S) G LUT */
#define BLUELUT		__REG(LCD_CTL_BASE + 0x28) 	/* (S) B LUT */
#define DITHMODE	__REG(LCD_CTL_BASE + 0x4c) 	/* (S) Dithering MOde */
#define TPAL		__REG(LCD_CTL_BASE + 0x50) 	/* (T) Temp. Palette */
#define LCDINTPND	__REG(LCD_CTL_BASE + 0x54) 	/* LCD INT Pending */
#define LCDSRCPND	__REG(LCD_CTL_BASE + 0x58) 	/* LCD INT SRC */
#define LCDINTMSK	__REG(LCD_CTL_BASE + 0x5c) 	/* LCD INT Mask */
#define LCDLPCSEL	__REG(LCD_CTL_BASE + 0x60) 	/* LPC3600 CTL */

#define fLCD1_LINECNT	Fld(10,18)	/* the status of the line counter */
#define LCD1_LINECNT	FMsk(fLCD_LINECNT)

#define fLCD1_CLKVAL	Fld(10,8)	/* rates of VCLK and CLKVAL[9:0] */
#define LCD1_CLKVAL(x)	FInsrt((x), fLCD1_CLKVAL)
#define LCD1_CLKVAL_MSK	FMask(fLCD1_CLKVAL)

#define LCD1_MMODE (1<<7)

#define fLCD1_PNR	Fld(2,5)	/* select the display mode */
#define LCD1_PNR_4D	FInsrt(0x0, fLCD1_PNR)	/* STN: 4-bit dual scan */
#define LCD1_PNR_4S	FInsrt(0x1, fLCD1_PNR)	/* STN: 4-bit single scan */
#define LCD1_PNR_8S	FInsrt(0x2, fLCD1_PNR)	/* STN: 8-bit single scan */
#define LCD1_PNR_TFT	FInsrt(0x3, fLCD1_PNR)	/* TFT LCD */

#define fLCD1_BPP	Fld(4,1)	/* select BPP(Bit Per Pixel) */
#define LCD1_BPP_1S	FInsrt(0x0, fLCD1_BPP)	/* STN: 1 bpp, mono */
#define LCD1_BPP_2S	FInsrt(0x1, fLCD1_BPP)	/* STN: 2 bpp, 4-grey */
#define LCD1_BPP_4S	FInsrt(0x2, fLCD1_BPP)	/* STN: 4 bpp, 16-grey */
#define LCD1_BPP_8S	FInsrt(0x3, fLCD1_BPP)	/* STN: 8 bpp, color */
#define LCD1_BPP_12S	FInsrt(0x4, fLCD1_BPP)	/* STN: 12 bpp, color */
#define LCD1_BPP_1T	FInsrt(0x8, fLCD1_BPP)	/* TFT: 1 bpp */
#define LCD1_BPP_2T	FInsrt(0x9, fLCD1_BPP)	/* TFT: 2 bpp */
#define LCD1_BPP_4T	FInsrt(0xa, fLCD1_BPP)	/* TFT: 4 bpp */
#define LCD1_BPP_8T	FInsrt(0xb, fLCD1_BPP)	/* TFT: 8 bpp */
#define LCD1_BPP_16T	FInsrt(0xc, fLCD1_BPP)	/* TFT: 16 bpp */

#define LCD1_ENVID	(1 << 0)	/* 1: Enable the video output */

#define fLCD2_VBPD	Fld(8,24)	/* TFT: (Vertical Back Porch)
	   # of inactive lines at the start of a frame,
	   after vertical synchronization period. */
#define LCD2_VBPD(x)	FInsrt((x), fLCD2_VBPD)

#define fLCD2_LINEVAL	Fld(10,14)	/* TFT/STN: vertical size of LCD */
#define LCD2_LINEVAL(x)	FInsrt((x), fLCD2_LINEVAL)
#define LCD2_LINEVAL_MSK	FMsk(fLCD2_LINEVAL)

#define fLCD2_VFPD	Fld(8,6)	/* TFT: (Vertical Front Porch)
					   # of inactive lines at the end of a frame,
					   before vertical synchronization period. */
#define LCD2_VFPD(x)	FInsrt((x), fLCD2_VFPD)

#define fLCD2_VSPW	Fld(6,0)	/* TFT: (Vertical Sync Pulse Width)
					   the VSYNC pulse's high level width
					   by counting the # of inactive lines */
#define LCD2_VSPW(x)	FInsrt((x), fLCD2_VSPW)

#define fLCD3_HBPD	Fld(7,19)	/* TFT: (Horizontal Back Porch)
					   # of VCLK periods between the falling edge of HSYNC
					   and the start of active data */
#define LCD3_HBPD(x)	FInsrt((x), fLCD3_HBPD)

#define fLCD3_WDLY	Fld(7,19)	/* STN: delay between VLINE and
					   VCLK by counting the # of the HCLK */
#define LCD3_WDLY	FMsk(fLCD3_WDLY)
#define LCD3_WDLY	FMsk(fLCD3_WDLY)
#define LCD3_WDLY_16	FInsrt(0x0, fLCD3_WDLY)	/* 16 clock */
#define LCD3_WDLY_32	FInsrt(0x1, fLCD3_WDLY)	/* 32 clock */
#define LCD3_WDLY_64	FInsrt(0x2, fLCD3_WDLY)	/* 64 clock */
#define LCD3_WDLY_128	FInsrt(0x3, fLCD3_WDLY)	/* 128 clock */

#define fLCD3_HOZVAL	Fld(11,8)	/* horizontal size of LCD */
#define LCD3_HOZVAL(x)	FInsrt((x), fLCD3_HOZVAL)
#define LCD3_HOZVAL_MSK	FMsk(fLCD3_HOZVAL)

#define fLCD3_HFPD	Fld(8,0)	/* TFT: (Horizontal Front Porch)
					   # of VCLK periods between the end of active date
					   and the rising edge of HSYNC */
#define LCD3_HFPD(x)	FInsrt((x), fLCD3_HFPD)

#define fLCD3_LINEBLNK	Fld(8,0)	/* STN: the blank time
					   in one horizontal line duration time.
					   the unit of LINEBLNK is HCLK x 8 */
#define LCD3_LINEBLNK	FInsrt(fLCD3_LINEBLNK)

#define fLCD4_MVAL	Fld(8,8)	/* STN: the rate at which the VM signal
					   will toggle if the MMODE bit is set logic '1' */
#define LCD4_MVAL(x)	FInsrt((x), fLCD4_MVAL)

#define fLCD4_HSPW	Fld(8,0)	/* TFT: (Horizontal Sync Pulse Width)
					   HSYNC pulse's high lvel width by counting the # of the VCLK */
#define LCD4_HSPW(x)	FInsrt((x), fLCD4_HSPW)

#define fLCD4_WLH	Fld(8,0)	/* STN: VLINE pulse's high level width
					   by counting the # of the HCLK */
#define LCD4_WLH(x)	FInsrt((x), fLCD4_WLH)
#define LCD4_WLH_16	FInsrt(0x0, fLCD4_WLH)	/* 16 clock */
#define LCD4_WLH_32	FInsrt(0x1, fLCD4_WLH)	/* 32 clock */
#define LCD4_WLH_64	FInsrt(0x2, fLCD4_WLH)	/* 64 clock */
#define LCD4_WLH_128	FInsrt(0x3, fLCD4_WLH)	/* 128 clock */

#define fLCD5_VSTAT	Fld(2,19)	/* TFT: Vertical Status (ReadOnly) */
#define LCD5_VSTAT	FMsk(fLCD5_VSTAT)
#define LCD5_VSTAT_VS	0x00	/* VSYNC */
#define LCD5_VSTAT_BP	0x01	/* Back Porch */
#define LCD5_VSTAT_AC	0x02	/* Active */
#define LCD5_VSTAT_FP	0x03	/* Front Porch */

#define fLCD5_HSTAT	Fld(2,17)	/* TFT: Horizontal Status (ReadOnly) */
#define LCD5_HSTAT	FMsk(fLCD5_HSTAT)
#define LCD5_HSTAT_HS	0x00	/* HSYNC */
#define LCD5_HSTAT_BP	0x01	/* Back Porch */
#define LCD5_HSTAT_AC	0x02	/* Active */
#define LCD5_HSTAT_FP	0x03	/* Front Porch */

#define LCD5_BPP24BL	(1 << 12)
#define LCD5_FRM565	(1 << 11)
#define LCD5_INVVCLK	(1 << 10)	/* STN/TFT :
					   1 : video data is fetched at VCLK falling edge
					   0 : video data is fetched at VCLK rising edge */
#define LCD5_INVVLINE	(1 << 9)	/* STN/TFT :
					   1 : VLINE/HSYNC pulse polarity is inverted */
#define LCD5_INVVFRAME	(1 << 8)	/* STN/TFT :
					   1 : VFRAME/VSYNC pulse polarity is inverted */
#define LCD5_INVVD	(1 << 7)	/* STN/TFT :
					   1 : VD (video data) pulse polarity is inverted */
#define LCD5_INVVDEN	(1 << 6)	/* TFT :
					   1 : VDEN signal polarity is inverted */
#define LCD5_INVPWREN	(1 << 5)
#define LCD5_INVLEND	(1 << 4)	/* TFT : 1 : LEND signal polarity is inverted */
#define LCD5_PWREN	(1 << 3)
#define LCD5_LEND	(1 << 2)	/* TFT,1 : Enable LEND signal */
#define LCD5_BSWP	(1 << 1)	/* STN/TFT,1 : Byte swap enable */
#define LCD5_HWSWP	(1 << 0)	/* STN/TFT,1 : HalfWord swap enable */

#define fLCDADDR_BANK	Fld(9,21)	/* bank location for video buffer */
#define LCDADDR_BANK(x)	FInsrt((x), fLCDADDR_BANK)

#define fLCDADDR_BASEU	Fld(21,0)	/* address of upper left corner */
#define LCDADDR_BASEU(x)	FInsrt((x), fLCDADDR_BASEU)

#define fLCDADDR_BASEL	Fld(21,0)	/* address of lower right corner */
#define LCDADDR_BASEL(x)	FInsrt((x), fLCDADDR_BASEL)

#define fLCDADDR_OFFSET	Fld(11,11)	/* Virtual screen offset size (# of half words) */
#define LCDADDR_OFFSET(x)	FInsrt((x), fLCDADDR_OFFSET)

#define fLCDADDR_PAGE	Fld(11,0)	/* Virtual screen page width (# of half words) */
#define LCDADDR_PAGE(x)	FInsrt((x), fLCDADDR_PAGE)

#define TPAL_LEN	(1 << 24)	/* 1 : Temp. Pallete Register enable */
#define fTPAL_VAL	Fld(24,0)	/* Temp. Pallete Register value */
#define TPAL_VAL(x)	FInsrt((x), fTPAL_VAL)
#define TPAL_VAL_RED(x)	FInsrt((x), Fld(8,16))
#define TPAL_VAL_GREEN(x)	FInsrt((x), Fld(8,8))
#define TPAL_VAL_BLUE(x)	FInsrt((x), Fld(8,0))
/* Touch Screen and ADC

#define ADC_CTL_BASE            0x58000000
/
#define ADCCON                  __REG(ADC_CTL_BASE+0x00)
#define ADCTSC                  __REG(ADC_CTL_BASE+0x04)
#define ADCDLY                  __REG(ADC_CTL_BASE+0x08)
#define ADCDAT0                 __REG(ADC_CTL_BASE+0x0c)
#define ADCDAT1                 __REG(ADC_CTL_BASE+0x10)

#define fADCCON_PRSCVL          Fld(8, 6)
#define fADCCON_INPUT           Fld(3, 3)
#define fTSC_XY_PST             Fld(2, 0)
#define fADC_DELAY              Fld(16, 0)
#define fDAT_UPDOWN             Fld(1, 15)
#define fDAT_AUTO_PST           Fld(1, 14)
#define fDAT_XY_PST             Fld(2, 12)
#define fDAT_XPDATA             Fld(10, 0)
#define fDAT_YPDATA             Fld(10, 0)
*/

/* ADC and Touch Screen Interface */
#define ADC_CTL_BASE    0x58000000
#define bADC_CTL(Nb)    __REG(ADC_CTL_BASE + (Nb))
/* Offset */
#define oADCCON     0x00  /* R/W, ADC control register */
#define oADCTSC     0x04  /* R/W, ADC touch screen ctl reg */
#define oADCDLY     0x08  /* R/W, ADC start or interval delay reg */
#define oADCDAT0    0x0c  /* R  , ADC conversion data reg */
#define oADCDAT1    0x10  /* R  , ADC conversion data reg */

#define oADC_UPDN         0x14    /* R/W Stylus Up or Down Interrpt status register*/
#define TSC_UP       0x02
#define TSC_DN       0x01

/* Registers */
#define ADCCON      bADC_CTL(oADCCON)
#define ADCTSC      bADC_CTL(oADCTSC)
#define ADCDLY      bADC_CTL(oADCDLY)
#define ADCDAT0     bADC_CTL(oADCDAT0)
#define ADCDAT1     bADC_CTL(oADCDAT1)

#define PEN_UPDN    bADC_CTL(oADC_UPDN)

/* Field */
#define fADCCON_PRSCVL    Fld(8, 6)
#define fADCCON_INPUT   Fld(3, 3)
#define fTSC_XY_PST   Fld(2, 0)
#define fADC_DELAY    Fld(6, 0)
#define fDAT_UPDOWN   Fld(1, 15)
#define fDAT_AUTO_PST   Fld(1, 14)
#define fDAT_XY_PST   Fld(2, 12)
#define fDAT_XPDATA   Fld(10, 0)
#define fDAT_YPDATA   Fld(10, 0)

#define ADC_IN0                 0
#define ADC_IN1                 1
#define ADC_IN2                 2
#define ADC_IN3                 3
#define ADC_IN4                 4
#define ADC_IN5                 5
#define ADC_IN6                 6
#define ADC_IN7                 7
#define ADC_BUSY    1
#define ADC_READY   0
#define NOP_MODE    0
#define X_AXIS_MODE   1
#define Y_AXIS_MODE   2
#define WAIT_INT_MODE   3
#define ADCCON_ECFLG    (1 << 15)
#define PRESCALE_ENDIS    (1 << 14)
#define PRESCALE_DIS    (PRESCALE_ENDIS*0)
#define PRESCALE_EN   (PRESCALE_ENDIS*1)

#define PRSCVL(x)   (x << 6)
#define ADC_INPUT(x)    (x << 3)
#define ADCCON_STDBM    (1 << 2)        /* 1: standby mode, 0: normal mode */
#define ADC_NORMAL_MODE   FClrBit(ADCCON, ADCCON_STDBM)
#define ADC_STANDBY_MODE  (ADCCON_STDBM*1)
#define ADCCON_READ_START (1 << 1)
#define ADC_START_BY_RD_DIS FClrBit(ADCCON, ADCCON_READ_START)
#define ADC_START_BY_RD_EN  (ADCCON_READ_START*1)
#define ADC_START   (1 << 0)

#define UD_SEN      (1 << 8)
#define DOWN_INT    (UD_SEN*0)
#define UP_INT      (UD_SEN*1)
#define YM_SEN      (1 << 7)
#define YM_HIZ      (YM_SEN*0)
#define YM_GND      (YM_SEN*1)
#define YP_SEN      (1 << 6)
#define YP_EXTVLT   (YP_SEN*0)
#define YP_AIN      (YP_SEN*1)
#define XM_SEN      (1 << 5)
#define XM_HIZ      (XM_SEN*0)
#define XM_GND      (XM_SEN*1)
#define XP_SEN      (1 << 4)
#define XP_EXTVLT   (XP_SEN*0)
#define XP_AIN      (XP_SEN*1)
#define XP_PULL_UP    (1 << 3)
#define XP_PULL_UP_EN   (XP_PULL_UP*0)
#define XP_PULL_UP_DIS    (XP_PULL_UP*1)
#define AUTO_PST    (1 << 2)
#define CONVERT_MAN   (AUTO_PST*0)
#define CONVERT_AUTO    (AUTO_PST*1)
#define XP_PST(x)   (x << 0)

/* Modified from here by Samsung */
#define ADCCON_STDBM		(1 << 2)
/* Modified to here by Samsung */

/* DMA */
#define DMA_CTL_BASE	0x4b000000
#define bDMA_CTL(Nb,x)	__REG(DMA_CTL_BASE + (0x40*Nb) + (x))
/* DMA channel 0 */
#define DISRC0			__REG(DMA_CTL_BASE+0x00)
#define DISRCC0			__REG(DMA_CTL_BASE+0x04)
#define DIDST0			__REG(DMA_CTL_BASE+0x08)
#define DIDSTC0			__REG(DMA_CTL_BASE+0x0c)
#define DCON0			__REG(DMA_CTL_BASE+0x10)
#define DSTAT0			__REG(DMA_CTL_BASE+0x14)
#define DCSRC0			__REG(DMA_CTL_BASE+0x18)
#define DCDST0			__REG(DMA_CTL_BASE+0x1c)
#define DMTRIG0			__REG(DMA_CTL_BASE+0x20)
/* DMA channel 1 */
#define DISRC1			__REG(DMA_CTL_BASE+0x40)
#define DISRCC1			__REG(DMA_CTL_BASE+0x44)
#define DIDST1			__REG(DMA_CTL_BASE+0x48)
#define DIDSTC1			__REG(DMA_CTL_BASE+0x4c)
#define DCON1			__REG(DMA_CTL_BASE+0x50)
#define DSTAT1			__REG(DMA_CTL_BASE+0x54)
#define DCSRC1			__REG(DMA_CTL_BASE+0x58)
#define DCDST1			__REG(DMA_CTL_BASE+0x5c)
#define DMTRIG1			__REG(DMA_CTL_BASE+0x60)
/* DMA channel 2 */
#define DISRC2			__REG(DMA_CTL_BASE+0x80)
#define DISRCC2			__REG(DMA_CTL_BASE+0x84)
#define DIDST2			__REG(DMA_CTL_BASE+0x88)
#define DIDSTC2			__REG(DMA_CTL_BASE+0x8c)
#define DCON2			__REG(DMA_CTL_BASE+0x90)
#define DSTAT2			__REG(DMA_CTL_BASE+0x94)
#define DCSRC2			__REG(DMA_CTL_BASE+0x98)
#define DCDST2			__REG(DMA_CTL_BASE+0x9c)
#define DMTRIG2			__REG(DMA_CTL_BASE+0xa0)
/* DMA channel 3 */
#define DISRC3			__REG(DMA_CTL_BASE+0xc0)
#define DISRCC3			__REG(DMA_CTL_BASE+0xc4)
#define DIDST3			__REG(DMA_CTL_BASE+0xc8)
#define DIDSTC3			__REG(DMA_CTL_BASE+0xcc)
#define DCON3			__REG(DMA_CTL_BASE+0xd0)
#define DSTAT3			__REG(DMA_CTL_BASE+0xd4)
#define DCSRC3			__REG(DMA_CTL_BASE+0xd8)
#define DCDST3			__REG(DMA_CTL_BASE+0xdc)
#define DMTRIG3			__REG(DMA_CTL_BASE+0xe0)

/* DISRC, DIDST Control registers */
#define fDMA_BASE_ADDR		Fld(30, 0)      /* base address of src/dst data */
#define DMA_BASE_ADDR(x)	FInsrt(x, fDMA_BASE_ADDR)
#define LOC_SRC			(1 << 1)	/* select the location of source */
#define ON_AHB			(LOC_SRC*0)
#define ON_APB			(LOC_SRC*1)
#define ADDR_MODE		(1 << 0)       /* select the address increment */
#define ADDR_INC		(ADDR_MODE*0)
#define ADDR_FIX		(ADDR_MODE*1)

/* DCON Definitions */
#define DCON_MODE		(1 << 31)	/* 0: demand, 1: handshake */
#define DEMAND_MODE		(DCON_MODE*0)
#define HS_MODE			(DCON_MODE*1)
#define DCON_SYNC		(1 << 30)       /* sync to 0:PCLK, 1:HCLK */
#define SYNC_PCLK		(DCON_SYNC*0)
#define SYNC_HCLK		(DCON_SYNC*1)
#define DCON_INT		(1 << 29)
#define POLLING_MODE		(DCON_INT*0)
#define INT_MODE		(DCON_INT*1)
#define DCON_TSZ		(1 << 28)	/* tx size 0: a unit, 1: burst */
#define TSZ_UNIT		(DCON_TSZ*0)
#define TSZ_BURST		(DCON_TSZ*1)
#define DCON_SERVMODE		(1 << 27)	/* 0: single, 1: whole service */
#define SINGLE_SERVICE		(DCON_SERVMODE*0)
#define WHOLE_SERVICE		(DCON_SERVMODE*1)
#define fDCON_HWSRC		Fld(3, 24)	/* select request source */
#define CH0_nXDREQ0		0
#define CH0_UART0		1
#define CH0_MMC			2
#define CH0_TIMER		3
#define CH0_USBEP1		4
#define CH1_nXDREQ1		0
#define CH1_UART1		1
#define CH1_I2SSDI		2
#define CH1_SPI			3
#define CH1_USBEP2		4
#define CH2_I2SSDO		0
#define CH2_I2SSDI		1
#define CH2_MMC			2
#define CH2_TIMER		3
#define CH2_USBEP3		4
#define CH3_UART2		0
#define CH3_MMC			1
#define CH3_SPI			2
#define CH3_TIMER		3
#define CH3_USBEP4		4
#define HWSRC(x)		FInsrt(x, fDCON_HWSRC)
#define DCON_SWHW_SEL		(1 << 23)	/* DMA src 0: s/w 1: h/w */
#define DMA_SRC_SW		(DCON_SWHW_SEL*0)
#define DMA_SRC_HW		(DCON_SWHW_SEL*1)
#define DCON_RELOAD		(1 << 22)	/* set auto-reload */
#define SET_ATRELOAD		(DCON_RELOAD*0)
#define CLR_ATRELOAD		(DCON_RELOAD*1)
#define fDCON_DSZ		Fld(2, 20)
#define DSZ_BYTE		0
#define DSZ_HALFWORD		1
#define DSZ_WORD		2
#define DSZ(x)			FInsrt(x, fDCON_DSZ)
#define readDSZ(x)		FExtr(x, fDCON_DSZ)
#define fDCON_TC		Fld(20,0)
#define TX_CNT(x)		FInsrt(x, fDCON_TC)
/* STATUS Register Definitions  */
#define fDSTAT_ST		Fld(2,20)	/* Status of DMA Controller */
#define fDSTAT_TC		Fld(20,0)	/* Current value of transfer count */
#define DMA_STATUS(chan)	FExtr((DSTAT0 + (0x20 * chan)), fDSTAT_ST)
#define DMA_BUSY		(1 << 0)
#define DMA_READY		(0 << 0)
#define DMA_CURR_TC(chan)	FExtr((DSTAT0 + (0x20 * chan)), fDSTAT_TC)      
/* DMA Trigger Register Definitions */
#define DMASKTRIG_STOP		(1 << 2)	/* Stop the DMA operation */
#define DMA_STOP		(DMASKTRIG_STOP*1)
#define DMA_STOP_CLR		(DMASKTRIG_STOP*0)
#define DMASKTRIG_ONOFF		(1 << 1)	/* DMA channel on/off */
#define CHANNEL_ON		(DMASKTRIG_ONOFF*1)
#define CHANNEL_OFF		(DMASKTRIG_ONOFF*0)
#define DMASKTRIG_SW		(1 << 0)	/* Trigger DMA ch. in S/W req. mode */
#define DMA_SW_REQ_CLR		(DMASKTRIG_SW*0)
#define DMA_SW_REQ		(DMASKTRIG_SW*1)

/* IIS Bus Interface */
#define IIS_CTL_BASE		0x55000000
#define bIIS_CTL(Nb)		__REG(IIS_CTL_BASE + (Nb))
#define IISCON			bIIS_CTL(0x00)
#define IISMOD			bIIS_CTL(0x04)
#define IISPSR			bIIS_CTL(0x08)
#define IISFIFOC		bIIS_CTL(0x0c)
#define IISFIFOE		bIIS_CTL(0x10)

#define IISCON_CH_RIGHT (1 << 8)        /* Right channel */
#define IISCON_CH_LEFT  (0 << 8)        /* Left channel */
#define IISCON_TX_RDY   (1 << 7)        /* Transmit FIFO is ready(not empty) */
#define IISCON_RX_RDY   (1 << 6)        /* Receive FIFO is ready (not full) */
#define IISCON_TX_DMA   (1 << 5)        /* Transmit DMA service reqeust */
#define IISCON_RX_DMA   (1 << 4)        /* Receive DMA service reqeust */
#define IISCON_TX_IDLE  (1 << 3)        /* Transmit Channel idle */
#define IISCON_RX_IDLE  (1 << 2)        /* Receive Channel idle */
#define IISCON_PRESCALE (1 << 1)        /* IIS Prescaler Enable */
#define IISCON_EN       (1 << 0)        /* IIS enable(start) */

#define IISMOD_SEL_MA   (0 << 8)        /* Master mode
					                                              (IISLRCK, IISCLK are Output) */
#define IISMOD_SEL_SL   (1 << 8)        /* Slave mode
					                                              (IISLRCK, IISCLK are Input) */
#define fIISMOD_SEL_TR  Fld(2, 6)       /* Transmit/Receive mode */
#define IISMOD_SEL_TR   FMsk(fIISMOD_SEL_TR)
#define IISMOD_SEL_NO   FInsrt(0x0, fIISMOD_SEL_TR)     /* No Transfer */
#define IISMOD_SEL_RX   FInsrt(0x1, fIISMOD_SEL_TR)     /* Receive */
#define IISMOD_SEL_TX   FInsrt(0x2, fIISMOD_SEL_TR)     /* Transmit */
#define IISMOD_SEL_BOTH FInsrt(0x3, fIISMOD_SEL_TR)     /* Tx & Rx */
#define IISMOD_CH_RIGHT (0 << 5)        /* high for right channel */
#define IISMOD_CH_LEFT  (1 << 5)        /* high for left channel */
#define IISMOD_FMT_IIS  (0 << 4)        /* IIS-compatible format */
#define IISMOD_FMT_MSB  (1 << 4)        /* MSB(left)-justified format */
#define IISMOD_BIT_8    (0 << 3)        /* Serial data bit/channel is 8 bit*/
#define IISMOD_BIT_16   (1 << 3)        /* Serial data bit/channel is 16 bit*/
#define IISMOD_FREQ_256 (0 << 2)        /* Master clock freq = 256 fs */
#define IISMOD_FREQ_384 (1 << 2)        /* Master clock freq = 384 fs */
#define fIISMOD_SFREQ   Fld(2, 0)       /* Serial bit clock frequency */
#define IISMOD_SFREQ    FMsk(fIISMOD_SFREQ)     /* fs = sampling frequency */
#define IISMOD_SFREQ_16 FInsrt(0x0, fIISMOD_SFREQ)      /* 16 fs */
#define IISMOD_SFREQ_32 FInsrt(0x1, fIISMOD_SFREQ)      /* 32 fs */
#define IISMOD_SFREQ_48 FInsrt(0x2, fIISMOD_SFREQ)      /* 48 fs */

#define fIISPSR_A       Fld(5, 5)       /* Prescaler Control A */
#define IISPSR_A(x)     FInsrt((x), fIISPSR_A)
#define fIISPSR_B       Fld(5, 0)       /* Prescaler Control B */
#define IISPSR_B(x)     FInsrt((x), fIISPSR_B)  

#define IISFCON_TX_NORM (0 << 15)       /* Transmit FIFO access mode: normal */
#define IISFCON_TX_DMA  (1 << 15)       /* Transmit FIFO access mode: DMA */
#define IISFCON_RX_NORM (0 << 14)       /* Receive FIFO access mode: normal */
#define IISFCON_RX_DMA  (1 << 14)       /* Receive FIFO access mode: DMA */
#define IISFCON_TX_EN   (1 << 13)        /* Transmit FIFO enable */
#define IISFCON_RX_EN   (1 << 12)        /* Recevice FIFO enable */
#define fIISFCON_TX_CNT Fld(6, 6)       /* Tx FIFO data count (Read-Only) */
#define IISFCON_TX_CNT  FMsk(fIISFCON_TX_CNT)
#define fIISFCON_RX_CNT Fld(6, 0)       /* Rx FIFO data count (Read-Only) */
#define IISFCON_RX_CNT  FMsk(fIISFCON_RX_CNT)

/* USB Device Controller Special Registers */
#define USB_CTL_BASE    0x52000000
#define UD_FUNC		__REG(USB_CTL_BASE + 0x140) // FUNC_ADDR_REG R/W 
#define UD_PWR		__REG(USB_CTL_BASE + 0x144) // PWR_REG R/W 
#define UD_INT		__REG(USB_CTL_BASE + 0x148) // EP_INT_REG
#define UD_USBINT	__REG(USB_CTL_BASE + 0x158) // USB_INT_REG 
#define UD_INTE		__REG(USB_CTL_BASE + 0x15c) // EP_INT_EN_REG
#define UD_USBINTE	__REG(USB_CTL_BASE + 0x16c) // USB_INT_EN_REG
#define UD_FRAMEL	__REG(USB_CTL_BASE + 0x170) // FRAME_NUM1_REG (R only)
//#define UD_FRAMEH	__REG(USB_CTL_BASE + 0x174) // 
#define UD_INDEX	__REG(USB_CTL_BASE + 0x178) // INDEX_REG
#define UD_MAXP         __REG(USB_CTL_BASE + 0x180) // Endpoint MAX Packet
#define UD_ICSR1        __REG(USB_CTL_BASE + 0x184) // EP In control status register 1 
#define UD_ICSR2        __REG(USB_CTL_BASE + 0x188) // EP In control status register 2 
#define UD_OCSR1        __REG(USB_CTL_BASE + 0x190) // EP Out control status register 1 
#define UD_OCSR2        __REG(USB_CTL_BASE + 0x194) // EP Out control status register 2 
#define UD_OFCNTL       __REG(USB_CTL_BASE + 0x198) // EP Out Write counter low-byte 
#define UD_OFCNTH       __REG(USB_CTL_BASE + 0x19c) // EP Out Write counter high-byte 
#define UD_FIFO0	__REG(USB_CTL_BASE + 0x1c0) // Endpoint 0 FIFO 
#define UD_FIFO1	__REG(USB_CTL_BASE + 0x1c4) // Endpoint 1 FIFO 
#define UD_FIFO2	__REG(USB_CTL_BASE + 0x1c8) // Endpoint 2 FIFO 
#define UD_FIFO3	__REG(USB_CTL_BASE + 0x1cc) // Endpoint 3 FIFO 
#define UD_FIFO4	__REG(USB_CTL_BASE + 0x1d0) // Endpoint 4 FIFO 
#define UD_DMACON1	__REG(USB_CTL_BASE + 0x200) // Endpoint 1 DMA control 
#define UD_DMAUC1	__REG(USB_CTL_BASE + 0x204) // Endpoint 1 DMA unit counter 
#define UD_DMAFC1	__REG(USB_CTL_BASE + 0x208) // Endpoint 1 DMA FIFO counter
#define UD_DMATCL1	__REG(USB_CTL_BASE + 0x20c) // Endpoint 1 DMA Transfer counter low-byte
#define UD_DMATCM1	__REG(USB_CTL_BASE + 0x210) // Endpoint 1 DMA Transfer counter middle-byte
#define UD_DMATCH1	__REG(USB_CTL_BASE + 0x214) // Endpoint 1 DMA Transfer counter high-byte
#define UD_DMACON2	__REG(USB_CTL_BASE + 0x218) // Endpoint 2 DMA control 
#define UD_DMAUC2	__REG(USB_CTL_BASE + 0x21c) // Endpoint 2 DMA unit counter 
#define UD_DMAFC2	__REG(USB_CTL_BASE + 0x220) // Endpoint 2 DMA FIFO counter
#define UD_DMATCL2	__REG(USB_CTL_BASE + 0x224) // Endpoint 2 DMA Transfer counter low-byte
#define UD_DMATCM2	__REG(USB_CTL_BASE + 0x228) // Endpoint 2 DMA Transfer counter middle-byte
#define UD_DMATCH2	__REG(USB_CTL_BASE + 0x22c) // Endpoint 2 DMA Transfer counter high-byte
#define UD_DMACON3	__REG(USB_CTL_BASE + 0x240) // Endpoint 3 DMA control 
#define UD_DMAUC3	__REG(USB_CTL_BASE + 0x244) // Endpoint 3 DMA unit counter 
#define UD_DMAFC3	__REG(USB_CTL_BASE + 0x248) // Endpoint 3 DMA FIFO counter
#define UD_DMATCL3	__REG(USB_CTL_BASE + 0x24c) // Endpoint 3 DMA Transfer counter low-byte
#define UD_DMATCM3	__REG(USB_CTL_BASE + 0x250) // Endpoint 3 DMA Transfer counter middle-byte
#define UD_DMATCH3	__REG(USB_CTL_BASE + 0x254) // Endpoint 3 DMA Transfer counter high-byte
#define UD_DMACON4	__REG(USB_CTL_BASE + 0x258) // Endpoint 4 DMA control 
#define UD_DMAUC4	__REG(USB_CTL_BASE + 0x25c) // Endpoint 4 DMA unit counter 
#define UD_DMAFC4	__REG(USB_CTL_BASE + 0x260) // Endpoint 4 DMA FIFO counter
#define UD_DMATCL4	__REG(USB_CTL_BASE + 0x264) // Endpoint 4 DMA Transfer counter low-byte
#define UD_DMATCM4	__REG(USB_CTL_BASE + 0x268) // Endpoint 4 DMA Transfer counter middle-byte
#define UD_DMATCH4	__REG(USB_CTL_BASE + 0x26c) // Endpoint 4 DMA Transfer counter high-byte

#define UD_FUNC_UD	(1 << 7)
#define fUD_FUNC_ADDR	Fld(7,0)	/* USB Device Addr. assigned by host */
#define UD_FUNC_ADDR	FMsk(fUD_FUNC_ADDR)

#define UD_PWR_ISOUP	(1<<7) 
#define UD_PWR_RESET	(1<<3) 
#define UD_PWR_RESUME	(1<<2) 
#define UD_PWR_SUSPND	(1<<1) 
#define UD_PWR_ENSUSPND	(1<<0) 

#define UD_PWR_DEFAULT	0x00

#define UD_INT_EP4	(1<<4)
#define UD_INT_EP3	(1<<3)	
#define UD_INT_EP2	(1<<2)	
#define UD_INT_EP1	(1<<1)
#define UD_INT_EP0	(1<<0)	

#define UD_USBINT_RESET	(1<<2) 
#define UD_USBINT_RESUM	(1<<1) 
#define UD_USBINT_SUSPND (1<<0)

#define UD_INTE_EP4	(1<<4) 
#define UD_INTE_EP3	(1<<3) 
#define UD_INTE_EP2	(1<<2)
#define UD_INTE_EP1	(1<<1)
#define UD_INTE_EP0	(1<<0) 

#define UD_USBINTE_RESET	(1<<2)
#define UD_USBINTE_SUSPND	(1<<0) 

#define fUD_FRAMEL_NUM	Fld(8,0)
#define UD_FRAMEL_NUM	FMsk(fUD_FRAMEL_NUM)

#define fUD_FRAMEH_NUM	Fld(8,0) 
#define UD_FRAMEH_NUM	FMsk(fUD_FRAMEH_NUM)

#define UD_INDEX_EP0	(0x00)
#define UD_INDEX_EP1	(0x01)
#define UD_INDEX_EP2	(0x02)
#define UD_INDEX_EP3	(0x03)
#define UD_INDEX_EP4	(0x04)
#define UD_ICSR1_CLRDT	(1<<6)   
#define UD_ICSR1_SENTSTL (1<<5)   
#define UD_ICSR1_SENDSTL (1<<4)  
#define UD_ICSR1_FFLUSH (1<<3)  	
#define UD_ICSR1_UNDRUN  (1<<2)   
#define UD_ICSR1_PKTRDY	 (1<<0)   

#define UD_ICSR2_AUTOSET (1<<7) 
#define UD_ICSR2_ISO	 (1<<6)	
#define UD_ICSR2_MODEIN	 (1<<5) 
#define UD_ICSR2_DMAIEN	 (1<<4) 

#define UD_OCSR1_CLRDT	(1<<7) 
#define UD_OCSR1_SENTSTL	(1<<6)	 
#define UD_OCSR1_SENDSTL	(1<<5)	
#define UD_OCSR1_FFLUSH		(1<<4) 
#define UD_OCSR1_DERROR		(1<<3) 
#define UD_OCSR1_OVRRUN		(1<<2)  
#define UD_OCSR1_PKTRDY		(1<<0)  

#define UD_OCSR2_AUTOCLR	(1<<7) 
#define UD_OCSR2_ISO		(1<<6) 
#define UD_OCSR2_DMAIEN		(1<<5) 

#define fUD_FIFO_DATA	Fld(8,0) 
#define UD_FIFO0_DATA	FMsk(fUD_FIFO_DATA)
#define UD_FIFO1_DATA	FMsk(fUD_FIFO_DATA)
#define UD_FIFO2_DATA	FMsk(fUD_FIFO_DATA)
#define UD_FIFO3_DATA	FMsk(fUD_FIFO_DATA)
#define UD_FIFO4_DATA	FMsk(fUD_FIFO_DATA)

#define UD_MAXP_8	(1<<0)
#define UD_MAXP_16	(1<<1)
#define UD_MAXP_32	(1<<2)
#define UD_MAXP_64	(1<<3)

#define fUD_OFCNT_DATA	Fld(8,0)
#define UD_OFCNTL_DATA	FMsk(fUD_OFCNT_DATA) 
#define UD_OFCNTH_DATA	FMsk(fUD_OFCNT_DATA) 

#define UD_DMACONx_INRUNOB	(1<<7) 
#define fUD_DMACON_STATE	Fld(3,4) 
#define UD_DMACONx_STATE	FMsk(fUD_DMACON_STATE) 
#define UD_DMACONx_DEMEN	(1<<3) 
#define UD_DMACONx_ORUN		(1<<2) 
#define UD_DMACONx_IRUN		(1<<1) 
#define UD_DMACONx_DMAMODE	(1<<0) 

#define fUD_DMAUC_DATA	Fld(8,0)
#define UD_DMAUCx_DATA	FMsk(fUD_DMAUC_DATA)

#define fUD_DMAFC_DATA	Fld(8,0)
#define UD_DMAFCx_DATA	FMsk(fUD_DMAFC_DATA)

#define fUD_DMATC_DATA	Fld(8,0)
#define UD_DMATCL_DATA	FMsk(fUD_DMATC_DATA)
#define UD_DMATCM_DATA	FMsk(fUD_DMATC_DATA)
#define UD_DMATCH_DATA	FMsk(fUD_DMATC_DATA)

#define EP0_CSR_OPKRDY	(1<<0)
#define EP0_CSR_IPKRDY	(1<<1)
#define EP0_CSR_SENTSTL	(1<<2)
#define EP0_CSR_DE	(1<<3)
#define EP0_CSR_SE	(1<<4)
#define EP0_CSR_SENDSTL	(1<<5)
#define EP0_CSR_SOPKTRDY (1<<6)
#define EP0_CSR_SSE	(1<<7)


/* USB Host - OHCI registers */
#define USBHOST_CTL_BASE 0x49000000


/* SPI */
#define SPI_CTL_BASE    0x59000000
#define SPCON0          __REG(SPI_CTL_BASE+0x00)
#define SPCON1          __REG(SPI_CTL_BASE+0x20)
#define SPSTA0          __REG(SPI_CTL_BASE+0x04)
#define SPSTA1          __REG(SPI_CTL_BASE+0x24)
#define SPPIN0          __REG(SPI_CTL_BASE+0x08)
#define SPPIN1          __REG(SPI_CTL_BASE+0x28)
#define SPPRE0          __REG(SPI_CTL_BASE+0x0c)
#define SPPRE1          __REG(SPI_CTL_BASE+0x2c)
#define SPTDAT0         __REG(SPI_CTL_BASE+0x10)
#define SPTDAT1         __REG(SPI_CTL_BASE+0x30)
#define SPRDAT0         __REG(SPI_CTL_BASE+0x14)
#define SPRDAT1         __REG(SPI_CTL_BASE+0x34)

#define fSPCON_SMOD     Fld(2,5)        /* SPI Mode Select */
#define SPCON_SMOD      FMsk(fSPCON_SMOD)
#define SPCON_SMOD_POLL FInsrt(0x0, fSPCON_SMOD)        /* polling mode */
#define SPCON_SMOD_INT  FInsrt(0x1, fSPCON_SMOD)        /* interrupt mode */
#define SPCON_SMOD_DMA  FInsrt(0x2, fSPCON_SMOD)        /* DMA mode */
#define SPCON_ENSCK     (1 << 4)        /* Enable SCK */
#define SPCON_MSTR      (1 << 3)        /* Master/Slave select
                                           0: slave, 1: master */
#define SPCON_CPOL      (1 << 2)        /* Clock polarity select
                                           1: active low, 0: active high */
#define SPCON_CPOL_LOW  (1 << 2)
#define SPCON_CPOL_HIGH (0 << 2)
#define SPCON_CPHA      (1 << 1)        /* Clock Phase Select
                                           0: format A, 1: format B */
#define SPCON_CPHA_FMTA (0 << 1)
#define SPCON_CPHA_FMTB (1 << 1)
#define SPCON_TAGD      (1 << 0)        /* Tx auto garbage data mode enable
                        in normal mode, you only want to receive data,
                                        you should tranmit dummy 0xFF data */

#define SPSTA_DCOL      (1 << 2)        /* Data Collision Error */
#define SPSTA_MULF      (1 << 1)        /* Multi Master Error */
#define SPSTA_READY     (1 << 0)        /* data Tx/Rx ready */

#define SPPIN_ENMUL     (1 << 2)        /* Multi Master Error detect Enable */
#define SPPIN_KEEP      (1 << 0)        /* Master Out keep */

/* Watchdog timer */

#define WT_CTL_BASE     0x53000000
#define WTCON           __REG(WT_CTL_BASE+0x00)
#define WTDAT           __REG(WT_CTL_BASE+0x04)
#define WTCNT           __REG(WT_CTL_BASE+0x08) 


/*GPIO temporary I get this from MIZI*/

#define GPCON(x)	__REG2(0x56000000, (x) * 0x10)
#define GPDAT(x)	__REG2(0x56000004, (x) * 0x10)
#define GPUP(x)	__REG2(0x56000008, (x) * 0x10)

#define GPIO_OFS_SHIFT		0
#define GPIO_PORT_SHIFTT	8
#define GPIO_PULLUP_SHIFT	16 
#define GPIO_MODE_SHIFT		24
#define GPIO_OFS_MASK		0x000000ff
#define GPIO_PORT_MASK		0x0000ff00
#define GPIO_PULLUP_MASK	0x00ff0000
#define GPIO_MODE_MASK		0xff000000
#define GPIO_MODE_IN		(0 << GPIO_MODE_SHIFT)
#define GPIO_MODE_OUT		(1 << GPIO_MODE_SHIFT)
#define GPIO_MODE_ALT0		(2 << GPIO_MODE_SHIFT)
#define GPIO_MODE_ALT1		(3 << GPIO_MODE_SHIFT)
#define GPIO_PULLUP_EN		(0 << GPIO_PULLUP_SHIFT)
#define GPIO_PULLUP_DIS		(1 << GPIO_PULLUP_SHIFT) 

#define PORTA_OFS		0
#define PORTB_OFS		1
#define PORTC_OFS		2
#define PORTD_OFS		3
#define PORTE_OFS		4
#define PORTF_OFS		5
#define PORTG_OFS		6
#define PORTH_OFS		7

#define MAKE_GPIO_NUM(p, o)	((p << GPIO_PORT_SHIFTT) | (o << GPIO_OFS_SHIFT))

#define GRAB_MODE(x)		(((x) & GPIO_MODE_MASK) >> GPIO_MODE_SHIFT)
#define GRAB_PULLUP(x)		(((x) & GPIO_PULLUP_MASK) >> GPIO_PULLUP_SHIFT)
#define GRAB_PORT(x)		(((x) & GPIO_PORT_MASK) >> GPIO_PORT_SHIFTT)
#define GRAB_OFS(x)		(((x) & GPIO_OFS_MASK) >> GPIO_OFS_SHIFT)

#define set_gpio_ctrl(x) \
	({ GPCON(GRAB_PORT((x))) &= ~(0x3 << (GRAB_OFS((x))*2)); \
	   GPCON(GRAB_PORT(x)) |= (GRAB_MODE(x) << (GRAB_OFS((x))*2)); \
	   GPUP(GRAB_PORT((x))) &= ~(1 << GRAB_OFS((x))); \
	   GPUP(GRAB_PORT((x))) |= (GRAB_PULLUP((x)) << GRAB_OFS((x))); })
#define set_gpio_pullup(x) \
	({ GPUP(GRAB_PORT((x))) &= ~(1 << GRAB_OFS((x))); \
	   GPUP(GRAB_PORT((x))) |= (GRAB_PULLUP((x)) << GRAB_OFS((x))); })
#define set_gpio_pullup_user(x, v) \
	({ GPUP(GRAB_PORT((x))) &= ~(1 << GRAB_OFS((x))); \
	   GPUP(GRAB_PORT((x))) |= ((v) << GRAB_OFS((x))); })
#define set_gpio_mode(x) \
	({ GPCON(GRAB_PORT((x))) &= ~(0x3 << (GRAB_OFS((x))*2)); \
	   GPCON(GRAB_PORT((x))) |= (GRAB_MODE((x)) << (GRAB_OFS((x))*2)); })
#define set_gpio_mode_user(x, v) \
	({ GPCON(GRAB_PORT((x))) & = ~(0x3 << (GRAB_OFS((x))*2)); \
	   GPCON(GRAB_PORT((x))) |= ((v) << (GRAB_OFS((x))*2)); })
#define set_gpioA_mode(x) \
	({ GPCON(GRAB_PORT((x))) &= ~(0x1 << GRAB_OFS((x))); \
	   GPCON(GRAB_PORT((x))) |= (GRAB_MODE((x)) << GRAB_OFS((x))); })
#define read_gpio_bit(x)	((GPDAT(GRAB_PORT((x))) & (1<<GRAB_OFS((x)))) >> GRAB_OFS((x)))
#define read_gpio_reg(x)	(GPDAT(GRAB_PORT((x)))
#define write_gpio_bit(x, v) \
	({ GPDAT(GRAB_PORT((x))) &= ~(0x1 << GRAB_OFS((x))); \
	   GPDAT(GRAB_PORT((x))) |= ((v) << GRAB_OFS((x))); })
#define write_gpio_reg(x, v)	(GPDAT(GRAB_PORT((x))) = (v))
	

#define GPIO_A0				MAKE_GPIO_NUM(PORTA_OFS, 0)
#define GPIO_A1				MAKE_GPIO_NUM(PORTA_OFS, 1)
#define GPIO_A2				MAKE_GPIO_NUM(PORTA_OFS, 2)
#define GPIO_A3				MAKE_GPIO_NUM(PORTA_OFS, 3)
#define GPIO_A4				MAKE_GPIO_NUM(PORTA_OFS, 4)
#define GPIO_A5				MAKE_GPIO_NUM(PORTA_OFS, 5)
#define GPIO_A6				MAKE_GPIO_NUM(PORTA_OFS, 6)
#define GPIO_A7				MAKE_GPIO_NUM(PORTA_OFS, 7)
#define GPIO_A8				MAKE_GPIO_NUM(PORTA_OFS, 8)
#define GPIO_A9				MAKE_GPIO_NUM(PORTA_OFS, 9)
#define GPIO_A10			MAKE_GPIO_NUM(PORTA_OFS, 10)
#define GPIO_A11			MAKE_GPIO_NUM(PORTA_OFS, 11)
#define GPIO_A12			MAKE_GPIO_NUM(PORTA_OFS, 12)
#define GPIO_A13			MAKE_GPIO_NUM(PORTA_OFS, 13)
#define GPIO_A14			MAKE_GPIO_NUM(PORTA_OFS, 14)
#define GPIO_A15			MAKE_GPIO_NUM(PORTA_OFS, 15)
#define GPIO_A16			MAKE_GPIO_NUM(PORTA_OFS, 16)
#define GPIO_A17			MAKE_GPIO_NUM(PORTA_OFS, 17)
#define GPIO_A18			MAKE_GPIO_NUM(PORTA_OFS, 18)
#define GPIO_A19			MAKE_GPIO_NUM(PORTA_OFS, 19)
#define GPIO_A20			MAKE_GPIO_NUM(PORTA_OFS, 20)
#define GPIO_A21			MAKE_GPIO_NUM(PORTA_OFS, 21)
#define GPIO_A22			MAKE_GPIO_NUM(PORTA_OFS, 22)

#define GPIO_B0				MAKE_GPIO_NUM(PORTB_OFS, 0)
#define GPIO_B1				MAKE_GPIO_NUM(PORTB_OFS, 1)
#define GPIO_B2				MAKE_GPIO_NUM(PORTB_OFS, 2)
#define GPIO_B3				MAKE_GPIO_NUM(PORTB_OFS, 3)
#define GPIO_B4				MAKE_GPIO_NUM(PORTB_OFS, 4)
#define GPIO_B5				MAKE_GPIO_NUM(PORTB_OFS, 5)
#define GPIO_B6				MAKE_GPIO_NUM(PORTB_OFS, 6)
#define GPIO_B7				MAKE_GPIO_NUM(PORTB_OFS, 7)
#define GPIO_B8				MAKE_GPIO_NUM(PORTB_OFS, 8)
#define GPIO_B9				MAKE_GPIO_NUM(PORTB_OFS, 9)
#define GPIO_B10			MAKE_GPIO_NUM(PORTB_OFS, 10)

#define GPIO_C0				MAKE_GPIO_NUM(PORTC_OFS, 0)
#define GPIO_C1				MAKE_GPIO_NUM(PORTC_OFS, 1)
#define GPIO_C2				MAKE_GPIO_NUM(PORTC_OFS, 2)
#define GPIO_C3				MAKE_GPIO_NUM(PORTC_OFS, 3)
#define GPIO_C4				MAKE_GPIO_NUM(PORTC_OFS, 4)
#define GPIO_C5				MAKE_GPIO_NUM(PORTC_OFS, 5)
#define GPIO_C6				MAKE_GPIO_NUM(PORTC_OFS, 6)
#define GPIO_C7				MAKE_GPIO_NUM(PORTC_OFS, 7)
#define GPIO_C8				MAKE_GPIO_NUM(PORTC_OFS, 8)
#define GPIO_C9				MAKE_GPIO_NUM(PORTC_OFS, 9)
#define GPIO_C10			MAKE_GPIO_NUM(PORTC_OFS, 10)
#define GPIO_C11			MAKE_GPIO_NUM(PORTC_OFS, 11)
#define GPIO_C12			MAKE_GPIO_NUM(PORTC_OFS, 12)
#define GPIO_C13			MAKE_GPIO_NUM(PORTC_OFS, 13)
#define GPIO_C14			MAKE_GPIO_NUM(PORTC_OFS, 14)
#define GPIO_C15			MAKE_GPIO_NUM(PORTC_OFS, 15)

#define GPIO_D0				MAKE_GPIO_NUM(PORTD_OFS, 0)
#define GPIO_D1				MAKE_GPIO_NUM(PORTD_OFS, 1)
#define GPIO_D2				MAKE_GPIO_NUM(PORTD_OFS, 2)
#define GPIO_D3				MAKE_GPIO_NUM(PORTD_OFS, 3)
#define GPIO_D4				MAKE_GPIO_NUM(PORTD_OFS, 4)
#define GPIO_D5				MAKE_GPIO_NUM(PORTD_OFS, 5)
#define GPIO_D6				MAKE_GPIO_NUM(PORTD_OFS, 6)
#define GPIO_D7				MAKE_GPIO_NUM(PORTD_OFS, 7)
#define GPIO_D8				MAKE_GPIO_NUM(PORTD_OFS, 8)
#define GPIO_D9				MAKE_GPIO_NUM(PORTD_OFS, 9)
#define GPIO_D10			MAKE_GPIO_NUM(PORTD_OFS, 10)
#define GPIO_D11			MAKE_GPIO_NUM(PORTD_OFS, 11)
#define GPIO_D12			MAKE_GPIO_NUM(PORTD_OFS, 12)
#define GPIO_D13			MAKE_GPIO_NUM(PORTD_OFS, 13)
#define GPIO_D14			MAKE_GPIO_NUM(PORTD_OFS, 14)
#define GPIO_D15			MAKE_GPIO_NUM(PORTD_OFS, 15)

#define GPIO_E0				MAKE_GPIO_NUM(PORTE_OFS, 0)
#define GPIO_E1				MAKE_GPIO_NUM(PORTE_OFS, 1)
#define GPIO_E2				MAKE_GPIO_NUM(PORTE_OFS, 2)
#define GPIO_E3				MAKE_GPIO_NUM(PORTE_OFS, 3)
#define GPIO_E4				MAKE_GPIO_NUM(PORTE_OFS, 4)
#define GPIO_E5				MAKE_GPIO_NUM(PORTE_OFS, 5)
#define GPIO_E6				MAKE_GPIO_NUM(PORTE_OFS, 6)
#define GPIO_E7				MAKE_GPIO_NUM(PORTE_OFS, 7)
#define GPIO_E8				MAKE_GPIO_NUM(PORTE_OFS, 8)
#define GPIO_E9				MAKE_GPIO_NUM(PORTE_OFS, 9)
#define GPIO_E10			MAKE_GPIO_NUM(PORTE_OFS, 10)
#define GPIO_E11			MAKE_GPIO_NUM(PORTE_OFS, 11)
#define GPIO_E12			MAKE_GPIO_NUM(PORTE_OFS, 12)
#define GPIO_E13			MAKE_GPIO_NUM(PORTE_OFS, 13)
#define GPIO_E14			MAKE_GPIO_NUM(PORTE_OFS, 14)
#define GPIO_E15			MAKE_GPIO_NUM(PORTE_OFS, 15)

#define GPIO_F0				MAKE_GPIO_NUM(PORTF_OFS, 0)
#define GPIO_F1				MAKE_GPIO_NUM(PORTF_OFS, 1)
#define GPIO_F2				MAKE_GPIO_NUM(PORTF_OFS, 2)
#define GPIO_F3				MAKE_GPIO_NUM(PORTF_OFS, 3)
#define GPIO_F4				MAKE_GPIO_NUM(PORTF_OFS, 4)
#define GPIO_F5				MAKE_GPIO_NUM(PORTF_OFS, 5)
#define GPIO_F6				MAKE_GPIO_NUM(PORTF_OFS, 6)
#define GPIO_F7				MAKE_GPIO_NUM(PORTF_OFS, 7)

#define GPIO_G0				MAKE_GPIO_NUM(PORTG_OFS, 0)
#define GPIO_G1				MAKE_GPIO_NUM(PORTG_OFS, 1)
#define GPIO_G2				MAKE_GPIO_NUM(PORTG_OFS, 2)
#define GPIO_G3				MAKE_GPIO_NUM(PORTG_OFS, 3)
#define GPIO_G4				MAKE_GPIO_NUM(PORTG_OFS, 4)
#define GPIO_G5				MAKE_GPIO_NUM(PORTG_OFS, 5)
#define GPIO_G6				MAKE_GPIO_NUM(PORTG_OFS, 6)
#define GPIO_G7				MAKE_GPIO_NUM(PORTG_OFS, 7)
#define GPIO_G8				MAKE_GPIO_NUM(PORTG_OFS, 8)
#define GPIO_G9				MAKE_GPIO_NUM(PORTG_OFS, 9)
#define GPIO_G10			MAKE_GPIO_NUM(PORTG_OFS, 10)
#define GPIO_G11			MAKE_GPIO_NUM(PORTG_OFS, 11)
#define GPIO_G12			MAKE_GPIO_NUM(PORTG_OFS, 12)
#define GPIO_G13			MAKE_GPIO_NUM(PORTG_OFS, 13)
#define GPIO_G14			MAKE_GPIO_NUM(PORTG_OFS, 14)
#define GPIO_G15			MAKE_GPIO_NUM(PORTG_OFS, 15)

#define GPIO_H0				MAKE_GPIO_NUM(PORTH_OFS, 0)
#define GPIO_H1				MAKE_GPIO_NUM(PORTH_OFS, 1)
#define GPIO_H2				MAKE_GPIO_NUM(PORTH_OFS, 2)
#define GPIO_H3				MAKE_GPIO_NUM(PORTH_OFS, 3)
#define GPIO_H4				MAKE_GPIO_NUM(PORTH_OFS, 4)
#define GPIO_H5				MAKE_GPIO_NUM(PORTH_OFS, 5)
#define GPIO_H6				MAKE_GPIO_NUM(PORTH_OFS, 6)
#define GPIO_H7				MAKE_GPIO_NUM(PORTH_OFS, 7)
#define GPIO_H8				MAKE_GPIO_NUM(PORTH_OFS, 8)
#define GPIO_H9				MAKE_GPIO_NUM(PORTH_OFS, 9)
#define GPIO_H10			MAKE_GPIO_NUM(PORTH_OFS, 10)

#define GPIO_MODE_TOUT			GPIO_MODE_ALT0
#define GPIO_MODE_nXBACK		GPIO_MODE_ALT0
#define GPIO_MODE_nXBREQ		GPIO_MODE_ALT0
#define GPIO_MODE_nXDACK		GPIO_MODE_ALT0
#define GPIO_MODE_nXDREQ		GPIO_MODE_ALT0
#define GPIO_MODE_LEND			GPIO_MODE_ALT0
#define GPIO_MODE_VCLK			GPIO_MODE_ALT0
#define GPIO_MODE_VLINE			GPIO_MODE_ALT0
#define GPIO_MODE_VFRAME		GPIO_MODE_ALT0
#define GPIO_MODE_VM			GPIO_MODE_ALT0
#define GPIO_MODE_LCDVF			GPIO_MODE_ALT0
#define GPIO_MODE_VD			GPIO_MODE_ALT0
#define GPIO_MODE_IICSDA		GPIO_MODE_ALT0
#define GPIO_MODE_IICSCL		GPIO_MODE_ALT0
#define GPIO_MODE_SPICLK		GPIO_MODE_ALT0
#define GPIO_MODE_SPIMOSI		GPIO_MODE_ALT0
#define GPIO_MODE_SPIMISO		GPIO_MODE_ALT0
#define GPIO_MODE_SDDAT			GPIO_MODE_ALT0
#define GPIO_MODE_SDCMD			GPIO_MODE_ALT0
#define GPIO_MODE_SDCLK			GPIO_MODE_ALT0
#define GPIO_MODE_I2SSDO		GPIO_MODE_ALT0
#define GPIO_MODE_I2SSDI		GPIO_MODE_ALT0
#define GPIO_MODE_CDCLK			GPIO_MODE_ALT0
#define GPIO_MODE_I2SSCLK		GPIO_MODE_ALT0
#define GPIO_MODE_I2SLRCK		GPIO_MODE_ALT0
#define GPIO_MODE_I2SSDI_ABNORMAL	GPIO_MODE_ALT1
#define GPIO_MODE_nSS			GPIO_MODE_ALT1
#define GPIO_MODE_EINT			GPIO_MODE_ALT0
#define GPIO_MODE_nYPON			GPIO_MODE_ALT1
#define GPIO_MODE_YMON			GPIO_MODE_ALT1
#define GPIO_MODE_nXPON			GPIO_MODE_ALT1
#define GPIO_MODE_XMON			GPIO_MODE_ALT1
#define GPIO_MODE_UART			GPIO_MODE_ALT0	
#define GPIO_MODE_TCLK_ABNORMAL		GPIO_MODE_ALT1
#define GPIO_MODE_SPICLK_ABNORMAL	GPIO_MODE_ALT1
#define GPIO_MODE_SPIMOSI_ABNORMAL	GPIO_MODE_ALT1
#define GPIO_MODE_SPIMISO_ABNORMAL	GPIO_MODE_ALT1
#define GPIO_MODE_LCD_PWRDN		GPIO_MODE_ALT1

/*minsung for serial 2440*/
#define UTRSTAT_TX_EMPTY        (1 << 2)
#define UTRSTAT_RX_READY        (1 << 0)
#define UART_ERR_MASK           0xF

	// NAND related defines
	// joo-young hwang
	// 2004 Jan 3
	  #define __REGb(x)       (*(volatile unsigned char *)(io_p2v(x)))
	  
	  #define bNAND_CTL(Nb)   __REG(0x4e000000 + (Nb))
	  #define NFCONF          bNAND_CTL(0x00)
	  #define NFCONT          bNAND_CTL(0x04)
	  #define NFCMD           bNAND_CTL(0x08)
	  #define NFADDR          bNAND_CTL(0x0c)
	  #define NFDATA8         __REGb(0x4e000000 + 0x10)
	  #define NFDATA          __REGb(0x4e000000 + 0x10) /* Default 8 bit read/write */
	  #define NFDATA32        bNAND_CTL(0x10)
	  #define NFSTAT          bNAND_CTL(0x20)
	  #define NFECC           bNAND_CTL(0x2c)

	 #define fNFCONF_TWRPH1   Fld(3,4)
	  #define NFCONF_TWRPH1    FMsk(fNFCONF_TWRPH1)
	  #define NFCONF_TWRPH1_7  FInsrt(0x7, fNFCONF_TWRPH1) /* 7 */
	  #define fNFCONF_TWRPH0   Fld(3,8)
	  #define NFCONF_TWRPH0    FMsk(fNFCONF_TWRPH0)
	  #define NFCONF_TWRPH0_7  FInsrt(0x7, fNFCONF_TWRPH0) /* 7 */
	  #define fNFCONF_TACLS    Fld(3,12)
	  #define NFCONF_TACLS     FMsk(fNFCONF_TACLS)
	  #define NFCONF_TACLS_7   FInsrt(0x7, fNFCONF_TACLS) /* 7 */
	  #define fNFCONT_nFCE     Fld(1,1)
	  #define NFCONT_nFCE      FMsk(fNFCONT_nFCE)
	  #define NFCONT_nFCE_LOW  FInsrt(0x0, fNFCONT_nFCE) /* active */
	  #define NFCONT_nFCE_HIGH FInsrt(0x1, fNFCONT_nFCE) /* inactive */

	  #define fNFCONT_ECC      Fld(1,4)
	  #define NFCONT_ECC       FMsk(fNFCONT_ECC)
	  #define NFCONT_ECC_INIT  FInsrt(0x1, fNFCONT_ECC)    /* initialize */

#define fNFCONT_MAINECC  Fld(1,5)
	  #define NFCONT_MECC       FMsk(fNFCONT_MAINECC)
	  #define NFCONT_MECC_UNLOCK FInsrt(0x0, fNFCONT_MAINECC)
	  #define NFCONT_MECC_LOCK   FInsrt(0x1, fNFCONT_MAINECC)

	  #define fNFCONF_ADDRSTEP Fld(1,13)                 /* Addressing Step */
	  #define NFCONF_ADDRSTEP  FMsk(fNFCONF_ADDRSTEP)

	  #define fNFCONF_PAGESIZE Fld(1,2)
	  #define NFCONF_PAGESIZE  FMsk(fNFCONF_PAGESIZE)
	  #define NFCONF_PAGESIZE_256  FInsrt(0x0, fNFCONF_PAGESIZE) /* 256 bytes */
	  #define NFCONF_PAGESIZE_512  FInsrt(0x1, fNFCONF_PAGESIZE) /* 512 bytes */

	  #define fNFCONT_FCTRL    Fld(1,0)  /* Flash controller enable/disable */
	  #define NFCONT_FCTRL     FMsk(fNFCONT_FCTRL)
	  #define NFCONT_FCTRL_DIS FInsrt(0x0, fNFCONT_FCTRL) /* Disable */
	  #define NFCONT_FCTRL_EN  FInsrt(0x1, fNFCONT_FCTRL) /* Enable */

	  #define NFSTAT_RnB      (1 << 2)

	  /* ECC0 Status Register bit field definition */
	  #define fNFESTAT0_MEDNO         Fld(11,7)               /* 2048 Byte  */
	  #define NFESTAT0_MEDNO          FMsk(fNFESTAT0_MEDNO)

	  #define fNFESTAT0_MAINERR       Fld(2,0)
	  #define NFESTAT0_MAINERR_NOERR  FInsrt(0x00,fNFESTAT0_MAINERR)
	  #define NFESTAT0_MAINERR_ONEBIT FInsrt(0x01,fNFESTAT0_MAINERR)
	  #define NFESTAT0_MAINERR_MULBIT FInsrt(0x10,fNFESTAT0_MAINERR)
	  #define NFESTAT0_MAINERR_ECCAREA FInsrt(0x11,fNFESTAT0_MAINERR)
	// NAND related defines
	// joo-young hwang
	// 2004 Jan 3
	//

	
	// camera related defines
	// joo-young hwang
	// 2004 Jan 3
	
	/*
	 *    * CAMERA IP
	 *       */

	  #define CAM_ASIZE       __REG(0x4f000000)
	  #define CAM_STAY1       __REG(0x4f000004)
	  #define CAM_STAY2       __REG(0x4f000008)
	  #define CAM_STAY3       __REG(0x4f00000c)
	  #define CAM_STAY4       __REG(0x4f000010)
	  #define CAM_AYBURST     __REG(0x4f000014)
	  #define CAM_ACBBURST    __REG(0x4f000018)
	  #define CAM_ACRBURST    __REG(0x4f00001c)
	  #define CAM_ADISTWIDTH  __REG(0x4f000040)
	  #define CAM_YRATIO      __REG(0x4f00004c)
	  #define CAM_CRATIO      __REG(0x4f000050)
	  #define CAM_YORIGINAL   __REG(0x4f000054)
	  #define CAM_CORIGINAL   __REG(0x4f00005c)
	  #define CAM_STACB1      __REG(0x4f000074)
	  #define CAM_STACB2      __REG(0x4f000078)
	  #define CAM_STACB3      __REG(0x4f00007c)
	  #define CAM_STACB4      __REG(0x4f000080)
	  #define CAM_STACR1      __REG(0x4f000084)
	  #define CAM_STACR2      __REG(0x4f000088)
	  #define CAM_STACR3      __REG(0x4f00008c)
	  #define CAM_STACR4      __REG(0x4f000090)
	  #define CAM_CTRL        __REG(0x4f0000BC)
	  #define CAM_RDSTAT      __REG(0x4f000000)
#define CAM_STAY(__x)   __REG(0x4f000004 + (__x)*4)
	  #define CAM_STACB(__x)  __REG(0x4f000074 + (__x)*4)
	  #define CAM_STACR(__x)  __REG(0x4f000084 + (__x)*4)
	  #define CAM_RDSTAY      __REG(0x4f000014)
	  #define CAM_RDSTACB     __REG(0x4f000018)
	  #define CAM_RDSTACR     __REG(0x4f00001c)

	  #define CAM_STAY(__x)   __REG(0x4f000004 + (__x)*4)
	  #define CAM_STACB(__x)  __REG(0x4f000074 + (__x)*4)
	  #define CAM_STACR(__x)  __REG(0x4f000084 + (__x)*4)
	  #define CAM_RDSTAY      __REG(0x4f000014)
	  #define CAM_RDSTACB     __REG(0x4f000018)
	  #define CAM_RDSTACR     __REG(0x4f00001c)

	  #define  fCAM_SIZE_H Fld(10, 10)
	  #define  fCAM_SIZE_V Fld(10, 0)
	  #define CAM_SIZE_H(x) FInsrt((x), fCAM_SIZE_H)
	  #define CAM_SIZE_V(x) FInsrt((x), fCAM_SIZE_V)

	  #define  fCAM_BURST_M Fld(16,16)
	  #define  fCAM_BURST_R Fld(16,0)
	  #define CAM_BURST_M(x) FInsrt((x), fCAM_BURST_M)
	  #define CAM_BURST_R(x) FInsrt((x), fCAM_BURST_R)

	  #define  fCAM_DISTWIDTH_D Fld(16,16)
	  #define  fCAM_DISTWIDTH_W Fld(16,0)
	  #define CAM_DISTWIDTH_D(x) FInsrt((x), fCAM_DISTWIDTH_D)
	  #define CAM_DISTWIDTH_W(x) FInsrt((x), fCAM_DISTWIDTH_W)
	  #define  fCAM_RATIO_H Fld(16,16)
	  #define  fCAM_RATIO_V Fld(16,0)
	  #define CAM_RATIO_H(x) FInsrt((x), fCAM_RATIO_H)
	  #define CAM_RATIO_V(x) FInsrt((x), fCAM_RATIO_V)
#define  fCAM_ORIGINAL_H Fld(10,16)
	  #define  fCAM_ORIGINAL_V Fld(10,0)
	  #define CAM_ORIGINAL_H(x) FInsrt((x), fCAM_ORIGINAL_H)
	  #define CAM_ORIGINAL_V(x) FInsrt((x), fCAM_ORIGINAL_V)

	  #define  fCAM_STAT_FRAME Fld(2,28)
	  #define CAM_STAT_FRAME(x) FExtr((x), fCAM_STAT_FRAME)

	// camera related defines
	// joo-young hwang
	// 2004 Jan 3



