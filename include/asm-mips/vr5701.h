#ifndef __VR5701_H
#define __VR5701_H

#ifndef __ASSEMBLY__
#define __preg8		(volatile unsigned char*)
#define __preg16	(volatile unsigned short*)
#define __preg32	(volatile unsigned int*)
#else
#define __preg8
#define __preg16
#define __preg32
#endif /* __ASSEMBLY__ */


#define VR5701_IO_BASE	0xbe000000

/****************************************************************************
 *                                                                          *
 *  Title:   Vr5701.h                                                       *
 *                                                                          *
 *        Vr5701  Assembler Macros                                          *
 *                                                                          *
 *                                                                          *
 *      Author  : Hideo Mochizuki                                           *
 *      Date    : 01.23.2003                                                *
 *                                                                          *
 ****************************************************************************
 *      Copyright (C) NEC Electronics Corporation 2003                      *
 *      All Rights Reserved.                                                *
 ***************************************************************************/
/* External Agent Memory Mapping                                         */
#define  SystemAddr   0x9fc18000 /* External Aagent Base Address         */
#define  PassOff      0x0000     /* Pass Offset address                  */
#define  FailOff      0x0008     /* Fail Offset address                  */

/* Default Base Address */
#define PeriBaseAddress 0x9fa00000
/* PADR registers */
#define PADR_SDRAM01	0x40
#define PADR_SDRAM23	0x48
#define PADR_LOCALCS0	0x80
#define PADR_LOCALCS1	0x88
#define PADR_LOCALCS2	0x90
#define PADR_LOCALCS3	0x98
#define PADR_PCIW0	0xC0
#define PADR_PCIW1	0xC8
#define PADR_EPCIW0	0xC0
#define PADR_EPCIW1	0xC8
#define PADR_IOPCIW0	0xE0
#define PADR_IOPCIW1	0xE8
#define PADR_IPCIW0	0xE0
#define PADR_IPCIW1	0xE8
/* INT registers */
#define INT0_STAT	0x100
#define INT1_STAT	0x108
#define INT2_STAT	0x110
#define INT3_STAT	0x118
#define INT4_STAT	0x120
#define NMI_STAT	0x130
#define INT_REQ		0x138
#define INT_CLR		0x140
#define INT_TYP		0x148
#define INT_MASK	0x150
#define INT_ROUTE0	0x160
#define INT_ROUTE1	0x168
#define INT_ROUTE2	0x170
#define INT_ROUTE3	0x178
/* Reset registers */
#define RESET_CTRL	0x180
#define BOOT_MODE	0x190
/* MEMC registers */
#define MEMC_CTRL	0x200
#define MEMC_MEMCFG	0x210
#define MEMC_REF	0x220
/* LOCAL registers */
#define LOCAL_CST0	0x400
#define LOCAL_CST1	0x408
#define LOCAL_CST2	0x410
#define LOCAL_CST3	0x418
#define LOCAL_CFG	0x440
#define LOCAL_ESTAT	0x460
#define LOCAL_EADR	0x470
/* DMA registers */
#define DMA_CTRL0L	0x500
#define DMA_CTRL0H	0x504
#define DMA_SRCA0	0x508
#define DMA_DESA0	0x510
#define DMA_NXDP0	0x518
#define DMA_CTRL1L	0x520
#define DMA_CTRL1H	0x524
#define DMA_SRCA1	0x528
#define DMA_DESA1	0x530
#define DMA_NXDP1	0x538
#define DMA_CTRL2L	0x540
#define DMA_CTRL2H	0x544
#define DMA_SRCA2	0x548
#define DMA_DESA2	0x550
#define DMA_NXDP2	0x558
#define DMA_CTRL3L	0x560
#define DMA_CTRL3H	0x564
#define DMA_SRCA3	0x568
#define DMA_DESA3	0x570
#define DMA_NXDP3	0x578
/* EPCI registers */
#define EPCI_CTRLL	0x600
#define EPCI_CTRLH	0x604
#define EPCI_ARBL	0x608
#define EPCI_ARBH	0x60C
#define EPCI_INIT0	0x610
#define EPCI_INIT1	0x618
#define EPCI_SWAP	0x620
#define EPCI_ERR	0x628
#define EPCI_INTS	0x630
#define EPCI_INTM	0x638
/* PCI registers */
#define PCI_VID		0x700
#define PCI_DID		0x702
#define PCI_PCICMD	0x704
#define PCI_PCISTS	0x706
#define PCI_REVID	0x708
#define PCI_CLASS	0x709
#define PCI_CLSIZ	0x70C
#define PCI_MLTIM	0x70D
#define PCI_HTYPE	0x70E
#define PCI_BAR_MEM01	0x710
#define PCI_BAR_MEM23	0x718
#define PCI_SSVID	0x72C
#define PCI_SSID	0x72E
#define PCI_INTLIM	0x73C
#define PCI_INTPIN	0x73D
#define VR5701_PCI_MIN_GNT	0x73E
#define VR5701_PCI_MAX_LAT	0x73F
#define PCI_BAR_LCS0	0x740
#define PCI_BAR_LCS1	0x748
#define PCI_BAR_LCS2	0x750
#define PCI_BAR_LCS3	0x758
#define PCI_BAR_IPCIW0	0x7A0
#define PCI_BAR_IPCIW1	0x7A8
#define PCI_BAR_IREG	0x7C0

/* PCI registers cont.*/
#define VID		0x700
#define DID		0x702
#define PCICMD		0x704
#define PCISTS		0x706
#define REVID		0x708
#define CLASS		0x709
#define CLSIZ		0x70C
#define MLTIM		0x70D
#define HTYPE		0x70E
#define BAR_MEM01	0x710
#define BAR_MEM23	0x718
#define SSVID		0x72C
#define SSID		0x72E
#define INTLIM		0x73C
#define INTPIN		0x73D
#define MIN_GNT		0x73E
#define MAX_LAT		0x73F
#define BAR_LCS0	0x740
#define BAR_LCS1	0x748
#define BAR_LCS2	0x750
#define BAR_LCS3	0x758
#define BAR_IPCIW0	0x7A0
#define BAR_IPCIW1	0x7A8
#define BAR_IREG	0x7C0

/* PIB registers*/
#define PIB_RESET	0x800
#define PIB_READY	0x810
#define PIB_MISC	0x830

/* TIMER (GPT-General Purpose Timer) registers*/
#define GPT0_CTRL	0x880
#define GPT0_RELOAD	0x888
#define GPT0_COUNT	0x890
#define GPT1_CTRL	0x8A0
#define GPT1_RELOAD	0x8A8
#define GPT1_COUNT	0x8B0
#define GPT2_CTRL	0x8C0
#define GPT2_RELOAD	0x8C8
#define GPT2_COUNT	0x8D0
#define GPT3_CTRL	0x8E0
#define GPT3_RELOAD	0x8E8
#define GPT3_COUNT	0x8F0

/* GPIO registers*/
#define GIU_INTSTAT	0x900
#define GIU_INTREQ	0x908
#define GIU_INTCLR	0x920
#define GIU_INTTYPE	0x928
#define GIU_INTMASK	0x930
#define GIU_INTPOL	0x938
#define GIU_PIO0	0x940
#define GIU_PIO1	0x948
#define GIU_DIR0	0x950
#define GIU_DIR1	0x958
#define GIU_FUNCSEL0	0x960
#define GIU_FUNCSEL1	0x968

/* UART0 registers*/
#define UART0_RBR	0xA00
#define UART0_THR	0xA00
#define UART0_DLL	0xA00
#define UART0_IER	0xA08
#define UART0_DLM	0xA08
#define UART0_IIR	0xA10
#define UART0_LCR	0xA18
#define UART0_MCR	0xA20
#define UART0_LSR	0xA28
#define UART0_MSR	0xA30
#define UART0_SCR	0xA38
/* UART1 registers*/
#define UART1_RBR	0xA40
#define UART1_THR	0xA40
#define UART1_DLL	0xA40
#define UART1_IER	0xA48
#define UART1_DLM	0xA48
#define UART1_IIR	0xA50
#define UART1_LCR	0xA58
#define UART1_MCR	0xA60
#define UART1_LSR	0xA68
#define UART1_MSR	0xA70
#define UART1_SCR	0xA88
/* UART2 registers*/
#define UART2_RBR	0xA80
#define UART2_THR	0xA80
#define UART2_DLL	0xA80
#define UART2_IER	0xA88
#define UART2_DLM	0xA88
#define UART2_IIR	0xA90
#define UART2_LCR	0xA98
#define UART2_MCR	0xAA0
#define UART2_LSR	0xAA8
#define UART2_MSR	0xAB0
#define UART2_SCR	0xAB8
/* UART3 registers*/
#define UART3_RBR	0xAC0
#define UART3_THR	0xAC0
#define UART3_DLL	0xAC0
#define UART3_IER	0xAC8
#define UART3_DLM	0xAC8
#define UART3_IIR	0xAD0
#define UART3_LCR	0xAD8
#define UART3_MCR	0xAE0
#define UART3_LSR	0xAE8
#define UART3_MSR	0xAF0
#define UART3_SCR	0xAF8

/* CSI0 registers*/
#define CSI0_MODE	0xB00
#define CSI0_SIRB	0xB08
#define CSI0_SOTB	0xB10
#define CSI0_SIRBE	0xB18
#define CSI0_SOTBF	0xB20
#define CSI0_SIO	0xB28
#define CSI0_CNT	0xB40
#define CSI0_INT	0xB48
#define CSI0_IFIFOV	0xB50
#define CSI0_OFIFOV	0xB58
#define CSI0_IFIFO	0xB60
#define CSI0_OFIFO	0xB68
#define CSI0_FIFOTRG	0xB70
/* CSI1 registers*/
#define CSI1_MODE	0xB80
#define CSI1_SIRB	0xB88
#define CSI1_SOTB	0xB90
#define CSI1_SIRBE	0xB98
#define CSI1_SOTBF	0xBA0
#define CSI1_SIO	0xBA8
#define CSI1_CNT	0xBC0
#define CSI1_INT	0xBC8
#define CSI1_IFIFOV	0xBD0
#define CSI1_OFIFOV	0xBD8
#define CSI1_IFIFO	0xBE0
#define CSI1_OFIFO	0xBE8
#define CSI1_FIFOTRG	0xBF0

/* IPCI registers*/
#define IPCI_CTRLL	0xE00
#define IPCI_CTRLH	0xE04
#define IPCI_ARBL	0xE08
#define IPCI_ARBH	0xE0C
#define IPCI_INIT0	0xE10
#define IPCI_INIT1	0xE18
#define IPCI_SWAP	0xE20
#define IPCI_ERR	0xE28
#define IPCI_INTS	0xE30
#define IPCI_INTM	0xE38
/* PCI registers */
#define IPCI_VID	0xF00
#define IPCI_DID	0xF02
#define IPCI_PCICMD	0xF04
#define IPCI_PCISTS	0xF06
#define IPCI_REVID	0xF08
#define IPCI_CLASS	0xF09
#define IPCI_CLSIZ	0xF0C
#define IPCI_MLTIM	0xF0D
#define IPCI_HTYPE	0xF0E
#define IPCI_BAR_MEM01	0xF10
#define IPCI_BAR_MEM23	0xF18
#define IPCI_SSVID	0xF2C
#define IPCI_SSID	0xF2E
#define IPCI_INTLIM	0xF3C
#define IPCI_INTPIN	0xF3D
#define IPCI_MIN_GNT	0xF3E
#define IPCI_MAX_LAT	0xF3F
#define IPCI_BAR_LCS0	0xF40
#define IPCI_BAR_LCS1	0xF48
#define IPCI_BAR_LCS2	0xF50
#define IPCI_BAR_LCS3	0xF58
#define IPCI_BAR_IPCIW0	0xF80
#define IPCI_BAR_IPCIW1	0xF88
#define IPCI_BAR_IREG	0xFC0

#endif /*  __VR5701_H */
