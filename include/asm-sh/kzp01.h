/*
 * linux/include/asm-sh/kzp01.h
 *
 * Copyright (C) 2001 Nobuhiro Sakawa 
 *
 * kmc Solution platform
 */

#ifndef __ASM_SH_KZP01_H
#define __ASM_SH_KZP01_H

/* kzp01 local memory map */
#define  SH4_SRAM_ADR	 0x10000000  /* 512kbyte */
#define  SH4_DRAM_ADR	 0x0c000000  /* 64Mbyte  */

#define  PCI_IO_WIN	 0xfe240000  /* pci io window 256k */
#define  PCI_MEM_WIN	 0xfd000000  /* pci memory window 16Mbyte */
#define  PCI_MEM_BNK_ADR 0xfe2001c4  /* bit[31:24] ... (bit0=LOCK) MBR */
#define  PCI_MEM_IO_ADR  0xfe2001c8  /* bit[31:18] ... (bit0=LOCK) MBR */
//#define  PCI_MEM_IO_ADR  0xfe2001c4  /* bit[31:18] ... (bit0=LOCK) MBR */

#define  LED	(volatile unsigned short *)0xa3000000 /* kzp01 LED */
#define  SRAM	(volatile unsigned short *)0xb0000000 /* kzp01 SRAM */


/* PCI memory map */
#define  PCI_SRAM_ADR    0xfe000000  /* sh4 SRAM  0x10000000 */
//#define  PCI_DRAM_ADR    0xff000000  /* sh4 SDRAM 0x0c000000 */

#define  PCI_PCIC_ADR	 0xfff00000  /* sh4 PCIC */

#define  PCIBIOS_READ_CONFIG_DWORD	kzp_pcibios_read_config_dword
#define  PCIBIOS_READ_CONFIG_WORD	kzp_pcibios_read_config_word
#define  PCIBIOS_READ_CONFIG_BYTE	kzp_pcibios_read_config_byte
#define  PCIBIOS_WRITE_CONFIG_DWORD	kzp_pcibios_write_config_dword
#define  PCIBIOS_WRITE_CONFIG_WORD	kzp_pcibios_write_config_word
#define  PCIBIOS_WRITE_CONFIG_BYTE	kzp_pcibios_write_config_byte

#define	KZP01_PCI_IO_WIN	PCI_IO_WIN
#define	KZP01_PCI_MEM_WIN	PCI_MEM_WIN


#endif  /* __ASM_SH_KZP01_H */
