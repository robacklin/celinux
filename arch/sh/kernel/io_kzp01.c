/* 
 *
 * linux/arch/sh/kernel/io_kzp01.c
 *
 * Copyright (C) 2001  Nobuhiro Sakawa
 *
 * I/O routine for kmc Solution Platform.
 *
 *  from
 *
 *  linux/arch/sh/kernel/io_se.c
 *
 *  Copyright (C) 2000  Kazumoto Kojima
 *
 *  I/O routine for Hitachi SolutionEngine.
 *
 */

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/pci.h>
#include <asm/io.h>
#include <asm/kzp01.h>

static inline void delay(void)
{
	ctrl_inw(0xa0000000);
}

static inline unsigned long PORT2ADR(unsigned long port)
{
	if(port < 0x0000ffff) { 
		return port + PCI_IO_WIN;
	}
	else {
		return port;
	}
}

static unsigned long now_bank=0UL;
static unsigned long fb_bank=0UL;		/* frame buffer bank address */

static inline unsigned long ADR2ADR(unsigned long addr)
{
	if((addr & 0xff000000) == 0x80000000) { /* bank=0 ?  */
	  *(unsigned long *)(PCI_MEM_BNK_ADR) = 0xbf000000UL;
	} else {
	  now_bank = *(unsigned long *)(PCI_MEM_BNK_ADR) = addr & 0xff000000UL;
	}
	return (addr & 0x00ffffff) + PCI_MEM_WIN;
}

static inline void RETURN_ADR2ADR()
{
	if(now_bank != fb_bank)
	    *(unsigned long *)(PCI_MEM_BNK_ADR) = fb_bank;
}

void kzp_set_fb_bank(unsigned long addr)
{
	fb_bank = addr & 0xff000000UL;
}

unsigned char kzp_inb(unsigned long port)
{
	unsigned long v;

	v = *(__u8 *)PORT2ADR(port);
	return (__u8)v; 
}

unsigned char kzp_inb_p(unsigned long port)
{
	unsigned long v;

	v = *(__u8 *)PORT2ADR(port); 
	delay();
	return (__u8)v;
}

unsigned short kzp_inw(unsigned long port)
{
	unsigned long v;

	v = *(__u16 *)PORT2ADR(port);
	return (__u16)v;
}

unsigned short kzp_inw_p(unsigned long port)
{
	unsigned long v;

	v = *(__u16 *)PORT2ADR(port);
	delay();
	return (__u16)v;
}

unsigned int kzp_inl(unsigned long port)
{
	unsigned long v;

	v = *(__u32 *)PORT2ADR(port);
	return (__u32)v;
}

void kzp_outb(unsigned char value, unsigned long port)
{
	*(__u8 *)PORT2ADR(port) = value; 
}

void kzp_outb_p(unsigned char value, unsigned long port)
{
	*(__u8 *)PORT2ADR(port) = value; 
	delay();
}

void kzp_outw(unsigned short value, unsigned long port)
{
	*(__u16 *)PORT2ADR(port) = value;
}

void kzp_outw_p(unsigned short value, unsigned long port)
{
	*(__u16 *)PORT2ADR(port) = value;
	delay();
}

void kzp_outl(unsigned int value, unsigned long port)
{
	*(__u32 *)PORT2ADR(port) = value;
}

void kzp_insb(unsigned long port, void *addr, unsigned long count)
{
	volatile __u8 *p = (__u8 *)PORT2ADR(port);

	while (count--)
		*((__u8 *) addr)++ = *p;
}

void kzp_insw(unsigned long port, void *addr, unsigned long count)
{
	volatile __u16 *p = (__u16 *)PORT2ADR(port);

	while (count--)
		*((__u16 *) addr)++ = *p;
}

void kzp_insl(unsigned long port, void *addr, unsigned long count)
{
	unsigned long data;
	volatile __u32 *p = (__u32 *)PORT2ADR(port);

	while (count--) {
		data	= *p;
		*((__u8 *) addr)++ = (__u8)(data & 0x000000ff);
		*((__u8 *) addr)++ = (__u8)((data>>8) & 0x000000ff);
		*((__u8 *) addr)++ = (__u8)((data>>16) & 0x000000ff);
		*((__u8 *) addr)++ = (__u8)((data>>24) & 0x000000ff);
	}
}

void kzp_outsb(unsigned long port, const void *addr, unsigned long count)
{
	volatile __u8 *p = (__u8 *)PORT2ADR(port);

	while (count--)
		*p = *((__u8 *) addr)++;
}

void kzp_outsw(unsigned long port, const void *addr, unsigned long count)
{
	volatile __u16 *p =(__u16 *)PORT2ADR(port);

	while (count--)
		*p = *((__u16 *) addr)++;
}

void kzp_outsl(unsigned long port, const void *addr, unsigned long count)
{
	unsigned long data;
	volatile __u32 *p = (__u32 *)PORT2ADR(port);

	while (count--) {
		data  = (unsigned long)((*((__u8 *) addr)++)     & 0x000000ff);
		data |= (unsigned long)((*((__u8 *) addr)++)<<8  & 0x0000ff00);
		data |= (unsigned long)((*((__u8 *) addr)++)<<16 & 0x00ff0000);
		data |= (unsigned long)((*((__u8 *) addr)++)<<24 & 0xff000000);
		*p = data;
	}
}

unsigned char kzp_readb(unsigned long addr)
{
	unsigned long val,flags;

	save_and_cli(flags);
	val =  *(volatile unsigned char*)ADR2ADR(addr);
	RETURN_ADR2ADR();
	restore_flags(flags);
	return (__u8)val;
}

unsigned short kzp_readw(unsigned long addr)
{
	unsigned long val,flags;

	save_and_cli(flags);
	val =  *(volatile unsigned short*)ADR2ADR(addr);
	RETURN_ADR2ADR();
	restore_flags(flags);
	return (__u16)val;
}

unsigned int kzp_readl(unsigned long addr)
{
	unsigned long val,flags;

	save_and_cli(flags);
	val =  *(volatile unsigned long*)ADR2ADR(addr);
	RETURN_ADR2ADR();
	restore_flags(flags);
	return (__u32)val;
}

void kzp_writeb(unsigned char b, unsigned long addr)
{
	unsigned long flags;

	save_and_cli(flags);
	*(volatile unsigned char*)ADR2ADR(addr) = b;
	RETURN_ADR2ADR();
	restore_flags(flags);
}

void kzp_writew(unsigned short b, unsigned long addr)
{
	unsigned long flags;

	save_and_cli(flags);
	*(volatile unsigned short*)ADR2ADR(addr) = b;
	RETURN_ADR2ADR();
	restore_flags(flags);
}

void kzp_writel(unsigned int b, unsigned long addr)
{
	unsigned long flags;

	save_and_cli(flags);
        *(volatile unsigned long*)ADR2ADR(addr) = b;
	RETURN_ADR2ADR();
	restore_flags(flags);
}

unsigned long
kzp_isa_port2addr(unsigned long offset)
{
	return 0;
}


int
kzp_pcibios_read_config_dword(unsigned char bus,	
			      unsigned char dev,
			      unsigned char offset,
			      unsigned int *val)
{			/* dev = dev-No + Function-No */
	unsigned long x;    

	x = 0x80000000 | ((unsigned long)bus<<16)
		       | ((unsigned long)dev<<8) | offset;
	*(unsigned long *)0xfe2001c0 = x;
	*val =  *(unsigned long *)0xfe200220;	
	return PCIBIOS_SUCCESSFUL;
}

int
kzp_pcibios_read_config_word(unsigned char bus,		/* little only */
			     unsigned char dev,	
			     unsigned char offset,
			     unsigned short *val)
{
	unsigned long x,wval;    
	
	x = 0x80000000 | ((unsigned long)bus<<16)
		       | ((unsigned long)dev<<8) | (offset & 0xfc);
	*(unsigned long *)0xfe2001c0 = x;
	wval =  *(unsigned long *)0xfe200220;
	switch(offset & 0x02) {
	case 0: *val = (unsigned short)(wval & 0x0000ffff);
		break;
	case 1: *val = (unsigned short)((wval>>16) & 0x0000ffff);
		break;
	default:
		return -1;
	}
	return PCIBIOS_SUCCESSFUL;
}

int
kzp_pcibios_read_config_byte(unsigned char bus,		/* little only */
			     unsigned char dev,	
			     unsigned char offset,
			     unsigned char *val)
{
	unsigned long x,wval;    

	x = 0x80000000 | ((unsigned long)bus<<16)
		       | ((unsigned long)dev<<8) | (offset & 0xfc);
	*(unsigned long *)0xfe2001c0 = x;
	wval =  *(unsigned long *)0xfe200220;	

	switch(offset & 0x03) {
	case 0: *val = (unsigned char)(wval & 0x000000ff);
		break;
	case 1: *val = (unsigned char)((wval>>8) & 0x000000ff);
		break;
	case 2: *val = (unsigned char)((wval>>16) & 0x000000ff);
		break;
	case 3: *val = (unsigned char)((wval>>24) & 0x000000ff);
		break;
	default:
		return -1;
	}
	return PCIBIOS_SUCCESSFUL;
}

int
kzp_pcibios_write_config_dword(unsigned char bus,	
			       unsigned char dev,
			       unsigned char offset,
			       unsigned int val)
{
	unsigned long x;    

	x = 0x80000000 | ((unsigned long)bus<<16)
		       | ((unsigned long)dev<<8) | offset;
	*(unsigned long *)0xfe2001c0 = x;
	*(unsigned int *)0xfe200220 = val;	
	return PCIBIOS_SUCCESSFUL;
}

int
kzp_pcibios_write_config_word(unsigned char bus,	/* little only */	
			      unsigned char dev,	
			      unsigned char offset,
			      unsigned short val)
{
	unsigned long x,wval;    

	x = 0x80000000 | ((unsigned long)bus<<16)
		       | ((unsigned long)dev<<8) | (offset & 0xfc);
	*(unsigned long *)0xfe2001c0 = x;
	wval =  *(unsigned long *)0xfe200220;	

	switch(offset & 0x02) {
	case 0: wval = (wval & 0xffff0000) | (unsigned long)(val & 0x0000ffff);
		break;
	case 1: wval = (wval & 0x0000ffff) | (unsigned long)((val<<16) & 0xffff0000);
		break;
	default:
		return -1;
	}
	*(unsigned long *)0xfe200220 = wval;	
	return PCIBIOS_SUCCESSFUL;
}

int
kzp_pcibios_write_config_byte(unsigned char bus,	/* little only */
			      unsigned char dev,	
			      unsigned char offset,
			      unsigned char val)
{
	unsigned long x,wval;    

	x = 0x80000000 | ((unsigned long)bus<<16)
		       | ((unsigned long)dev<<8) | (offset & 0xfc);
	*(unsigned long *)0xfe2001c0 = x;
	wval =  *(unsigned long *)0xfe200220;	

	switch(offset & 0x03) {
	case 0: wval = (wval & 0xffffff00) | (unsigned long)(val & 0x000000ff);
		break;
	case 1: wval = (wval & 0xffff00ff) | (unsigned long)((val<<8)  & 0x0000ff00);
		break;
	case 2: wval = (wval & 0xff00ffff) | (unsigned long)((val<<16) & 0x00ff0000);
		break;
	case 3: wval = (wval & 0x00ffffff) | (unsigned long)((val<<24) & 0xff000000);
		break;
	default:
		return -1;
	}
	*(unsigned long *)0xfe200220 = wval;	
	return PCIBIOS_SUCCESSFUL;
}

