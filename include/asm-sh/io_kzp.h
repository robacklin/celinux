/*
 * include/asm-sh/io_kzp.h
 *
 * Copyright 2000 Stuart Menefy (stuart.menefy@st.com)
 *
 * May be copied or modified under the terms of the GNU General Public
 * License.  See linux/COPYING for more information.
 *
 * IO functions for use when we don't know what machine we are on
 */

#ifndef _ASM_SH_IO_KZP_H
#define _ASM_SH_IO_KZP_H

#include <asm/io_generic.h>

extern unsigned char kzp_inb(unsigned long port);
extern unsigned short kzp_inw(unsigned long port);
extern unsigned int kzp_inl(unsigned long port);

extern void kzp_outb(unsigned char value, unsigned long port);
extern void kzp_outw(unsigned short value, unsigned long port);
extern void kzp_outl(unsigned int value, unsigned long port);

extern unsigned char kzp_inb_p(unsigned long port);
extern unsigned short kzp_inw_p(unsigned long port);
extern unsigned int kzp_inl_p(unsigned long port);
extern void kzp_outb_p(unsigned char value, unsigned long port);
extern void kzp_outw_p(unsigned short value, unsigned long port);
extern void kzp_outl_p(unsigned int value, unsigned long port);

extern void kzp_insb(unsigned long port, void *addr, unsigned long count);
extern void kzp_insw(unsigned long port, void *addr, unsigned long count);
extern void kzp_insl(unsigned long port, void *addr, unsigned long count);
extern void kzp_outsb(unsigned long port, const void *addr, unsigned long count);
extern void kzp_outsw(unsigned long port, const void *addr, unsigned long count);
extern void kzp_outsl(unsigned long port, const void *addr, unsigned long count);

extern unsigned char kzp_readb(unsigned long addr);
extern unsigned short kzp_readw(unsigned long addr);
extern unsigned int kzp_readl(unsigned long addr);
extern void kzp_writeb(unsigned char b, unsigned long addr);
extern void kzp_writew(unsigned short b, unsigned long addr);
extern void kzp_writel(unsigned int b, unsigned long addr);

extern unsigned long kzp_isa_port2addr(unsigned long offset);
//extern void *kzp_ioremap(unsigned long offset, unsigned long size);
//extern void *kzp_ioremap_nocache (unsigned long offset, unsigned long size);
//extern void kzp_iounmap(void *addr);

extern void kzp_set_fb_bank(unsigned long addr);

#ifdef __WANT_IO_DEF

# define __inb			kzp_inb
# define __inw			kzp_inw
# define __inl			kzp_inl
# define __outb			kzp_outb
# define __outw			kzp_outw
# define __outl			kzp_outl

# define __inb_p		kzp_inb_p
# define __inw_p		kzp_inw_p
# define __inl_p		kzp_inl_p
# define __outb_p		kzp_outb_p
# define __outw_p		kzp_outw_p
# define __outl_p		kzp_outl_p

# define __insb			kzp_insb
# define __insw			kzp_insw
# define __insl			kzp_insl
# define __outsb		kzp_outsb
# define __outsw		kzp_outsw
# define __outsl		kzp_outsl

# define __readb		kzp_readb
# define __readw		kzp_readw
# define __readl		kzp_readl
# define __writeb		kzp_writeb
# define __writew		kzp_writew
# define __writel		kzp_writel

# define __isa_port2addr	kzp_isa_port2addr
# define __ioremap		generic_ioremap
//# define __ioremap_nocache	kzp_ioremap_nocache
# define __iounmap		generic_iounmap

#endif

#endif /* _ASM_SH_IO_KZP_H */
