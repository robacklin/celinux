/*
 *	Access to VGA videoram
 *
 *	(c) 1998 Martin Mares <mj@ucw.cz>
 */

#ifdef __KERNEL__
#ifndef _LINUX_ASM_VGA_H_
#define _LINUX_ASM_VGA_H_

#include <asm/io.h>

#include <linux/config.h>

#define VT_BUF_HAVE_RW

#if defined(CONFIG_SH_KZP01)
#define VGA_OFFSET(ofst)  (ofst)
//#define VGA_OFFSET(ofst)  (0xbf000000 + ofst - 0xa00000)
#endif

/*
 *  These are only needed for supporting VGA or MDA text mode, which use little
 *  endian byte ordering.
 *  In other cases, we can optimize by using native byte ordering and
 *  <linux/vt_buffer.h> has already done the right job for us.
 */

static __inline__ void scr_writew(u16 val, volatile u16 *addr)
{
#if defined(CONFIG_SH_KZP01)
    if(((ulong)addr & 0xff000000)==0x8c000000){
	*addr = val;
	return;
    }
    else
	return writew(val, (unsigned long)VGA_OFFSET( addr )); 
#else
    if ( (long) addr < 0 )
	*addr = val;
    else
	writew(val, (unsigned long) addr);
#endif
}

static __inline__ u16 scr_readw(volatile const u16 *addr)
{
#if defined(CONFIG_SH_KZP01)
    if(((ulong)addr & 0xff000000)==0x8c000000)
	return *addr;
    else
	return readw((unsigned long)VGA_OFFSET( addr ));
#else
    if ( (long)addr < 0 )
	return *addr;
    else
	return readw((unsigned long)addr);
#endif
}

#define VT_BUF_HAVE_MEMCPYW
#if defined(CONFIG_SH_KZP01)

#define VGA_OFFSET_A(ofst)  (0xfd000000 + ofst)
#define scr_memcpyw(a, b, c) memcpy(VGA_OFFSET_A(a), VGA_OFFSET_A(b), c)

extern unsigned long vgacon_remap_base;

#define VGA_MAP_MEM(x) VGA_OFFSET(x)
#define vga_readb(x) (*VGA_OFFSET(x))
#define vga_writeb(x,y) (*VGA_OFFSET(y) = (x))

#else

#define scr_memcpyw	memcpy

extern unsigned long vgacon_remap_base;
#define VGA_MAP_MEM(x) (x)
#define vga_readb(x) (*(x))
#define vga_writeb(x,y) (*(y) = (x))

#endif

#endif
#endif /* __KERNEL__ */
