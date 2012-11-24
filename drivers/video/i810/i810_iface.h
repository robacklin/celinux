/*-*- linux-c -*-
 *  linux/drivers/video/i810_iface.h -- Hardware Interface
 *
 *      Copyright (C) 2001 Antonino Daplas
 *      All Rights Reserved      
 *
 *  This file is subject to the terms and conditions of the GNU General Public
 *  License. See the file COPYING in the main directory of this archive for
 *  more details.
 */

#ifndef __I810_IFACE_H__
#define __I810_IfACE_H__

struct blit_header_struct {
	u32 opcode;
	u32 size;
};

static struct blit_header_struct blit_header[] = {
	{ 0x00, 6 },
	{ 0x10, 7 },
	{ 0x30, 0 },
	{ 0x21, 1 },
	{ 0x22, 4 },
	{ 0x30, 0xFFFF0002},
	{ 0x40, 3 },
	{ 0x41, 3 },
	{ 0x42, 6 },
	{ 0x43, 4 },
	{ 0x44, 6 },
	{ 0x45, 6 },
	{ 0x46, 7 },
	{ 0x47, 9 },
	{ 0x48, 10},
	{ 0x60, 0xFFFF0002 },
	{ 0x61, 0xFFFF0004 },
};	


extern struct iface_data  *i810_iface;
extern void        i810fb_release_all  (u32 key);
extern int         i810fb_bind_all     (void);
extern void        i810fb_clear_gttmap (agp_memory *surface); 
extern void        i810fb_set_gttmap   (agp_memory *surface); 
extern void        i810fb_sync         (void);
extern void        i810fb_flush        (void);
extern void        emit_instruction    (u32 dsize, u32 pointer, u32 trusted);
extern void        i810_writel         (u32 where, u32 val);
extern u32         i810_readl          (u32 where);
extern void        i810_writeb         (u32 where, u8 val);
extern u8          i810_readb          (u32 where);
extern void        i810_writew         (u32 where, u16 val);
extern u16         i810_readw          (u32 where);
extern void        i810fb_load_overlay (int ovl_address);

#if LINUX_VERSION_CODE > KERNEL_VERSION(2,4,9)
static struct list_head *list2;
#endif 

#endif /* __I810_IFACE_H__ */
