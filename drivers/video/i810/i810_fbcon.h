/*-*- linux-c -*-
 *  linux/drivers/video/i810_fbcon.h -- Console Drawing Functions
 *
 *      Copyright (C) 2001 Antonino Daplas
 *      All Rights Reserved      
 *
 *
 *  This file is subject to the terms and conditions of the GNU General Public
 *  License. See the file COPYING in the main directory of this archive for
 *  more details.
 */

#ifndef __I810_FBCON_H__
#define __I810_FBCON_H__

extern struct i810_fbinfo *i810_info;
extern struct orientation *i810_orient;
extern struct accel_data  *i810_accel;

extern void i810_writel             (u32 where, u32 val);
extern int  i810fb_reacquire_gart   (void);
extern u32  get_line_length         (int xres_virtual, int bpp);
extern void i810fb_enable_cursor    (int mode);

extern void mono_pat_blit           (int dpitch, int dheight, int dwidth, int dest, 
				     int fg, int bg, int rop, int patt_1, int patt_2, 
				     int blit_bpp);
extern void source_copy_blit        (int dwidth, int dheight, int dpitch, int xdir, 
				     int spitch, int src, int dest, int rop, int blit_bpp);
extern void color_blit              (int width, int height, int pitch, 
				     int dest, int rop, int what, int blit_bpp);
extern void i810fb_load_cursor_image(struct display *p);
extern void i810fb_set_cursor_color (struct display *p);

#ifdef CONFIG_FB_I810_IFACE
extern void i810fb_remove_stale_memory(void);
#else
static inline void i810fb_remove_stale_memory(void) { }
#endif

#endif /* __I810_FBCON_H__ */
