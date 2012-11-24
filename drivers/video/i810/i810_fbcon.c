/*-*- linux-c -*-
 *  linux/drivers/video/i810_fbcon.c -- Console Drawing Functions
 *
 *      Copyright (C) 2001 Antonino Daplas
 *      All Rights Reserved      
 *
 *  This file is subject to the terms and conditions of the GNU General Public
 *  License. See the file COPYING in the main directory of this archive for
 *  more details.
 */

#include <linux/tty.h>
#include <linux/fb.h>
#include <linux/console.h>

#include <video/fbcon.h>
#include <video/fbcon-cfb8.h>
#include <video/fbcon-cfb16.h>
#include <video/fbcon-cfb24.h>
#include <video/fbcon-cfb32.h>

#include "i810_regs.h"
#include "i810_common.h"
#include "i810_fbcon.h"

/*
 * Crash Handling
 */

/**
 * i810_gart_countdown_handler - crash handler
 * @data: pointer to arbitrary data, presently NULL
 *
 * DESCRIPTION:
 * This handler will attempt to recover the hardware state from whatever
 * process that failed to release the device.  This is called via
 * a timer.  It will only attempt recovery when fbcon is in context, otherwise,
 * the handler will just exit. 
 */
static void i810_gart_countdown_handler(unsigned long data)
{
	if (!i810_info->gart_countdown_active) 
		return;
	if (i810_info->in_context && i810_info->gart_is_claimed)
		i810fb_reacquire_gart();
	del_timer(&i810_info->gart_countdown_timer);
	i810_info->gart_countdown_active = 0; 
}	

/**
 * i810fb_start_countdown - initiates crash handler timer
 *
 * DESCRIPTION:
 * If fbcon comes into context, and the device is still claimed, this
 * function will initiate a 5-second timer, which upon expiration will
 * start the recovery process
 */
static void i810fb_start_countdown(void) 
{
	if (i810_info->gart_countdown_active)
		return;
	i810_info->gart_countdown_active = 1;
	init_timer(&i810_info->gart_countdown_timer);	
	i810_info->gart_countdown_timer.data = 0;
	i810_info->gart_countdown_timer.expires = jiffies + (HZ * 5);
	i810_info->gart_countdown_timer.function = i810_gart_countdown_handler;
	add_timer(&i810_info->gart_countdown_timer);
}

/**
 * not_safe - determines if it's safe to use the graphics device
 *
 * DESCRIPTION:
 * Checks if the current GART is claimed or not.
 * If the GART is claimed, then it will initiate a 5 second
 * timer in an attempt to recover the framebuffer state.
 *
 * RETURNS:
 * a nonzero if true
 */

inline int not_safe(void)
{
	if (i810_info->cur_state)
		return 1;
	if (!i810_info->in_context) 
		i810_info->in_context = 1;
	if (!i810_info->gart_is_claimed) 
		return 0;
	i810fb_remove_stale_memory();
	i810fb_start_countdown(); 
	return 1;
}		 

void i810_cursor(struct display *p, int mode, int xx, int yy)
{
	int temp = 0, w, h, c, fg = 0, bg = 0;
	
	h = fontheight(p);
	w = fontwidth(p);
	yy -= p->yscroll;
	yy *= h;
	xx *= w;
	switch (i810_orient->rotate) {
	case NO_ROTATION:
		temp = xx & 0xFFFF;
		temp |= yy << 16;
		break;
	case ROTATE_RIGHT:
		temp = (i810_orient->xres - (yy+h)) & 0xFFFF;
		temp |= xx << 16;
		break;
	case ROTATE_180:
		temp = (i810_orient->xres - (xx+w)) & 0xFFFF;
		temp |= (i810_orient->yres - (yy+h)) << 16;
		break;
	case ROTATE_LEFT:
		temp = yy & 0xFFFF;
		temp |= (i810_orient->yres - (xx+w)) << 16;
		break;
	}
	i810_writel(CURPOS, temp);
	
	switch (mode) {
	case CM_ERASE:
		i810fb_enable_cursor(OFF);
		i810_info->cursor.cursor_show = 0;
		i810_info->cursor.cursor_enable = 0;
		break;
	case CM_MOVE:
	case CM_DRAW:
		if ((p->conp->vc_cursor_type & 0x0F) != i810_info->cursor_format) {
			i810_info->cursor_format = p->conp->vc_cursor_type & 0x0F;
			i810_info->cursor_reset = 1;
		}
		c = scr_readw((u16 *) p->conp->vc_pos);
		
		fg = (int) attr_fgcol(p,c);
		bg = (int) attr_bgcol(p,c);
		
		if (bg != i810_info->cursor_bg ||
		    fg != i810_info->cursor_fg   ) {
			i810_info->cursor_bg = bg;
			i810_info->cursor_fg = fg;
			i810fb_set_cursor_color(p);
		}
		
		c &= p->charmask;
		i810_info->cursor_bitmap = (u8 *) i810_accel->fontcache_start_virtual + ((c * w * h) >> 3);
		
		i810fb_load_cursor_image(p);
		i810_info->cursor.cursor_show = 1;
		i810_info->cursor.cursor_enable = 1;
	}
	i810_info->cursor.blink_count = 0;
} 

static void i810_noaccel_setup(struct display *p)
{
	p->next_line = p->line_length ? p->line_length : 
		get_line_length(p->var.xres_virtual, p->var.bits_per_pixel);
	p->next_plane = 0;
	memmove((u8 *)i810_accel->fontcache_start_virtual, p->fontdata, 4096);
}

/* 
 * The following are wrappers for the generic framebuffer operations.  Each
 * is preceeded by a call to 'not_safe' before calling the actual generic 
 * operation.
 */
 
#ifdef FBCON_HAS_CFB8
void i810_noaccel8_bmove(struct display *p, int sy, int sx, int dy, int dx,
			 int height, int width)
{
	if (not_safe()) return;
	fbcon_cfb8_bmove(p, sy, sx, dy, dx, height, width);
}

void i810_noaccel8_clear(struct vc_data *conp, struct display *p, 
			 int sy, int sx, int height, int width)
{
	if (not_safe()) return;
	fbcon_cfb8_clear(conp, p, sy, sx, height, width);
}

void i810_noaccel8_putc(struct vc_data *conp, struct display *p, 
			int c, int yy, int xx)
{	    
	if (not_safe()) return;
	fbcon_cfb8_putc(conp, p, c, yy, xx);
}

void i810_noaccel8_putcs(struct vc_data *conp, struct display *p,
			 const unsigned short *s, int count, int yy, int xx)
{
	if (not_safe()) return;
	fbcon_cfb8_putcs(conp, p, s, count, yy, xx);
}

void i810_noaccel8_revc(struct display *p, int xx, int yy)
{	                        	                                     
	if (not_safe()) return;
	fbcon_cfb8_revc(p, xx, yy);
}

void i810_noaccel8_clear_margins(struct vc_data *conp, struct display *p,
				 int bottom_only)
{           
	if (not_safe()) return;
	fbcon_cfb8_clear_margins(conp, p, bottom_only);
}

struct display_switch i810_noaccel8 = {
	i810_noaccel_setup,
	i810_noaccel8_bmove,
	i810_noaccel8_clear,
	i810_noaccel8_putc,
	i810_noaccel8_putcs,
	i810_noaccel8_revc,
	NULL,
	NULL,
	i810_noaccel8_clear_margins,
	FONTWIDTH(8) | FONTWIDTH(16)
};
#endif

#ifdef FBCON_HAS_CFB16
void i810_noaccel16_bmove(struct display *p, int sy, int sx, int dy, int dx,
                       int height, int width)
{
	if (not_safe()) return;
	fbcon_cfb16_bmove(p, sy, sx, dy, dx, height, width);
}

void i810_noaccel16_clear(struct vc_data *conp, struct display *p, 
			  int sy, int sx, int height, int width)
{
	if (not_safe()) return;
	fbcon_cfb16_clear(conp, p, sy, sx, height, width);
}

void i810_noaccel16_putc(struct vc_data *conp, struct display *p, 
			 int c, int yy, int xx)
{	    
	if (not_safe()) return;
	fbcon_cfb16_putc(conp, p, c, yy, xx);
}

void i810_noaccel16_putcs(struct vc_data *conp, struct display *p,
                        const unsigned short *s, int count, int yy, int xx)
{
	if (not_safe()) return;
	fbcon_cfb16_putcs(conp, p, s, count, yy, xx);
}

void i810_noaccel16_revc(struct display *p, int xx, int yy)
{	                                       	                        
	if (not_safe()) return;
	fbcon_cfb16_revc(p, xx, yy);
}

void i810_noaccel16_clear_margins(struct vc_data *conp, struct display *p,
                               int bottom_only)
{           
	if (not_safe()) return;
	fbcon_cfb16_clear_margins(conp, p, bottom_only);
}

struct display_switch i810_noaccel16 = {	
	i810_noaccel_setup,
	i810_noaccel16_bmove,
	i810_noaccel16_clear,
	i810_noaccel16_putc,
	i810_noaccel16_putcs,
	i810_noaccel16_revc,
	NULL,
	NULL,
	i810_noaccel16_clear_margins,
	FONTWIDTH(8) | FONTWIDTH(16)
};
#endif

#ifdef FBCON_HAS_CFB24
void i810_noaccel24_bmove(struct display *p, int sy, int sx, int dy, int dx,
                       int height, int width)
{
	if (not_safe()) return;
	fbcon_cfb24_bmove(p, sy, sx, dy, dx, height, width);
}

void i810_noaccel24_clear(struct vc_data *conp, struct display *p, 
			  int sy, int sx, int height, int width)
{
	if (not_safe()) return;
	fbcon_cfb24_clear(conp, p, sy, sx, height, width);
}

void i810_noaccel24_putc(struct vc_data *conp, struct display *p, 
			 int c, int yy, int xx)
{	    
	if (not_safe()) return;
	fbcon_cfb24_putc(conp, p, c, yy, xx);
}

void i810_noaccel24_putcs(struct vc_data *conp, struct display *p,
                        const unsigned short *s, int count, int yy, int xx)
{
	if (not_safe()) return;
	fbcon_cfb24_putcs(conp, p, s, count, yy, xx);
}

void i810_noaccel24_revc(struct display *p, int xx, int yy)
{	                                    	                        
	if (not_safe()) return;
	fbcon_cfb24_revc(p, xx, yy);
}

void i810_noaccel24_clear_margins(struct vc_data *conp, struct display *p,
                               int bottom_only)
{           
	if (not_safe()) return;
	fbcon_cfb24_clear_margins(conp, p, bottom_only);
}

struct display_switch i810_noaccel24 = {	
	i810_noaccel_setup,
	i810_noaccel24_bmove,
	i810_noaccel24_clear,
	i810_noaccel24_putc,
	i810_noaccel24_putcs,
	i810_noaccel24_revc,
	NULL,
	NULL,
	i810_noaccel24_clear_margins,
	FONTWIDTH(8) | FONTWIDTH(16)
};
#endif

#ifdef FBCON_HAS_CFB32
void i810_noaccel32_bmove(struct display *p, int sy, int sx, int dy, int dx,
                       int height, int width)
{
	if (not_safe()) return;
	fbcon_cfb32_bmove(p, sy, sx, dy, dx, height, width);
}

void i810_noaccel32_clear(struct vc_data *conp, struct display *p, 
			  int sy, int sx, int height, int width)
{
	if (not_safe()) return;
	fbcon_cfb32_clear(conp, p, sy, sx, height, width);
}

void i810_noaccel32_putc(struct vc_data *conp, struct display *p, 
			 int c, int yy, int xx)
{	    
	if (not_safe()) return;
	fbcon_cfb32_putc(conp, p, c, yy, xx);
}

void i810_noaccel32_putcs(struct vc_data *conp, struct display *p,
                        const unsigned short *s, int count, int yy, int xx)
{
	if (not_safe()) return;
	fbcon_cfb32_putcs(conp, p, s, count, yy, xx);
}

void i810_noaccel32_revc(struct display *p, int xx, int yy)
{	                                    	                        
	if (not_safe()) return;
	fbcon_cfb32_revc(p, xx, yy);
}

void i810_noaccel32_clear_margins(struct vc_data *conp, struct display *p,
                               int bottom_only)
{           
	if (not_safe()) return;
	fbcon_cfb32_clear_margins(conp, p, bottom_only);
}

struct display_switch i810_noaccel32 = {	
	i810_noaccel_setup,
	i810_noaccel32_bmove,
	i810_noaccel32_clear,
	i810_noaccel32_putc,
	i810_noaccel32_putcs,
	i810_noaccel32_revc,
	NULL,
	NULL,
	i810_noaccel32_clear_margins,
	FONTWIDTH(8) | FONTWIDTH(16)
};
#endif




