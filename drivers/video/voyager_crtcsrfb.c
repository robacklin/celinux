/*
 *  linux/drivers/video/voyafb.c
 *
 *  Copyright (c) 2003 ATOM Create, Inc.
 *
 *  May be copied or modified under the terms of the GNU General Public
 *  License.  See linux/COPYING for more information.
 *
 *  VOYAGER Frame Buffer video plane Device Driver for SH4
 *
 */
#include <linux/config.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/ctype.h>
#include <linux/mm.h>
#include <linux/tty.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/fb.h>
#include <linux/delay.h>
#include <linux/wrapper.h>
#include <linux/pm.h>
#include <asm/uaccess.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/voyagergx_reg.h>
#include <video/fbcon.h>
#include <video/fbcon-cfb2.h>
#include "voyager.h"


/* frame buffer operations */

static int  voyafb_get_fix(struct fb_fix_screeninfo*, int, struct fb_info*);
static int  voyafb_get_var(struct fb_var_screeninfo*, int, struct fb_info*);
static int  voyafb_set_var(struct fb_var_screeninfo*, int, struct fb_info*);
static int  voyafb_get_cmap(struct fb_cmap*, int, int, struct fb_info*);
static int  voyafb_set_cmap(struct fb_cmap*, int, int, struct fb_info*);
static int  voyafb_ioctl(struct inode*, struct file*,
			   unsigned int, unsigned long, int, struct fb_info*);
static void voyafb_set_disp(int, struct fb_info*);
static int  change_mode(struct fb_var_screeninfo *var);

#ifdef CONFIG_PM
static struct pm_dev* fb_pm_dev;
#endif

static struct display disp[MAX_NR_CONSOLES];
static struct fb_info fb_info;
static struct fb_ops  voyafb_ops = {
	owner:		THIS_MODULE,
	fb_get_fix:	voyafb_get_fix,
	fb_get_var:	voyafb_get_var,
	fb_set_var:	voyafb_set_var,
	fb_get_cmap:	voyafb_get_cmap,
	fb_set_cmap:	voyafb_set_cmap,
	fb_ioctl:	voyafb_ioctl,
};

static u_char* vram_base      = (u_char*)0;
static u_char* vram_base_phys = (u_char*)0;

static u16 fbcon_cmap_cfb2[2];

static struct fb_var_screeninfo current_var;
static int currcon = 0;

static char voyafb_name[] = "voyager_crt_csr_fb";

static int voyafb_get_fix(struct fb_fix_screeninfo* fix, int con,
			    struct fb_info* info)
{
	memset(fix, 0, sizeof(struct fb_fix_screeninfo));

	strcpy(fix->id, "voyager_crt_csr_fb");

	fix->smem_start  = (unsigned long)vram_base_phys;
	fix->smem_len    = current_var.xres_virtual * current_var.yres_virtual * current_var.bits_per_pixel / 8;
	fix->type        = FB_TYPE_PACKED_PIXELS;
	fix->type_aux    = 0;
	fix->visual      = FB_VISUAL_TRUECOLOR;
	fix->xpanstep    = 0;
	fix->ypanstep    = 0;
#if defined(CONFIG_FBCON_ROTATE_R) || defined(CONFIG_FBCON_ROTATE_L)
	fix->xwrapstep   = 0;
#else
	fix->ywrapstep   = 0;
#endif
        fix->line_length = current_var.xres_virtual * current_var.bits_per_pixel / 8;

	return 0;
}
        
static int voyafb_get_var(struct fb_var_screeninfo* var, int con,
                           struct fb_info* info)
{
	memcpy(var, &current_var, sizeof(struct fb_var_screeninfo));
	return 0;
}

static int voyafb_set_var(struct fb_var_screeninfo* var, int con,
			    struct fb_info* info)
{
	if (var->xres != CSR_XRES) 
		return -EINVAL;
	if (var->yres != CSR_YRES)
		return -EINVAL;
	if (var->xres_virtual != CSR_XRES)
		return -EINVAL;
	if (var->yres_virtual != CSR_YRES)
		return -EINVAL;
/*
	if (var->xoffset != 0)
		return -EINVAL;
	if (var->yoffset != 0)
		return -EINVAL;
*/
	if (var->bits_per_pixel != CSR_BPP)
		return -EINVAL;
	if (var->grayscale != 0)
		return -EINVAL;
	if (var->nonstd != 0)
		return -EINVAL;
	if (var->activate != FB_ACTIVATE_NOW)
		return -EINVAL;
	if (var->pixclock != 0)
		return -EINVAL;
	if (var->left_margin != 0)
		return -EINVAL;
	if (var->right_margin != 0)
		return -EINVAL;
	if (var->hsync_len != 0)
		return -EINVAL;
	if (var->vsync_len != 0)
		return -EINVAL;
	if (var->sync != 0)
		return -EINVAL;
	if (var->vmode != FB_VMODE_NONINTERLACED)
		return -EINVAL;
	if(change_mode(var) != 0) {
		return -EINVAL;
	}
//	size = var->xres * var->yres * var->bits_per_pixel / 8;
//	memset(vram_base_phys,0,size);
	memcpy(&current_var, var, sizeof(struct fb_var_screeninfo));

	return 0;

}

static int voyafb_getcolreg(unsigned regno, unsigned* red, unsigned* green,
			      unsigned* blue, unsigned* transp,
			      struct fb_info* info)
{
	if (regno >= 2) return 1;

	*transp = 0;
	if(regno == 0) {
		*green = *red = *blue = 0;
	}
	else {
		*green = *red = *blue = 1;
	}

	return 0;
}

static int voyafb_setcolreg(unsigned regno, unsigned red, unsigned green,
			     unsigned blue, unsigned transp,
			     struct fb_info* info)
{
	return 0;
}

static int voyafb_get_cmap(struct fb_cmap* cmap, int kspc, int con,
			    struct fb_info* info)
{
	if (con == currcon)
		return fb_get_cmap(cmap, kspc, voyafb_getcolreg, info);
	else if (fb_display[con].cmap.len) /* non default colormap? */
		fb_copy_cmap(&fb_display[con].cmap, cmap, kspc ? 0 : 2);
	else
		fb_copy_cmap(fb_default_cmap(
		1 << fb_display[con].var.bits_per_pixel), cmap, kspc ? 0 : 2);

	return 0;
}

static int voyafb_set_cmap(struct fb_cmap* cmap, int kspc, int con,
			 struct fb_info* info)
{
	int err;

	if (!fb_display[con].cmap.len) {
		if ((err = fb_alloc_cmap(&fb_display[con].cmap,
				 1 << fb_display[con].var.bits_per_pixel, 0)))
			return err;
	}

	if (con == currcon) {
	        return fb_set_cmap(cmap, kspc, voyafb_setcolreg, info);
	}
	else {
		fb_copy_cmap(cmap, &fb_display[con].cmap, kspc ? 0 : 1);
	}

	return 0;
}

static int voyafb_ioctl(struct inode* inode, struct file* file,
			 unsigned int cmd, unsigned long arg, int con,
			 struct fb_info* info)
{
	if(cmd == VOYAGER_IOCTL_ENABLE) {
		if(arg == 0) {
			*(volatile unsigned long *)(CRT_HWC_ADDRESS) &= 0x7fffffff;
		}
		else {
			*(volatile unsigned long *)(CRT_HWC_ADDRESS) |= 0x80000000;
		}
		return 0;
	}
	else if(cmd == VOYAGER_IOCTL_COLOR_1) {
		*(volatile unsigned long *)(CRT_HWC_COLOR_12) = arg;
		return 0;
	}
	else if(cmd == VOYAGER_IOCTL_COLOR_2) {
		*(volatile unsigned long *)(CRT_HWC_COLOR_3) = arg;
		return 0;
	}
	return -EINVAL;
}


static int __init voyafb_map_video_memory(void)
{
	if (vram_base != NULL)
		return -EINVAL;
        vram_base_phys = (u_char *)VOY_VRAM_TOP7;
	vram_base = (u_char*)ioremap((u_long)vram_base_phys,
				     ALLOCATED_FB_MEM_SIZE);
	return (vram_base == NULL ? -EINVAL : 0);
}

static void voyafb_set_disp(int con, struct fb_info* info)
{
	struct fb_fix_screeninfo fix;
	struct display* display;

	voyafb_get_fix(&fix, con, info);

	if (con >= 0) {
		display = &fb_display[con];
	}
	else {
		display = &disp[0];
	}

	if (con < 0) { con = 0; }

	display->screen_base    = (u_char*)vram_base;
	display->visual         = fix.visual;
	display->type           = fix.type;
	display->type_aux       = fix.type_aux;
#if defined(CONFIG_FBCON_ROTATE_R) || defined(CONFIG_FBCON_ROTATE_L)
	display->xpanstep       = fix.xpanstep;
	display->xwrapstep      = fix.xwrapstep;
#else
	display->ypanstep       = fix.ypanstep;
	display->ywrapstep      = fix.ywrapstep;
#endif
	display->can_soft_blank = 0;
	display->inverse        = 0;
	display->line_length    = fix.line_length;
#if defined(CONFIG_FBCON_ROTATE_R) || defined(CONFIG_FBCON_ROTATE_L)
	display->scrollmode     = SCROLL_XREDRAW;
#else
	display->scrollmode     = SCROLL_YREDRAW;
#endif

#if defined(FBCON_HAS_CFB2)
	display->dispsw         = &fbcon_cfb2;
	disp->dispsw_data       = fbcon_cmap_cfb2;
#else
#error NO FBCON_HAS_CFB2
	display->dispsw         = &fbcon_dummy;
#endif
}
  
static void voyafb_disable_voy_controller(void)
{
	*(volatile unsigned long *)(CRT_HWC_ADDRESS) &= 0x7fffffff;
}

static void voyafb_enable_voy_controller(void)
{
//	*(volatile unsigned long *)(CRT_HWC_ADDRESS) = 0x80000000 + (VOY_VRAM_TOP7 & 0x00ffffff);
	*(volatile unsigned long *)(CRT_HWC_ADDRESS) = (VOY_VRAM_TOP7 & 0x00ffffff);
	*(volatile unsigned long *)(CRT_HWC_COLOR_12) = 0x5555aaaa;
	*(volatile unsigned long *)(CRT_HWC_COLOR_3) = 0x0000ffff;
	change_mode(&current_var);
}

static int voyafb_updatevar(int con, struct fb_info* info)
{
	return 0;
}

static void voyafb_blank(int blank, struct fb_info* info)
{
	if (blank) {
		voyafb_disable_voy_controller();
	}
	else {
		voyafb_enable_voy_controller();
	}
}

static int voyafb_switch(int con, struct fb_info* info)
{ 
	currcon = con;
	fb_display[con].var.activate = FB_ACTIVATE_NOW;
	voyafb_set_var(&fb_display[con].var, con, info);
	return 0;
}

#ifdef CONFIG_PM
static int voyafb_pm_callback(struct pm_dev* pm_dev,
				pm_request_t req, void* data)
{
	switch (req) {
	case PM_SUSPEND:
		voyafb_disable_voy_controller();
		break;
	case PM_RESUME:
		voyafb_enable_voy_controller();
		break;
	}

	return 0;
}
#endif

int __init voyafb_init7(void)
{
int	size;

	current_var.xres           = CSR_XRES;
	current_var.yres           = CSR_YRES;
	current_var.xres_virtual   = CSR_XRES;
	current_var.yres_virtual   = CSR_YRES;
	current_var.bits_per_pixel = CSR_BPP;
	current_var.xoffset        = 0;
	current_var.yoffset        = 0;
	current_var.grayscale      = 0;
	current_var.nonstd         = 0;
	current_var.activate       = FB_ACTIVATE_NOW;
	current_var.height         = CSR_YRES;
	current_var.width          = CSR_XRES;
        current_var.red.length     = 0;
	current_var.blue.length    = 0;
	current_var.green.length   = 0;
	current_var.transp.length  = 0;
	current_var.red.offset     = 0;
	current_var.green.offset   = 0;
	current_var.blue.offset    = 0;
	current_var.transp.offset  = 0;
	current_var.pixclock       = 0;
	current_var.left_margin    = 0;
	current_var.right_margin   = 0;
	current_var.hsync_len      = 0;
	current_var.vsync_len      = 0;
	current_var.sync           = 0;
	current_var.vmode          = FB_VMODE_NONINTERLACED;

	fb_info.changevar   = NULL;
	strcpy(&fb_info.modename[0], voyafb_name);
	fb_info.fontname[0] = 0;
	fb_info.disp        = disp;
	fb_info.switch_con  = &voyafb_switch;
	fb_info.updatevar   = &voyafb_updatevar;
	fb_info.blank       = &voyafb_blank;	
	fb_info.node        = -1;
	fb_info.fbops       = &voyafb_ops;
	fb_info.flags       = FBINFO_FLAG_DEFAULT;
	
	if (voyafb_map_video_memory() < 0)
		return -EINVAL;

	size = current_var.xres_virtual * current_var.yres_virtual * current_var.bits_per_pixel / 8;
	memset(vram_base_phys,0,size);

        voyafb_get_var(&disp[0].var, 0, &fb_info);
	voyafb_set_disp(-1, &fb_info);

	if (register_framebuffer(&fb_info) < 0) {
#ifdef DEBUG
		printk(KERN_ERR
		       "unable to register VOYAGER frame buffer crt csr plane\n");
#endif
		return -EINVAL;
	}

#ifdef CONFIG_PM
	fb_pm_dev = pm_register(PM_SYS_DEV, 0, voyafb_pm_callback);
#endif
	voyafb_enable_voy_controller();

	printk("VOYAGER frame buffer crt csr plane initialized.\n");

	return 0;
}

//------------------------------------------------------------------------------------
static	int	change_mode(struct fb_var_screeninfo *var)
{
	if((var->xoffset < 0)||(var->xoffset > XRES)||
	   (var->yoffset < 0)||(var->yoffset > YRES))
	{
		return(-1);
	}
	*(volatile unsigned long *)(CRT_HWC_LOCATION) = (var->yoffset << 16) | 
								      var->xoffset;
	return(0);
}
