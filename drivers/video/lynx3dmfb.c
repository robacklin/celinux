/*
 * BRIEF MODULE DESCRIPTION
 *	Lynx3DM+ frame buffer driver
 *        for AiD Alchemy Titanium Board
 *
 * Copyright 2003 mycable GmbH
 * Author: mycable GmbH
 *         Joerg Ritter jr@mycable.de
 *
 *
 * Based on:
 * linux/drivers/video/skeletonfb.c -- Skeleton for a frame buffer device
 *  Created 28 Dec 1997 by Geert Uytterhoeven
 *
 *  This program is free software; you can redistribute	 it and/or modify it
 *  under  the terms of	 the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the	License, or (at your
 *  option) any later version.
 *
 *  THIS  SOFTWARE  IS PROVIDED	  ``AS	IS'' AND   ANY	EXPRESS OR IMPLIED
 *  WARRANTIES,	  INCLUDING, BUT NOT  LIMITED  TO, THE IMPLIED WARRANTIES OF
 *  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN
 *  NO	EVENT  SHALL   THE AUTHOR  BE	 LIABLE FOR ANY	  DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 *  NOT LIMITED	  TO, PROCUREMENT OF  SUBSTITUTE GOODS	OR SERVICES; LOSS OF
 *  USE, DATA,	OR PROFITS; OR	BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 *  ANY THEORY OF LIABILITY, WHETHER IN	 CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 *  THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *  You should have received a copy of the  GNU General Public License along
 *  with this program; if not, write  to the Free Software Foundation, Inc.,
 *  675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/tty.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/fb.h>
#include <linux/init.h>
#include <linux/pci.h>

#include <asm/au1000.h>
#include "lynx3dmfb.h"

#include <video/fbcon.h>
#include <video/fbcon-mfb.h>
#include <video/fbcon-cfb2.h>
#include <video/fbcon-cfb4.h>
#include <video/fbcon-cfb8.h>
#include <video/fbcon-cfb16.h>


#define	dbg(fmt, args...) printk(KERN_INFO "%s: " fmt, __func__, ## args)

#define DEB(fmt, args...) if (i2c_debug>=1) dbg(fmt, ## args)
#define DEB2(fmt, args...) if (i2c_debug>=2) dbg(fmt, ## args)
#define DEB3(fmt, args...) if (i2c_debug>=3) dbg(fmt, ## args)
#define DEBPROTO(fmt, args...) if (i2c_debug>=9) dbg(fmt, ## args)

#define PFX "lynx3dmfb"
#define err(format, arg...) printk(KERN_ERR PFX ": " format , ## arg)
#define info(format, arg...) printk(KERN_INFO PFX " %s: " format , __FUNCTION__, ## arg)
#define warn(format, arg...) printk(KERN_WARNING PFX ": " format , ## arg)
#define emerg(format, arg...) printk(KERN_EMERG PFX ": " format , ## arg)

#define PRINT_FUNCTIONNAME printk("%s\n", __FUNCTION__)

#if defined(CONFIG_MIPS_XXS1500)
// FIXME put in the correct CONFIG_ for Titanium board
#else
#error Unknown Au1500 board
#endif

#define CMAPSIZE 16

//static char *options;
//MODULE_PARM(options, "s");
u32 lynxfb_initialised;

struct lynxfb_info {
	struct fb_info_gen gen;
	unsigned long 	fb_size;

	unsigned long 	lynxbase_phys;
	unsigned long 	membase_phys;
	void 		*lynxbase_virt;
	void 		*membase_virt;

	int 		xres;
	int 		yres;
	
	int 		mmaped;
	int 		nohwcursor;

	struct { unsigned red, green, blue, pad; } palette[256];

#if defined(FBCON_HAS_CFB16)
	u16 		fbcon_cmap16[CMAPSIZE];
#endif
};


struct lynxfb_par {
	struct fb_var_screeninfo var;
	int line_length;  // in bytes
	int cmap_len;     // color-map length
};


/*-------------------------------------------------------------------------*/
/*  function prototypes                                                    */
/*-------------------------------------------------------------------------*/

int lynxfb_init(void);
static int lynx3dm_init(struct lynxfb_info *fb_info);
void lynxfb_setup(char *options, int *ints);
static int lynxfb_mmap(struct fb_info *fb, struct file *file, 
		       struct vm_area_struct *vma);
static int lynx_blank(int blank_mode, struct fb_info_gen *info);
static int lynxfb_ioctl(struct inode *inode, struct file *file, u_int cmd,
			u_long arg, int con, struct fb_info *info);

/*-------------------------------------------------------------------------*/
/*  global variables                                                       */
/*-------------------------------------------------------------------------*/

static u32 lynx_regbase;                   /* Ravin-E register base       */
static u32 lynx_mem_noburst;               /* Ravin-E memory base         */
static u32 lynx_mem_burst;                 /* Ravin-E memory base, burst  */



/*-------------------------------------------------------------------------*/
/*  static variables													   */
/*-------------------------------------------------------------------------*/

static struct lynxfb_info fb_info;
static struct lynxfb_par current_par;
static struct display disp;

static int cur_timing;

static volatile u8	*FrameBuffer;
static volatile u8	*MMIO;
static volatile u8	*DPR;
static volatile u8	*VPR;
static volatile u8	*CPR;
static volatile u8	*DataPort;
static unsigned int	Width, Height, BitsPerPixel;
static unsigned int	Bus, Slot;



/*-------------------------------------------------------------------------*/
void lynx_nocursor(struct display *p, int mode, int xx, int yy){};

static struct fb_ops lynxfb_ops = {
	owner:		THIS_MODULE,
	fb_get_fix:	fbgen_get_fix,
	fb_get_var:	fbgen_get_var,
	fb_set_var:	fbgen_set_var,
	fb_get_cmap:	fbgen_get_cmap,
	fb_set_cmap:	fbgen_set_cmap,
	fb_pan_display: fbgen_pan_display,
	fb_ioctl:       lynxfb_ioctl,
	fb_mmap:        lynxfb_mmap,
};

static void lynx_detect(void)
{
	PRINT_FUNCTIONNAME;
	/*
	 *  This function should detect the current video mode settings 
	 *  and store it as the default video mode
	 */


}

static int lynx_encode_fix(struct fb_fix_screeninfo *fix, 
		const void *_par, struct fb_info_gen *_info)
{
	struct lynxfb_info *info = (struct lynxfb_info *) _info;
	struct lynxfb_par *par = (struct lynxfb_par *) _par;
	struct fb_var_screeninfo *var = &par->var;
	memset(fix, 0, sizeof(struct fb_fix_screeninfo));

	fix->smem_start = info->membase_phys;
	fix->smem_len = info->fb_size;
	fix->type = FB_TYPE_PACKED_PIXELS;
	fix->type_aux = 0;
	fix->visual = (var->bits_per_pixel == 8) ?
      	FB_VISUAL_PSEUDOCOLOR	: FB_VISUAL_TRUECOLOR;
	fix->ywrapstep = 0;
	fix->xpanstep = 1;
	fix->ypanstep = 1;
	fix->line_length = current_par.line_length;
	return 0;
}

static void set_color_bitfields(struct fb_var_screeninfo *var)
{
	switch (var->bits_per_pixel) {
	case 8:
		var->red.offset = 0;
		var->red.length = 8;
		var->green.offset = 0;
		var->green.length = 8;
		var->blue.offset = 0;
		var->blue.length = 8;
		var->transp.offset = 0;
		var->transp.length = 0;
		break;
	case 16:	/* RGB 565 */
		var->red.offset = 11;
		var->red.length = 5;
		var->green.offset = 5;
		var->green.length = 6;
		var->blue.offset = 0;
		var->blue.length = 5;
		var->transp.offset = 0;
		var->transp.length = 0;
		break;
	case 24:	/* RGB 888 */
		var->red.offset = 16;
		var->red.length = 8;
		var->green.offset = 8;
		var->green.length = 8;
		var->blue.offset = 0;
		var->blue.length = 8;
		var->transp.offset = 0;
		var->transp.length = 0;
		break;
	}

	var->red.msb_right = 0;
	var->green.msb_right = 0;
	var->blue.msb_right = 0;
	var->transp.msb_right = 0;
}

static int lynx_decode_var(const struct fb_var_screeninfo *var, 
		void *_par, struct fb_info_gen *_info)
{
	struct lynxfb_par *par = (struct lynxfb_par *)_par;

	/*
	 * FIXME 
	 * check if settings make sense
	 */

	memset(par, 0, sizeof(struct lynxfb_par));
	par->var = *var;
	
	switch (var->bits_per_pixel) {
	case 8:
		par->var.bits_per_pixel = 8;
		break;
	case 16:
		par->var.bits_per_pixel = 16;
		break;
#if 0		
	case 24:
		par->var.bits_per_pixel = 24;
		break;
#endif
	default:
		printk("color depth %d bpp not supported\n",
			var->bits_per_pixel);
		return -EINVAL;
	}
	set_color_bitfields(&par->var);
	par->cmap_len = (par->var.bits_per_pixel == 8) ? 256 : 16;
	return 0;
}

static int lynx_encode_var(struct fb_var_screeninfo *var, 
		const void *par, struct fb_info_gen *_info)
{

	*var = ((struct lynxfb_par *)par)->var;
	return 0;
}

static void 
lynx_get_par(void *_par, struct fb_info_gen *_info)
{
	*(struct lynxfb_par *)_par = current_par;
}

static void lynx_set_par(const void *par, struct fb_info_gen *info)
{
	/* nothing to do: we don't change any settings */
}

static int lynx_getcolreg(unsigned regno, unsigned *red, unsigned *green,
			 unsigned *blue, unsigned *transp,
			 struct fb_info *info)
{

	struct lynxfb_info* i = (struct lynxfb_info*)info;

	if (regno > 255)
		return 1;
   
	*red    = i->palette[regno].red; 
	*green  = i->palette[regno].green; 
	*blue   = i->palette[regno].blue; 
	*transp = 0;

	return 0;
}

static int lynx_setcolreg(unsigned regno, unsigned red, unsigned green,
			 unsigned blue, unsigned transp,
			 struct fb_info *info)
{
	struct lynxfb_info* i = (struct lynxfb_info *)info;

	if (regno > 255)
		return 1;

	i->palette[regno].red    = red;
	i->palette[regno].green  = green;
	i->palette[regno].blue   = blue;
   
#if defined(FBCON_HAS_CFB16)
	i->fbcon_cmap16[regno] =
		((red & 0xf800) >> 0) |
		((green & 0xfc00) >> 5) |
		((blue & 0xf800) >> 11);
#endif
#if defined(FBCON_HAS_CFB24)
	i->fbcon_cmap24[regno] =
		((red & 0xff00) << 8) |
		((green & 0xff00) >> 0) |
		((blue & 0xff00) >> 8);
#endif
	return 0;
}


static int  lynx_blank(int blank_mode, struct fb_info_gen *_info)
{

	switch (blank_mode) {
	case VESA_NO_BLANKING:
		/* turn on panel */
		printk("turn on panel\n");
		break;

	case VESA_VSYNC_SUSPEND:
	case VESA_HSYNC_SUSPEND:
	case VESA_POWERDOWN:
		/* turn off panel */
		printk("turn off panel\n");
		break;
	default: 
		break;

	}
	return 0;
}

static void lynx_set_disp(const void *unused, struct display *disp,
			 struct fb_info_gen *info)
{
	disp->screen_base = (char *)fb_info.membase_virt;
	switch (disp->var.bits_per_pixel) {
#ifdef FBCON_HAS_CFB8
	case 8:
		disp->dispsw = &fbcon_cfb8;
		if (fb_info.nohwcursor)
			fbcon_cfb8.cursor = lynx_nocursor;
		break;
#endif
#ifdef FBCON_HAS_CFB16
	case 16:
		disp->dispsw = &fbcon_cfb16;
		disp->dispsw_data = fb_info.fbcon_cmap16;
		if (fb_info.nohwcursor)
			fbcon_cfb16.cursor = lynx_nocursor;
		break;
#endif
#ifdef FBCON_HAS_CFB24
	case 24:
		disp->dispsw = &fbcon_cfb24;
		disp->dispsw_data = fb_info.fbcon_cmap24;
		if (fb_info.nohwcursor)
			fbcon_cfb24.cursor = lynx_nocursor;
		break;
#endif
	default:
		disp->dispsw = &fbcon_dummy;
		disp->dispsw_data = NULL;
		break;
	}
}



static int
lynxfb_mmap(struct fb_info *_fb,
	     struct file *file,
	     struct vm_area_struct *vma)
{
	unsigned int len;
	unsigned long start=0, off;
	struct lynxfb_info *fb = (struct lynxfb_info *)_fb;

	if (vma->vm_pgoff > (~0UL >> PAGE_SHIFT)) {
		return -EINVAL;
	}
    
	start = fb_info.membase_phys & PAGE_MASK;
	len = PAGE_ALIGN((start & ~PAGE_MASK) + fb_info.fb_size);

	off = vma->vm_pgoff << PAGE_SHIFT;

	if ((vma->vm_end - vma->vm_start + off) > len) {
		return -EINVAL;
	}

	off += start;
	vma->vm_pgoff = off >> PAGE_SHIFT;

	pgprot_val(vma->vm_page_prot) &= ~_CACHE_MASK;
	pgprot_val(vma->vm_page_prot) |= _CACHE_UNCACHED;

	/* This is an IO map - tell maydump to skip this VMA */
	vma->vm_flags |= VM_IO;
    
	if (io_remap_page_range(vma->vm_start, off,
				vma->vm_end - vma->vm_start,
				vma->vm_page_prot)) {
		return -EAGAIN;
	}


	fb->mmaped = 1;
	return 0;
}

int lynx_pan_display(const struct fb_var_screeninfo *var,
		       struct fb_info_gen *info)
{
	return 0;
}

static int lynxfb_ioctl(struct inode *inode, struct file *file, u_int cmd,
			  u_long arg, int con, struct fb_info *info)
{
	/* nothing to do yet */
	printk("%s cmd=0x%x", __FUNCTION__, cmd);
	return -EINVAL;
}

static struct fbgen_hwswitch lynx_switch = {
	lynx_detect, 
	lynx_encode_fix, 
	lynx_decode_var, 
	lynx_encode_var, 
	lynx_get_par, 
	lynx_set_par, 
	lynx_getcolreg, 
	lynx_setcolreg, 
	lynx_pan_display, 
	lynx_blank, 
	lynx_set_disp
};


int __init lynx3dmfb_init(void)
{
	//u32 sys_clksrc;

	struct pci_dev *pdev = NULL;
	struct pci_dev *pdev2 = NULL;
	u8 *pci_io;
	char sbuf[32];

	u8 rev_code;


//	if (options) {
//		lynxfb_setup(options, 0);
//	}
	if (!(pdev = pci_find_device(PCI_SMI_VENDORID,
				     PCI_LYNX3DMP_DEVICEID, pdev))) {
		printk("  Lynx3M+ not found\n");
		return -ENODEV;
	} else {
		printk("  Lynx3DM+ found\n");
	}
	if (pci_enable_device(pdev) != 0) {
		printk("  couldn't enable Lynx3DM+\n");
		return -ENODEV;
	} else {
		printk("  Lynx3DM+ enabled\n");
	}
#if 0
	int i;
	for (i = 0; i < DEVICE_COUNT_RESOURCE; ++i) {
		printk("  resource%d: type=0x%lx start=0x%lx end=0x%lx\n", i, pdev->resource[i].flags, 
		       pdev->resource[i].start, pdev->resource[i].end);
	}
#endif
	pci_write_config_dword(pdev, 0x04, 0x03);
	
	pci_io = ioremap(Au1500_PCI_IO_START, 0x1000); 
			 
	if (!pci_io) {
		printk(KERN_ERR "Unable to ioremap pci space\n");
		return -1;
	}
	pci_io[0x3c4] = 0x18;
	pci_io[0x3c5] = 0x41;
	pci_write_config_dword(pdev, 0x04, 0x02);
	
	
	// map Lynx registers
	fb_info.lynxbase_phys = pci_resource_start(pdev, 0);
	fb_info.membase_phys  = fb_info.lynxbase_phys + 0x200000;
	fb_info.lynxbase_virt = ioremap_nocache(fb_info.lynxbase_phys, 8*1024*1024);
	fb_info.membase_virt  = fb_info.lynxbase_virt + 0x200000;

	fb_info.xres = 800;//defd_timings[cur_timing].screen_width;
	fb_info.yres = 600;//defd_timings[cur_timing].screen_height;
	//fb_info.xres = 640;//defd_timings[cur_timing].screen_width;
	//fb_info.yres = 480;//defd_timings[cur_timing].screen_height;
	
	fb_info.fb_size = fb_info.xres * fb_info.yres * 16 / 8;
	 
	// hard coded values so far
	current_par.var.xres = fb_info.xres;
	current_par.var.xres_virtual = fb_info.xres;
	current_par.var.yres = fb_info.yres;
	current_par.var.yres_virtual = fb_info.yres;
	current_par.var.bits_per_pixel = 16;

	/* FIXME only works for 8/16 bpp */
	current_par.line_length = fb_info.xres * 16 / 8; /* in bytes */

	lynx3dm_init(&fb_info);

	fb_info.gen.parsize = sizeof(struct lynxfb_par);
	fb_info.gen.fbhw = &lynx_switch;

	sprintf(sbuf, "Lynx3DM+ %dx%d-16", fb_info.xres, fb_info.yres);
	strcpy(fb_info.gen.info.modename, sbuf);
   	fb_info.gen.info.changevar = NULL;
	fb_info.gen.info.node = -1;

	fb_info.gen.info.fbops = &lynxfb_ops;
	fb_info.gen.info.disp = &disp;
	fb_info.gen.info.switch_con = &fbgen_switch;
	fb_info.gen.info.updatevar = &fbgen_update_var;
	fb_info.gen.info.blank = &fbgen_blank;
	fb_info.gen.info.flags = FBINFO_FLAG_DEFAULT;

	/* This should give a reasonable default video mode */
	fbgen_get_var(&disp.var, -1, &fb_info.gen.info);
	fbgen_do_set_var(&disp.var, 1, &fb_info.gen);
	fbgen_set_disp(-1, &fb_info.gen);
	fbgen_install_cmap(0, &fb_info.gen);
	if (register_framebuffer(&fb_info.gen.info) < 0)
		return -EINVAL;
	printk(KERN_INFO "fb%d: %s frame buffer device\n", 
			GET_FB_IDX(fb_info.gen.info.node), 
			fb_info.gen.info.modename);

	/* uncomment this if your driver cannot be unloaded */
	/* MOD_INC_USE_COUNT; */

	lynxfb_initialised = 1;
	return 0;
}

void lynxfb_cleanup(struct fb_info *info)
{
	unregister_framebuffer(info);
}


void lynx3dmfb_setup(char *options, int *ints)
{
	// FIXME 
	// parse options
	// function is called BEFORE lynxfb_init()
	// valid options
	//   "640x480"   default
	//   "800x600"
	//   "800x480"
	
	mdelay(2);
	
	printk("%s options=\"%s\"\n", __FUNCTION__, options);
	
	cur_timing = 0; 	// set default
	if (strncmp(options, "800x480", 7) == 0) {
		printk("  use 800x480\n");
		cur_timing = 1;
	}
	else if (strncmp(options, "800x600", 7) == 0) {
		printk("  use 800x600\n");
		cur_timing = 2;
	}
}


/*-------------------------------------------------------------------------*/


/*-------------------------------------------------------------------------*/
void SetupDPR( unsigned int	Width,
	       unsigned int	Height,
	       unsigned int	BitsPerPixel)
{
	u32	Pitch;
	u16	DataFormat;

	/* Setup variables. */
	if (BitsPerPixel == 24)	{
		Pitch = Width * 3;
		DataFormat = 3;
	} else {
		Pitch = Width;
		DataFormat = BitsPerPixel / 16;
	}

	/* Setup DPR registers. */
	DPR32(0x10) = (Pitch << 16) | Pitch;
	DPR16(0x1E) = (DataFormat << 4) | 0x000F;
//	DPR32(0x1E - 2) = (((DataFormat << 4) | 0x000F) << 16);
	DPR32(0x24) = 0xFFFFFFFF;
	DPR32(0x28) = 0xFFFFFFFF;
	DPR32(0x2C) = 0;
	DPR32(0x3C) = (Pitch << 16) | Pitch;
	DPR32(0x40) = 0;
	DPR32(0x44) = 0;
}

void SetupVPR(unsigned int Width, unsigned int BitsPerPixel)
{
	u32 GraphicsDataFormat;
	unsigned int DataSourceWidth;

	switch (BitsPerPixel) {
	case 8:
		GraphicsDataFormat = 0;
		break;
	case 16:
		GraphicsDataFormat = 2;
		break;

	case 24:
		GraphicsDataFormat = 4;
		break;

	case 32:
		GraphicsDataFormat = 3;
		break;
	default:
		GraphicsDataFormat = 4;
		break;
	}
	DataSourceWidth = Width * (BitsPerPixel / 8) / 8;

	VPR32(0x00) = GraphicsDataFormat << 16;
	VPR32(0x08) = 0xFFFFFFFF;
	VPR32(0x0C) = 0;
	VPR32(0x10) = ((DataSourceWidth + 2) << 16) | DataSourceWidth;
	VPR32(0x3C) = 0;
	VPR32(0x40) = 0;
	VPR32(0x54) = 0x07216543;
	VPR32(0x58) = 0x00000444;
	VPR32(0x5C) = 0x00EDEDED;
}

/* Program SM710 to 800x600 mode. */
void SetMode(unsigned int *Width, unsigned int *Height, unsigned int *BitsPerPixel)
{
	u8	Temp;

	printk("%s %dx%d BitsPerPixel %d\n", 
			__FUNCTION__, *Width, *Height, *BitsPerPixel);
	/* Program MISCELLANEOUS OUTPUT register. */
	MMIO[0x3C2] = 0x2B;

	/* Disable palette. */
	Temp = MMIO[0x3DA]; MMIO[0x3C0] = 0x00;

	/* Program SEQUENCER registers. */
	MMIO[0x3C4] = 0x00; MMIO[0x3C5] = 0x01;
	MMIO[0x3C4] = 0x01; MMIO[0x3C5] = 0x01;
	MMIO[0x3C4] = 0x02; MMIO[0x3C5] = 0x0F;
	MMIO[0x3C4] = 0x03; MMIO[0x3C5] = 0x03;
	MMIO[0x3C4] = 0x04; MMIO[0x3C5] = 0x0E;
	MMIO[0x3C4] = 0x00; MMIO[0x3C5] = 0x03;

	/* Program CRTC registers. */
	MMIO[0x3D4] = 0x11; MMIO[0x3D5] = 0x0C;
	MMIO[0x3C4] = 0x33; MMIO[0x3C5] = 0x09;
	MMIO[0x3D4] = 0x00; MMIO[0x3D5] = 0x7F;
	MMIO[0x3D4] = 0x01; MMIO[0x3D5] = 0x63;
	MMIO[0x3D4] = 0x02; MMIO[0x3D5] = 0x63;
	MMIO[0x3D4] = 0x03; MMIO[0x3D5] = 0x00;
	MMIO[0x3D4] = 0x04; MMIO[0x3D5] = 0x69;
	MMIO[0x3D4] = 0x05; MMIO[0x3D5] = 0x19;
	MMIO[0x3D4] = 0x06; MMIO[0x3D5] = 0x72;
	MMIO[0x3D4] = 0x07; MMIO[0x3D5] = 0xF0;
	MMIO[0x3D4] = 0x08; MMIO[0x3D5] = 0x00;
	MMIO[0x3D4] = 0x09; MMIO[0x3D5] = 0x60;
	MMIO[0x3D4] = 0x0A; MMIO[0x3D5] = 0x00;
	MMIO[0x3D4] = 0x0B; MMIO[0x3D5] = 0x00;
	MMIO[0x3D4] = 0x0C; MMIO[0x3D5] = 0x00;
	MMIO[0x3D4] = 0x0D; MMIO[0x3D5] = 0x00;
	MMIO[0x3D4] = 0x0E; MMIO[0x3D5] = 0x00;
	MMIO[0x3D4] = 0x0F; MMIO[0x3D5] = 0x00;
	MMIO[0x3D4] = 0x10; MMIO[0x3D5] = 0x58;
	MMIO[0x3D4] = 0x11; MMIO[0x3D5] = 0x0C;
	MMIO[0x3D4] = 0x12; MMIO[0x3D5] = 0x57;
	MMIO[0x3D4] = 0x13; MMIO[0x3D5] = 0x64;
	MMIO[0x3D4] = 0x14; MMIO[0x3D5] = 0x40;
	MMIO[0x3D4] = 0x15; MMIO[0x3D5] = 0x57;
	MMIO[0x3D4] = 0x16; MMIO[0x3D5] = 0x00;
	MMIO[0x3D4] = 0x17; MMIO[0x3D5] = 0xE3;
	MMIO[0x3D4] = 0x18; MMIO[0x3D5] = 0xFF;

	/* Program GRAPHICS CONTROLLER registers. */
	MMIO[0x3CE] = 0x00; MMIO[0x3CF] = 0x00;
	MMIO[0x3CE] = 0x01; MMIO[0x3CF] = 0x00;
	MMIO[0x3CE] = 0x02; MMIO[0x3CF] = 0x00;
	MMIO[0x3CE] = 0x03; MMIO[0x3CF] = 0x00;
	MMIO[0x3CE] = 0x04; MMIO[0x3CF] = 0x00;
	MMIO[0x3CE] = 0x05; MMIO[0x3CF] = 0x40;
	MMIO[0x3CE] = 0x06; MMIO[0x3CF] = 0x05;
	MMIO[0x3CE] = 0x07; MMIO[0x3CF] = 0x0F;
	MMIO[0x3CE] = 0x08; MMIO[0x3CF] = 0xFF;

	/* Program ATTRIBUTE CONTROLLER registers. */
	Temp = MMIO[0x3DA];
	MMIO[0x3C0] = 0x00; MMIO[0x3C0] = 0x00;
	MMIO[0x3C0] = 0x01; MMIO[0x3C0] = 0x01;
	MMIO[0x3C0] = 0x02; MMIO[0x3C0] = 0x02;
	MMIO[0x3C0] = 0x03; MMIO[0x3C0] = 0x03;
	MMIO[0x3C0] = 0x04; MMIO[0x3C0] = 0x04;
	MMIO[0x3C0] = 0x05; MMIO[0x3C0] = 0x05;
	MMIO[0x3C0] = 0x06; MMIO[0x3C0] = 0x06;
	MMIO[0x3C0] = 0x07; MMIO[0x3C0] = 0x07;
	MMIO[0x3C0] = 0x08; MMIO[0x3C0] = 0x08;
	MMIO[0x3C0] = 0x09; MMIO[0x3C0] = 0x09;
	MMIO[0x3C0] = 0x0A; MMIO[0x3C0] = 0x0A;
	MMIO[0x3C0] = 0x0B; MMIO[0x3C0] = 0x0B;
	MMIO[0x3C0] = 0x0C; MMIO[0x3C0] = 0x0C;
	MMIO[0x3C0] = 0x0D; MMIO[0x3C0] = 0x0D;
	MMIO[0x3C0] = 0x0E; MMIO[0x3C0] = 0x0E;
	MMIO[0x3C0] = 0x0F; MMIO[0x3C0] = 0x0F;
	MMIO[0x3C0] = 0x10; MMIO[0x3C0] = 0x41;
	MMIO[0x3C0] = 0x11; MMIO[0x3C0] = 0x00;
	MMIO[0x3C0] = 0x12; MMIO[0x3C0] = 0x0F;
	MMIO[0x3C0] = 0x13; MMIO[0x3C0] = 0x00;
	MMIO[0x3C0] = 0x14; MMIO[0x3C0] = 0x00;

	/* Program EXTENDED Sequencer registers. */


	MMIO[0x3C4] = 0x15; MMIO[0x3C5] = 0x0A;
	MMIO[0x3C4] = 0x17; MMIO[0x3C5] = 0x00;
	MMIO[0x3C4] = 0x18; MMIO[0x3C5] = 0x51;
	MMIO[0x3C4] = 0x19; MMIO[0x3C5] = 0x00;
	MMIO[0x3C4] = 0x1B; MMIO[0x3C5] = 0x00;
	MMIO[0x3C4] = 0x1D; MMIO[0x3C5] = 0x00;
	MMIO[0x3C4] = 0x1E; MMIO[0x3C5] = 0x00;
	MMIO[0x3C4] = 0x1F; MMIO[0x3C5] = 0x00;
	MMIO[0x3C4] = 0x20; MMIO[0x3C5] = 0xC4;
	MMIO[0x3C4] = 0x21; MMIO[0x3C5] = 0x30;
	MMIO[0x3C4] = 0x22; MMIO[0x3C5] = 0x02;
	MMIO[0x3C4] = 0x23; MMIO[0x3C5] = 0x00;
	MMIO[0x3C4] = 0x24; MMIO[0x3C5] = 0x01;
	MMIO[0x3C4] = 0x25; MMIO[0x3C5] = 0x00;
	MMIO[0x3C4] = 0x26; MMIO[0x3C5] = 0x00;
	MMIO[0x3C4] = 0x27; MMIO[0x3C5] = 0x00;
	MMIO[0x3C4] = 0x28; MMIO[0x3C5] = 0x00;
	MMIO[0x3C4] = 0x29; MMIO[0x3C5] = 0x00;
	MMIO[0x3C4] = 0x2A; MMIO[0x3C5] = 0x00;
	MMIO[0x3C4] = 0x2B; MMIO[0x3C5] = 0x00;
	MMIO[0x3C4] = 0x2C; MMIO[0x3C5] = 0x00;
	MMIO[0x3C4] = 0x2D; MMIO[0x3C5] = 0x00;
	MMIO[0x3C4] = 0x2E; MMIO[0x3C5] = 0x00;
	MMIO[0x3C4] = 0x2F; MMIO[0x3C5] = 0x00;
	MMIO[0x3C4] = 0x30; MMIO[0x3C5] = 0x36;  // FPR30: Flat panel type select
	MMIO[0x3C4] = 0x31; MMIO[0x3C5] = 0x43;
	MMIO[0x3C4] = 0x32; MMIO[0x3C5] = 0x20;
	MMIO[0x3C4] = 0x33; MMIO[0x3C5] = 0x09;
	MMIO[0x3C4] = 0x34; MMIO[0x3C5] = 0xC0;
	MMIO[0x3C4] = 0x35; MMIO[0x3C5] = 0x36;
	MMIO[0x3C4] = 0x36; MMIO[0x3C5] = 0x36;
	MMIO[0x3C4] = 0x37; MMIO[0x3C5] = 0x36;
	MMIO[0x3C4] = 0x38; MMIO[0x3C5] = 0x36;
	MMIO[0x3C4] = 0x39; MMIO[0x3C5] = 0x36;
	MMIO[0x3C4] = 0x3A; MMIO[0x3C5] = 0x36;
	MMIO[0x3C4] = 0x3B; MMIO[0x3C5] = 0x36;
	MMIO[0x3C4] = 0x3C; MMIO[0x3C5] = 0x00;
	MMIO[0x3C4] = 0x3D; MMIO[0x3C5] = 0x00;
	MMIO[0x3C4] = 0x3E; MMIO[0x3C5] = 0x03;
	MMIO[0x3C4] = 0x3F; MMIO[0x3C5] = 0xFF;
	MMIO[0x3C4] = 0x40; MMIO[0x3C5] = 0x00;
	MMIO[0x3C4] = 0x41; MMIO[0x3C5] = 0xFC;
	MMIO[0x3C4] = 0x42; MMIO[0x3C5] = 0x00;
	MMIO[0x3C4] = 0x43; MMIO[0x3C5] = 0x00;
	MMIO[0x3C4] = 0x44; MMIO[0x3C5] = 0x20;
	MMIO[0x3C4] = 0x45; MMIO[0x3C5] = 0x18;
	MMIO[0x3C4] = 0x46; MMIO[0x3C5] = 0x00;
	MMIO[0x3C4] = 0x47; MMIO[0x3C5] = 0xFC;
	MMIO[0x3C4] = 0x48; MMIO[0x3C5] = 0x20;
	MMIO[0x3C4] = 0x49; MMIO[0x3C5] = 0x0C;
	MMIO[0x3C4] = 0x4A; MMIO[0x3C5] = 0x44;
	MMIO[0x3C4] = 0x4B; MMIO[0x3C5] = 0x20;
	MMIO[0x3C4] = 0x4C; MMIO[0x3C5] = 0x00;
	MMIO[0x3C4] = 0x4D; MMIO[0x3C5] = 0x36;
	MMIO[0x3C4] = 0x4E; MMIO[0x3C5] = 0x16; //0x36;   
	MMIO[0x3C4] = 0x4F; MMIO[0x3C5] = 0x36;
	MMIO[0x3C4] = 0x50; MMIO[0x3C5] = 0x04;
	MMIO[0x3C4] = 0x51; MMIO[0x3C5] = 0x48;
	MMIO[0x3C4] = 0x52; MMIO[0x3C5] = 0x83;
	MMIO[0x3C4] = 0x53; MMIO[0x3C5] = 0x63;
	MMIO[0x3C4] = 0x54; MMIO[0x3C5] = 0x68;
	MMIO[0x3C4] = 0x55; MMIO[0x3C5] = 0x73;
	MMIO[0x3C4] = 0x56; MMIO[0x3C5] = 0x57;
	MMIO[0x3C4] = 0x57; MMIO[0x3C5] = 0x58;
	MMIO[0x3C4] = 0x58; MMIO[0x3C5] = 0x04;
	MMIO[0x3C4] = 0x59; MMIO[0x3C5] = 0x58;
	MMIO[0x3C4] = 0x5A; MMIO[0x3C5] = 0x7B;
	MMIO[0x3C4] = 0x5B; MMIO[0x3C5] = 0x36;
	MMIO[0x3C4] = 0x5C; MMIO[0x3C5] = 0x36;
	MMIO[0x3C4] = 0x5D; MMIO[0x3C5] = 0x00;
	MMIO[0x3C4] = 0x5E; MMIO[0x3C5] = 0x00;
	MMIO[0x3C4] = 0x5F; MMIO[0x3C5] = 0x36;
	MMIO[0x3C4] = 0x60; MMIO[0x3C5] = 0x01;
	MMIO[0x3C4] = 0x61; MMIO[0x3C5] = 0x00;
//	MMIO[0x3C4] = 0x62; MMIO[0x3C5] = 0x7E;
	MMIO[0x3C4] = 0x63; MMIO[0x3C5] = 0x1A;
	MMIO[0x3C4] = 0x64; MMIO[0x3C5] = 0x1A;
	MMIO[0x3C4] = 0x65; MMIO[0x3C5] = 0x00;
	MMIO[0x3C4] = 0x66; MMIO[0x3C5] = 0x00;
	MMIO[0x3C4] = 0x67; MMIO[0x3C5] = 0x00;
	MMIO[0x3C4] = 0x68; MMIO[0x3C5] = 0x50;
	MMIO[0x3C4] = 0x69; MMIO[0x3C5] = 0x04;
	MMIO[0x3C4] = 0x6A; MMIO[0x3C5] = 0x0C;
	MMIO[0x3C4] = 0x6B; MMIO[0x3C5] = 0x02;
	MMIO[0x3C4] = 0x6C; MMIO[0x3C5] = 0x1C;  
	MMIO[0x3C4] = 0x6D; MMIO[0x3C5] = 0x85;  
	MMIO[0x3C4] = 0x6E; MMIO[0x3C5] = 0x0B;
	MMIO[0x3C4] = 0x6F; MMIO[0x3C5] = 0x04;
	MMIO[0x3C4] = 0x70; MMIO[0x3C5] = 0x82;
	MMIO[0x3C4] = 0x71; MMIO[0x3C5] = 0x4D;
	MMIO[0x3C4] = 0x72; MMIO[0x3C5] = 0x06;
	MMIO[0x3C4] = 0x73; MMIO[0x3C5] = 0x35;
	MMIO[0x3C4] = 0x74; MMIO[0x3C5] = 0x04;
	MMIO[0x3C4] = 0x75; MMIO[0x3C5] = 0x00;


/*

	MMIO[0x3C4] = 0x76; MMIO[0x3C5] = 0x36;


	MMIO[0x3C4] = 0x77; MMIO[0x3C5] = 0x36;
	MMIO[0x3C4] = 0x78; MMIO[0x3C5] = 0x36;
	MMIO[0x3C4] = 0x79; MMIO[0x3C5] = 0x36;
	MMIO[0x3C4] = 0x7A; MMIO[0x3C5] = 0x36;
	MMIO[0x3C4] = 0x7B; MMIO[0x3C5] = 0x36;
	MMIO[0x3C4] = 0x7C; MMIO[0x3C5] = 0x36;
	MMIO[0x3C4] = 0x7D; MMIO[0x3C5] = 0x36;
	MMIO[0x3C4] = 0x7E; MMIO[0x3C5] = 0x36;
	MMIO[0x3C4] = 0x7F; MMIO[0x3C5] = 0x36;

*/
	
	MMIO[0x3C4] = 0x80; MMIO[0x3C5] = 0xFF;
	MMIO[0x3C4] = 0x81; MMIO[0x3C5] = 0x07;
	MMIO[0x3C4] = 0x82; MMIO[0x3C5] = 0x00;
	MMIO[0x3C4] = 0x83; MMIO[0x3C5] = 0x00;
	MMIO[0x3C4] = 0x84; MMIO[0x3C5] = 0xFF;
	MMIO[0x3C4] = 0x85; MMIO[0x3C5] = 0xFF;
	MMIO[0x3C4] = 0x86; MMIO[0x3C5] = 0xFF;
	MMIO[0x3C4] = 0x87; MMIO[0x3C5] = 0x36;
	MMIO[0x3C4] = 0x88; MMIO[0x3C5] = 0x00;
	MMIO[0x3C4] = 0x89; MMIO[0x3C5] = 0x00;
	MMIO[0x3C4] = 0x8A; MMIO[0x3C5] = 0x00;
	MMIO[0x3C4] = 0x8B; MMIO[0x3C5] = 0x00;
	MMIO[0x3C4] = 0x8C; MMIO[0x3C5] = 0xFF;
	MMIO[0x3C4] = 0x8D; MMIO[0x3C5] = 0xFF;
	MMIO[0x3C4] = 0x8E; MMIO[0x3C5] = 0x36;
	MMIO[0x3C4] = 0x8F; MMIO[0x3C5] = 0x36;
	MMIO[0x3C4] = 0x90; MMIO[0x3C5] = 0x00;
	MMIO[0x3C4] = 0x91; MMIO[0x3C5] = 0x00;
	MMIO[0x3C4] = 0x92; MMIO[0x3C5] = 0x00;
	MMIO[0x3C4] = 0x93; MMIO[0x3C5] = 0x00;
	MMIO[0x3C4] = 0x94; MMIO[0x3C5] = 0x36;
	MMIO[0x3C4] = 0x95; MMIO[0x3C5] = 0x36;
	MMIO[0x3C4] = 0x96; MMIO[0x3C5] = 0x36;
	MMIO[0x3C4] = 0x97; MMIO[0x3C5] = 0x36;
	MMIO[0x3C4] = 0x98; MMIO[0x3C5] = 0x36;
	MMIO[0x3C4] = 0x99; MMIO[0x3C5] = 0x36;
	MMIO[0x3C4] = 0x9A; MMIO[0x3C5] = 0x36;
	MMIO[0x3C4] = 0x9B; MMIO[0x3C5] = 0x36;
	MMIO[0x3C4] = 0x9C; MMIO[0x3C5] = 0x36;
	MMIO[0x3C4] = 0x9D; MMIO[0x3C5] = 0x36;
	MMIO[0x3C4] = 0x9E; MMIO[0x3C5] = 0x36;
	MMIO[0x3C4] = 0x9F; MMIO[0x3C5] = 0x36;
	MMIO[0x3C4] = 0xA0; MMIO[0x3C5] = 0x00;
	MMIO[0x3C4] = 0xA1; MMIO[0x3C5] = 0xFF;
	MMIO[0x3C4] = 0xA2; MMIO[0x3C5] = 0xFF;
	MMIO[0x3C4] = 0xA3; MMIO[0x3C5] = 0xFF;
	MMIO[0x3C4] = 0xA4; MMIO[0x3C5] = 0xFF;
	MMIO[0x3C4] = 0xA5; MMIO[0x3C5] = 0xED;
	MMIO[0x3C4] = 0xA6; MMIO[0x3C5] = 0xED;
	MMIO[0x3C4] = 0xA7; MMIO[0x3C5] = 0xED;
	MMIO[0x3C4] = 0xA8; MMIO[0x3C5] = 0xFF;
	MMIO[0x3C4] = 0xA9; MMIO[0x3C5] = 0xFF;
	MMIO[0x3C4] = 0xAA; MMIO[0x3C5] = 0xFF;
	MMIO[0x3C4] = 0xAB; MMIO[0x3C5] = 0xFF;
	MMIO[0x3C4] = 0xAC; MMIO[0x3C5] = 0xFF;
	MMIO[0x3C4] = 0xAD; MMIO[0x3C5] = 0xFF;
	MMIO[0x3C4] = 0xAE; MMIO[0x3C5] = 0xFF;
	MMIO[0x3C4] = 0xAF; MMIO[0x3C5] = 0xFF;


	/* Program EXTENDED CRTC registers. */
	MMIO[0x3D4] = 0x9E; MMIO[0x3D5] = 0x7B;
	MMIO[0x3D4] = 0x30; MMIO[0x3D5] = 0x00;
	MMIO[0x3D4] = 0x31; MMIO[0x3D5] = 0x00;
	MMIO[0x3D4] = 0x32; MMIO[0x3D5] = 0x00;
	MMIO[0x3D4] = 0x33; MMIO[0x3D5] = 0x00;
	MMIO[0x3D4] = 0x34; MMIO[0x3D5] = 0x00;
	MMIO[0x3D4] = 0x35; MMIO[0x3D5] = 0x88;
	MMIO[0x3D4] = 0x36; MMIO[0x3D5] = 0x02;
	MMIO[0x3D4] = 0x38; MMIO[0x3D5] = 0x00;
	MMIO[0x3D4] = 0x39; MMIO[0x3D5] = 0x00;
	MMIO[0x3D4] = 0x3A; MMIO[0x3D5] = 0x00;
	MMIO[0x3D4] = 0x3B; MMIO[0x3D5] = 0x40;
	MMIO[0x3D4] = 0x3C; MMIO[0x3D5] = 0x00;
	MMIO[0x3D4] = 0x3D; MMIO[0x3D5] = 0x00;
	MMIO[0x3D4] = 0x3E; MMIO[0x3D5] = 0xFF;
	MMIO[0x3D4] = 0x3F; MMIO[0x3D5] = 0xFF;
	MMIO[0x3D4] = 0x40; MMIO[0x3D5] = 0x7F;
	MMIO[0x3D4] = 0x41; MMIO[0x3D5] = 0x63;
	MMIO[0x3D4] = 0x42; MMIO[0x3D5] = 0x00;
	MMIO[0x3D4] = 0x43; MMIO[0x3D5] = 0x69;
	MMIO[0x3D4] = 0x44; MMIO[0x3D5] = 0x19;
	MMIO[0x3D4] = 0x45; MMIO[0x3D5] = 0x72;
	MMIO[0x3D4] = 0x46; MMIO[0x3D5] = 0x57;
	MMIO[0x3D4] = 0x47; MMIO[0x3D5] = 0x00;
	MMIO[0x3D4] = 0x48; MMIO[0x3D5] = 0x58;
	MMIO[0x3D4] = 0x49; MMIO[0x3D5] = 0x0C;
	MMIO[0x3D4] = 0x4A; MMIO[0x3D5] = 0xE0;
	MMIO[0x3D4] = 0x4B; MMIO[0x3D5] = 0x20;
	MMIO[0x3D4] = 0x4C; MMIO[0x3D5] = 0x63;
	MMIO[0x3D4] = 0x4D; MMIO[0x3D5] = 0x57;
	MMIO[0x3D4] = 0x90; MMIO[0x3D5] = 0x56;
	MMIO[0x3D4] = 0x91; MMIO[0x3D5] = 0x4A;
	MMIO[0x3D4] = 0x92; MMIO[0x3D5] = 0x5E;
	MMIO[0x3D4] = 0x93; MMIO[0x3D5] = 0x55;
	MMIO[0x3D4] = 0x94; MMIO[0x3D5] = 0x86;
	MMIO[0x3D4] = 0x95; MMIO[0x3D5] = 0x9D;
	MMIO[0x3D4] = 0x96; MMIO[0x3D5] = 0x8E;
	MMIO[0x3D4] = 0x97; MMIO[0x3D5] = 0xAA;
	MMIO[0x3D4] = 0x98; MMIO[0x3D5] = 0xDB;
	MMIO[0x3D4] = 0x99; MMIO[0x3D5] = 0x2A;
	MMIO[0x3D4] = 0x9A; MMIO[0x3D5] = 0xDF;
	MMIO[0x3D4] = 0x9B; MMIO[0x3D5] = 0x33;
	MMIO[0x3D4] = 0x9F; MMIO[0x3D5] = 0x00;
	MMIO[0x3D4] = 0xA0; MMIO[0x3D5] = 0x00;
	MMIO[0x3D4] = 0xA1; MMIO[0x3D5] = 0x00;
	MMIO[0x3D4] = 0xA2; MMIO[0x3D5] = 0x00;
	MMIO[0x3D4] = 0xA3; MMIO[0x3D5] = 0x00;
	MMIO[0x3D4] = 0xA4; MMIO[0x3D5] = 0x00;
	MMIO[0x3D4] = 0xA5; MMIO[0x3D5] = 0x00;
	MMIO[0x3D4] = 0xA6; MMIO[0x3D5] = 0x00;
	MMIO[0x3D4] = 0xA7; MMIO[0x3D5] = 0x00;

	/* Program VPR registers. */

	// There are obviously problems with 24bpp in or around TinyX
	// Therefore we currently support 16bpp only 

	SetupVPR(800, 16);
	//SetupVPR(640, 16);

	/* Enable palette. */
	Temp = MMIO[0x3DA]; MMIO[0x3C0] = 0x20;

	/* Initialize the Drawing Processor */

	SetupDPR(*Width = 800, *Height = 600, *BitsPerPixel = 16);
	//SetupDPR(*Width = 640, *Height = 480, *BitsPerPixel = 16);
}




static int lynx3dm_init(struct lynxfb_info *fb_info)
{
	
	u8*	LogicalAddress;

	LogicalAddress = (u8*) fb_info->lynxbase_virt;
	printk("  LogicalAddress 0x%p\n", LogicalAddress);
	

	// Assign addresses for SM720.
	FrameBuffer = LogicalAddress + 0x200000;
	MMIO = LogicalAddress + 0x0C0000;
	DPR = LogicalAddress + 0x000000;
	DataPort = LogicalAddress + 0x100000;
	VPR = LogicalAddress + 0x000800;
	CPR = LogicalAddress + 0x001000;

	/* Program chip to its default mode. */
	SetMode(&Width, &Height, &BitsPerPixel);

	/* Fill the screen with dark blue. */
//	SM_Rectangle(0, 0, Width, Height, SM_RGB(0x77, 0x77, 0x77));
//	SM_Rectangle(0, 0, Width, Height, SM_RGB(0x00, 0x00, 0x80));

	return(1);
}



EXPORT_SYMBOL(lynxfb_initialised);
#ifdef MODULE
MODULE_LICENSE("GPL");
int init_module(void)
{
	return lynx3dmfb_init();
}

void cleanup_module(void)
{
	lynxfb_cleanup(void);
}

MODULE_AUTHOR("Joerg Ritter <jr@mycable.de>");
MODULE_DESCRIPTION("Lynx3DM+ framebuffer device driver");
#endif /* MODULE */
