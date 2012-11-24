/*
 *  BRIEF MODULE DESCRIPTION
 *	NEC RavinE frame buffer driver
 *      for AMD Alchemy Titanium Board
 *
 *  Copyright 2003 mycable GmbH
 *  Author: mycable GmbH
 *         Joerg Ritter jr@mycable.de
 *
 *
 *  12-Jun-2003: first version, 640x480 only, RavinE DS2
 *
 *  Based on:
 *  linux/drivers/video/skeletonfb.c -- Skeleton for a frame buffer device
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
#include "necravinefb.h"

#include <video/fbcon.h>
#include <video/fbcon-mfb.h>
#include <video/fbcon-cfb2.h>
#include <video/fbcon-cfb4.h>
#include <video/fbcon-cfb8.h>
#include <video/fbcon-cfb16.h>


//#define EXPORT_SYMTAB

#if defined(CONFIG_MIPS_XXS1500)
// FIXME put in the correct CONFIG_ for Titanium board
#else
#error Unknown Au1500 board
#endif

#define CMAPSIZE 16

#define RAVINE_DS2	2
#define RAVINE_DS3	3




//static char *options;
//MODULE_PARM(options, "s");
u32 necravine_fb_initialised;

struct necravinefb_info {
	struct fb_info_gen gen;
	unsigned long fb_size;

	void *regbase_virt;
	unsigned long regbase_phys;
	unsigned long regbase_size;
	void *membase_virt;
	unsigned long membase_phys;

	int xres;
	int yres;

	int mmaped;
	int nohwcursor;

	struct { unsigned red, green, blue, pad; } palette[256];

#if defined(FBCON_HAS_CFB16)
	u16 fbcon_cmap16[16];
#endif
};


struct necravinefb_par {
	struct fb_var_screeninfo var;
	int line_length;  // in bytes
	int cmap_len;     // color-map length
};


/*-------------------------------------------------------------------------*/
/*  function prototypes                                                    */
/*-------------------------------------------------------------------------*/

int necravinefb_init(void);
void necravinefb_setup(char *options, int *ints);
static int necravinefb_mmap(struct fb_info *fb, struct file *file, 
		struct vm_area_struct *vma);
static int necravine_blank(int blank_mode, struct fb_info_gen *info);
static int necravinefb_ioctl(struct inode *inode, struct file *file, u_int cmd,
		u_long arg, int con, struct fb_info *info);
int amm_init_ravine (u32 reg_base, u32 mem_base, u32 mem_base_burst, 
		tRGL_TIMING *display_timing);
int ravine_init(struct necravinefb_info *fb_info, char *optstring);

/*-------------------------------------------------------------------------*/
/*  global variables                                                       */
/*-------------------------------------------------------------------------*/
u32 necravine_fb_initialised;


/*-------------------------------------------------------------------------*/
/*  static variables													   */
/*-------------------------------------------------------------------------*/


static u32 ravine_regbase;                   /* Ravin-E register base       */
static u32 ravine_mem_noburst;               /* Ravin-E memory base         */
static u32 ravine_mem_burst;                 /* Ravin-E memory base, burst  */

static struct necravinefb_info fb_info;
static struct necravinefb_par current_par;
static struct display disp;

static int cur_timing;
static int ravine_rev = RAVINE_DS2; 


/*-------------------------------------------------------------------------*/
void necravine_nocursor(struct display *p, int mode, int xx, int yy){};

static struct fb_ops necravinefb_ops = {
	owner:		THIS_MODULE,
	fb_get_fix:	fbgen_get_fix,
	fb_get_var:	fbgen_get_var,
	fb_set_var:	fbgen_set_var,
	fb_get_cmap:	fbgen_get_cmap,
	fb_set_cmap:	fbgen_set_cmap,
	fb_pan_display: fbgen_pan_display,
	fb_ioctl:       necravinefb_ioctl,
	fb_mmap:        necravinefb_mmap,
};

static void necravine_detect(void)
{
	PRINT_FUNCTIONNAME;
	/*
	 *  This function should detect the current video mode settings 
	 *  and store it as the default video mode
	 */

	/*
	 * FIXME 
	 * detect RavinE version (DS2, DS3, whatever)
	 */

}

static int necravine_encode_fix(struct fb_fix_screeninfo *fix, 
		const void *_par, struct fb_info_gen *_info)
{
	struct necravinefb_info *info = (struct necravinefb_info *) _info;
	struct necravinefb_par *par = (struct necravinefb_par *) _par;
	struct fb_var_screeninfo *var = &par->var;

	PRINT_FUNCTIONNAME;
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
	}

	var->red.msb_right = 0;
	var->green.msb_right = 0;
	var->blue.msb_right = 0;
	var->transp.msb_right = 0;
}

static int necravine_decode_var(const struct fb_var_screeninfo *var, 
		void *_par, struct fb_info_gen *_info)
{

	struct necravinefb_par *par = (struct necravinefb_par *)_par;

	/*
	 * FIXME 
	 * check if settings make sense
	 */

	memset(par, 0, sizeof(struct necravinefb_par));
	par->var = *var;
	
	switch (var->bits_per_pixel) {
		case 8:
			par->var.bits_per_pixel = 8;
			break;
		case 16:
			par->var.bits_per_pixel = 16;
			break;
		default:
			printk("color depth %d bpp not supported\n",
					var->bits_per_pixel);
			return -EINVAL;

	}
	set_color_bitfields(&par->var);
	par->cmap_len = (par->var.bits_per_pixel == 8) ? 256 : 16;
	return 0;
}

static int necravine_encode_var(struct fb_var_screeninfo *var, 
		const void *par, struct fb_info_gen *_info)
{

	PRINT_FUNCTIONNAME;
	*var = ((struct necravinefb_par *)par)->var;
	return 0;
}

static void 
necravine_get_par(void *_par, struct fb_info_gen *_info)
{
	*(struct necravinefb_par *)_par = current_par;
	PRINT_FUNCTIONNAME;
}

static void necravine_set_par(const void *par, struct fb_info_gen *info)
{
	/* nothing to do: we don't change any settings */
	PRINT_FUNCTIONNAME;
}

static int necravine_getcolreg(unsigned regno, unsigned *red, unsigned *green,
			 unsigned *blue, unsigned *transp,
			 struct fb_info *info)
{

	struct necravinefb_info* i = (struct necravinefb_info*)info;

	//  PRINT_FUNCTIONNAME;
	if (regno > 255)
		return 1;
   
	*red    = i->palette[regno].red; 
	*green  = i->palette[regno].green; 
	*blue   = i->palette[regno].blue; 
	*transp = 0;

	return 0;
}

static int necravine_setcolreg(unsigned regno, unsigned red, unsigned green,
			 unsigned blue, unsigned transp,
			 struct fb_info *info)
{
	struct necravinefb_info* i = (struct necravinefb_info *)info;

	if (regno > 255)
		return 1;

	i->palette[regno].red    = red;
	i->palette[regno].green  = green;
	i->palette[regno].blue   = blue;
   
	i->fbcon_cmap16[regno] =
		((red & 0xf800) >> 0) |
		((green & 0xfc00) >> 5) |
		((blue & 0xf800) >> 11);
	return 0;
}


static int  necravine_blank(int blank_mode, struct fb_info_gen *_info)
{

	PRINT_FUNCTIONNAME;
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

static void necravine_set_disp(const void *unused, struct display *disp,
			 struct fb_info_gen *info)
{
	disp->screen_base = (char *)fb_info.membase_virt;
	switch (disp->var.bits_per_pixel) {
#ifdef FBCON_HAS_CFB8
	case 8:
		disp->dispsw = &fbcon_cfb8;
		if (fb_info.nohwcursor)
			fbcon_cfb8.cursor = necravine_nocursor;
		break;
#endif
#ifdef FBCON_HAS_CFB16
	case 16:
		disp->dispsw = &fbcon_cfb16;
		disp->dispsw_data = fb_info.fbcon_cmap16;
		if (fb_info.nohwcursor)
			fbcon_cfb16.cursor = necravine_nocursor;
		break;
#endif
	default:
		disp->dispsw = &fbcon_dummy;
		disp->dispsw_data = NULL;
		break;
	}
}



static int
necravinefb_mmap(struct fb_info *_fb, struct file *file, 
		struct vm_area_struct *vma)
{
	unsigned int len;
	unsigned long start=0, off;
	struct necravinefb_info *fb = (struct necravinefb_info *)_fb;

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

int necravine_pan_display(const struct fb_var_screeninfo *var,
		       struct fb_info_gen *info)
{
  PRINT_FUNCTIONNAME;
	return 0;
}

static int necravinefb_ioctl(struct inode *inode, struct file *file, u_int cmd,
			  u_long arg, int con, struct fb_info *info)
{
	/* nothing to do yet */
	printk("%s cmd=0x%x", __FUNCTION__, cmd);
	return -EINVAL;
}

static struct fbgen_hwswitch necravine_switch = {
	necravine_detect, 
	necravine_encode_fix, 
	necravine_decode_var, 
	necravine_encode_var, 
	necravine_get_par, 
	necravine_set_par, 
	necravine_getcolreg, 
	necravine_setcolreg, 
	necravine_pan_display, 
	necravine_blank, 
	necravine_set_disp
};


int __init necravinefb_init(void)
{
	//u32 sys_clksrc;

	struct pci_dev *pdev = NULL;

	PRINT_FUNCTIONNAME;

//	if (options) {
//		necravinefb_setup(options, 0);
//	}

	
	if (!(pdev = pci_find_device(PCI_RAVINE_VENDORID,
				     PCI_RAVINE_DEVICEID, pdev))) {
		printk("  RavinE not found\n");
		return -ENODEV;
	} else {
		printk("  RavinE found\n");
	}
	if (pci_enable_device(pdev) != 0) {
		printk("  couldn't enable RavinE\n");
		return -ENODEV;
	} else {
		printk("  RavinE enabled\n");
	}

	// map RavinE registers
	fb_info.membase_phys = pci_resource_start(pdev, 0);
	fb_info.regbase_phys = fb_info.membase_phys + 0x1fff000;
	fb_info.regbase_virt = ioremap_nocache(fb_info.regbase_phys, 0x2000);
	fb_info.membase_virt = ioremap_nocache(fb_info.membase_phys, 2*1024*1024);

	fb_info.xres = defd_timings[cur_timing].screen_width;
	fb_info.yres = defd_timings[cur_timing].screen_height;
	
	fb_info.fb_size = fb_info.xres * fb_info.yres * 16 / 8;
	
	// hard coded values so far
	current_par.var.xres = fb_info.xres;
	current_par.var.xres_virtual = fb_info.xres;
	current_par.var.yres = fb_info.yres;
	current_par.var.yres_virtual = fb_info.yres;
	current_par.var.bits_per_pixel = 16;

	/* FIXME only works for 8/16 bpp */
	current_par.line_length = fb_info.xres * 16 / 8; /* in bytes */

	ravine_init(&fb_info, "");

	fb_info.gen.parsize = sizeof(struct necravinefb_par);
	fb_info.gen.fbhw = &necravine_switch;

	strcpy(fb_info.gen.info.modename, "NEC RavinE");
   	fb_info.gen.info.changevar = NULL;
	fb_info.gen.info.node = -1;

	fb_info.gen.info.fbops = &necravinefb_ops;
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

	necravine_fb_initialised = 1;
	return 0;
}

void necravinefb_cleanup(struct fb_info *info)
{
	unregister_framebuffer(info);
}


void necravinefb_setup(char *options, int *ints)
{
	// FIXME 
	// parse options
	// function is called BEFORE necravinefb_init()
	// valid options
	//   "640x480"   default
	//   "800x600"
	//   "800x480"
	
	printk("%s options=\"%s\"\n", __FUNCTION__, options);
	
	cur_timing = 0;		// set default
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

int
ravine_init(struct necravinefb_info *fb_info, char *optstring)
{
  	tRGL_TIMING *p_disp_tim;
  	volatile u16 *vptr;
  	int i;

  	PRINT_FUNCTIONNAME;
  	if (cur_timing >= NUM_TIMINGS) 
		cur_timing = 0;
  	p_disp_tim = &defd_timings[cur_timing];
		
  	/* Ravin-E PCI settings*/
  	if (amm_init_ravine((u32) fb_info->regbase_virt, 
				(u32) fb_info->membase_virt, 
				(u32) fb_info->membase_virt, 
				&defd_timings[cur_timing]) == 0) {
		printk(KERN_DEBUG "RavinE mem.init. ok\n");
		write_ravine(RAVINE_VoWin0StartAddr, 0x00000000);
		write_ravine(RAVINE_VoA0StartAddr  , 0x00100000);
		write_ravine(RAVINE_VoZ0StartAddr  , 0x00100000);
		write_ravine(RAVINE_VoWin0WStartX  , 0);
		write_ravine(RAVINE_VoWin0WStartY  , 0);
		write_ravine(RAVINE_VoWin0VPStartX , 0);
		write_ravine(RAVINE_VoWin0VPStartY , 0);
		write_ravine(RAVINE_VoWin0StrideX, p_disp_tim->screen_width);
		write_ravine(RAVINE_VoWin0StrideY, p_disp_tim->screen_height);
		write_ravine(RAVINE_VoWin0Width, p_disp_tim->screen_width);
		write_ravine(RAVINE_VoWin0Height, p_disp_tim->screen_height);
		/* PC mode, BMG0 16bpp */

		/* non-interlaced */
		write_ravine(RAVINE_VoPlaneMode, 0x00000010);

		/* switch on BMG0 */
		write_ravine(RAVINE_VoPlaneSelect, 0x00000001);

		/* BMG0 on top    */
		write_ravine(RAVINE_VoPlaneOrder , 0x00543210);

		/* enable video output */
		write_ravine(RAVINE_VoControl, read_ravine(RAVINE_VoControl) | 
				RAVINE_VoControl_DST);
	}
  	else {
		printk(KERN_ERR "RavinE init error\n");
	}
  	return 0;
}


/*-------------------------------------------------------------------------*/

int amm_init_ravine (u32 reg_base, u32 mem_base, u32 mem_base_burst, 
		tRGL_TIMING *display_timing)

{
	int rc;

	volatile unsigned long dummy;

	PRINT_FUNCTIONNAME;
	rc = 0;

	ravine_regbase     = reg_base;
	ravine_mem_noburst = mem_base;
	ravine_mem_burst   = mem_base_burst;

	/* detect RavinE revision */
	
  	write_ravine(RAVINE_ViControl, read_ravine(RAVINE_ViControl) | 0x10000000);
	if (read_ravine(RAVINE_ViControl) & 0x10000000) {
		printk(KERN_INFO "RavinE DS3 detected\n");
		ravine_rev = RAVINE_DS3;
	} else {
		printk(KERN_INFO "RavinE DS2 detected\n");
		ravine_rev = RAVINE_DS2;
	}
  	write_ravine(RAVINE_ViControl, read_ravine(RAVINE_ViControl) & ~0x10000000);

	/* set SDRAM clock speed   */
	write_ravine(RAVINE_MemClockSet , 0x00010d00);
	mdelay(5);

	/* enable direct mode  */
	write_ravine(RAVINE_HostControl , RAVINE_HostControl_DIRECT);

	/* reset host inerface     */
	write_ravine(RAVINE_HostCmdPC        , 0x00000000); 
	write_ravine(RAVINE_HostCmdStop      , 0x00000000);
	write_ravine(RAVINE_HostIntThreshold , 0x00000070);
	write_ravine(RAVINE_HostDma          , 0x00000000);
	write_ravine(RAVINE_HostCache        , 0x00000001);
	write_ravine(RAVINE_HostIntEnable    , 0x00000000);
	write_ravine(RAVINE_HostIntClear     , 0xffffffff);
	write_ravine(RAVINE_HostReadyTimeout , 0x00000000);
	write_ravine(RAVINE_HostCpuBaseAddr  , 0x00000000);
	/* configure memory        */
	/* inerface                */
	write_ravine(RAVINE_MemSdramMode     , 0x00000027);
	write_ravine(RAVINE_MemRefreshCycle  , 0x00020506 | 
		  (1 << 28) | (1 << 29)); 
	write_ravine(RAVINE_MemControl       , 0x346a5401);

	//printk(KERN_DEBUG "  memcontrol=0x%x\n", 
	//			read_ravine(RAVINE_MemSdramMode)); 
	//printk(KERN_DEBUG "  memcontrol=0x%x\n", 
	//		read_ravine(RAVINE_MemControl)); 

	/* write_ravine(RAVINE_MemClockDelay    , 0x00ffff00);  */  /* undocumented clock skew */
	/* write_ravine(RAVINE_MemClockDelayDDR , 0xff000000);  */  /* adjust feature          */
	/* write_ravine(RAVINE_MemClockDelay    , 0xffffff00);  */
						/* wait for the SdramMode  */
	mdelay(10);
	/* into SDRAM              */

	if (read_ravine(RAVINE_MemSdramMode) & RAVINE_MemSdramMode_ST) {
		rc = -1;
	} else {
		/* set video output (Vo)   */
		/* timing registers        */
		write_ravine(RAVINE_VoControl, read_ravine(RAVINE_VoControl) | 
				RAVINE_VoControl_DPLL);

		mdelay(5);  /* PLL unstable for 1 ms   */

		amm_ravine_settiming(display_timing);

		if (ravine_rev == RAVINE_DS3) {
			 /* switch on enable signal */
			write_ravine(RAVINE_VoSyncSelect , 0x01000111); 
		} else {
			 /* switch on enable signal */
			write_ravine(RAVINE_VoSyncSelect , 0x00004015);
		}

		write_ravine(RAVINE_VoControl , RAVINE_VoControl_SWN | 
				RAVINE_VoControl_DAC | RAVINE_VoControl_DIG | 
				RAVINE_VoControl_CNT(0x0b) | 
				RAVINE_VoControl_CLK(0x00));

		mdelay(5);

		write_ravine(RAVINE_VoScanlineInt    , 0x01);
		write_ravine(RAVINE_VoFrameCount     , (1 << 28));
			     
		write_ravine(RAVINE_VoWin0StartAddr  , RAVINE_TRASHPAGE);
		write_ravine(RAVINE_VoWin0StrideX    , 0);
		write_ravine(RAVINE_VoWin0StrideY    , 0);
		write_ravine(RAVINE_VoWin0VPStartX   , 0);
		write_ravine(RAVINE_VoWin0VPStartY   , 0);

		write_ravine(RAVINE_VoWin0Width      , 0);
		write_ravine(RAVINE_VoWin0Height     , 0); 
				     
		write_ravine(RAVINE_VoWin0WStartX    , 0);
		write_ravine(RAVINE_VoWin0WStartY    , 0);
				     
		write_ravine(RAVINE_VoWin0ColorKey   , 0);
		write_ravine(RAVINE_VoAlpha0Const    , 0);
				     
		write_ravine(RAVINE_VoA0StartAddr    , RAVINE_TRASHPAGE);
		write_ravine(RAVINE_VoZ0StartAddr    , RAVINE_TRASHPAGE);
				     
		write_ravine(RAVINE_VoWin1StartAddr  , RAVINE_TRASHPAGE);
		write_ravine(RAVINE_VoWin1StrideX    , 0);
		write_ravine(RAVINE_VoWin1StrideY    , 0);
		write_ravine(RAVINE_VoWin1VPStartX   , 0);
		write_ravine(RAVINE_VoWin1VPStartY   , 0);
				     
		write_ravine(RAVINE_VoWin1Width      , 0);
		write_ravine(RAVINE_VoWin1Height     , 0);
		write_ravine(RAVINE_VoWin1WStartX    , 0);
		write_ravine(RAVINE_VoWin1WStartY    , 0);
				       
		write_ravine(RAVINE_VoWin1ColorKey   , 0);
		write_ravine(RAVINE_VoAlpha1Const    , 0);
				     
		write_ravine(RAVINE_VoA1StartAddr    , RAVINE_TRASHPAGE);
		write_ravine(RAVINE_VoZ1StartAddr    , RAVINE_TRASHPAGE);
				     
		write_ravine(RAVINE_VoWin2StartAddr  , RAVINE_TRASHPAGE);
		write_ravine(RAVINE_VoWin2StrideX    , 0);
		write_ravine(RAVINE_VoWin2StrideY    , 0);
		write_ravine(RAVINE_VoWin2VPStartX   , 0);
		write_ravine(RAVINE_VoWin2VPStartY   , 0);
				     
		write_ravine(RAVINE_VoWin2Width      , 0);
		write_ravine(RAVINE_VoWin2Height     , 0);
		write_ravine(RAVINE_VoWin2WStartX    , 0);
		write_ravine(RAVINE_VoWin2WStartY    , 0);
				     
		write_ravine(RAVINE_VoWin2ColorKey   , 0);
		write_ravine(RAVINE_VoAlpha2Const    , 0);
				   
		write_ravine(RAVINE_VoA2StartAddr    , RAVINE_TRASHPAGE);
		write_ravine(RAVINE_VoZ2StartAddr    , RAVINE_TRASHPAGE);
				     
		write_ravine(RAVINE_VoWin3StartAddr  , RAVINE_TRASHPAGE);
		write_ravine(RAVINE_VoWin3StrideX    , 0);
		write_ravine(RAVINE_VoWin3StrideY    , 0);
		write_ravine(RAVINE_VoWin3VPStartX   , 0);
		write_ravine(RAVINE_VoWin3VPStartY   , 0);
				   
		write_ravine(RAVINE_VoWin3Width      , 0);
		write_ravine(RAVINE_VoWin3Height     , 0);
		write_ravine(RAVINE_VoWin3WStartX    , 0);
		write_ravine(RAVINE_VoWin3WStartY    , 0);
				   
		write_ravine(RAVINE_VoWin3ColorKey   , 0);
		write_ravine(RAVINE_VoAlpha3Const    , 0);
		write_ravine(RAVINE_VoA3StartAddr    , RAVINE_TRASHPAGE);
		write_ravine(RAVINE_VoZ3StartAddr    , RAVINE_TRASHPAGE);
				   
		write_ravine(RAVINE_VoWin4StartAddr  , RAVINE_TRASHPAGE);
		write_ravine(RAVINE_VoWin4StrideX    , 0);
		write_ravine(RAVINE_VoWin4StrideY    , 0);
		write_ravine(RAVINE_VoWin4VPStartX   , 0);
		write_ravine(RAVINE_VoWin4VPStartY   , 0);
				   
		write_ravine(RAVINE_VoWin4Height     , 0);
		write_ravine(RAVINE_VoWin4Width      , 0);
		write_ravine(RAVINE_VoWin4WStartX    , 0);
		write_ravine(RAVINE_VoWin4WStartY    , 0);
				   
		write_ravine(RAVINE_VoWin4ColorKey   , 0);
		write_ravine(RAVINE_VoAlpha4Const    , 0);
				   
		write_ravine(RAVINE_VoA4StartAddr    , RAVINE_TRASHPAGE);
		write_ravine(RAVINE_VoZ4StartAddr    , RAVINE_TRASHPAGE);
				   
		write_ravine(RAVINE_VoWin5StartAddr  , RAVINE_TRASHPAGE);
		write_ravine(RAVINE_VoWin5StrideX    , 0);
		write_ravine(RAVINE_VoWin5StrideY    , 0);
		write_ravine(RAVINE_VoWin5VPStartX   , 0);
		write_ravine(RAVINE_VoWin5VPStartY   , 0);
				   
		write_ravine(RAVINE_VoWin5Height     , 0);
		write_ravine(RAVINE_VoWin5Width      , 0);
		write_ravine(RAVINE_VoWin5WStartX    , 0);
		write_ravine(RAVINE_VoWin5WStartY    , 0);
				   
		write_ravine(RAVINE_VoWin5ColorKey   , 0);
		write_ravine(RAVINE_VoAlpha5Const    , 0);
				   
		write_ravine(RAVINE_VoA5StartAddr    , RAVINE_TRASHPAGE);
		write_ravine(RAVINE_VoZ5StartAddr    , RAVINE_TRASHPAGE);
				   
		write_ravine(RAVINE_VoWorkStartAddr  , RAVINE_TRASHPAGE);
		write_ravine(RAVINE_VoWorkStrideX    , 0);
		write_ravine(RAVINE_VoWorkStrideY    , 0);
		write_ravine(RAVINE_VoAWorkStartAddr , RAVINE_TRASHPAGE);
		write_ravine(RAVINE_VoZWorkStartAddr , RAVINE_TRASHPAGE);

		write_ravine(RAVINE_VoColorLookUpTable , 0);
		write_ravine(RAVINE_VoBgColor          , 0);
		write_ravine(RAVINE_VoPlaneMode, RAVINE_VoPlaneMode_HLDO(0) | 
				RAVINE_VoPlaneMode_INTER(0) | 
				RAVINE_VoPlaneMode_SYN     | 
				RAVINE_VoPlaneMode_WIN0(0)  | 
				RAVINE_VoPlaneMode_WIN1(0) | 
				RAVINE_VoPlaneMode_WIN2(0)  | 
				RAVINE_VoPlaneMode_WIN3(0) | 
				RAVINE_VoPlaneMode_WIN4(0)  | 
				RAVINE_VoPlaneMode_WIN5(0));

		write_ravine(RAVINE_VoPlaneSelect , 0);

		write_ravine(RAVINE_VoPlaneOrder  , 
				RAVINE_VoPlaneOrder_POWIN0(5) | 
				RAVINE_VoPlaneOrder_POWIN1(4) | 
				RAVINE_VoPlaneOrder_POWIN2(3) | 
				RAVINE_VoPlaneOrder_POWIN3(2) | 
				RAVINE_VoPlaneOrder_POWIN4(1) | 
				RAVINE_VoPlaneOrder_POWIN5(0));

		write_ravine(RAVINE_VoUpscale     , 0);

		write_ravine(RAVINE_VoDimSignal   , 0);
		write_ravine(RAVINE_VoDimBase     , 0);
		write_ravine(RAVINE_VoDimCtrl     , 0);
		write_ravine(RAVINE_VoPwmCtrl     , 0);

		write_ravine(RAVINE_ViStartAddr1   , RAVINE_TRASHPAGE);
		write_ravine(RAVINE_ViStartAddr2   , RAVINE_TRASHPAGE);

		write_ravine(RAVINE_ViStartX       , 0);
		write_ravine(RAVINE_ViStartY       , XY(0, 0));
		write_ravine(RAVINE_ViScaledWidth  , 0);
		write_ravine(RAVINE_ViScaledHeight , 0);
		write_ravine(RAVINE_ViControl      , 0);
		write_ravine(RAVINE_ViFrameRate    , 0);
		write_ravine(RAVINE_ViDownscale    , XY(0, 0));

		/* set point in time for     */

		 /* measurement at 1/4 and 3/4*/
		write_ravine(RAVINE_ViScanlineInt  , 1);    
		write_ravine(RAVINE_ViCSyncDecode  , 
				RAVINE_VoCSyncDecode_14SCN(192) | /* of the */ 
				RAVINE_VoCSyncDecode_34SCN(576)); /* horizont */
				/* scan period for the dec.  */ 
			        /* of the csync signal       */
		write_ravine(RAVINE_ViStrideX           , 0);
		write_ravine(RAVINE_ViStrideY           , 0);

		write_ravine(RAVINE_DrawClipLeftTop     , XY(0, 0));
		write_ravine(RAVINE_DrawClipRightBottom , XY(0x7fff,0x7fff));
		write_ravine(RAVINE_DrawOffset          , 0);
		write_ravine(RAVINE_DrawDdaAdjust       , 0x0f);

		write_ravine(RAVINE_VoControl, read_ravine(RAVINE_VoControl) & 
				~RAVINE_VoControl_VBS);
	}

	dummy = *(volatile u32 *)ravine_regbase;

	return(rc);

}

/*-------------------------------------------------------------------------*/

int amm_ravine_settiming (tRGL_TIMING *timing)

{

	int           rc;

	unsigned long vo_pixel_clock;
	unsigned long div;
	unsigned long tmp;

	PRINT_FUNCTIONNAME;

	rc = 0;
#if 0
	if (ravine_check_timing(timing) != FR_NOERR) {
		rc = -1;
	} else
#endif
	{
		write_ravine(RAVINE_VoHTotal, 
				XY(timing->subtiming_h.total , 
					timing->subtiming_h.blank_width));
		write_ravine(RAVINE_VoHSync1, 
				XY(timing->subtiming_h.front_porch, 
					timing->subtiming_h.sync_width));
		write_ravine(RAVINE_VoHSync2, 
				XY(timing->subtiming_h.front_porch, 
					timing->subtiming_h.sync_width));

		write_ravine(RAVINE_VoVTotal, 
				XY(timing->subtiming_v.total , 
					timing->subtiming_v.blank_width));
		write_ravine(RAVINE_VoVSync1, 
				XY(timing->subtiming_v.front_porch, 
					timing->subtiming_v.sync_width ));
		write_ravine(RAVINE_VoVSync2, 
				XY(timing->subtiming_v.front_porch, 
					timing->subtiming_v.sync_width ));

		write_ravine(RAVINE_VoExternalPllSet, 0); 
		/* use internal PCLK */
		/* = 14.318 MHz for pixel clock gener.*/
		vo_pixel_clock          = timing->vo_pixel_clock; 
		div = vo_pixel_clock & 0x0000001f;

		if (!div) {
			rc = -1;
		} else {
			// fix this later
			//write_ravine(RAVINE_VoPixelClock, 0x18031000);   

			tmp = (vo_pixel_clock & ~0x0000001f) | (div >> 1) | (1 << 27) | (1 << 28);
			//printk(" pixel clock reg 0x%x\n", tmp);
		      	//write_ravine(RAVINE_VoPixelClock, tmp);
			write_ravine(RAVINE_VoPixelClock, 0x18031000);     // fix this later

			// fix this later
			// write_ravine(RAVINE_VoPixelClock, 0x18021000);

			/* RAVINE_VoPixelClock = (vo_pixel_clock & 0x0000001f) |(div >> 1) | */
			/* (1 << 27) |  */ /* enable clock delay    */ 
			/* (1 << 28);   */ /* set clock delay=90 degree   */
			/* 0 = 0 deg., 1 = 90 deg.,    */
			/* 2 = 180 deg.,3 = 270 deg.   */

			write_ravine(RAVINE_VoDisplayWidth,  
					timing->screen_width);
			write_ravine(RAVINE_VoDisplayHeight, 
					timing->screen_height);
		}
	}
	return(rc);

}
EXPORT_SYMBOL(necravine_fb_initialised);
#ifdef MODULE
MODULE_LICENSE("GPL");
int init_module(void)
{
	return necravinefb_init();
}

void cleanup_module(void)
{
	necravinefb_cleanup(void);
}

MODULE_AUTHOR("Joerg Ritter <jr@mycable.de>");
MODULE_DESCRIPTION("NEC RavinE LCD framebuffer device driver");
#endif /* MODULE */
