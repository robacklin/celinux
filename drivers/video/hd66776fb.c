/*
 * drivers/video/hd66776fb.c
 *
 * Copyright (C) 2003 YOSHII Takashi <yoshii-takashi@hitachi-ul.co.jp>
 *
 * Based on skeletonfb.c by Geert Uytterhoeven.
 */
 
#include <linux/config.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/tty.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/timer.h>

#include <asm/machvec.h>
#include <asm/uaccess.h>
#include <asm/pgtable.h>
#include <asm/io.h>

#include <linux/fb.h>

#include <video/fbcon.h>
#include <video/fbcon-cfb16.h>

#define XRES 240
#define YRES 320
#define UPDATE_FREQ 4

struct hd66776fb_info {
	struct fb_info_gen gen;
	unsigned long base, size;
	unsigned long dat, idx;
};

struct hd66776fb_par {
	/* nothing, so far */
};

static struct hd66776fb_info fb_info;
static struct hd66776fb_par current_par;
static int current_par_valid = 0;
static struct display disp;
static struct fb_var_screeninfo default_var;

static u16 fbcon_cmap[16];

static void hd66776fb_set_par(const void *fb_par, struct fb_info_gen *info);
static int hd66776fb_encode_var(struct fb_var_screeninfo *var, const void *par,
			    struct fb_info_gen *info);


static u16 vfbuffer[XRES*YRES];
struct timer_list hd66776fb_timer;

static unsigned short hd66776_inw(int a)
{
	ctrl_outw(a, fb_info.idx);
	return ctrl_inw(fb_info.dat);
}

static void hd66776_outw(unsigned short d, int a)
{
	ctrl_outw(a, fb_info.idx);
	ctrl_outw(d, fb_info.dat);
}

static void hd66776_outw_p(unsigned short d, int a)
{
	int timeout=100;
	ctrl_outw(a, fb_info.idx);
	while(timeout-- && (ctrl_inw(fb_info.dat)&0x100))
			udelay(10);
	ctrl_outw(a, fb_info.idx);
	ctrl_outw(d|0x0100, fb_info.dat);
	udelay(100);
}

static void hd66776_power_on(void)
{
	hd66776_outw(0x0001, 0x0000);
	mdelay(10);
	hd66776_outw(0x0000, 0x0100);
	hd66776_outw(0x0000, 0x0101);
	hd66776_outw(0x0000, 0x0102);
	hd66776_outw(0x0000, 0x0103);
	hd66776_outw(0x0014, 0x0104);
	hd66776_outw(0x0d1f, 0x0105);
	hd66776_outw_p(0x0000, 0x000a);
	hd66776_outw_p(0x0001, 0x000a);
	hd66776_outw_p(0x0002, 0x000a);
	hd66776_outw(0x19ec, 0x0101);
	hd66776_outw_p(0x0000, 0x000a);
	mdelay(40);
	hd66776_outw(0x2d1f, 0x0105);
	hd66776_outw_p(0x0000, 0x000a);
}

static void hd66776_init_screen(int xres, int yres)
{
	hd66776_outw(0x0127, 0x0001);
	hd66776_outw(0x0000, 0x0002);
	hd66776_outw(0x0330, 0x0003);
	hd66776_outw(0x0000, 0x0007);
	hd66776_outw(0x0808, 0x0008);

	hd66776_outw(0x1006, 0x0010);
	hd66776_outw(0x1a06, 0x0011);
	hd66776_outw(0x0606, 0x0012);
	hd66776_outw(0x0000, 0x0013);
	hd66776_outw(0x0203, 0x0014);
	hd66776_outw(0x0000, 0x0015);
	
	hd66776_outw(0x0000, 0x0204);
	hd66776_outw(0x0000, 0x0205);

	hd66776_outw(0x0112, 0x000d);
	hd66776_outw(0x0000, 0x000c);

	hd66776_outw(0x0000, 0x0400);
	hd66776_outw(0x0000, 0x0401);
	hd66776_outw(0x0000, 0x0402);
	hd66776_outw(0x013f, 0x0403);
	hd66776_outw(0x013f, 0x0404);
	hd66776_outw(0x013f, 0x0405);

	hd66776_outw(0,   0x0406);
	hd66776_outw(xres-1, 0x0407);
	hd66776_outw(0,   0x0408);
	hd66776_outw(yres-1, 0x0409);
}

static void hd66776_set_gamma(void)
{
	hd66776_outw(0x0100, 0x0300);
	hd66776_outw(0x0707, 0x0301);
	hd66776_outw(0x0102, 0x0302);
	hd66776_outw(0x0206, 0x0303);
	hd66776_outw(0x0302, 0x0304);
	hd66776_outw(0x0506, 0x0305);
	hd66776_outw(0x0000, 0x0306);
	hd66776_outw(0x0706, 0x0307);
	hd66776_outw(0x0206, 0x0308);
	hd66776_outw(0x0203, 0x0309);
}

static void hd66776_display_on(void)
{
	hd66776_outw(0x0001, 0x0007);
	mdelay(100);
	hd66776_outw(0x0022, 0x0007);
	hd66776_outw_p(0x0000, 0x000a);
	hd66776_outw(0x0033, 0x0007);
}

static void hd66776_init(int xres, int yres)
{
	hd66776_power_on();
	hd66776_init_screen(xres, yres);
	hd66776_set_gamma();
	hd66776_display_on();
}

static void hd66776fb_update(unsigned long d);

static void hd66776fb_restart_timer(struct timer_list *t, unsigned long d)
{
	init_timer(t);
	t->expires = jiffies + HZ/UPDATE_FREQ;
	t->data = d;
	t->function = hd66776fb_update;
	add_timer(t);
}

static void hd66776fb_update(unsigned long d)
{
	int cnt=XRES*YRES;
	if(d >= sizeof(vfbuffer)/2)
		d=0;
	hd66776_outw(d&0xff, 0x0200);
	hd66776_outw(d>>8,   0x0201);
	ctrl_outw(0x0202, fb_info.idx);
	while(cnt--)
		ctrl_outw(vfbuffer[d++], fb_info.dat);
	hd66776fb_restart_timer(&hd66776fb_timer, d);
}

/////////////////////////////////////////////////////////////////////////

static void hd66776fb_get_par(void *par, struct fb_info_gen *info);

static void hd66776fb_detect(void)
{
    fb_info.base = (unsigned long)vfbuffer;
    fb_info.size = sizeof(vfbuffer);
    fb_info.dat = 0xb8000000;
    fb_info.idx = 0xb8000004;

    hd66776_init(XRES, YRES);

    hd66776fb_get_par(&current_par, (struct fb_info_gen*)&fb_info);
    hd66776fb_encode_var(&default_var, &current_par, NULL);

    hd66776fb_restart_timer(&hd66776fb_timer, 0);
}

static int hd66776fb_encode_fix(struct fb_fix_screeninfo *fix, const void *par,
				struct fb_info_gen *info)
{
    memset(fix, 0, sizeof(struct fb_fix_screeninfo));

    strcpy(fix->id, "hd66776fb");
    fix->smem_start = fb_info.base;
    fix->smem_len = fb_info.size;
    fix->type = FB_TYPE_PACKED_PIXELS;
    fix->type_aux = 0;
    fix->visual = FB_VISUAL_TRUECOLOR;
    fix->xpanstep = 0;
    fix->ypanstep = 0;
    fix->ywrapstep = 0;
    fix->line_length = XRES*2;

    return 0;
}


static int hd66776fb_decode_var(const struct fb_var_screeninfo *var, void *par,
				struct fb_info_gen *info)
{
	return 0;
}


static int hd66776fb_encode_var(struct fb_var_screeninfo *var, const void *par,
			     struct fb_info_gen *info)
{
    memset(var, 0, sizeof(*var));

    var->xres = XRES;
    var->yres = YRES;
    var->xres_virtual = var->xres;
    var->yres_virtual = var->yres;
    var->xoffset = 0;
    var->yoffset = 0;
    var->bits_per_pixel = 16;
    var->grayscale = 0;
    var->transp.offset = 0;
    var->transp.length = 0;
    var->transp.msb_right = 0;
    var->nonstd = 0;
    var->activate = 0;
    var->height = -1;
    var->width = -1;
    var->vmode = FB_VMODE_NONINTERLACED;
    var->pixclock = 0;
    var->sync = 0;
    var->left_margin = 0;
    var->right_margin = 0;
    var->upper_margin = 0;
    var->lower_margin = 0;
    var->hsync_len = 0;
    var->vsync_len = 0;

    var->red.offset = 11;
    var->red.length = 5;
    var->green.offset = 5;
    var->green.length = 6;
    var->blue.offset = 0;
    var->blue.length = 5;
    var->transp.offset = 0;
    var->transp.length = 0;

    var->red.msb_right = 0;
    var->green.msb_right = 0;
    var->blue.msb_right = 0;
    var->transp.msb_right = 0;

    return 0;
}


static void hd66776fb_get_par(void *par, struct fb_info_gen *info)
{
    if (current_par_valid)
	*(struct hd66776fb_par *)par = current_par;
    else
	/* fill par with initial value */;
}


static void hd66776fb_set_par(const void *par, struct fb_info_gen *info)
{
    current_par = *(struct hd66776fb_par*)par;
    current_par_valid = 1;
    /* set hardware */
}


static int hd66776fb_getcolreg(unsigned regno, unsigned *red, unsigned *green,
			   unsigned *blue, unsigned *transp,
			   struct fb_info *info)
{
    if (regno > 255)
	return 1;	

    *red = *green = *blue = regno << 8;
    *transp = 0;
    
    return 0;
}


static int hd66776fb_setcolreg(unsigned regno, unsigned red, unsigned green,
			   unsigned blue, unsigned transp,
			   struct fb_info *info)
{
    if (regno > 255)
	return 1;
    
    fbcon_cmap[regno] =
	((red   & 0xf800)      ) |
	((green & 0xfc00) >>  5) |
	((blue  & 0xf800) >> 11);

    return 0;
}


static int hd66776fb_pan_display(const struct fb_var_screeninfo *var,
			     struct fb_info_gen *info)
{
    return 0;
}


static int hd66776fb_blank(int blank_mode, struct fb_info_gen *info)
{
    return 0;
}

static void hd66776fb_set_disp(const void *par, struct display *disp,
			    struct fb_info_gen *info)
{
    disp->screen_base = (void *)fb_info.base;
    disp->scrollmode = SCROLL_YREDRAW;
    disp->dispsw = &fbcon_cfb16;
    disp->dispsw_data = fbcon_cmap;
}

struct fbgen_hwswitch hd66776fb_switch = {
    hd66776fb_detect,
    hd66776fb_encode_fix,
    hd66776fb_decode_var,
    hd66776fb_encode_var,
    hd66776fb_get_par,
    hd66776fb_set_par,
    hd66776fb_getcolreg,
    hd66776fb_setcolreg,
    hd66776fb_pan_display,
    hd66776fb_blank,
    hd66776fb_set_disp
};


static struct fb_ops hd66776fb_ops = {
    owner:		THIS_MODULE,
    fb_get_fix:		fbgen_get_fix,
    fb_get_var:		fbgen_get_var,
    fb_set_var:		fbgen_set_var,
    fb_get_cmap:	fbgen_get_cmap,
    fb_set_cmap:	fbgen_set_cmap,
    fb_pan_display:	fbgen_pan_display,
};

int __init hd66776fb_init(void)
{
    fb_info.gen.parsize = sizeof(struct hd66776fb_par);
    fb_info.gen.fbhw = &hd66776fb_switch;
    fb_info.gen.fbhw->detect();
    strcpy(fb_info.gen.info.modename, "hd66776fb");
    fb_info.gen.info.changevar = NULL;
    fb_info.gen.info.node = -1;
    fb_info.gen.info.fbops = &hd66776fb_ops;
    fb_info.gen.info.disp = &disp;
    fb_info.gen.info.switch_con = &fbgen_switch;
    fb_info.gen.info.updatevar = &fbgen_update_var;
    fb_info.gen.info.blank = &fbgen_blank;
    fb_info.gen.info.flags = FBINFO_FLAG_DEFAULT;
    
    fbgen_get_var(&disp.var, -1, &fb_info.gen.info);
    fbgen_do_set_var(&disp.var, 1, &fb_info.gen);
    fbgen_set_disp(-1, &fb_info.gen);
    fbgen_install_cmap(0, &fb_info.gen);
    
    if(register_framebuffer(&fb_info.gen.info)<0)
	return -EINVAL;
    
    printk(KERN_INFO "fb%d: %s frame buffer device\n",
	   GET_FB_IDX(fb_info.gen.info.node), fb_info.gen.info.modename);
    
    return 0;
}


void hd66776fb_cleanup(void)
{
    unregister_framebuffer(&fb_info.gen.info);
}

MODULE_LICENSE("GPL");
#ifdef MODULE
module_init(hd66776fb_init);
#endif
module_exit(hd66776fb_cleanup);


