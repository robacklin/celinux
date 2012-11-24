/*
 *  linux/drivers/char/s3c2440_kbd.c
 *
 *  Copyright (C) 2003 SAMSUNG ELECTRONICS
 *
 *  10/30/2003
 *
 *  This program is free software; you can redistribute it and/or modify
 *
 *  it under the terms of the GNU General Public License version 2 as
 *
 *  published by the Free Software Foundation.
 *
 */
#include <linux/config.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/interrupt.h>	/* for in_interrupt */
#include <linux/timer.h>
#include <linux/init.h>
#include <linux/delay.h>	/* for udelay */
#include <linux/kbd_kern.h>	/* for keyboard_tasklet */
#include <linux/kbd_ll.h>

#include <asm/io.h>
#include <asm/irq.h>
#include <asm/hardware.h>
#include <asm/arch/keyboard.h>
#include <asm/keyboard.h>

#define SPI_COLLISION_ERROR     0x4
#define SPI_MULTI_MASTER_ERROR  0x2
#define SPI_RX_TX_READY         0x1

#define UNSET_OUTPUT()         (GPBDAT &= ~(0x1<<6))
#define SET_OUTPUT()           (GPBDAT |=  (0x1<<6))
#define SPI_WRITE(c)           (SPTDAT1  = c)
#define SPI_READ(c)            (c = SPRDAT1 )

#define  KBCTL_NODATA -1
#define  KBCTL_AGAIN  -2

static int key_fn_down = 0;
static int key_numl_down = 0;

static struct kbd_info s3c2440_keyboard __initdata = {
	irq:	IRQ_EINT1,
};
static struct kbd_info *s3c_keyb= &s3c2440_keyboard;

static void key_interrupt(int nr, void *devid, struct pt_regs *regs)
{
  	u_int val = 0 ; 
	int x = 0 ,y = 0;
	static int lastc = -1;
	int ret = 0;

	kbd_pt_regs = regs;
	UNSET_OUTPUT();
     	while (! ( (SPSTA1)  & SPI_RX_TX_READY ));
        SPI_WRITE(0x0);
	while (! ( (SPSTA1)  & SPI_RX_TX_READY ));
	SPI_READ(val);
	SET_OUTPUT();

        x = val&0xff;  /* get char by sending one */	
        if (x == lastc || x == 0 || x == 0xff) {
  	        ret =  KBCTL_NODATA;
		tasklet_schedule(&keyboard_tasklet);
	}
        lastc = x;

        if ( x & 0x80) {  /* key up */  		
	        x &= 0x7f;
		if (x == 0x21) {   /* fn key up */	
	                key_fn_down = 0;
			ret =  KBCTL_AGAIN;
		}
		else {
		        if (key_fn_down) {   /* fn modified key */	
		                y = kbmapFN[x];
				if (y == KK_NUML)  ret=  KBCTL_AGAIN;
			}
			else {   
			       if (key_numl_down)   /* numlock modified key */	
			     	      y = kbmapNL[x];
			       else   y = kbmap[x];
			}
		}
		ret =  (y | 0x80);
	} else {	/* key down */
		if (x == 0x21) {   
		        key_fn_down = 1;
    			ret =  KBCTL_AGAIN;
    		}
		else {
		        if (key_fn_down) {
			          y = kbmapFN[x];
				  if (y == KK_NUML) { /* toggle numlock */	
				         key_numl_down = !key_numl_down;
					 ret=  KBCTL_AGAIN;
				  }
                         } 
			 else {
			         if (key_numl_down)
				         y = kbmapNL[x];
				 else
				         y = kbmap[x];
			 }
		}
                ret = y;
       }

       if ( ret != KBCTL_NODATA ) {
	         if (  ret != KBCTL_AGAIN && ret != KK_NONE)
		         handle_scancode(ret, !(ret & 0x80));
		 tasklet_schedule(&keyboard_tasklet);
	} 
}

static int __init HW_kbd_init(struct kbd_info *kbd)
{

	int ret = -ENODEV;
	int delay ;

	/* set GPF1 to EINT1 for kbd interrupt*/
	GPFCON  &= ~(0x3<<2); 	
	GPFCON  |=  (0x2<<2);  
	
	EXTINT0 &= ~(0x7<<4); 
	EXTINT0 |=  (0x2<<4); 

	/* setup SPI interface */
	GPGCON &= ~((0x3 << 10) | (0x3 << 12) | (0x3 << 14));   
	GPGCON |= ((0x3 << 10) | (0x3 << 12) | (0x3 << 14));    
   
        /* setup _SS signal(nSS_KBD) */	
	GPBCON &= ~(0x3 << 12);  
	GPBCON |= (0x1 << 12); 

	/* setup _PWR_OK signal(KBD) */	
	GPBCON &= ~(0x3 << 0);         
	GPBCON |= (0x1 << 0);       
	GPDDAT &=~(0x1 << 0);        
   
        /* Setup SPI registers
	 * Interrupt mode, prescaler enable, master mode, active high clock, format B, normal mode 
	 */ 	
	SPCON1 = (0x1<<5)|(0x1<<4)|(0x1<<3)|(0x0<<2)|(0x1<<1);

	SPPRE1 = 255; /* prescaler value */
 
	for(delay =0 ;delay<10;delay++)
	        SPI_WRITE(0xff);
	
	if (ret = request_irq(kbd->irq, key_interrupt, SA_INTERRUPT,"keyboard", kbd))
	        printk(KERN_INFO " ---> can't get assinged irq %d\n", kbd->irq);

	if (ret)goto bad;
	return ret;
bad:
	printk(" Oh dear, the interface was bad  ..\n");
	return ret;
}

#ifdef CONFIG_VT
/*
 * The fragment between #ifdef above and #endif * CONFIG_VT *
 * is from the pc_keyb.c driver.  It is not copyrighted under the
 * above notice.  This code is by various authors; please see
 * drivers/char/pc_keyb.c for further information.
 */

/*
 * Translation of escaped scancodes to keycodes.
 * This is now user-settable.
 * The keycodes 1-88,96-111,119 are fairly standard, and
 * should probably not be changed - changing might confuse X.
 * X also interprets scancode 0x5d (KEY_Begin).
 *
 * For 1-88 keycode equals scancode.
 */

#define E0_KPENTER 96
#define E0_RCTRL   97
#define E0_KPSLASH 98
#define E0_PRSCR   99
#define E0_RALT    100
#define E0_BREAK   101  /* (control-pause) */
#define E0_HOME    102
#define E0_UP      103
#define E0_PGUP    104
#define E0_LEFT    105
#define E0_RIGHT   106
#define E0_END     107
#define E0_DOWN    108
#define E0_PGDN    109
#define E0_INS     110
#define E0_DEL     111

#define E1_PAUSE   119

/*
 * The keycodes below are randomly located in 89-95,112-118,120-127.
 * They could be thrown away (and all occurrences below replaced by 0),
 * but that would force many users to use the `setkeycodes' utility, where
 * they needed not before. It does not matter that there are duplicates, as
 * long as no duplication occurs for any single keyboard.
 */
#define SC_LIM 89

#define FOCUS_PF1 85           /* actual code! */
#define FOCUS_PF2 89
#define FOCUS_PF3 90
#define FOCUS_PF4 91
#define FOCUS_PF5 92
#define FOCUS_PF6 93
#define FOCUS_PF7 94
#define FOCUS_PF8 95
#define FOCUS_PF9 120
#define FOCUS_PF10 121
#define FOCUS_PF11 122
#define FOCUS_PF12 123

#define JAP_86     124


/* tfj@olivia.ping.dk:
 * The four keys are located over the numeric keypad, and are
 * labelled A1-A4. It's an rc930 keyboard, from
 * Regnecentralen/RC International, Now ICL.
 * Scancodes: 59, 5a, 5b, 5c.
 */
#define RGN1 124
#define RGN2 125
#define RGN3 126
#define RGN4 127


static unsigned char high_keys[128 - SC_LIM] = {
  RGN1, RGN2, RGN3, RGN4, 0, 0, 0,                   /* 0x59-0x5f */
  0, 0, 0, 0, 0, 0, 0, 0,                            /* 0x60-0x67 */
  0, 0, 0, 0, 0, FOCUS_PF11, 0, FOCUS_PF12,          /* 0x68-0x6f */
  0, 0, 0, FOCUS_PF2, FOCUS_PF9, 0, 0, FOCUS_PF3,    /* 0x70-0x77 */
  FOCUS_PF4, FOCUS_PF5, FOCUS_PF6, FOCUS_PF7,        /* 0x78-0x7b */
  FOCUS_PF8, JAP_86, FOCUS_PF10, 0                   /* 0x7c-0x7f */
};


/* BTC */
#define E0_MACRO   112
/* LK450 */
#define E0_F13     113
#define E0_F14     114
#define E0_HELP    115
#define E0_DO      116
#define E0_F17     117
#define E0_KPMINPLUS 118
/*
 * My OmniKey generates e0 4c for  the "OMNI" key and the
 * right alt key does nada. [kkoller@nyx10.cs.du.edu]
 */
#define E0_OK   124
/*
 * New microsoft keyboard is rumoured to have
 * e0 5b (left window button), e0 5c (right window button),
 * e0 5d (menu button). [or: LBANNER, RBANNER, RMENU]
 * [or: Windows_L, Windows_R, TaskMan]
 */
#define E0_MSLW 125
#define E0_MSRW 126
#define E0_MSTM 127


static unsigned char e0_keys[128] = {
  0, 0, 0, 0, 0, 0, 0, 0,                             /* 0x00-0x07 */
  0, 0, 0, 0, 0, 0, 0, 0,                             /* 0x08-0x0f */
  0, 0, 0, 0, 0, 0, 0, 0,                             /* 0x10-0x17 */
  0, 0, 0, 0, E0_KPENTER, E0_RCTRL, 0, 0,             /* 0x18-0x1f */
  0, 0, 0, 0, 0, 0, 0, 0,                             /* 0x20-0x27 */
  0, 0, 0, 0, 0, 0, 0, 0,                             /* 0x28-0x2f */
  0, 0, 0, 0, 0, E0_KPSLASH, 0, E0_PRSCR,             /* 0x30-0x37 */
  E0_RALT, 0, 0, 0, 0, E0_F13, E0_F14, E0_HELP,       /* 0x38-0x3f */
  E0_DO, E0_F17, 0, 0, 0, 0, E0_BREAK, E0_HOME,       /* 0x40-0x47 */
  E0_UP, E0_PGUP, 0, E0_LEFT, E0_OK, E0_RIGHT, E0_KPMINPLUS, E0_END,/* 0x48-0x4f */
  E0_DOWN, E0_PGDN, E0_INS, E0_DEL, 0, 0, 0, 0,       /* 0x50-0x57 */
  0, 0, 0, E0_MSLW, E0_MSRW, E0_MSTM, 0, 0,           /* 0x58-0x5f */
  0, 0, 0, 0, 0, 0, 0, 0,                             /* 0x60-0x67 */
  0, 0, 0, 0, 0, 0, 0, E0_MACRO,                      /* 0x68-0x6f */
  0, 0, 0, 0, 0, 0, 0, 0,                             /* 0x70-0x77 */
  0, 0, 0, 0, 0, 0, 0, 0                              /* 0x78-0x7f */
};


#ifdef CONFIG_MAGIC_SYSRQ
u_char s3c_kbd_sysrq_xlate[128] =
	"\000\0331234567890-=\177\t"			/* 0x00 - 0x0f */
	"qwertyuiop[]\r\000as"				/* 0x10 - 0x1f */
	"dfghjkl;'`\000\\zxcv"				/* 0x20 - 0x2f */
	"bnm,./\000*\000 \000\201\202\203\204\205"	/* 0x30 - 0x3f */
	"\206\207\210\211\212\000\000789-456+1" 	/* 0x40 - 0x4f */
	"230\177\000\000\213\214\000\000\000\000\000\000\000\000\000\000" /* 0x50 - 0x5f */
	"\r\000/";					/* 0x60 - 0x6f */
#endif

int s3c_kbd_setkeycode(u_int scancode, u_int keycode)
{
        if (scancode < SC_LIM || scancode > 255 || keycode > 127)
	        return -EINVAL;
        if (scancode < 128)
                high_keys[scancode - SC_LIM] = keycode;
        else
                e0_keys[scancode - 128] = keycode;
        return 0;
}

int s3c_kbd_getkeycode(u_int scancode)
{
  return
          (scancode < SC_LIM || scancode > 255) ? -EINVAL :
          (scancode < 128) ? high_keys[scancode - SC_LIM] :
            e0_keys[scancode - 128];

}
		
int s3c_kbd_translate(u_char scancode, u_char *keycode, char raw_mode)
{
          *keycode = scancode & 0x7f;
        return 1;
}

char s3c_kbd_unexpected_up(u_char keycode)
{
        if (keycode >= SC_LIM || keycode == 85)
	        return 0;
        else
	        return 0200;
}

void s3c_kbd_leds(u_char leds)
{

}

extern int s3c2440_kbd_init(void);

int __init s3c2440_kbd_init(void)
{
	int  ret = -ENODEV;

	ret = HW_kbd_init(s3c_keyb);
	if (ret == 0) {
		k_translate	= s3c_kbd_translate;
		k_unexpected_up	= s3c_kbd_unexpected_up;
		k_leds		= s3c_kbd_leds;
#ifdef CONFIG_MAGIC_SYSRQ
		k_sysrq_xlate	= s3c_kbd_sysrq_xlate;
		k_sysrq_key	= 0x54;
#endif
	}
	printk("keyboard initialized\n");
	return ret;
}

#endif /* CONFIG_VT */

