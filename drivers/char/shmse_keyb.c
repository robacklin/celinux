/* 
 */

#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/init.h>

#include <asm/machvec.h>
#include <asm/delay.h>
#include <asm/io.h>
#include "scan_keyb.h"

#define KEYCR1   0xa44b0000
#define KEYCR2   0xa44b0004
#define KEYOUTDR 0xa44b000c
#define KEYINDR  0xa44b0008

static const unsigned char se7300_scan_table[] = {
//	in0   in1   in2   in3   in4     x     x     x 
	0x01, 0x02, 0x03, 0x04, 0x05, 0x00, 0x00, 0x00,	// out0
	0x06, 0x07, 0x08, 0x09, 0x0a, 0x00, 0x00, 0x00,	// out1
	0x0b, 0x0c, 0x0d, 0x0e, 0x0f, 0x00, 0x00, 0x00, // out2
	0x11, 0x12, 0x13, 0x14, 0x15, 0x00, 0x00, 0x00, // out3
	0x16, 0x17, 0x18, 0x19, 0x1a, 0x00, 0x00, 0x00, // out4
	0x1b, 0x1c, 0x1d, 0x1e, 0x1f, 0x00, 0x00, 0x00, // out5
};

static const unsigned short se7300_keyout[] = {
	0x0ffc, 0x0ff3, 0x0fcf, 0x0f3f, 0x0cff, 0x03ff
};
#define LEN (sizeof(se7300_keyout)/sizeof(*se7300_keyout))

static int se7300_scan_kbd(unsigned char *s)
{
	int i;

	for(i=0; i<LEN; i++) {
		ctrl_outw(se7300_keyout[i], KEYOUTDR);
		udelay(8);
		*s++=ctrl_inw(KEYINDR);
	}
	ctrl_outw(0x0000, KEYOUTDR);

	return 0;
}


void __init se7300_kbd_init_hw(void)
{
	ctrl_outb(0x00, KEYCR1);
	ctrl_outb(0x00, KEYCR2);
	ctrl_outb(0x00, KEYOUTDR);
	scan_kbd_init();
	register_scan_keyboard(se7300_scan_kbd, se7300_scan_table, LEN);
	printk(KERN_INFO "se7300 matrix scan keyboard registered\n");
}

