/*
 * S3C2410 clock interface implements
 *
 * Initial OMAP1510 code by Gordon McNutt <gmcnutt@ridgerun.com>
 * Copyright (c) 2001 RidgeRun, Inc. <http://www.ridgerun.com>
 *
 * Addapted for S3C2410 by Seongil Na.
 * Copyright (c) 2003 Samsung Electronics, Inc. <http://www.samsungsemi.com>
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 * 
 * History:
 * 2003-10-29:  Seongil Na <seongil@samsung.com>
 * 
 */

#if defined(CONFIG_MODVERSIONS)
#define MODVERSIONS
#include <linux/modversions.h>
#endif

#include <linux/module.h>		/* Kernel module definitions */
#include <linux/init.h>
#include <linux/kernel.h>		/* We will be in the kernel mode of execution */
#include <linux/proc_fs.h>		/* Support for /proc file system */
#include <asm/hardware.h>		/* Definitions for S3C2410 */
#include <asm/arch/ck.h>		/* Enumerate the clock */
#include <asm/arch/cpu_s3c2440.h>
#include <asm/uaccess.h>
#include <linux/delay.h>		/* mdelay */

MODULE_AUTHOR("Samsung Electronics");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("");

//#define _DEBUG_
#ifdef _DEBUG_
    #define DPRINTK(x, args...) printk("DEBUG>>%s:%d: "x, __FUNCTION__, __LINE__, ##args)
#else
    #define DPRINTK(x, args...) do {} while (0)
#endif

typedef struct {
	char *name;				/* name */
	__u8 flags;				/* flags */
	ck_t parent;			/* input clock */
	volatile __u32 *rreg;	/* rate register */
	volatile __u32 *ereg;	/* enable register */
	volatile __u32 *ireg;	/* idle register */
	volatile __u32 *sreg;	/* select register */
	__u32 rshift;			/* rate shift bit */
	__u32 eshift;			/* enable shift bit */
	__u32 ishift;			/* idle shift bit */
	__u32 sshift;			/* select shift bit */
} ck_info_t;

static ck_info_t ck_info_table[] = {
	{
		name:"clkin",
		flags:0,
		parent:clkin,
	}, {
		name:"ck_fclk",
		flags:CK_RATEF | CK_IDLEF,
		parent:clkin,
		rreg:&MPLLCON,
		ireg:&CLKCON,
		ishift:CLKCON_IDLE,		 
	}, {
		name:"ck_hclk",
		flags:CK_RATEF,
		parent:ck_fclk,
		rreg:&CLKDIVN,
		rshift:CLKDIVN_HDIVN,
	}, {
		name:"ck_pclk",
		flags:CK_RATEF,
		parent:ck_fclk,
		rreg:&CLKDIVN,
		rshift:CLKDIVN_PDIVN,
	}, {
		name:"ck_uclk",
		flags:0,
		parent:clkin,
	}, {
		name:"ck_lcdc",
		flags:CK_ENABLEF,
		parent:ck_hclk,
		eshift:CLKCON_LCDC,
	}, {
		name:"ck_nandc",
		flags:CK_ENABLEF,
		parent:ck_hclk,
		eshift:CLKCON_NAND,
	}, {
		name:"ck_spi",
		flags:CK_ENABLEF,
		parent:ck_pclk,
		eshift:CLKCON_SPI,
	}, {
		name:"ck_iis",
		flags:CK_ENABLEF,
		parent:ck_pclk,
		eshift:CLKCON_IIS,
	}, {
		name:"ck_iic",
		flags:CK_ENABLEF,
		parent:ck_pclk,
		eshift:CLKCON_IIC,
	}, {
		name:"ck_adc",
		flags:CK_ENABLEF,
		parent:ck_pclk,
		eshift:CLKCON_ADC,
	}, {
		name:"ck_rtc",
		flags:CK_ENABLEF,
		parent:ck_pclk,
		eshift:CLKCON_RTC,
	}, {
		name:"ck_gpio",
		flags:CK_ENABLEF,
		parent:ck_pclk,
		eshift:CLKCON_GPIO,
	}, {
		name:"ck_uart0",
		flags:CK_ENABLEF,
		parent:ck_pclk,
		eshift:CLKCON_UART0,
	}, {
		name:"ck_uart1",
		flags:CK_ENABLEF,
		parent:ck_pclk,
		eshift:CLKCON_UART1,
	}, {
		name:"ck_uart2",
		flags:CK_ENABLEF,
		parent:ck_pclk,
		eshift:CLKCON_UART2,
	}, {
		name:"ck_sdi",
		flags:CK_ENABLEF,
		parent:ck_pclk,
		eshift:CLKCON_SDI,
	}, {
		name:"ck_pwm",
		flags:CK_ENABLEF,
		parent:ck_pclk,
		eshift:CLKCON_PWM,
	}, {
		name:"ck_usbh",
		flags:CK_ENABLEF,
		parent:ck_uclk,
		eshift:CLKCON_USBH,
	}, {
		name:"ck_usbd",
		flags:CK_ENABLEF,
		parent:ck_uclk,
		eshift:CLKCON_USBD,
	},
};

#define CK_NAME(ck)				ck_info_table[ck].name
#define CK_FLAGS(ck)			ck_info_table[ck].flags
#define CK_PARENT(ck)			ck_info_table[ck].parent
#define CK_RATE_REG(ck)			ck_info_table[ck].rreg
#define CK_ENABLE_REG(ck)		ck_info_table[ck].ereg
#define CK_IDLE_REG(ck)			ck_info_table[ck].ireg
#define CK_SELECT_REG(ck)		ck_info_table[ck].sreg
#define CK_RATE_SHIFT(ck)		ck_info_table[ck].rshift
#define CK_ENABLE_SHIFT(ck)		ck_info_table[ck].eshift
#define CK_IDLE_SHIFT(ck)		ck_info_table[ck].ishift
#define CK_SELECT_SHIFT(ck)		ck_info_table[ck].sshift
#define CK_CAN_CHANGE_RATE(ck)	(CK_FLAGS(ck) & CK_RATEF)
#define CK_CAN_DISABLE(ck)		(CK_FLAGS(ck) & CK_ENABLEF)
#define CK_CAN_IDLE(ck)			(CK_FLAGS(ck) & CK_IDLEF)
#define CK_CAN_SWITCH(ck)		(CK_FLAGS(ck) & CK_SELECTF)

/* Clock range */
#define CK_MIN	clkin
#define CK_MAX	ck_usbd
#define CK_IN_RANGE(ck) (!((ck < CK_MIN) || (ck > CK_MAX)))

int ck_get_rate(ck_t ck)
{
	int ret = -EINVAL;

	if (CK_IN_RANGE(ck) == 0)
		goto exit;

	switch (ck)
	{
	case ck_fclk:
		 ret = s3c2440_get_fclk() / 1000000;
		 break;
		 
	// HCLK friends
	case ck_hclk:
	case ck_lcdc:
	case ck_nandc:
		 ret = s3c2440_get_hclk() / 1000000;
		 break;
		 
	// PCLK friends
	case ck_pclk:
	case ck_spi:
	case ck_iis:
	case ck_iic:
	case ck_adc:
	case ck_rtc:
	case ck_gpio:
	case ck_uart0:
	case ck_uart1:
	case ck_uart2:
	case ck_sdi:
	case ck_pwm:
		 ret = s3c2440_get_pclk() / 1000000;
		 break;

	// UCLK friends
	case ck_uclk:
	case ck_usbh:
	case ck_usbd:
		 ret = s3c2440_get_uclk() / 1000000;
		 break;

	default:
		 ret = -EINVAL;
		 break;
	}

exit:
	return ret;
}

int ck_valid_rate(int mhz)
{
	int ret = -EINVAL;

	if (mhz < MPLL_101MHZ || mhz > MPLL_400MHZ)
		goto exit;

	ret = 0;

exit:
	return ret;
}

#define CLKDIVN_111	0x0
#define CLKDIVN_112	0x1
#define CLKDIVN_122	0x2
#define CLKDIVN_124	0x3
#define CLKDIVN_133	0x6
#define CLKDIVN_136	0x7
#define CLKDIVN_144	0x4
#define CLKDIVN_148	0x5
int ck_set_rate(ck_t ck, int mhz)
{
	int ret = -EINVAL;
	unsigned int clkdivn = CLKDIVN & 0x8; 

	switch (ck)
	{
	case ck_fclk:
		//
		ret = ck_valid_rate(mhz);
		if (ret != 0)
			goto exit;

		mdelay(10);
		switch (mhz)
		{
		case MPLL_400MHZ:
			 CLKDIVN = clkdivn | CLKDIVN_136;
			 MPLLCON = mpll_value[mhz];
			 break;
		case MPLL_135MHZ:
			 MPLLCON = mpll_value[mhz];
			 CLKDIVN = clkdivn | CLKDIVN_112;
			 break;
		default:
			 printk(KERN_WARNING "Not support a clock speed\n");
			 break;
		}
		mdelay(10);
		break;
	case ck_hclk:
		break;
	case ck_pclk:
		break;
	default:
		break;			
	}

exit:
	return ret;
}

int ck_enable(ck_t ck)
{
	int ret = -EINVAL;
	
	if (CK_IN_RANGE(ck) == 0)
		goto exit;

	if (CK_CAN_DISABLE(ck) == 0)
		goto exit;

	CLKCON |= (CK_ENABLE_SHIFT(ck));
	ret = 0;
	
exit:
	return ret;
}

int ck_is_enabled(ck_t ck)
{
	int ret = -EINVAL;
	
	if (CK_IN_RANGE(ck) == 0)
		goto exit;

	if (CK_CAN_DISABLE(ck) == 0)
		goto exit;

	switch (ck)
	{
	case ck_lcdc:
		ret = (CLKCON & CLKCON_LCDC) ? 1 : 0;
		break;
	case ck_nandc:
		ret = (CLKCON & CLKCON_NAND) ? 1 : 0;
		break;
	case ck_spi:
		ret = (CLKCON & CLKCON_SPI) ? 1 : 0;
		break;
	case ck_iis:
		ret = (CLKCON & CLKCON_IIS) ? 1 : 0;
		break;
	case ck_iic:
		ret = (CLKCON & CLKCON_IIC) ? 1 : 0;
		break;
	case ck_adc:
		ret = (CLKCON & CLKCON_ADC) ? 1 : 0;
		break;
	case ck_rtc:
		ret = (CLKCON & CLKCON_RTC) ? 1 : 0;
		break;
	case ck_gpio:
		ret = (CLKCON & CLKCON_GPIO) ? 1 : 0;
		break;
	case ck_uart0:
		ret = (CLKCON & CLKCON_UART0) ? 1 : 0;
		break;
	case ck_uart1:
		ret = (CLKCON & CLKCON_UART1) ? 1 : 0;
		break;
	case ck_uart2:
		ret = (CLKCON & CLKCON_UART2) ? 1 : 0;
		break;
	case ck_sdi:
		ret = (CLKCON & CLKCON_SDI) ? 1 : 0;
		break;
	case ck_pwm:
		ret = (CLKCON & CLKCON_PWM) ? 1 : 0;
		break;
	case ck_usbh:
		ret = (CLKCON & CLKCON_USBH) ? 1 : 0;
		break;
	case ck_usbd:
		ret = (CLKCON & CLKCON_USBD) ? 1 : 0;
		break;
	default:
		break;
	}

exit:
	return ret;

}

int ck_disable(ck_t ck)
{
	int ret = -EINVAL;

	if (CK_IN_RANGE(ck) == 0)
		goto exit;

	if (CK_CAN_DISABLE(ck) == 0)
		goto exit;

	CLKCON &= ~(CK_ENABLE_SHIFT(ck));
	ret = 0;

exit:
	return ret;
}

#if defined(CONFIG_PROC_FS)
#define CK_ENTRY_NAME		"ck"
#define CK_UP_ENTRY_NAME	"ckup"
#define CK_CALL_ENTRY_NAME	"ckcall"
#define CK_TEST_ENTRY_NAME	"cktest"
#define CK_LPJ_ENTRY_NAME	"lpj"
static struct proc_dir_entry* s3c2440_ck_proc = NULL;

static int s3c2440_ck_test_write_proc(struct file* file, const char* buffer,
									 unsigned long count, void* data)
{
	int ret;
	
	ret = ck_disable(ck_iic);
	DPRINTK("CLKCON is 0x%X\n", CLKCON);
	ret = ck_disable(ck_iis);
	DPRINTK("CLKCON is 0x%X\n", CLKCON);
	ret = ck_disable(ck_spi);
	DPRINTK("CLKCON is 0x%X\n", CLKCON);
	ret = ck_disable(ck_uart2);
	DPRINTK("CLKCON is 0x%X\n", CLKCON);
	ret = ck_disable(ck_gpio);
	DPRINTK("CLKCON is 0x%X\n", CLKCON);
	ret = ck_disable(ck_rtc);
	DPRINTK("CLKCON is 0x%X\n", CLKCON);
	ret = ck_disable(ck_adc);
	DPRINTK("CLKCON is 0x%X\n", CLKCON);
	ret = ck_disable(ck_pwm);
	DPRINTK("CLKCON is 0x%X\n", CLKCON);
	ret = ck_disable(ck_sdi);
	DPRINTK("CLKCON is 0x%X\n", CLKCON);
	//ret = ck_disable(ck_uart0);
	//DPRINTK("CLKCON is 0x%X\n", CLKCON);
	ret = ck_disable(ck_uart1);
	DPRINTK("CLKCON is 0x%X\n", CLKCON);
	ret = ck_disable(ck_nandc);
	DPRINTK("CLKCON is 0x%X\n", CLKCON);
	ret = ck_disable(ck_lcdc);
	DPRINTK("CLKCON is 0x%X\n", CLKCON);
	ret = ck_disable(ck_usbh);
	DPRINTK("CLKCON is 0x%X\n", CLKCON);
	ret = ck_disable(ck_usbd);
	DPRINTK("CLKCON is 0x%X\n", CLKCON);

	ret - ck_enable(ck_iic);
	DPRINTK("CLKCON is 0x%X\n", CLKCON);
	ret - ck_enable(ck_iis);
	DPRINTK("CLKCON is 0x%X\n", CLKCON);
	ret - ck_enable(ck_spi);
	DPRINTK("CLKCON is 0x%X\n", CLKCON);
	ret - ck_enable(ck_uart2);
	DPRINTK("CLKCON is 0x%X\n", CLKCON);
	ret - ck_enable(ck_gpio);
	DPRINTK("CLKCON is 0x%X\n", CLKCON);
	ret - ck_enable(ck_rtc);
	DPRINTK("CLKCON is 0x%X\n", CLKCON);
	ret - ck_enable(ck_adc);
	DPRINTK("CLKCON is 0x%X\n", CLKCON);
	ret - ck_enable(ck_pwm);
	DPRINTK("CLKCON is 0x%X\n", CLKCON);
	ret - ck_enable(ck_sdi);
	DPRINTK("CLKCON is 0x%X\n", CLKCON);
	//ret - ck_enable(ck_uart0);
	//DPRINTK("CLKCON is 0x%X\n", CLKCON);
	ret - ck_enable(ck_uart1);
	DPRINTK("CLKCON is 0x%X\n", CLKCON);
	ret - ck_enable(ck_nandc);
	DPRINTK("CLKCON is 0x%X\n", CLKCON);
	ret - ck_enable(ck_lcdc);
	DPRINTK("CLKCON is 0x%X\n", CLKCON);
	ret - ck_enable(ck_usbh);
	DPRINTK("CLKCON is 0x%X\n", CLKCON);
	ret - ck_enable(ck_usbd);
	DPRINTK("CLKCON is 0x%X\n", CLKCON);

	ret = ck_disable((ck_t)30);
	if (ret == 0)
		DPRINTK("ck_disable() for wrong clock is failed\n");
	ret = ck_enable((ck_t)30);
	if (ret == 0)
		DPRINTK("ck_enable() for wrong clock is failed\n");
	ret = ck_disable(ck_fclk);
	if (ret == 0)
		DPRINTK("ck_disable() for flags is failed\n");
	ret = ck_enable(ck_fclk);
	if (ret == 0)
		DPRINTK("ck_enable() for flags is failed\n");

	return count;
}

static int s3c2440_ck_up_write_proc(struct file* file, const char* buffer,
									 unsigned long count, void* data)
{
	int ret = -EINVAL;;
	int fclk, hclk, pclk, uclk;
	
	DPRINTK("Writing a /proc/ck/ckup\n");
	
	ret = ck_set_rate(ck_fclk, MPLL_400MHZ);
	if (ret != 0)
	{
		DPRINTK("ck_set_rate error\n");
		return ret;
	}

	fclk = ck_get_rate(ck_fclk);
	hclk = ck_get_rate(ck_hclk);
	pclk = ck_get_rate(ck_pclk);
	uclk = ck_get_rate(ck_uclk);
	
	DPRINTK("FCLK:%dMHz HCLK:%dMHz PCLK:%dMHZ UCLK:%dMHz\n",
			fclk, hclk, pclk, uclk);
	
	return count;
}

static int s3c2440_ck_call_write_proc(struct file* file, const char* buffer,
										unsigned long count, void* data)
{
	int ret = -EINVAL;
	int size;
	int mpll;
	int fclk, hclk, pclk, uclk;
	char buf[256];

	DPRINTK("Writing a /proc/ck/ckcall\n");
	
	size = count;
	if (size >= sizeof(buf))
		size = sizeof(buf) - 1;

	copy_from_user(buf, buffer, size);
	buf[size] = '\0';

	mpll = (int)simple_strtol(buf, NULL, 10);
	
	ret = ck_set_rate(ck_fclk, mpll);
	if (ret != 0)
	{
		DPRINTK("ck_set_rate error\n");
		return ret;
	}

	fclk = ck_get_rate(ck_fclk);
	hclk = ck_get_rate(ck_hclk);
	pclk = ck_get_rate(ck_pclk);
	uclk = ck_get_rate(ck_uclk);

	DPRINTK("FCLK:%dMHz HCLK:%dMHz PCLK:%dMHZ UCLK:%dMHz\n",
			fclk, hclk, pclk, uclk);
	
	return count;
}

static int s3c2440_ck_lpj_read_proc(char *page, char **start, off_t off,
									int count, int *eof, void *data)
{
	char* i = page;
	
	i += sprintf(i, "Loops per jiffy is %d\n", loops_per_jiffy);
	*start = NULL;
	*eof = 1;
	
	return (i - page);
}
#endif /* CONFIG_PROC_FS && _DEBUG_ */

static int __init s3c2440_ck_init(void)
{
	DPRINTK("Inserted S3C2410 clock module now..!\n");

#if defined(CONFIG_PROC_FS)
	struct proc_dir_entry* entry;
	
	s3c2440_ck_proc = proc_mkdir(CK_ENTRY_NAME, NULL);
	if (s3c2440_ck_proc != NULL)
	{
		entry = create_proc_entry(CK_UP_ENTRY_NAME, 0, s3c2440_ck_proc);
		if (entry)
			entry->write_proc = s3c2440_ck_up_write_proc;
		entry = create_proc_entry(CK_CALL_ENTRY_NAME, 0,s3c2440_ck_proc);
		if (entry)
			entry->write_proc = s3c2440_ck_call_write_proc;
		entry = create_proc_entry(CK_TEST_ENTRY_NAME, 0, s3c2440_ck_proc);
		if (entry)
			entry->write_proc = s3c2440_ck_test_write_proc;
		entry = create_proc_entry(CK_LPJ_ENTRY_NAME, 0, s3c2440_ck_proc);
		if (entry)
			entry->read_proc = s3c2440_ck_lpj_read_proc;
	}
	else
	{
		DPRINTK("Unable to create /proc/ck entry\n");
		return -ENOMEM;
	}
#endif /* CONFIG_PROC_FS && _DEBUG_ */

	LOCKTIME = 0xFFFFFFFF;
	DPRINTK("PLL Lock time is 0x%X\n", LOCKTIME);
	
	return 0;
}

static void __exit s3c2440_ck_exit(void)
{
	DPRINTK("Removed S3C2410 clock modules now..!\n");

#if defined(CONFIG_PROC_FS)
	remove_proc_entry(CK_LPJ_ENTRY_NAME, s3c2440_ck_proc);
	remove_proc_entry(CK_TEST_ENTRY_NAME, s3c2440_ck_proc);
	remove_proc_entry(CK_UP_ENTRY_NAME, s3c2440_ck_proc);
	remove_proc_entry(CK_CALL_ENTRY_NAME, s3c2440_ck_proc);
	remove_proc_entry(CK_ENTRY_NAME, NULL);
#endif /* CONFIG_PROC_FS && _DEBUG_ */
}

module_init(s3c2440_ck_init);
module_exit(s3c2440_ck_exit);
