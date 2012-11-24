/*
 * Robert Lembree, lembree@metrolink.com
 * Copyright (C) 2001 Metro Link, Inc.  All rights reserved.
 *
 * ########################################################################
 *
 *  This program is free software; you can distribute it and/or modify it
 *  under the terms of the GNU General Public License (Version 2) as
 *  published by the Free Software Foundation.
 *
 *  This program is distributed in the hope it will be useful, but WITHOUT
 *  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 *  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 *  for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  59 Temple Place - Suite 330, Boston MA 02111-1307, USA.
 *
 * ########################################################################
 *
 * Handle the front-panel display on a ATI Xilleon platform
 *
 */

#include <linux/init.h>
#include <linux/fs.h>
#include <linux/module.h>
#include <linux/miscdevice.h>
#include <linux/major.h>

static ssize_t xilleon_fp_write(struct file *file, const char *data, size_t len, loff_t *ppos)
{
	if (len) {
#ifdef CONFIG_MIPS_MALTA
		mips_display_message(data);
#endif
#ifdef CONFIG_ATI_XILLEON
		
#endif
	}
	return(len);
}


static struct file_operations xilleon_fp_fops =
{
	write:		xilleon_fp_write,
};


static struct miscdevice xilleon_fp_miscdev =
{
	MISC_DYNAMIC_MINOR,
	"xilleon_fp",
	&xilleon_fp_fops
};


int __init xilleon_fp_init(void)
{
	int result = 0;

	result = misc_register(&xilleon_fp_miscdev);

	if (result) {
		printk("Error registering ATI Xilleon Front Panel Driver: %d\n",
		       result);
		return result;
	}

	printk("ATI Xilleon Front Panel Driver at [%d:%d]\n",
	       MISC_MAJOR, xilleon_fp_miscdev.minor);

	return 0;
}


static void __exit xilleon_fp_exit(void)
{
	misc_deregister(&xilleon_fp_miscdev);
}
