/*
 * S3C2440 C-level Power-OFF/Wakeup Management Routines
 *
 * Initial by Seongil Na:
 * Copyright (c) 2003 Samsung Electronics Inc.
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
 * 2003-12-11:	Seongil Na <seongil@samsung.com>
 * 	- CPU Idle 
 *
 */

#if defined(CONFIG_MODVERSIONS)
#define MODVERSIONS
#include <linux/modversions.h>
#endif

#include <linux/module.h>			/* Kernel module definitions */
#include <linux/init.h>				/* Some macros of _init, _exit, Etc */
#include <linux/kernel.h>			/* We will be in the kernel mode of execution */
#include <linux/sched.h>			/* current macro */
#include <linux/pm.h>				/* pm_idle() */
#include <asm/hardware.h>			/* Definitions for S3C2440 */

MODULE_AUTHOR("Samsung Electronics, Inc.");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("");

//#define _DEBUG_
#ifdef _DEBUG_
	#define DPRINTK(x, args...) printk("DEBUG>>%s:%d: "x, __FUNCTION__, __LINE__, ##args)
#else
	#define DPRINTK(x, args...) do {} while (0)
#endif

/*
 * CPU idle
 *	S3C2440 is simple.
 */
static int value = 10; 
void s3c2440_pm_idle(void)
{
	// jyhwang deleted this line
	//local_irq_disable();

	int i;
	for(i=0;i<1000;++i){
		if (current->need_resched){
			schedule();
			return;
		}
	}
	if (!current->need_resched)
		CLKCON |= CLKCON_IDLE;	// Transition to IDLE mode

	//local_irq_enable();

	return;
}

/*
 * CPU idle module
 */
static int __init s3c2440_pm_idle_init(void)
{
	// Replace CPU idle function
	pm_idle = s3c2440_pm_idle;
	
	printk(KERN_INFO "S3C2440 Idle module module installed\n");
	return 0;
}

static void __exit s3c2440_pm_idle_exit(void)
{
	// Return to NULL
	pm_idle = NULL;

	printk(KERN_INFO "S3C2440 Idle module removed\n");
}

module_init(s3c2440_pm_idle_init);
module_exit(s3c2440_pm_idle_exit);
