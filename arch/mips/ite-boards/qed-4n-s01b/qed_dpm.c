/*
 * arch/mips/ite-boards/qed-4n-s01b/qed_dpm.c  QED-specific DPM support
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 * 
 * Copyright 2003 Sony Corporation 
 * Copyright 2003 Matsushita Electric Industrial Co., Ltd.
 * Copyright (C) 2002 - 2003 MontaVista Software, Inc.
 * Author: MontaVista Software, Inc.
 *     source@mvista.com
 *
 * Based on arch/ppc/platforms/ibm405lp_dpm.c by Bishop Brock.
 */

#include <linux/config.h>
#include <linux/dpm.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/kmod.h>
#include <linux/module.h>
#include <linux/proc_fs.h>
#include <linux/stat.h>
#include <linux/string.h>
#include <linux/trace.h>

#include <linux/delay.h>
#include <asm/hardirq.h>
#include <asm/page.h>
#include <asm/processor.h>
#include <asm/uaccess.h>
#include <linux/device.h>
#include <asm/it8172/it8172_dpm.h>


#define DEBUG
#undef DEBUG

#ifdef DEBUG
#define DPRINT(args...) printk(args)
#else
#define DPRINT(args...) do {} while (0)
#endif

struct dpm_bd dpm_bd;
struct dpm_md dpm_md;


unsigned int dpm_fscaler_flags;
#define DPM_FSCALER_NOP 		0
#define DPM_FSCALER_SLEEP 		2
#define DPM_FSCALER_WAKEUP 		4

static void qed_fscaler(struct dpm_regs *regs)
{
	if(dpm_fscaler_flags & DPM_FSCALER_NOP)
		return;

	if(dpm_fscaler_flags & DPM_FSCALER_SLEEP) {
		qed_suspend();

	 	/* Here when we wake up.  Recursive call to switch back to
	 	* to task state.
	 	*/

		dpm_set_os(DPM_TASK_STATE);
	}
}

static dpm_fscaler fscalers[1] = { 
	qed_fscaler,
};

/* This routine computes the "forward" frequency scaler that moves the system
 * from the current operating point to the new operating point.  The resulting
 * fscaler is applied to the registers of the new operating point. 
 */

dpm_fscaler
compute_fscaler(struct dpm_md_opt *cur, struct dpm_md_opt *new)
{
	dpm_fscaler_flags = DPM_FSCALER_NOP;

	if (cur->cpu && ! new->cpu)
		dpm_fscaler_flags = DPM_FSCALER_SLEEP;
	else if (! cur->cpu && new->cpu)
		dpm_fscaler_flags = DPM_FSCALER_WAKEUP;

	return fscalers[0];
}

/* Initialize the machine-dependent operating point from a list of parameters,
   which has already been installed in the pp field of the operating point.
   Some of the parameters may be specified with a value of -1 to indicate a
   default value. */

int
dpm_qed_init_opt(struct dpm_opt *opt)
{
	struct dpm_md_opt *md_opt = &opt->md_opt;

	md_opt->v = opt->pp[0];
	md_opt->cpu = opt->pp[1];
	return 0;
}

/* Fully determine the current machine-dependent operating point, and fill in a
   structure presented by the caller. This should only be called when the
   dpm_sem is held. This call can return an error if the system is currently at
   an operating point that could not be constructed by dpm_md_init_opt(). */

int
dpm_qed_get_opt(struct dpm_opt *opt)
{
	struct dpm_md_opt *md_opt = &opt->md_opt;

	md_opt->v = 5000;
        md_opt->cpu = 1;
	return 0;
}

/****************************************************************************
 *  DPM Idle Handler
 ****************************************************************************/

/* Check for pending external interrupts.  If so, the entry to a low-power
   idle is preempted. */

int
return_from_idle_immediate(void)
{
	return 0;
}

/****************************************************************************
 * Machine-dependent /proc/driver/dpm/md entries
 ****************************************************************************/

static inline int 
p5d(char *buf, unsigned mhz)
{
	return sprintf(buf, "%5d", mhz ); /* Round */
}

static int
dpm_proc_print_opt(char *buf, struct dpm_opt *opt)
{
        int len = 0;
        struct dpm_md_opt *md_opt = &opt->md_opt;

        len += sprintf(buf + len, "%12s %9llu", 
                      opt->name, opt->stats.count);
        len += p5d(buf + len, md_opt->cpu);
        return len;
}

int
read_proc_dpm_md_opts(char *page, char **start, off_t offset,
		      int count, int *eof, void *data)
{
	int len = 0;
	int limit = offset + count;
	struct dpm_opt *opt;
	struct list_head *opt_list;
	
	/* FIXME: For now we assume that the complete table,
	 * formatted, fits within one page */
	if (offset >= PAGE_SIZE)
		return 0;

	if (dpm_lock_interruptible())
		return -ERESTARTSYS;

	if (!dpm_initialized)
		len += sprintf(page + len, "DPM is not initialized\n");
	else if (!dpm_enabled)
		len += sprintf(page + len, "DPM is disabled\n");
	else {
		len += sprintf(page + len,
			       "The active DPM policy is \"%s\"\n",
			       dpm_active_policy->name);
		len += sprintf(page + len, 
			       "The current operating point is \"%s\"\n",
			       dpm_active_opt->name);
	}

	dpm_unlock();
	*eof = 1;
	if (offset >= len)
		return 0;
	*start = page + offset;
	return min(count, len - (int)offset);
}

/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * /proc/driver/dpm/md/cmd (Write-only)
 *
 *  This is a catch-all, simple command processor for the IT8172 DPM
 *  implementation. These commands are for experimentation and development
 *  _only_, and may leave the system in an unstable state.
 *
 *  No commands defined now.
 *
 ****************************************************************************/

int 
write_proc_dpm_md_cmd (struct file *file, const char *buffer,
		       unsigned long count, void *data)
{
	char *buf, *tok, *s;
	char *whitespace = " \t\r\n";
	int ret = 0;

	if (current->uid != 0)
		return -EACCES;
	if (count == 0)
		return 0;
	if (!(buf = kmalloc(count + 1, GFP_KERNEL)))
		return -ENOMEM;
	if (copy_from_user(buf, buffer, count)) {
		kfree(buf);
		return -EFAULT;
	}
	buf[count] = '\0';
	s = buf + strspn(buf, whitespace);
	tok = strsep(&s, whitespace);
	
	if (strcmp(tok, "define-me") == 0) {
		;
	} else {
		ret = -EINVAL;
	}
		kfree(buf);
	if (ret == 0)
		return count;
	else 
		return ret;
}


/****************************************************************************
 * Initialization/Exit
 ****************************************************************************/

void
dpm_qed_cleanup(void)
{
	dpm_bd.exit();
}

//extern void (*pm_idle)(void);   

int __init
dpm_qed_init(void)
{
	printk("IT8172 (QED5132) Dynamic Power Management\n");
	
	dpm_md.init		= NULL;
	dpm_md.init_opt		= dpm_qed_init_opt;
	dpm_md.set_opt		= dpm_default_set_opt;
	dpm_md.get_opt		= dpm_qed_get_opt;
	dpm_md.idle_set_parms	= NULL;
	dpm_md.cleanup		= dpm_qed_cleanup;

	dpm_it8172_board_setup();
	dpm_bd.init();

//#ifdef CONFIG_DPM_IDLE
	//pm_idle = dpm_idle;
//#endif

	return 0;
}
__initcall(dpm_qed_init);

