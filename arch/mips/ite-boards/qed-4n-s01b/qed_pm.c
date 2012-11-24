/* qed_pm.c:	Power management for the QED
 * 
 * Copyright 2003 Sony Corporation 
 * Copyright 2003 Matsushita Electric Industrial Co., Ltd.
 * Copyright (C) 2003 MontaVista Software, Inc.
 * Author: MontaVista Software, Inc.
 *     source@mvista.com
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 *  THIS  SOFTWARE  IS PROVIDED   ``AS  IS'' AND   ANY  EXPRESS OR IMPLIED
 *  WARRANTIES,   INCLUDING, BUT NOT  LIMITED  TO, THE IMPLIED WARRANTIES OF
 *  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN
 *  NO  EVENT  SHALL   THE AUTHOR  BE    LIABLE FOR ANY   DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 *  NOT LIMITED   TO, PROCUREMENT OF  SUBSTITUTE GOODS  OR SERVICES; LOSS OF
 *  USE, DATA,  OR PROFITS; OR  BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 *  ANY THEORY OF LIABILITY, WHETHER IN  CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 *  THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *  You should have received a copy of the  GNU General Public License along
 *  with this program; if not, write  to the Free Software Foundation, Inc.,
 *  675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/config.h>
#include <linux/init.h>         /* __initcall */
#include <linux/kernel.h>	/* printk */
#include <linux/timer.h>
#include <linux/sysctl.h>	/* ctl_table */
#include <linux/device.h>	/* Device model */
#include <linux/tqueue.h>	/* Task queue stuff */
#include <linux/delay.h>	/* udelay */
#include <linux/irq.h>
#include <linux/sched.h>
#include <linux/pm.h>

#include <asm/system.h>
#include <asm/it8172/it8172.h>
#include <asm/it8172/it8172_int.h>
#include <asm/it8172/pm.h>

#define PREFIX		"QED APM: "	/* for printk */


/* number for the QED sleep sysctl()s
 *
 * This is picked at random out of thin air, hoping that it won't
 * clash with someone.  That's really ugly, but appears to be
 * "standard" practice (!?).  Oh well, with any luck we can throw
 * these away and replace them with sysfs parameters, in the
 * not-too-distant future...
 */

#define DEBUG	1
#define CTL_PM_QED 0xb1c3

enum pm_button_state { running, pressing,
		        suspended } pm_button;

int pm_booting = 1;
idle_state_t pm_idle_state;

static struct it8172_intc_regs volatile *it8172_hw0_icregs
    = (struct it8172_intc_regs volatile *)(KSEG1ADDR(IT8172_PCI_IO_BASE + IT_INTC_BASE));

static void pm_button_handler(unsigned long mode);

static int suspend_handler(void)
{
	int retval;

	pm_button_handler(1);
	if ((retval = device_suspend(0, SUSPEND_POWER_DOWN))) {
		  printk (KERN_ERR PREFIX
			"suspend_handler: "
			"device_suspend failed (%d)\n",
			retval);
		  return retval;
	}
	clear_c0_status(IE_IRQ5);
	do {
		udelay(100);
		cpu_wait();
	} while (pm_button);
	set_c0_status(IE_IRQ5);
 	device_resume(RESUME_POWER_ON);
	pm_button_handler(2);

	return 0;	/* success */
}

int qed_suspend(void)
{
	return suspend_handler();
}

void it8172_pm_idle(void)
{

	unsigned short lb_mask, lpc_mask, pci_mask, nmi_mask;

	lb_mask = it8172_hw0_icregs->lb_mask;
	lpc_mask = it8172_hw0_icregs->lpc_mask;
	pci_mask = it8172_hw0_icregs->pci_mask;
	nmi_mask = it8172_hw0_icregs->nmi_mask;
	it8172_hw0_icregs->lb_mask = 0xfff9; 
	// Unmask interrupts from TIMER0 and TIMER1
	it8172_hw0_icregs->lpc_mask = 0xffff;
	it8172_hw0_icregs->pci_mask = 0xffff;
	it8172_hw0_icregs->nmi_mask = 0xffff;
	clear_c0_status(IE_IRQ5);
	do {
		cpu_wait();
	} while (pm_idle_state);
	set_c0_status(IE_IRQ5);
	it8172_hw0_icregs->lb_mask = lb_mask;
	it8172_hw0_icregs->lpc_mask = lpc_mask;
	it8172_hw0_icregs->pci_mask = pci_mask;
	it8172_hw0_icregs->nmi_mask = nmi_mask;
}

/*
 * /proc interface for power managment
 */

static int suspend_proc_handler(ctl_table *ctl, int write, struct file * filp,
				void *buffer, size_t *lenp)
{
	return suspend_handler();
}

static struct ctl_table pm_table[] =
{
	{
		.ctl_name = CTL_PM_QED,
		.procname = "sleep",
		.mode = 0200, /* write-only to trigger sleep */
		.proc_handler = &suspend_proc_handler,
	},
	{0}
};

/*
 * APM Power Button
 *
 */

static struct tq_struct pm_button_task;

static void
button_suspend(void *data  __attribute__((unused)))
{
	suspend_handler();
}

static void
pm_button_handler(unsigned long mode)
{
	extern int pm_booting;

	switch (mode) {
	    case 0:
		/* Called from IRQ handler to handle
		   button pressing. */
		switch (pm_button) {
		    case running:
			/* The button is pressed for the first time in a
			   running system. */
			pm_button = pressing;
			if (pm_booting) {
				pm_booting = 0;	
				break;	
			}
			/* Fall through */
		    case pressing:		
			/* The user is pressing the button to suspend the
                           system. */
			pm_button = suspended;
			INIT_TQUEUE(&pm_button_task,
				    button_suspend, 0);
			schedule_task(&pm_button_task);
			break;
		    case suspended:
			/* Resume from suspend */
		        pm_button = running;
			break;
		}
		break;
	   case 1:
		/* Called from suspend_handler() */
		switch (pm_button) {
		    case suspended: /* Suspended from button press */
		    case running:	/* Suspended from /proc interface */
			pm_button = suspended;
			break;
		}
		break;
	    case 2:
		/* 
		 * Called from suspend_handler().  If the power button did
                 * not cause the wakeup (maybe it was RTC or another wakeup), 
                 * set the state to 'running',which arms the button to 
                 * suspend the machine again.
		 */
		pm_button = running;
		break;
	}
	return;
}
	
static void
pm_button_irq(int irq, void *dev_id, struct pt_regs *regs)
{
	pm_button_handler(0);
}

/*
 * Initialize power interface
 *
 */

static int __init qed_pm_init(void)
{	
	int err;

	register_sysctl_table(pm_table, 1);
	local_disable_irq(IT8172_POWER_NMI_IRQ);
	/* Redirect NMI# from power button to INT0# */
	it8172_hw0_icregs->nmi_redir |= 0x0020;
	/* Set interrupt from power button edge triggered. */
	it8172_hw0_icregs->nmi_trigger |= 0x0020;
	if ((err = request_irq(IT8172_POWER_NMI_IRQ, pm_button_irq, SA_SHIRQ,
			       "it8172_pm_button", &pm_button))) {
		printk(KERN_ERR PREFIX
		       "Request PM Button IRQ %d failed (%d)\n",
		       IT8172_POWER_NMI_IRQ, err);
		return -ENODEV;
	}	
	pm_button = running;
	local_enable_irq(IT8172_POWER_NMI_IRQ);	

	//pm_idle = it8172_pm_idle;
	//printk("initializing pm_idle at 0x%x\n", pm_idle);
	return 0;
}

__initcall(qed_pm_init);
