/* arch/mips/ite-boards/qed-4n-s01b/qed_vst.c
 * VST support for QED5231 board
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
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/spinlock.h>
#include <linux/timer.h>
#include <linux/irq.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <asm/hrtime.h>
#include <asm/it8172/it8172.h>
#include <asm/it8172/it8172_int.h>
#include <asm/it8172/pm.h>

#define PREFIX "IT8172 (QED5231) VST: "
#define DEBUG_VST
#ifdef DEBUG_VST
#define DPRINT(args...) printk(PREFIX args)
#else
#define DPRINT(args...) do {} while (0)
#endif

#ifndef CONFIG_HIGH_RES_RESOLUTION
!Err VST needs the HIGH_RES_RESOLUTION
#else
#define TIMER_TICKS_PER_JIFFIE	((CONFIG_HIGH_RES_RESOLUTION * 1000000) / 2 / HZ)
/*
 * MAX_TIMER_TICKS has some leeway to make sure the timer doesn't wrap
 * before we update jiffies.
 */
#define MAX_TIMER_TICKS (~(unsigned long)0 - (4 * TIMER_TICKS_PER_JIFFIE))
#endif /* CONFIG_HIGH_RES_RESOLUTION */

int update_vst_jiffies;
idle_state_t vst_idle_state;

static volatile unsigned short it8172_intc_lb_mask, it8172_intc_lpc_mask,
                               it8172_intc_pci_mask, it8172_intc_nmi_mask;
/* Save IRQ controller state and disable CPU timer before entering idle state. Note TIMER0,
   serial and watchdog irqs should be unmasked in lb_mask and nmi_mask here */
void it8172_intc_save(void)
{
	if (vst_idle_state == not_idle) {
		it8172_intc_lb_mask = it8172_hw0_icregs->lb_mask;
		it8172_intc_lpc_mask = it8172_hw0_icregs->lpc_mask;
		it8172_intc_pci_mask = it8172_hw0_icregs->pci_mask;
		it8172_intc_nmi_mask = it8172_hw0_icregs->nmi_mask;
		it8172_hw0_icregs->lb_mask = 0xff7d;
		it8172_hw0_icregs->lpc_mask = 0xffff;
		it8172_hw0_icregs->pci_mask = 0xffff;
		it8172_hw0_icregs->nmi_mask = 0xffbf;
		clear_c0_status(IE_IRQ5);
		vst_idle_state = idle;
	}
}

/* Restore IRQ constoller state and enable CPU timer */
void it8172_intc_restore(void)
{
	if (vst_idle_state == idle) {
		set_c0_status(IE_IRQ5);
		it8172_hw0_icregs->lb_mask = it8172_intc_lb_mask;
		it8172_hw0_icregs->lpc_mask = it8172_intc_lpc_mask;
		it8172_hw0_icregs->pci_mask = it8172_intc_pci_mask;
		it8172_hw0_icregs->nmi_mask = it8172_intc_nmi_mask;
		vst_idle_state = not_idle;
	}
}

/* Update jiffies after sleeping for a while */
void inline update_jiffies_vst(void)
{
	static unsigned long base_c0 = 0;
	unsigned long c0;
	long delta_c0;

	c0 = read_c0_count();
	if (update_vst_jiffies) {
		delta_c0 = (base_c0 - c0) / TIMER_TICKS_PER_JIFFIE;
		update_vst_jiffies = 0;
	} else {
		delta_c0 = 1;
	}
	vst_idle_state = not_idle;
	(*(u64 *)&jiffies_64) += delta_c0;
	base_c0 = c0;
	return;
}

/* This is the main idle function called whenever we're going into idle state */
void vst_idle(void)
{
	extern void timer_bh(void);
	unsigned long next_timer, flags;

	spin_lock_irqsave(&timerlist_lock, flags);
	switch (find_next_timer(&next_timer)) {
	    case SHORT_NEXT_TIMER:
		/* Next timer is less than threshold; leave the periodic tick
		   interrupt enabled and don't set up a non-periodic timer */
		vst_short_timers++;
		spin_unlock_irqrestore(&timerlist_lock, flags);
		break;
	    case NO_NEXT_TIMER:
		/* No timers in lists; If we didn't have to worry about
		   timer overflow, we could disable the periodic tick
		   interrupt and don't bother with a non-periodic timer.
		   We would then wait for an interrupt to wake us up.
		   However, that is not the case.  next_timer has been
		   set to the maximum interval, so just fall through and
		   handle it the same as RETURNED_NEXT_TIMER */
		vst_no_timers++;
		DPRINT("No next timer\n");	
		/* FALLTHROUGH ! */
	    case RETURNED_NEXT_TIMER:
		/*
		 * *next contains when next timer will expire; disable
		 * the periodic tick interrupt and set a non-periodic
		 * timer.  We can reuse the subjiffie timer because we
		 * only use it when the next timer is greater than a
		 * tick and high res only uses it when the next timer is
		 * less than a tick.
		 */
		update_vst_jiffies = 1;
		vst_long_timers++;
		if (next_timer > (unsigned long)MAX_TIMER_TICKS) {
			next_timer = MAX_TIMER_TICKS;
		}
		DPRINT("About to go VST for %lu jiffies\n", next_timer);
		schedule_hr_timer_int(next_timer, 0);
		do {
			/* Note cpu_wait is not NULL for this arch */
			cpu_wait();
		} while (vst_idle_state == idle);

		spin_unlock_irqrestore(&timerlist_lock, flags);
		if (need_resched())
			schedule();
	}
}

static int __init qed_vst_init(void)
{
	vst_idle_state = not_idle;
	pm_idle = vst_idle;
	printk(KERN_INFO PREFIX "initialized pm_idle at 0x%x\n", pm_idle);
	return 0;
}

__initcall(qed_vst_init);
