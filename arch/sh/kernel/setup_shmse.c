/*
 * linux/arch/sh/kernel/setup_shmse.c
 *
 * Copyright (C) 2003 Takashi Kusuda <kusuda-takashi@hitachi-ul.co.jp>
 *
 * SH-Mobile SolutionEngine Support.
 */

#include <linux/config.h>
#include <linux/init.h>
#include <linux/irq.h>

#include <asm/uaccess.h>
#include <asm/io.h>
#include <asm/hitachi_shmse.h>

void shmse_rtc_gettimeofday(struct timeval* tv)
{
	tv->tv_sec  = 0;
	tv->tv_usec = 0;
}

int shmse_rtc_settimeofday(const struct timeval* tv)
{
	return 0;
}

void __init init_shmse_IRQ(void)
{
	ctrl_outw(0x0028, 0xb0a00000);	// mode set [active low].
	ctrl_outw(0x000a, INTC_ICR1);	// IRQ mode; IRQ0,1 enable.
 	/* PC_IRQ[0-3] -> IRQ0 (32) */
        make_ipr_irq( IRQ0_IRQ, IRQ0_IPR_ADDR, IRQ0_IPR_POS, 0x0f-IRQ0_IRQ);
        /* A_IRQ[0-3] -> IRQ1 (33) */
        make_ipr_irq( IRQ1_IRQ, IRQ1_IPR_ADDR, IRQ1_IPR_POS, 0x0f-IRQ1_IRQ);
}

void __init setup_shmse(void)
{
}
