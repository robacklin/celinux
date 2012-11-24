/*
 * S3C2410 Specific definitions for DPM
 * 
 * include/asm-arm/arch-s3c2410/s3c2410_dpm.h  
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
 * Copyright (C) 2002, MontaVista Software <source@mvista.com>
 *
 * Adapted for S3C2410 by Samsung
 * Copyright (c) 2003 Samsung Electronics Inc.
 *
 *
 * History:
 * 2003-12-16:	Seongil Na <seongil@samsung.com>
 * 	- Based on OMAP1510 DPM code by MontaVista Software
 *
 */

#ifndef __ASM_S3C2440_DPM_H__
#define __ASM_S3C2440_DPM_H__

/*
 * machine dependent operating state
 *
 * An operating state is a cpu execution state that has implications for power
 * management. The DPM will select operating points based largely on the
 * current operating state.
 *
 * DPM_STATES is the number of supported operating states. Valid operating
 * states are from 0 to DPM_STATES-1 but when setting an operating state the
 * kernel should only specify a state from the set of "base states" and should
 * do so by name.  During the context switch the new operating state is simply
 * extracted from current->dpm_state.
 *
 * task states:
 *
 * APIs that reference task states use the range -(DPM_TASK_STATE_LIMIT + 1)
 * through +DPM_TASK_STATE_LIMIT.  This value is added to DPM_TASK_STATE to
 * obtain the downward or upward adjusted task state value. The
 * -(DPM_TASK_STATE_LIMIT + 1) value is interpreted specially, and equates to
 * DPM_NO_STATE.
 *
 * Tasks inherit their task operating states across calls to
 * fork(). DPM_TASK_STATE is the default operating state for all tasks, and is
 * inherited from init.  Tasks can change (or have changed) their tasks states
 * using the DPM_SET_TASK_STATE variant of the sys_dpm() system call.  */

#define DPM_NO_STATE        -1

#define DPM_RELOCK_STATE     0
#define DPM_IDLE_TASK_STATE  1
#define DPM_IDLE_STATE       2
#define DPM_SLEEP_STATE      3
#define DPM_BASE_STATES      4

#define DPM_TASK_STATE_LIMIT 4
#define DPM_TASK_STATE       (DPM_BASE_STATES + DPM_TASK_STATE_LIMIT)
#define DPM_STATES           (DPM_TASK_STATE + DPM_TASK_STATE_LIMIT + 1)
#define DPM_TASK_STATES      (DPM_STATES - DPM_BASE_STATES)

#define DPM_STATE_NAMES                  \
{ "relock", "idle-task", "idle", "sleep",\
  "task-4", "task-3", "task-2", "task-1",\
  "task",                                \
  "task+1", "task+2", "task+3", "task+4" \
}

/* MD operating point parameters */
#define DPM_MD_V		0  
#define DPM_MD_MPLL		1	/* S3C2410 MPLL freq */
#define DPM_MD_CPU		2	/* CPU freq. */
#define DPM_MD_CLKDIVN	3	/* HCLK, PCLK freq. */
#define DPM_MD_LCD		4
#define DPM_MD_NAND		5
#define DPM_MD_SPI		6
#define DPM_MD_IIS		7
#define DPM_MD_IIC		8
#define DPM_MD_ADC		9
#define DPM_MD_RTC		10
#define DPM_MD_GPIO		11
#define DPM_MD_UART0	12
#define DPM_MD_UART1	13
#define DPM_MD_UART2	14
#define DPM_MD_SDI		15
#define DPM_MD_PWM		16
#define DPM_MD_USBH		17
#define DPM_MD_USBD		18

#define DPM_PP_NBR		19

#ifdef __KERNEL__
#ifndef __ASSEMBLER__

#include <linux/types.h>
#include <linux/proc_fs.h>
#include <asm/arch-s3c2440/ck.h>

#define DPM_MD_STATS
typedef __u64 dpm_md_count_t;
typedef __u64 dpm_md_time_t;

#if 0
extern unsigned int read_mputimer1(void);
#define dpm_md_time() read_mputimer1()

/* mputimer 1 runs @ 6Mhz  6 ticks = 1 microsecond */
#define DPM_MD_HZ 6 * 1000000

/* Hardcode this for now. */

#define tb_ticks_per_second DPM_MD_HZ   
#else
// FIXME: Hardcode dummy for now, and maybe should supports for S3C2410 later
#define dpm_md_time()		1
#define DPM_MD_HZ			1000000
#define tb_ticks_per_second	DPM_MD_HZ   
#endif

/* Disable *all* asynchronous interrupts for a super-critical section. */
/* TODO: Just regular save and restore for now, revisit later. */
#define critical_save_and_cli(flags) save_flags_cli(flags)
#define critical_restore_flags(flags) restore_flags(flags)

/* The register values only include the bits manipulated by the DPM
   system - other bits that also happen to reside in these registers are
   not represented here.  The layout of struct dpm_regs is used by
   assembly code; don't change it without updating constants further
   below (TODO). */

struct dpm_regs {
	int cpu;
	unsigned int mpllcon;	/* MPLL divider register */
	unsigned int clkdivn;	/* Clock divider register */
	unsigned int upllcon;	/* UPLL divider register */
	unsigned int clkcon;	/* Clock control register */
	unsigned int clkslow;	/* Clock slow register */
	unsigned long lpj;		/* FIXME: Loops per jiffy is duplicated with
							 * dpm_md_opt->lpj
							 */
};   

/* Instances of this structure define valid Innovator operating points for DPM.
   Voltages are represented in mV, and frequencies are represented in KHz. */ 
struct dpm_md_opt {
	unsigned int v;			/* Target voltage in mV */
	unsigned int mpll;		/* in KHz */
	unsigned int cpu;		/* CPU frequency in KHz */
	unsigned int clkdivn;	/* Clock divier */
	unsigned int lcd;		
	unsigned int nand;
	unsigned int spi;
	unsigned int iis;
	unsigned int iic;
	unsigned int adc;
	unsigned int rtc;
	unsigned int gpio;
	unsigned int uart0;
	unsigned int uart1;
	unsigned int uart2;
	unsigned int sdi;
	unsigned int pwm;
	unsigned int usbh;
	unsigned int usbd;
 	unsigned int lpj;		/* New value for loops_per_jiffy */
	struct dpm_regs regs;   /* Register values */
};

typedef void (*dpm_fscaler)(struct dpm_regs *regs);

#define basic_idle(parms) s3c2440_pm_idle()

/* Machine-dependent operating point creating/query/setting */

#endif /* __ASSEMBLER__ */

#endif /* __KERNEL__ */
#endif /* __ASM_S3C2440_DPM_H__ */
