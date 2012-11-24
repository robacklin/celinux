/*
 * include/asm-mips/it8172/it8172_dpm.h  IT8172-specific definitions for DPM
 * 
 * Copyright 2003 Sony Corporation 
 * Copyright 2003 Matsushita Electric Industrial Co., Ltd.
 * Copyright (C) 2003 MontaVista Software, Inc.
 * Author: MontaVista Software, Inc.
 *     source@mvista.com
 * This file is licensed under the
 * terms of the GNU General Public License version 2. This program is
 * licensed "as is" without any warranty of any kind, whether express
 * or implied.  All Rights Reserved.
 * */

#ifndef __ASM_IT8172_DPM_H__
#define __ASM_IT8172_DPM_H__

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

/* IT8172 machine-dependent operating point initialization vector
   locations.  TODO. */

#define DPM_PP_NBR 2

#ifndef __ASSEMBLER__

/* A struct form of the PP vector */

struct it8172_dpm_pp {
};
#endif /* __ASSEMBLER__ */

#ifdef __KERNEL__
#ifndef __ASSEMBLER__

#include <linux/types.h>
#include <linux/proc_fs.h>

#define DPM_MD_STATS
typedef __u64 dpm_md_count_t;
typedef __u64 dpm_md_time_t; 
#define readclock(void) read_c0_count()
extern unsigned long dpm_md_time(void);
extern int qed_suspend(void);
extern unsigned long cycles_per_jiffy;
#define tb_ticks_per_second (cycles_per_jiffy*HZ)
#define DPM_MD_HZ tb_ticks_per_second

/* Disable *all* asynchronous interrupts for a super-critical section.  Must
   occur in pairs.  This is necessarily done this way because RT Linux hacks
   break the symmetry of save/restore flags. */

#define critical_save_and_cli(flags) save_and_cli(flags)

#define critical_restore_flags(flags) restore_flags(flags)

/* Instances of this structure define valid operating points for DPM.
   Voltages are represented in mV, and frequencies are represented in KHz. 
   The values only include the bits manipulated by the DPM system - other
   bits that also happen to reside in these registers are not represented
   here.  The layout of struct dpm_regs is used by assembly code; don't change
   it without updating constants further below. */

struct dpm_regs {
        u16 pad;
};	

struct dpm_md_opt {
        unsigned int v;         /* Target voltage in mV */
	unsigned int cpu;	/* CPU frequency in KHz */
	struct dpm_regs regs;   /* Register values */
};

struct dpm_opt;
struct dpm_md_idle_parms;


void dpm_it8172_board_setup(void);

typedef void (*dpm_fscaler)(struct dpm_regs *regs);

#define basic_idle(parms) it8172_pm_idle()

#endif /* __ASSEMBLER__ */

#endif /* __KERNEL__ */
#endif /* __ASM_IT8172_DPM_H__ */
