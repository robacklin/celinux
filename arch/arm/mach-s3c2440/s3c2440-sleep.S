/*
 * S3C2410 Assembler Power-OFF/Wakeup Management Routines
 *
 * Initial SA1100 code:
 * Copyright (c) 2001 Cliff Brake <cbrake@accelent.com>
 *
 * Adapted for S3C2410 by Seongil Na:
 * Copyright (c) 2003 Samsung Electronics Inc. Ltd.
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
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 * 
 *
 * History:
 * 2003-10-20:	Seongil Na <seongil@samsung.com>
 *	- Initiated start
 *
 * 2003-12-12:	Seongil Na <seongil@samsung.com>
 *				Jaehoon Jeong <hooni_jeong@samsung.com>
 *	- Modified from swl patch
 *	- Add cache invalidate code from proc-arm920.S
 *
 */

#include <linux/linkage.h>
#include <asm/assembler.h>
#include <asm/hardware.h>

#define POWER_OFF_CLK	0xFFFF8

/*
 * s3c2440_pm_cpu_poweroff
 *
 * Causes s3c2440 to enter Power_OFF mode
 */
	.text
ENTRY(s3c2440_pm_cpu_poweroff)

	stmfd	sp!, {r4 - r12, lr}				@ save registers on stack

	@ get coprocessor registers
	mrc 	p15, 0, r4, c3, c0, 0			@ domain ID
	mrc 	p15, 0, r5, c2, c0, 0			@ translation table base addr
	mrc		p15, 0, r6, c13, c0, 0			@ PID
	mrc 	p15, 0, r7, c1, c0, 0			@ control reg

	@ store them plus current virtual stack ptr on stack
	mov		r8, sp
	stmfd	sp!, {r4 - r8}

	@ preserve phys address of stack
	mov		r0, sp
	bl		s3c2440_pm_sleep_phys_sp
	ldr		r1, =sleep_save_sp
	str		r0, [r1]

	@ clean data cache and invalidate WB
	bl		cpu_s3c2440_cache_clean_invalidate_all

	@ Already mask all interrupt sources by cli()
	@ Ensure that EINTMASK corresponding to wake up source has not to be masked 
	@ Set Wakeup external mode
	@ Change the Port Congfiguration to get the maximum of power efficiency
	@ Read rREFRESH and rCLKCON register in order to fill the TLB
	@ Only a sigle entry in the TLB takes care of Secntions and large pages
	ldr		r0, =MISCCR
	ldr		r0, [r0]
	ldr		r0, =CLKCON
	ldr		r0, [r0]

	@ Put the SDRAM self-refresh mode into r0 register
	ldr		r0, =REFRESH
	ldr		r1, [r0]
	orr		r1, r1, #REFRESH_TREFMD
	
	@ Set MISCCR[19:17]=111b to make SDRAM signals(SCLK0,SCLK1,SCKE) protected
	ldr		r2, =MISCCR
	ldr		r3, [r2]
	orr		r3, r3, #(MISCCR_nEN_SCKE | MISCCR_nEN_SCLK1 | MISCCR_nEN_SCLK0)

	@ Set the Power_Off mode bit in CLKCON Register
	ldr		r4, =CLKCON
	ldr		r5, =POWER_OFF_CLK				@ default value + POWER_OFFA
	b		PowerOffAction

	.align 5
PowerOffAction: 
	str		r1, [r0]						@ SDRAM self-refresh enable

	mov		r7, #254
loop1:	
	subs	r1, r1, #1
	bne		loop1
	
	@ Now we cannot use MMU we must use TLB hit
	str		r3, [r2]						@ MISCCR Set
	str		r5, [r4]						@ Power Off
	nop										@ waiting for power off
	nop
	nop
	b 	.


/*
 * s3c2440_pm_cpu_resume()
 *
 * entry point from bootloader into kernel during resume
 *
 * Note: Yes, part of the following code is located into the .data section.
 *       This is to allow sleep_save_sp to be accessed with a relative load
 *       while we can't rely on any MMU translation.  We could have put
 *       sleep_save_sp in the .text section as well, but some setups might
 *       insist on .text to be truely read-only.
 */
	.data
	.align	5
ENTRY(s3c2440_pm_cpu_resume)
	mov		r0, #I_BIT | F_BIT | MODE_SVC	@ set SVC, irqs off
	msr		cpsr_c, r0
	
	ldr		r0, sleep_save_sp				@ stack phys addr
 	ldr		r2, =resume_after_mmu			@ its absolute virtual address
	ldmfd	r0, {r4 - r7, sp}				@ CP regs + virt stack ptr

	mov		r1, #0
	mcr		p15, 0, r1, c7, c7, 0   		@ flush I&D cache

	mcr		p15, 0, r1, c7, c7, 0   		@ flush I&D cache
	mcr		p15, 0, r1, c7, c10, 4			@ drain write buffer 
	mcr		p15, 0, r1, c8, c7, 0			@ flush I+D TLBs

	mcr 	p15, 0, r4, c3, c0, 0			@ domain ID
	mcr 	p15, 0, r5, c2, c0, 0			@ translation table base addr
	mcr		p15, 0, r6, c13, c0, 0			@ PID
	b		resume_turn_on_mmu				@ cache align execution

	.align 5
resume_turn_on_mmu:
	mcr 	p15, 0, r7, c1, c0, 0			@ turn on MMU, caches, etc.
	mov		pc, r2							@ jump to virtual addr
	nop

sleep_save_sp:
	.word	0								@ preserve stack phys ptr here

	.text
ENTRY(resume_after_mmu)
	ldmfd	sp!, {r4 - r12, pc}				@ return to caller


/*
 * s3c2440_cache_flush_all()
 *
 * clean and invalidate all cache lines
 */
 	.text
	.align	5
ENTRY(cpu_s3c2440_cache_clean_invalidate_all)
	mov		r2, #1
	mov		ip, #0

	/*
	 * 'Clean & Invalidate whole DCache'
	 * Re-written to use Index Ops.
	 * Uses registers r1, r3 and ip
	 */
	mov		r1, #7 << 5						@ 8 segments
1:	orr		r3, r1, #63 << 26				@ 64 entries
2:	mcr		p15, 0, r3, c7, c14, 2			@ clean & invalidate D index
	subs	r3, r3, #1 << 26
	bcs		2b								@ entries 63 to 0
	subs	r1, r1, #1 << 5
	bcs		1b								@ segments 7 to 0

	teq		r2, #0
	mcrne	p15, 0, ip, c7, c5, 0			@ invalidate I cache
	mcr		p15, 0, ip, c7, c10, 4			@ drain WB

	mov	pc, lr
