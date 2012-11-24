/*
 * pmon-low.S
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 */
#include <linux/config.h>
#include <asm/asm.h>
#include <asm/mipsregs.h>
#include <asm/regdef.h>
#include <asm/addrspace.h>
#include <asm/stackframe.h>

	.align 5
	NESTED(pmon_trap_low, PT_SIZE, sp)
	.set	noat
	.set	noreorder
	mfc0	k0,CP0_STATUS
	sll	k0,3     		/* extract cu0 bit */
	bltz	k0,1f
	 nop
	/*
	 * Trap from user mode - invoke user exception handler
	 * saved in pmon.c
	 */
	mfc0	k1,CP0_CAUSE
	andi	k1,k1,0x7c
#ifdef __mips64
	dsll	k1,k1,1
#endif
	PTR_LA	k0,pmon_user_exception_handler
	PTR_ADDU	k0,k1
	PTR_L	k0,(k0)
	nop
	jr	k0
	nop
1:
#if defined(CONFIG_CPU_R3000) || defined(CONFIG_CPU_TX39XX)
	/* CP0_EPC is read only.  Use "r pc @pc+4" to skip break insn. */
#else
	/* If we're in pmon_breakpoint() increment the PC */
	PTR_LA	k0, pmon_breakinst
	MFC0	k1,CP0_EPC
	bne	k1,k0,1f
	 nop
	PTR_ADD	k1,4
	MTC0	k1,CP0_EPC
1:
#endif
	/* jump into PMON */
	PTR_LA	k0, pmon_vector
	PTR_L	k0, (k0)
	PTR_L	k0, PTRSIZE*12(k0)	/* 12:_exception */
	jr	k0
	 nop
	.set	at
	.set	reorder
	END(pmon_trap_low)
