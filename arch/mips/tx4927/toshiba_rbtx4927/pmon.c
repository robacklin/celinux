/*
 * pmon.c
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 */
#include <linux/config.h>
#include <linux/init.h>
#include <linux/version.h>
#include <linux/string.h>
#include <asm/cpu.h>
#include <asm/processor.h>
#include <asm/system.h>
#include <asm/mipsregs.h>
#include <asm/tx4927/pmon.h>

struct pmon_vector *pmon_vector;

#ifdef __mips64
/* true vector */
struct pmon_vector_32 {
	__s32 read;	/* use sign-promotion to convert kseg0 to ckseg0 */
	__s32 write;
	__s32 open;
	__s32 close;
	__s32 ioctl;
	__s32 printf;
	__s32 vsprintf;
	__s32 ttctl;
	__s32 exit;
	__s32 getenv;
	__s32 onintr;
	__s32 flush_cache;
	__s32 _exception;
	__s32 _fpstatesz;
	__s32 _fpinit;
	__s32 _fpstate;
	__s32 cop1;
	__s32 adr2symoff;
	__s32 sym2adr;
	__s32 getclkfreq;
	__s32 _clkinit;
	__s32 interpret;
} *pmon_vector_32;

/* stub functions */
static __s32 _pmon_call(__s32 func, __s32 *args)
{
	long saveregs[11];
	__u32 save_status;
	__s32 ret;
	__s32 (*f)(__s32, __s32, __s32, __s32);
	f = (typeof(f))(long)func;
	save_status = read_c0_status();
	write_c0_status(save_status & ~(ST0_KX | ST0_IE));
	__asm__ __volatile__(
		".set\tnoreorder\n\t"
		"sd\t$16,0x00(%0)\n\t"
		"sd\t$17,0x08(%0)\n\t"
		"sd\t$18,0x10(%0)\n\t"
		"sd\t$19,0x18(%0)\n\t"
		"sd\t$20,0x20(%0)\n\t"
		"sd\t$21,0x28(%0)\n\t"
		"sd\t$22,0x30(%0)\n\t"
		"sd\t$23,0x38(%0)\n\t"
		"sd\t$28,0x40(%0)\n\t"
		"sd\t$29,0x48(%0)\n\t"
		"sd\t$30,0x50(%0)\n\t"
		".set\treorder\n\t"
		: /* No outputs */
		: "r" (saveregs));
	ret = f(args[0], args[1], args[2], args[3]);
	__asm__ __volatile__(
		".set\tnoreorder\n\t"
		"ld\t$29,0x48(%0)\n\t"	/* restore sp first */
		"ld\t$16,0x00(%0)\n\t"
		"ld\t$17,0x08(%0)\n\t"
		"ld\t$18,0x10(%0)\n\t"
		"ld\t$19,0x18(%0)\n\t"
		"ld\t$20,0x20(%0)\n\t"
		"ld\t$21,0x28(%0)\n\t"
		"ld\t$22,0x30(%0)\n\t"
		"ld\t$23,0x38(%0)\n\t"
		"ld\t$28,0x40(%0)\n\t"
		"ld\t$30,0x50(%0)\n\t"
		".set\treorder\n\t"
		: /* No outputs */
		: "r" (saveregs));
	write_c0_status(save_status);
	return ret;
}

static int _pmon_write(int fd, char *buf, int size)
{
	__s32 args[4];
	args[0] = fd;
	args[1] = (__s32)(long)buf;
	args[2] = size;
	return _pmon_call(pmon_vector_32->write, args);
}
static void _pmon_exit(int status)
{
	__s32 args[4];
	args[0] = status;
	_pmon_call(pmon_vector_32->exit, args);
}
static char *_pmon_getenv(char *name)
{
	__s32 args[4];
	args[0] = (__s32)(long)name;
	return (char *)(long)_pmon_call(pmon_vector_32->getenv, args);
}
static void _pmon_flush_cache(int cache)
{
	__s32 args[4];
	args[0] = cache;
	_pmon_call(pmon_vector_32->flush_cache, args);
}
static int _pmon_interpret(const char *cmd, char *outbuf, int len)
{
	__s32 args[4];
	args[0] = (__s32)(long)cmd;
	args[1] = (__s32)(long)outbuf;
	args[2] = len;
	return _pmon_call(pmon_vector_32->interpret, args);
}
static struct pmon_vector pmon_vector_stub = {
	.write = _pmon_write,
	.exit = _pmon_exit,
	.getenv = _pmon_getenv,
	.flush_cache = _pmon_flush_cache,
	.interpret = _pmon_interpret,
};
#endif	/* __mips64 */

/* this function is executed 32-bit kernel mode even on mips64 */
void __init pmon_setup_vector(void)
{
	int i;
	unsigned long vec;

	pmon_vector = NULL;
	vec = 0;
	switch(current_cpu_data.cputype) {
	case CPU_TX3927:
		vec = PMON_VECTOR_TX39;
		break;
	case CPU_TX49XX:
		vec = PMON_VECTOR_R4K;
		break;
	default:
		return;
	}
#ifdef __mips64
	vec = (unsigned long)(long)(__s32)vec;
#endif

	/* simple check for PMON vector */
	for (i = 0; i < sizeof(*pmon_vector) / sizeof(void *); i++) {
		__u32 p = *((__u32 *)vec + i);
		if ((p & 0xdffc0003) != 0x9fc00000 && p != 0) {
			return;
		}
	}
#ifdef __mips64
	pmon_vector_32 = (struct pmon_vector_32 *)vec;
	pmon_vector = &pmon_vector_stub;
	/* use original entry (PMON must handle exception on 64-bit mode) */
	pmon_vector_stub._exception =
		(void (*)(void))(long)pmon_vector_32->_exception;
#else
	pmon_vector = (struct pmon_vector *)vec;
#endif
}

#ifdef __mips64
static char *pmon_argv[32 /* max argc */];
void __init pmon_setup_args(int *argcp, char ***argvp, char ***envpp)
{
	int i;
	for (i = 0; i < *argcp; i++) {
		pmon_argv[i] = (char *)(long)(*((__s32 *)(*argvp) + i));
	}
	*argvp = pmon_argv;
	*envpp = NULL;	/* not used */
}
#else
void __init pmon_setup_args(int *argcp, char ***argvp, char ***envpp)
{
}
#endif

void pmon_halt(void)
{
	if (pmon_vector && pmon_vector->exit)
		pmon_vector->exit(0);
	cli();
	while (1)
		;
}

extern asmlinkage void pmon_trap_low(void);
void *pmon_user_exception_handler[32];
static int pmon_debug_initialized;
void set_pmon_debug_traps(void)
{
	unsigned long flags;
	int tt = 9;	/* beakpoint only */

	if (!pmon_vector)
		return;
	save_and_cli(flags);
	/* save original handlers (for user-mode exceptions) */
	pmon_user_exception_handler[tt] = set_except_vector(tt, pmon_trap_low);
	restore_flags(flags);
	pmon_debug_initialized = 1;
}

void pmon_breakpoint(void)
{
	if (!pmon_debug_initialized)
		return;
#if defined(CONFIG_CPU_R3000) || defined(CONFIG_CPU_TX39XX)
	/* CP0_EPC is read only. */
	pmon_printf("entering PMON.  Type \"r pc @pc+4;c\" to continue.\n");
#else
	pmon_printf("entering PMON.  Type \"c\" to continue.\n");
#endif

	__asm__ __volatile__(
		".globl	pmon_breakinst\n\t"
		".set	noreorder\n\t"
		"nop\n"
		"pmon_breakinst:\tbreak\n\t"
		"nop\n\t"
		".set	reorder"
		);
}

/* promlib support routine */
void prom_putchar(char c)
{
	if (!pmon_vector)
		return;
	pmon_vector->write(1/*STDOUT*/, &c, 1);
}
