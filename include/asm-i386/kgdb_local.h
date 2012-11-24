#ifndef __KGDB_LOCAL
#define ___KGDB_LOCAL
#include <linux/config.h>
#include <linux/types.h>
#include <linux/serial.h>
#include <linux/serialP.h>
#include <linux/spinlock.h>
#include <asm/processor.h>
#include <asm/msr.h>
#include <asm/kgdb.h>

#define PORT 0x3f8
#ifdef CONFIG_KGDB_PORT
#undef PORT
#define PORT CONFIG_KGDB_PORT
#endif
#define IRQ 4
#ifdef CONFIG_KGDB_IRQ
#undef IRQ
#define IRQ CONFIG_KGDB_IRQ
#endif
#define SB_CLOCK 1843200
#define SB_BASE (SB_CLOCK/16)
#define SB_BAUD9600 SB_BASE/9600
#define SB_BAUD192  SB_BASE/19200
#define SB_BAUD384  SB_BASE/38400
#define SB_BAUD576  SB_BASE/57600
#define SB_BAUD1152 SB_BASE/115200
#ifdef CONFIG_KGDB_9600BAUD
#define SB_BAUD SB_BAUD9600
#endif
#ifdef CONFIG_KGDB_19200BAUD
#define SB_BAUD SB_BAUD192
#endif
#ifdef CONFIG_KGDB_38400BAUD
#define SB_BAUD SB_BAUD384
#endif
#ifdef CONFIG_KGDB_57600BAUD
#define SB_BAUD SB_BAUD576
#endif
#ifdef CONFIG_KGDB_115200BAUD
#define SB_BAUD SB_BAUD1152
#endif
#ifndef SB_BAUD
#define SB_BAUD SB_BAUD1152        /* Start with this if not given*/
#endif

#ifndef CONFIG_X86_TSC
#undef rdtsc
#define rdtsc(a,b) if (a++ > 10000){a = 0; b++;}
#undef rdtscll
#define rdtscll(s) s++
#endif

/* RTAI support needs us to really stop/start interrupts */
#define RTAI
#ifdef hard_save_flags
#undef cli()
#undef sti()
#undef save_flags(x) 
#undef restore_flags(x) 
#define sti() hard_sti()
#define cli() hard_cli()
#define save_flags(x) hard_save_flags(x)
#define restore_flags(x) hard_restore_flags(x)
#endif

#ifdef _raw_read_unlock  // must use a name that is "define"ed, not an inline
#undef spin_lock
#undef spin_trylock
#undef spin_unlock
#define spin_lock	 _raw_spin_lock
#define spin_trylock	 _raw_spin_trylock
#define spin_unlock	 _raw_spin_unlock
#else
#endif
#undef spin_unlock_wait
#define spin_unlock_wait(x)  do { cpu_relax(); barrier();} \
                                     while(spin_is_locked(x))


 
#define SB_IER 1
#define SB_MCR UART_MCR_OUT2 | UART_MCR_DTR | UART_MCR_RTS

#define FLAGS 0
#define SB_STATE { \
     magic: SSTATE_MAGIC, \
     baud_base: SB_BASE,  \
     port:      PORT,     \
     irq:       IRQ,      \
     flags:     FLAGS,    \
     custom_divisor:SB_BAUD}
#define SB_INFO  { \
      magic: SERIAL_MAGIC, \
      port:  PORT,0,FLAGS, \
      state: &state,       \
      tty:   (struct tty_struct *)&state, \
      IER:   SB_IER,       \
      MCR:   SB_MCR}
extern void putDebugChar(int);
/* RTAI support needs us to really stop/start interrupts */

#define kgdb_sti() __asm__ __volatile__("sti": : :"memory")
#define kgdb_cli() __asm__ __volatile__("cli": : :"memory")
#define kgdb_local_save_flags(x) __asm__ __volatile__(\
                                   "pushfl ; popl %0":"=g" (x): /* no input */)
#define kgdb_local_irq_restore(x) __asm__ __volatile__(\
                                   "pushl %0 ; popfl": \
                                     /* no output */ :"g" (x):"memory", "cc")
#define kgdb_local_irq_save(x) kgdb_local_save_flags(x); kgdb_cli()

#ifdef CONFIG_SERIAL
extern void shutdown_for_kgdb(struct async_struct * info);
#endif
#define INIT_KDEBUG putDebugChar("+");
#endif /* __KGDB_LOCAL */

