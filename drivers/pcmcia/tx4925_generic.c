/* $Id: tx4925_generic.c,v 1.1.2.1 2003/01/15 16:45:48 mpruznick Exp $
 * tx4925_generic.c: TX4925/TX4926 pcmcia socket driver
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2001-2002 Toshiba Corporation
 *
 * 2003 (c) MontaVista Software, Inc. This file is licensed under
 * the terms of the GNU General Public License version 2. This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/config.h>
#include <linux/delay.h>
#include <linux/ioport.h>
#include <linux/kernel.h>
#include <linux/tqueue.h>
#include <linux/timer.h>
#include <linux/mm.h>
#include <linux/proc_fs.h>
#include <linux/version.h>
#include <linux/types.h>
#include <linux/vmalloc.h>

#include <pcmcia/version.h>
#include <pcmcia/cs_types.h>
#include <pcmcia/cs.h>
#include <pcmcia/ss.h>
#include <pcmcia/bus_ops.h>

#include <asm/io.h>
#include <asm/irq.h>
#include <asm/system.h>
#include <asm/bootinfo.h>

#include <asm/tx4925/tx4925.h>
#include <asm/tx4925/tx4925_pci.h>
#include "tx4925_pcmcia.h"

#undef TX4925_GENERIC_DEBUG

#ifdef PCMCIA_DEBUG
static int pc_debug = PCMCIA_DEBUG;
#define DEBUG(n, args...) do { if (pc_debug>(n)) printk(KERN_DEBUG args); } while (0)
#else
#define DEBUG(n, args...) do { } while (0)
#endif

MODULE_AUTHOR("TOSHIBA Corporation");
MODULE_DESCRIPTION
    ("Linux PCMCIA Card Services: TX4925/TX4926 Socket Controller");

/* When configuring memory maps, Card Services appears to adopt the policy
 * that a memory access time of "0" means "use the default." The default
 * PCMCIA I/O command width time is 165ns. The default PCMCIA 5V attribute
 * and memory command width time is 150ns; the PCMCIA 3.3V attribute and
 * memory command width time is 300ns.
 */
#define TX4925_PCMCIA_IO_SPEED       (165)
#define TX4925_PCMCIA_MEM_SPEED      (300)

/* The socket driver actually works nicely in interrupt-driven form,
 * so the (relatively infrequent) polling is "just to be sure."
 */
#define TX4925_PCMCIA_POLL_PERIOD    (2*HZ)
static int poll_interval = 0;	/* default: use interrupt */

MODULE_PARM(poll_interval, "i");

/* Maximum number of IO windows per socket */
#define MAX_IO_WIN	2

/* Maximum number of memory windows per socket */
#define MAX_WIN		4

/* This structure encapsulates per-socket state which we might need to
 * use when responding to a Card Services query of some kind.
 */
struct tx4925_pcmcia_socket {
	socket_state_t cs_state;
	struct pcmcia_state k_state;
	unsigned int irq;
	void (*handler) (void *, unsigned int);
	void *handler_info;
	pccard_io_map io_map[MAX_IO_WIN];
	pccard_mem_map mem_map[MAX_WIN];
	ioaddr_t virt_io;
	ioaddr_t phys_attr;
	ioaddr_t phys_mem;
	unsigned short speed_io;
	unsigned short speed_attr;
	unsigned short speed_mem;
};

/* This structure maintains housekeeping state for each socket, such
 * as the last known values of the card detect pins, or the Card Services
 * callback value associated with the socket:
 */
static struct tx4925_pcmcia_socket *pcmcia_socket = NULL;
static int socket_count;

/* Returned by the low-level PCMCIA interface: */
static struct pcmcia_low_level *pcmcia_low_level = NULL;

/* Event poll timer structure */
static struct timer_list poll_timer;

/* Task structure */
static struct tq_struct tx4925_pcmcia_task;

/* EBUSC number table */
static int ebusc_no[3] = { -1, -1, -1 };	/* 0:mem 1:io 2:attr */

/* Prototypes for routines which are used internally: */
static int tx4925_pcmcia_driver_init(void);
static void tx4925_pcmcia_driver_shutdown(void);
static void tx4925_pcmcia_task_handler(void *data);
static void tx4925_pcmcia_poll_event(unsigned long dummy);
static void tx4925_pcmcia_interrupt(int irq, void *dev, struct pt_regs *regs);

#ifdef CONFIG_PROC_FS
static int tx4925_pcmcia_proc_status(char *buf, char **start,
				     off_t pos, int count, int *eof,
				     void *data);
#ifdef TX4925_GENERIC_DEBUG
static int tx4925_pcmcia_proc_iodump(char *buf, char **start,
				     off_t pos, int count, int *eof,
				     void *data);
static int tx4925_pcmcia_proc_memdump(char *buf, char **start, off_t pos,
				      int count, int *eof, void *data);
static int tx4925_pcmcia_proc_attrdump(char *buf, char **start, off_t pos,
				       int count, int *eof, void *data);
static int tx4925_pcmcia_proc_ebuscdump(char *buf, char **start, off_t pos,
					int count, int *eof, void *data);
#ifdef CONFIG_TOSHIBA_RBTX4925
#include <asm/tx4925/toshiba_rbtx4925/toshiba_rbtx4925.h>
static int rbtx4925_pcmcia_proc_interrupt(char *buf, char **start,
					  off_t pos, int count, int *eof,
					  void *data);
#endif
#endif				/* TX4925_GENERIC_DEBUG */
#endif

/* Prototypes for operations which are exported to the
 * new-and-impr^H^H^H^H^H^H^H^H^H^H in-kernel PCMCIA core:
 */

static int tx4925_pcmcia_init(u32 sock);
static int tx4925_pcmcia_suspend(u32 sock);
static int tx4925_pcmcia_register_callback(u32 sock,
					   void (*handler) (void *, u32),
					   void *info);
static int tx4925_pcmcia_inquire_socket(u32 sock, socket_cap_t * cap);
static int tx4925_pcmcia_get_status(u32 sock, u_int * value);
static int tx4925_pcmcia_get_socket(u32 sock, socket_state_t * state);
static int tx4925_pcmcia_set_socket(u32 sock, socket_state_t * state);
static int tx4925_pcmcia_get_io_map(u32 sock, struct pccard_io_map *io);
static int tx4925_pcmcia_set_io_map(u32 sock, struct pccard_io_map *io);
static int tx4925_pcmcia_get_mem_map(u32 sock, struct pccard_mem_map *mem);
static int tx4925_pcmcia_set_mem_map(u32 sock, struct pccard_mem_map *mem);
#ifdef CONFIG_PROC_FS
static void tx4925_pcmcia_proc_setup(u32 sock, struct proc_dir_entry *base);
#endif

static struct pccard_operations tx4925_pcmcia_operations = {
	tx4925_pcmcia_init,
	tx4925_pcmcia_suspend,
	tx4925_pcmcia_register_callback,
	tx4925_pcmcia_inquire_socket,
	tx4925_pcmcia_get_status,
	tx4925_pcmcia_get_socket,
	tx4925_pcmcia_set_socket,
	tx4925_pcmcia_get_io_map,
	tx4925_pcmcia_set_io_map,
	tx4925_pcmcia_get_mem_map,
	tx4925_pcmcia_set_mem_map,
#ifdef CONFIG_PROC_FS
	tx4925_pcmcia_proc_setup
#endif
};

static void
tx4925_pcmcia_resetup_speed(int ebusc_no, unsigned int ns_speed)
{
	if ((ebusc_no < 0) || (ebusc_no > 7)) {
		return;
	}
#ifdef CONFIG_TOSHIBA_RBTX4925
	{
		u32 ebccr, clk_div, wait;

		ebccr = tx4925_ebuscptr->ch[ebusc_no].cr;
		clk_div = (ebccr >> 4) & 0x03;
		clk_div = (4 - clk_div);

		ebccr &= ~(63 << 12);
		/*
		 * gbus_clk = ( 80 * 1000 * 1000 )
		 * 1wait = 1 / ( gbus_clk / clk_div ) sec.
		 *       = 1000000000 / ( gbus_clk / clk_div ) ns
		 *       = ( clk_div * 1000000000 ) / gbus_clk ns
		 *       = ( clk_div * 1000000000 ) / 80000000 ns
		 *       = ( clk_div * 100 ) / 8 ns
		 * Nwait = ns_speed / 1wait
		 *       = ns_speed / ( ( clk_div * 100 ) / 8 )
		 *       = ( ns_speed * 8 ) / ( clk_div * 100 )
		 */
		wait = ((ns_speed * 8) / (clk_div * 100)) + 1;
		if (wait > 63) {
			wait = 63;
		}
		ebccr |= (wait << 12);
		tx4925_ebuscptr->ch[ebusc_no].cr = ebccr;
	}
#endif
}

static int __init
tx4925_pcmcia_driver_init(void)
{
	servinfo_t info;
	struct pcmcia_init pcmcia_init;
	struct pcmcia_state state;
	unsigned int i, j;
	unsigned long ccfg_ng, pcfg_ng;

	pcmcia_low_level = (struct pcmcia_low_level *) NULL;

	switch (mips_machtype) {
#ifdef CONFIG_TOSHIBA_RBTX4925
	case MACH_TOSHIBA_RBTX4925:
		pcmcia_low_level = &rbtx4925_pcmcia_ops;
		break;
#endif
	default:
		break;
	}
	if (!pcmcia_low_level) {
		printk(KERN_DEBUG
		       "This hardware is not supported "
		       "by the TX4925 Card Service driver\n");
		return -ENODEV;
	}

	printk(KERN_INFO "TX4925 PCMCIA (CS release %s)\n", CS_RELEASE);

	CardServices(GetCardServicesInfo, &info);

	if (info.Revision != CS_RELEASE_CODE) {
		printk(KERN_ERR "Card Services release codes do not match\n");
		return -1;
	}

	ccfg_ng = (tx4925_ccfgptr->ccfg & TX4925_CONFIG_CCFG_PCTRCE ? 1 : 0);
	pcfg_ng =
	    (!(tx4925_ccfgptr->pcfg & TX4925_CONFIG_PCFG_SELCARD) ? 1 : 0);

	if (ccfg_ng || pcfg_ng) {
		printk(KERN_DEBUG "TX4925 PCMCIA: disabled by CCFG or PCFG.\n");
		printk(KERN_DEBUG
		       "(CCFG[PCTRCE]: %s   ", (ccfg_ng ? "NG" : "OK"));
		printk(KERN_DEBUG
		       "PCFG[SELCARD]: %s)\n", (pcfg_ng ? "NG" : "OK"));
		return -ENODEV;
	}

	memset(&pcmcia_init, 0, sizeof (pcmcia_init));
	pcmcia_init.handler = tx4925_pcmcia_interrupt;
	if ((socket_count = pcmcia_low_level->init(&pcmcia_init)) < 0) {
		printk(KERN_ERR "Unable to initialize PCMCIA service.\n");
		return -EIO;
	}

	i = sizeof (struct tx4925_pcmcia_socket) * socket_count;
	pcmcia_socket = kmalloc(i, GFP_KERNEL);
	if (!pcmcia_socket) {
		printk(KERN_ERR "Card Services can't get memory \n");
		return -1;
	}
	memset(pcmcia_socket, 0, i);

	for (i = 0; i < socket_count; i++) {

		if (pcmcia_low_level->socket_state(i, &state) < 0) {
			printk(KERN_ERR "Unable to get PCMCIA status\n");
			return -EIO;
		}
		pcmcia_socket[i].k_state = state;
		pcmcia_socket[i].cs_state.csc_mask = SS_DETECT;

		for (j = 0; j < 8; j++) {
			unsigned long ebccr = tx4925_ebuscptr->ch[j].cr;
			unsigned long ebcbar = tx4925_ebuscptr->ch[j].bar;
			unsigned long size;

			/* check ME and PCS */
			if (!(ebccr & 8) || ((ebccr >> 23) & 1) != i) {
				continue;
			}

			size = 0x00100000 << ((ebccr >> 8) & 0x0f);

			/* check PCM */
			switch ((ebccr >> 24) & 7) {
			case 1:
				printk(KERN_DEBUG
				       "TX4925 PCMCIA: MEM  %08lx @ %08lx\n",
				       ebcbar, size);
				pcmcia_socket[i].phys_mem = ebcbar;
				ebusc_no[0] = j;
				tx4925_pcmcia_resetup_speed(j,
							    TX4925_PCMCIA_MEM_SPEED);
				break;
			case 2:
				printk(KERN_DEBUG
				       "TX4925 PCMCIA: I/O  %08lx @ %08lx\n",
				       ebcbar, size);
				pcmcia_socket[i].virt_io =
				    (u32) ioremap(ebcbar, size);
				pcmcia_socket[i].virt_io -= mips_io_port_base;
				ebusc_no[1] = j;
				tx4925_pcmcia_resetup_speed(j,
							    TX4925_PCMCIA_IO_SPEED);
				break;
			case 3:
				printk(KERN_DEBUG
				       "TX4925 PCMCIA: ATTR %08lx @ %08lx\n",
				       ebcbar, size);
				pcmcia_socket[i].phys_attr = ebcbar;
				ebusc_no[2] = j;
				break;
			}
		}
		if (!pcmcia_socket[i].phys_mem ||
		    !pcmcia_socket[i].phys_attr || !pcmcia_socket[i].virt_io) {
			printk(KERN_ERR
			       "no EBUS channel available for slot %d\n", i);
			return -EIO;
		}
	}

	/* Only advertise as many sockets as we can detect: */
	if (register_ss_entry(socket_count, &tx4925_pcmcia_operations) < 0) {
		printk(KERN_ERR "Unable to register socket service routine\n");
		return -ENXIO;
	}

	/* Start the event poll timer.  
	 * It will reschedule by itself afterwards. 
	 */
	if (poll_interval) {
		tx4925_pcmcia_poll_event(0);
	}

	DEBUG(1, "tx4925: initialization complete\n");
	return 0;

}				/* tx4925_pcmcia_driver_init() */

module_init(tx4925_pcmcia_driver_init);

static void __exit
tx4925_pcmcia_driver_shutdown(void)
{
	int i;

	if (poll_interval) {
		del_timer_sync(&poll_timer);
	}
	unregister_ss_entry(&tx4925_pcmcia_operations);
	pcmcia_low_level->shutdown();
	flush_scheduled_tasks();
	for (i = 0; i < socket_count; i++) {
		if (pcmcia_socket[i].virt_io) {
			iounmap((void *) pcmcia_socket[i].virt_io);
		}
	}
	DEBUG(1, "tx4925: shutdown complete\n");
}

module_exit(tx4925_pcmcia_driver_shutdown);

static int
tx4925_pcmcia_init(unsigned int sock)
{
	return 0;
}

static int
tx4925_pcmcia_suspend(unsigned int sock)
{
	return 0;
}

static inline unsigned
tx4925_pcmcia_events(struct pcmcia_state *state,
		     struct pcmcia_state *prev_state,
		     unsigned int mask, unsigned int flags)
{
	unsigned int events = 0;

	if (state->detect != prev_state->detect) {
		DEBUG(2, "%s(): card detect value %u\n", __FUNCTION__,
		      state->detect);
		events |= mask & SS_DETECT;
	}

	if (state->ready != prev_state->ready) {
		DEBUG(2, "%s(): card ready value %u\n", __FUNCTION__,
		      state->ready);
		events |= mask & ((flags & SS_IOCARD) ? 0 : SS_READY);
	}

	if (state->bvd1 != prev_state->bvd1) {
		DEBUG(2, "%s(): card BVD1 value %u\n", __FUNCTION__,
		      state->bvd1);
		events |= mask & ((flags & SS_IOCARD) ? SS_STSCHG : SS_BATDEAD);
	}

	if (state->bvd2 != prev_state->bvd2) {
		DEBUG(2, "%s(): card BVD2 value %u\n", __FUNCTION__,
		      state->bvd2);
		events |= mask & ((flags & SS_IOCARD) ? 0 : SS_BATWARN);
	}

	*prev_state = *state;
	return events;
}				/* tx4925_pcmcia_events() */

/* 
 * tx4925_pcmcia_task_handler()
 * Processes socket events.
 */
static void
tx4925_pcmcia_task_handler(void *data)
{
	struct pcmcia_state state;
	int i, events, irq_status;

	for (i = 0; i < socket_count; i++) {
		if ((irq_status =
		     pcmcia_low_level->socket_state(i, &state)) < 0) {
			printk(KERN_ERR "low-level PCMCIA error\n");
		}

		events = tx4925_pcmcia_events(&state,
					      &pcmcia_socket[i].k_state,
					      pcmcia_socket[i].cs_state.
					      csc_mask,
					      pcmcia_socket[i].cs_state.flags);
		if (events && (pcmcia_socket[i].handler != NULL)) {
			pcmcia_socket[i].handler(pcmcia_socket[i].handler_info,
						 events);
		}
		/* need to check again? */
	}

}				/* tx4925_pcmcia_task_handler() */

static struct tq_struct tx4925_pcmcia_task = {
	routine:tx4925_pcmcia_task_handler
};

static void
tx4925_pcmcia_poll_event(unsigned long dummy)
{
	poll_timer.function = tx4925_pcmcia_poll_event;
	poll_timer.expires = jiffies + poll_interval;
	add_timer(&poll_timer);
	schedule_task(&tx4925_pcmcia_task);
}

/* 
 * tx4925_pcmcia_interrupt()
 * The actual interrupt work is performed by tx4925_pcmcia_task(), 
 * because the Card Services event handling code performs scheduling 
 * operations which cannot be executed from within an interrupt context.
 */
static void
tx4925_pcmcia_interrupt(int irq, void *dev, struct pt_regs *regs)
{
#ifdef TX4925_GENERIC_DEBUG
	printk(__FUNCTION__ ": interrupt detected  irq = %d\n", irq);
#endif
	schedule_task(&tx4925_pcmcia_task);
}

static int
tx4925_pcmcia_register_callback(unsigned int sock,
				void (*handler) (void *, unsigned int),
				void *info)
{
	if (handler == NULL) {
		pcmcia_socket[sock].handler = NULL;
		MOD_DEC_USE_COUNT;
	} else {
		MOD_INC_USE_COUNT;
		pcmcia_socket[sock].handler = handler;
		pcmcia_socket[sock].handler_info = info;
	}
	return 0;
}

/* tx4925_pcmcia_inquire_socket()
 *
 * From the sa1100 socket driver : 
 *
 * Implements the inquire_socket() operation for the in-kernel PCMCIA
 * service (formerly SS_InquireSocket in Card Services). Of note is
 * the setting of the SS_CAP_PAGE_REGS bit in the `features' field of
 * `cap' to "trick" Card Services into tolerating large "I/O memory" 
 * addresses. Also set is SS_CAP_STATIC_MAP, which disables the memory
 * resource database check. (Mapped memory is set up within the socket
 * driver itself.)
 *
 * In conjunction with the STATIC_MAP capability is a new field,
 * `io_offset', recommended by David Hinds. Rather than go through
 * the SetIOMap interface (which is not quite suited for communicating
 * window locations up from the socket driver), we just pass up 
 * an offset which is applied to client-requested base I/O addresses
 * in alloc_io_space().
 *
 * Returns: 0 on success, -1 if no pin has been configured for `sock'
 */
static int
tx4925_pcmcia_inquire_socket(unsigned int sock, socket_cap_t * cap)
{
	struct pcmcia_irq_info irq_info;

	if (sock > socket_count) {
		printk(KERN_ERR "tx4925: socket %u not configured\n", sock);
		return -1;
	}

	/* from the sa1100_generic driver: */

	/* SS_CAP_PAGE_REGS: used by setup_cis_mem() in cistpl.c to set the
	   *   force_low argument to validate_mem() in rsrc_mgr.c -- since in
	   *   general, the mapped * addresses of the PCMCIA memory regions
	   *   will not be within 0xffff, setting force_low would be
	   *   undesirable.
	   *
	   * SS_CAP_STATIC_MAP: don't bother with the (user-configured) memory
	   *   resource database; we instead pass up physical address ranges
	   *   and allow other parts of Card Services to deal with remapping.
	   *
	   * SS_CAP_PCCARD: we can deal with 16-bit PCMCIA & CF cards, but
	   *   not 32-bit CardBus devices.
	 */
	cap->features = (SS_CAP_PAGE_REGS | SS_CAP_STATIC_MAP | SS_CAP_PCCARD);

	irq_info.sock = sock;
	irq_info.irq = -1;

	if (pcmcia_low_level->get_irq_info(&irq_info) < 0) {
		printk(KERN_ERR "Error obtaining IRQ info socket %u\n", sock);
		return -1;
	}

	cap->irq_mask = 0;
	cap->map_size = PAGE_SIZE;
	cap->pci_irq = irq_info.irq;
	cap->io_offset = pcmcia_socket[sock].virt_io;

	return 0;

}				/* tx4925_pcmcia_inquire_socket() */

static int
tx4925_pcmcia_get_status(unsigned int sock, unsigned int *status)
{
	struct pcmcia_state state;

	if ((pcmcia_low_level->socket_state(sock, &state)) < 0) {
		printk(KERN_ERR "Unable to get PCMCIA status from kernel.\n");
		return -1;
	}

	pcmcia_socket[sock].k_state = state;

	*status = (state.detect ? SS_DETECT : 0);
	*status |= (state.ready ? SS_READY : 0);
	*status |= (pcmcia_socket[sock].cs_state.Vcc ? SS_POWERON : 0);

	if (pcmcia_socket[sock].cs_state.flags & SS_IOCARD) {
		*status |= (state.bvd1 ? SS_STSCHG : 0);
	} else {
		if (state.bvd1 == 0) {
			*status |= SS_BATDEAD;
		} else if (state.bvd2 == 0) {
			*status |= SS_BATWARN;
		}
	}

	*status |= (state.vs_3v ? SS_3VCARD : 0);
	*status |= (state.vs_Xv ? SS_XVCARD : 0);

	DEBUG(2, "\tstatus: %s%s%s%s%s%s%s%s\n",
	      (*status & SS_DETECT) ? "DETECT " : "",
	      (*status & SS_READY) ? "READY " : "",
	      (*status & SS_BATDEAD) ? "BATDEAD " : "",
	      (*status & SS_BATWARN) ? "BATWARN " : "",
	      (*status & SS_POWERON) ? "POWERON " : "",
	      (*status & SS_STSCHG) ? "STSCHG " : "",
	      (*status & SS_3VCARD) ? "3VCARD " : "",
	      (*status & SS_XVCARD) ? "XVCARD " : "");

	return 0;

}				/* tx4925_pcmcia_get_status() */

static int
tx4925_pcmcia_get_socket(unsigned int sock, socket_state_t * state)
{
	*state = pcmcia_socket[sock].cs_state;
	return 0;
}

static int
tx4925_pcmcia_set_socket(unsigned int sock, socket_state_t * state)
{
	struct pcmcia_configure configure;

	DEBUG(2, "\tmask:  %s%s%s%s%s%s\n\tflags: %s%s%s%s%s%s\n"
	      "\tVcc %d  Vpp %d  irq %d\n",
	      (state->csc_mask == 0) ? "<NONE>" : "",
	      (state->csc_mask & SS_DETECT) ? "DETECT " : "",
	      (state->csc_mask & SS_READY) ? "READY " : "",
	      (state->csc_mask & SS_BATDEAD) ? "BATDEAD " : "",
	      (state->csc_mask & SS_BATWARN) ? "BATWARN " : "",
	      (state->csc_mask & SS_STSCHG) ? "STSCHG " : "",
	      (state->flags == 0) ? "<NONE>" : "",
	      (state->flags & SS_PWR_AUTO) ? "PWR_AUTO " : "",
	      (state->flags & SS_IOCARD) ? "IOCARD " : "",
	      (state->flags & SS_RESET) ? "RESET " : "",
	      (state->flags & SS_SPKR_ENA) ? "SPKR_ENA " : "",
	      (state->flags & SS_OUTPUT_ENA) ? "OUTPUT_ENA " : "",
	      state->Vcc, state->Vpp, state->io_irq);

	configure.sock = sock;
	configure.vcc = state->Vcc;
	configure.vpp = state->Vpp;
	configure.output = (state->flags & SS_OUTPUT_ENA) ? 1 : 0;
	configure.speaker = (state->flags & SS_SPKR_ENA) ? 1 : 0;
	configure.reset = (state->flags & SS_RESET) ? 1 : 0;
	configure.iocard = (state->flags & SS_IOCARD) ? 1 : 0;

	if (pcmcia_low_level->configure_socket(&configure) < 0) {
		printk(KERN_ERR "Unable to configure socket %u\n", sock);
		return -1;
	}

	pcmcia_socket[sock].cs_state = *state;
	return 0;

}				/* tx4925_pcmcia_set_socket() */

static int
tx4925_pcmcia_get_io_map(unsigned int sock, struct pccard_io_map *map)
{
	DEBUG(1, "tx4925_pcmcia_get_io_map: sock %d\n", sock);
	if (map->map >= MAX_IO_WIN) {
		printk(KERN_ERR "%s(): map (%d) out of range\n", __FUNCTION__,
		       map->map);
		return -1;
	}
	*map = pcmcia_socket[sock].io_map[map->map];
	return 0;
}

int
tx4925_pcmcia_set_io_map(unsigned int sock, struct pccard_io_map *map)
{
	unsigned int speed;
	unsigned long size;

	if (map->map >= MAX_IO_WIN) {
		printk(KERN_ERR "%s(): map (%d) out of range\n", __FUNCTION__,
		       map->map);
		return -1;
	}

	if (map->flags & MAP_ACTIVE) {
		speed = (map->speed > 0 ? map->speed : TX4925_PCMCIA_IO_SPEED);
		pcmcia_socket[sock].speed_io = speed;
		/* FIXME: tune EBCCRn PWT,WT,SHWT */
		tx4925_pcmcia_resetup_speed(ebusc_no[1], speed);
	}

	if (map->stop == 1) {
		map->stop = PAGE_SIZE - 1;
	}

	size = map->stop - map->start;
	map->start = pcmcia_socket[sock].virt_io;
	map->stop = map->start + size;
	pcmcia_socket[sock].io_map[map->map] = *map;
	DEBUG(3, "set_io_map %d start %x stop %x\n", map->map, map->start,
	      map->stop);
	return 0;

}				/* tx4925_pcmcia_set_io_map() */

static int
tx4925_pcmcia_get_mem_map(unsigned int sock, struct pccard_mem_map *map)
{
	if (map->map >= MAX_WIN) {
		printk(KERN_ERR "%s(): map (%d) out of range\n", __FUNCTION__,
		       map->map);
		return -1;
	}
	*map = pcmcia_socket[sock].mem_map[map->map];
	return 0;
}

static int
tx4925_pcmcia_set_mem_map(unsigned int sock, struct pccard_mem_map *map)
{
	unsigned int speed;
	unsigned long size;

	if (map->map >= MAX_WIN) {
		printk(KERN_ERR "%s(): map (%d) out of range\n",
		       __FUNCTION__, map->map);
		return -1;
	}

	if (map->flags & MAP_ACTIVE) {
		speed = (map->speed > 0 ? map->speed : TX4925_PCMCIA_MEM_SPEED);

		/* FIXME: tune EBCCRn PWT,WT,SHWT */
		if (map->flags & MAP_ATTRIB) {
			pcmcia_socket[sock].speed_attr = speed;
			tx4925_pcmcia_resetup_speed(ebusc_no[2], speed);
		} else {
			pcmcia_socket[sock].speed_mem = speed;
			tx4925_pcmcia_resetup_speed(ebusc_no[0], speed);
		}
	}

	if (map->sys_stop == 0) {
		map->sys_stop = PAGE_SIZE - 1;
	}

	size = map->sys_stop - map->sys_start;

	if (map->flags & MAP_ATTRIB) {
		map->sys_start =
		    pcmcia_socket[sock].phys_attr + map->card_start;
	} else {
		map->sys_start = pcmcia_socket[sock].phys_mem + map->card_start;
	}

	map->sys_stop = map->sys_start + size;
	pcmcia_socket[sock].mem_map[map->map] = *map;

	DEBUG(3, "set_mem_map %d start %x stop %x card_start %x\n",
	      map->map, map->sys_start, map->sys_stop, map->card_start);
	return 0;

}				/* tx4925_pcmcia_set_mem_map() */

#if defined( CONFIG_PROC_FS )
static void
tx4925_pcmcia_proc_setup(unsigned int sock, struct proc_dir_entry *base)
{
	struct proc_dir_entry *entry;

	if ((entry = create_proc_entry("status", 0, base)) == NULL) {
		printk(KERN_ERR "Unable to install \"status\" procfs entry\n");
		return;
	}

	entry->read_proc = tx4925_pcmcia_proc_status;
	entry->data = (void *) sock;

#ifdef TX4925_GENERIC_DEBUG
	if ((entry = create_proc_entry("io", 0, base)) == NULL) {
		printk(KERN_ERR "Unable to install \"io\" procfs entry\n");
		return;
	}

	entry->read_proc = tx4925_pcmcia_proc_iodump;
	entry->data = NULL;

	if ((entry = create_proc_entry("mem", 0, base)) == NULL) {
		printk(KERN_ERR "Unable to install \"mem\" procfs entry\n");
		return;
	}

	entry->read_proc = tx4925_pcmcia_proc_memdump;
	entry->data = NULL;

	if ((entry = create_proc_entry("attr", 0, base)) == NULL) {
		printk(KERN_ERR "Unable to install \"attr\" procfs entry\n");
		return;
	}

	entry->read_proc = tx4925_pcmcia_proc_attrdump;
	entry->data = NULL;

	if ((entry = create_proc_entry("ebusc", 0, base)) == NULL) {
		printk(KERN_ERR "Unable to install \"ebusc\" procfs entry\n");
		return;
	}

	entry->read_proc = tx4925_pcmcia_proc_ebuscdump;
	entry->data = NULL;

#ifdef CONFIG_TOSHIBA_RBTX4925
	if ((entry = create_proc_entry("int_stat", 0, base)) == NULL) {
		printk(KERN_ERR
		       "Unable to install \"int_stat\" procfs entry\n");
		return;
	}

	entry->read_proc = rbtx4925_pcmcia_proc_interrupt;
	entry->data = NULL;
#endif				/* CONFIG_TOSHIBA_RBTX4925 */
#endif				/* TX4925_GENERIC_DEBUG */
}

#ifdef TX4925_GENERIC_DEBUG

#define PROC_DUMP_SIZE	0x80
static int
tx4925_pcmcia_proc_iodump(char *buf, char **start, off_t pos,
			  int count, int *eof, void *data)
{
	int len = 0, cnt;
	u32 io_addr;
	u32 tmp;

	len += sprintf(&buf[len], "\n              "
		       "+0 +1 +2 +3 +4 +5 +6 +7 +8 +9 +a +b +c +d +e +f");
	for (cnt = 0, io_addr = 0xb0000000; cnt < PROC_DUMP_SIZE;
	     cnt++, io_addr++) {
		if ((cnt % 16) == 0) {
			len += sprintf(&buf[len], "\n    %8.8x: ", io_addr);
		}
		tmp = (u32) * ((volatile u8 *) io_addr);
		len += sprintf(&buf[len], "%2.2x ", tmp & 0x0ff);
	}
	len += sprintf(&buf[len], "\n\n");

	return len;
}

static int
tx4925_pcmcia_proc_memdump(char *buf, char **start, off_t pos,
			   int count, int *eof, void *data)
{
	int len = 0, cnt;
	u32 io_addr;
	u32 tmp;

	len += sprintf(&buf[len], "\n              "
		       "+0 +1 +2 +3 +4 +5 +6 +7 +8 +9 +a +b +c +d +e +f");
	for (cnt = 0, io_addr = 0xac000000; cnt < PROC_DUMP_SIZE;
	     cnt++, io_addr++) {
		if ((cnt % 16) == 0) {
			len += sprintf(&buf[len], "\n    %8.8x: ", io_addr);
		}
		tmp = (u32) * ((volatile u8 *) io_addr);
		len += sprintf(&buf[len], "%2.2x ", tmp & 0x0ff);
	}
	len += sprintf(&buf[len], "\n\n");

	return len;
}

static int
tx4925_pcmcia_proc_attrdump(char *buf, char **start, off_t pos,
			    int count, int *eof, void *data)
{
	int len = 0, cnt;
	u32 io_addr;
	u32 tmp;

	len += sprintf(&buf[len], "\n              "
		       "+0 +1 +2 +3 +4 +5 +6 +7 +8 +9 +a +b +c +d +e +f");
	for (cnt = 0, io_addr = 0xa8000000; cnt < PROC_DUMP_SIZE;
	     cnt++, io_addr++) {
		if ((cnt % 16) == 0) {
			len += sprintf(&buf[len], "\n    %8.8x: ", io_addr);
		}
		tmp = (u32) * ((volatile u8 *) io_addr);
		len += sprintf(&buf[len], "%2.2x ", tmp & 0x0ff);
	}
	len += sprintf(&buf[len], "\n\n");

	return len;
}

static int
tx4925_pcmcia_proc_ebuscdump(char *buf, char **start, off_t pos,
			     int count, int *eof, void *data)
{
	int len = 0;
	int no;

	len +=
	    sprintf(&buf[len], "\n          ebusc no      cr          bar\n");

	no = ebusc_no[0];
	len += sprintf(&buf[len], "    MEM :    %d       %8.8x    %8.8x\n",
		       no, tx4925_ebuscptr->cr[no].c,
		       tx4925_ebuscptr->cr[no].b);
	no = ebusc_no[1];
	len += sprintf(&buf[len], "    I/O :    %d       %8.8x    %8.8x\n",
		       no, tx4925_ebuscptr->cr[no].c,
		       tx4925_ebuscptr->cr[no].b);
	no = ebusc_no[2];
	len += sprintf(&buf[len], "    ATTR:    %d       %8.8x    %8.8x\n\n",
		       no, tx4925_ebuscptr->cr[no].c,
		       tx4925_ebuscptr->cr[no].b);

	return len;
}

#ifdef CONFIG_TOSHIBA_RBTX4925
static int
rbtx4925_pcmcia_proc_interrupt(char *buf, char **start,
			       off_t pos, int count, int *eof, void *data)
{
	int len = 0;

	len += sprintf(&buf[len], "\nRBTX4925 PCMCIA Interrupt Status:\n");
	len += sprintf(&buf[len], "    interrupt: %s\n",
		       RBTX4925_IOC_REG(PCMCIA_INT_MASK) & 1 ? "enable" :
		       "disable");
	len +=
	    sprintf(&buf[len], "    pole     : %s\n",
		    RBTX4925_IOC_REG(PCMCIA_INT_POLE) & 1 ? "reverse" :
		    "through");
	len +=
	    sprintf(&buf[len], "    status1  : %d\n",
		    RBTX4925_IOC_REG(PCMCIA_INT_STAT1) & 1);
	len +=
	    sprintf(&buf[len], "    status2  : %d\n\n",
		    RBTX4925_IOC_REG(PCMCIA_INT_STAT2) & 1);

	len += sprintf(&buf[len], "\nL1121 PCMCIA/CompactFlash Controller:\n");
	len +=
	    sprintf(&buf[len], "    id             : %2.2x %2.2x %2.2x %2.2x\n",
		    (u32) L1121_inb(L1121_IDR1), (u32) L1121_inb(L1121_IDR2),
		    (u32) L1121_inb(L1121_IDR3), (u32) L1121_inb(L1121_IDR4));
	len +=
	    sprintf(&buf[len], "    status         : %2.2x\n",
		    (u32) L1121_inb(L1121_SR));
	len +=
	    sprintf(&buf[len], "    int source     : %2.2x\n",
		    (u32) L1121_inb(L1121_ISR));
	len +=
	    sprintf(&buf[len], "    edge int src   : %2.2x\n",
		    (u32) L1121_inb(L1121_EISR));
	len +=
	    sprintf(&buf[len], "    command1       : %2.2x\n",
		    (u32) L1121_inb(L1121_CR1));
	len +=
	    sprintf(&buf[len], "    command2       : %2.2x\n",
		    (u32) L1121_inb(L1121_CR2));
	len +=
	    sprintf(&buf[len], "    int enable     : %2.2x\n",
		    (u32) L1121_inb(L1121_IER));
	len +=
	    sprintf(&buf[len], "    edge int sence : %2.2x\n",
		    (u32) L1121_inb(L1121_ESNR));
	len +=
	    sprintf(&buf[len], "    int type select: %2.2x\n",
		    (u32) L1121_inb(L1121_ITSR));
	len +=
	    sprintf(&buf[len], "    edge clear     : %2.2x\n",
		    (u32) L1121_inb(L1121_ECLR));
	len +=
	    sprintf(&buf[len], "    command3       : %2.2x\n",
		    (u32) L1121_inb(L1121_CR3));
	len +=
	    sprintf(&buf[len], "    DAC control    : %2.2x\n",
		    (u32) L1121_inb(L1121_DACCR));
	len +=
	    sprintf(&buf[len], "    DAC data       : %2.2x\n\n",
		    (u32) L1121_inb(L1121_DACDR));

	return len;
}
#endif				/* CONFIG_TOSHIBA_RBTX4925 */
#endif				/* TX4925_GENERIC_DEBUG */

/* tx4925_pcmcia_proc_status()
 * Implements the /proc/bus/pccard/??/status file.
 *
 * Returns: the number of characters added to the buffer
 */
static int
tx4925_pcmcia_proc_status(char *buf, char **start, off_t pos,
			  int count, int *eof, void *data)
{
	char *p = buf;
	unsigned int sock = (unsigned int) data;
	struct tx4925_pcmcia_socket *psocket = &pcmcia_socket[sock];

	p += sprintf(p, "k_flags  : %s%s%s%s%s%s%s\n",
		     psocket->k_state.detect ? "detect " : "",
		     psocket->k_state.ready ? "ready " : "",
		     psocket->k_state.bvd1 ? "bvd1 " : "",
		     psocket->k_state.bvd2 ? "bvd2 " : "",
		     psocket->k_state.wrprot ? "wrprot " : "",
		     psocket->k_state.vs_3v ? "vs_3v " : "",
		     psocket->k_state.vs_Xv ? "vs_Xv " : "");

	p += sprintf(p, "status   : %s%s%s%s%s%s%s%s%s\n",
		     psocket->k_state.detect ? "SS_DETECT " : "",
		     psocket->k_state.ready ? "SS_READY " : "",
		     psocket->cs_state.Vcc ? "SS_POWERON " : "",
		     psocket->cs_state.flags & SS_IOCARD ? "SS_IOCARD " : "",
		     ((psocket->cs_state.flags & SS_IOCARD) &&
		      (psocket->k_state.bvd1)) ? "SS_STSCHG " : "",
		     (!(psocket->cs_state.flags & SS_IOCARD) &&
		      !(psocket->k_state.bvd1)) ? "SS_BATDEAD " : "",
		     (!(psocket->cs_state.flags & SS_IOCARD) &&
		      !(psocket->k_state.bvd2)) ? "SS_BATWARN " : "",
		     psocket->k_state.vs_3v ? "SS_3VCARD " : "",
		     psocket->k_state.vs_Xv ? "SS_XVCARD " : "");

	p += sprintf(p, "mask     : %s%s%s%s%s\n",
		     psocket->cs_state.csc_mask & SS_DETECT ? "SS_DETECT " : "",
		     psocket->cs_state.csc_mask & SS_READY ? "SS_READY " : "",
		     psocket->cs_state.
		     csc_mask & SS_BATDEAD ? "SS_BATDEAD " : "",
		     psocket->cs_state.
		     csc_mask & SS_BATWARN ? "SS_BATWARN " : "",
		     psocket->cs_state.
		     csc_mask & SS_STSCHG ? "SS_STSCHG " : "");

	p += sprintf(p, "cs_flags : %s%s%s%s%s\n",
		     psocket->cs_state.
		     flags & SS_PWR_AUTO ? "SS_PWR_AUTO " : "",
		     psocket->cs_state.flags & SS_IOCARD ? "SS_IOCARD " : "",
		     psocket->cs_state.flags & SS_RESET ? "SS_RESET " : "",
		     psocket->cs_state.
		     flags & SS_SPKR_ENA ? "SS_SPKR_ENA " : "",
		     psocket->cs_state.
		     flags & SS_OUTPUT_ENA ? "SS_OUTPUT_ENA " : "");

	p += sprintf(p, "Vcc      : %d\n", psocket->cs_state.Vcc);
	p += sprintf(p, "Vpp      : %d\n", psocket->cs_state.Vpp);
	p += sprintf(p, "irq      : %d\n", psocket->cs_state.io_irq);
	p += sprintf(p, "I/O      : %u\n", psocket->speed_io);
	p += sprintf(p, "attribute: %u\n", psocket->speed_attr);
	p += sprintf(p, "common   : %u\n", psocket->speed_mem);

	return p - buf;
}

#ifndef MODULE
static int __init
tx4925_pcmcia_setup(char *str)
{
	char *p = str;

	while (p) {
		if (!strncmp(p, "poll:", 5)) {
			poll_interval = simple_strtol(p + 5, NULL, 0);
			if (poll_interval < 0) {
				poll_interval = TX4925_PCMCIA_POLL_PERIOD;
			}
		}
		p = strchr(p, ',');
		if (p) {
			p++;
		}
	}
	return 1;
}

__setup("tx4925_pcmcia=", tx4925_pcmcia_setup);
#endif				/* !MODULE */

#endif				/* defined( CONFIG_PROC_FS ) */
