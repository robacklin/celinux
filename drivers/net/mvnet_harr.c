/*
 *
 *  mvnet/driver/mvnet_harr.c, version 2.0
 *
 *  Copyright 2002-2003, MontaVista Software, Inc.
 *
 *  This software may be used and distributed according to the terms of
 *  the GNU Public License, Version 2, incorporated herein by reference.
 *
 *  Contact:  <source@mvista.com>
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/pci.h>
#include <linux/sched.h>
#include <linux/stddef.h>
#include <linux/string.h>
#include <linux/tqueue.h>
#include <linux/types.h>
#include <asm/io.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>

#include <linux/harrier_defs.h>

#include <linux/reboot.h>

#include "mvnet.h"

MODULE_AUTHOR("MontaVista Software <source@mvista.com>");
MODULE_DESCRIPTION("MontaVista Net Backpanel Driver (Harrier driver)");
MODULE_LICENSE("GPL");

#undef USE_TASKLET
#undef USE_SCHEDULE_TASK

#ifndef NDEBUG
#define Debug(message)	printk message
#define assert(expr)	\
    if (!(expr))	\
	printk("Assertion failure:  (%s) in %s, %s:%d\n", \
		#expr, __FUNCTION__, __FILE__, __LINE__); else
#else /* NDEBUG */
#define Debug(message)
#define assert(expr)
#endif /* NDEBUG */

#define HARR_NAME	"mvnet_harr"
#define PRINT_PREFIX	"mvnet_harr:  "

#define HARR_SYSMEM_PPC_BASE	0xe0000000
#define HARR_SYSMEM_PCI_BASE	0x00000000

#define NOT_BREAK	1

/* Use 1MB mappings for memory accesses */
#define MVNET_HARR_MEM_SIZE		(1024 * 1024)
#define MVNET_HARR_MEM_MASK		(~(MVNET_HARR_MEM_SIZE - 1))

/* Secondary->Primary interrupts */
#define MVNET_HARR_INTR_ACCEPT		(1<<0)
#define MVNET_HARR_INTR_START		(1<<1)
#define MVNET_HARR_INTR_STOP		(1<<2)

/* Primary->Secondary interrupts */
#define MVNET_HARR_INTR_CONNECT		(1<<0)
#define MVNET_HARR_INTR_UPDATE		(1<<1)
#define MVNET_HARR_INTR_DISCONNECT	(1<<2)

/* Common interrupts */
#define MVNET_HARR_INTR_RECV		(1<<3)
#define MVNET_HARR_INTR_ALL		(MVNET_HARR_INTR_ACCEPT | \
					 MVNET_HARR_INTR_START	| \
					 MVNET_HARR_INTR_STOP   | \
					 MVNET_HARR_INTR_RECV)

/* Events */
#define MVNET_HARR_EVENT_ACCEPT		(1<<0)
#define MVNET_HARR_EVENT_CONNECT	(1<<1)
#define MVNET_HARR_EVENT_START		(1<<2)
#define MVNET_HARR_EVENT_UPDATE		(1<<3)
#define MVNET_HARR_EVENT_RECV		(1<<4)
#define MVNET_HARR_EVENT_STOP		(1<<5)
#define MVNET_HARR_EVENT_DISCONNECT	(1<<6)


/* Per-device information */
struct mvnet_harr_device {
    /* Control and Status Registers */
    unsigned long csr;
    unsigned long csr_paddr;
    unsigned long csr_size;

    /* Mapped memory */
    void *mem;
    unsigned long mem_paddr;
    unsigned long buf_paddr;
    unsigned long mem_size;

    int primary;
    void (*irq_handler)(int irq, void *dev_id, struct pt_regs *regs);

    unsigned int irq;
    volatile u32 *irq_out;
    volatile u32 *irq_in;
    volatile u32 *irq_msk;

    u32 events;
    struct mvnet_harr_device *next_event;

    struct mvnet_device *netdev;
    struct mvnet_funcs *netfn;
    struct pci_dev *pdev;
    u32 ppc_base;
    u32 pci_base;
    u32 rtmemstart;
    u32 rtmemend;
};

static union {
    char string[4];
    u32 integer;
} mvnet_harr_signature = { { 'N', 'E', 'T', /* version */ 2 } };

/* DEBUG */
#ifndef NDEBUG
static void
mvnet_harr_dump_csr(struct mvnet_harr_device *dev,
		     unsigned long start, unsigned long end)
{
	printk(PRINT_PREFIX "mvnet_harr_dump_csr\n");

	if (dev->primary) {
		start += (unsigned long) HARRIER_MP_PMEP(dev->csr);
		end   += (unsigned long) HARRIER_MP_PMEP(dev->csr);
	}
	else {
		start += (unsigned long) HARRIER_MP_XCSR(dev->csr);
		end   += (unsigned long) HARRIER_MP_XCSR(dev->csr);
	}

	for (; start < end; start++)
	{
		if (!((start - end) % 8))
				printk("\n0x%08lx ", start);
		printk("%2.2x ", *(unsigned char *) start);
			      
	}
	printk("\n");

}
#endif

static void
mvnet_harr_print_error(char *str, struct pci_dev *pdev)
{
	printk("%s %s (b:d:f = %d:%d:%d)\n", PRINT_PREFIX, str,
	       pdev->bus->number, PCI_SLOT(pdev->devfn), PCI_FUNC(pdev->devfn));
}

static int
mvnet_harr_window_init(u32 ppc_reg_base, u32 window, u32 size, u32 ppc_base,
		       u32 pci_base)
{

	u32 addr, offset;
	u32 scale = HARRIER_OTAD1_OFF - HARRIER_OTAD0_OFF;
	u32 reg_base = ppc_reg_base + (scale * window);

	Debug((PRINT_PREFIX "mvnet_harr_window_init: size = 0x%08x, ppc = 0x%08x, pci = 0x%08x\n", size, ppc_base, pci_base));


	/* Verify size is a multiple of 64K */
	if (size & 0x0000ffff) {
		printk("mvnet_harr_window_init: size not multiple of 64K (0x%08x)\n", size);
		return -EINVAL;
	}

	/* Verify base addresses are multiples of 64K */
	if ((ppc_base & 0x0000ffff) || 
	    (pci_base & 0x0000ffff)) {
		printk("mvnet_harr_window_init: PPC (0x%08x) or PCI (0x%08x) base error\n", ppc_base, pci_base);
		return -EINVAL;
	}

	/* disable the window */
	writel(0, reg_base + HARRIER_OTOF0_OFF);

	if (size) {
		addr = (ppc_base & 0xffff0000) | ((ppc_base + size - 1) >> 16);
#ifdef CONFIG_HARRIER_STORE_GATHERING
		offset = (pci_base - ppc_base) | 0x9a;
#else
		offset = (pci_base - ppc_base) | 0x92;
#endif
		writel(cpu_to_le32(addr), reg_base + HARRIER_OTAD0_OFF);
		writel(cpu_to_le32(offset), reg_base + HARRIER_OTOF0_OFF);
	}

	return 0;
}

static int
mvnet_harr_window_move(struct mvnet_harr_device *dev, u32 ppc_reg_base,
		       u32 window, u32 pci_size, u32 pci_base)
{

	u32 scale = HARRIER_OTAD1_OFF - HARRIER_OTAD0_OFF;
	u32 reg_base = ppc_reg_base + (scale * window);
	u32 ppc_base, ppc_size;

	Debug((PRINT_PREFIX "mvnet_harr_window_move: window %d size = 0x%08x, pci = 0x%08x\n", window, pci_size, pci_base));


	/* Verify size is a multiple of 64K */
	if (pci_size & ~HARRIER_OTOFX_OFF_MSK) {
		printk("mvnet_harr_window_move: size not multiple of 64K (0x%08x)\n", pci_size);
		return -EINVAL;
	}

	/* Verify base address is a multiple of 64K */
	if (pci_base & ~HARRIER_OTOFX_OFF_MSK) {
		printk("mvnet_harr_window_move: PCI (0x%08x) base error\n",
			pci_base);
		return -EINVAL;
	}

	/* determine current window size */
	ppc_base = le32_to_cpu(readl(reg_base + HARRIER_OTAD0_OFF));
	ppc_size = ((ppc_base + 1) & HARRIER_OTADX_END_MSK) << 16;
	ppc_base &= HARRIER_OTADX_START_MSK;
	ppc_size -= ppc_base;

	/* if new window size is too big, return error */
	if (pci_size > ppc_size) {
		printk("mvnet_harr_window_move: pci size (%08x) exceeds ppc size (%08x)\n", pci_size, ppc_size);
		return -EINVAL;
	}

	dev->ppc_base = ppc_base;
	dev->pci_base = pci_base;

	Debug((PRINT_PREFIX "mvnet_harr_window_move: wind %d, size = %08x, ppc_base = %08x, pci_base = %08x\n",
	      window, pci_size, ppc_base, pci_base));

	return mvnet_harr_window_init(ppc_reg_base, window, pci_size,
				      ppc_base, pci_base);
}

static void
mvnet_harr_set_bits(volatile u32 *addr, u32 bits)
{
	unsigned long flags;

	Debug((PRINT_PREFIX "mvnet_harr_set_bits: addr = 0x%08x, bits = 0x%08x\n", (u32)addr, bits));
	local_irq_save(flags);
	writel(readl(addr) | bits, addr);
	local_irq_restore(flags);
}

static void
mvnet_harr_clear_bits(volatile u32 *addr, u32 bits)
{
	unsigned long flags;

	Debug((PRINT_PREFIX "mvnet_harr_clear_bits: addr = 0x%08x, bits = 0x%08x\n",
		(u32)addr, bits));
	local_irq_save(flags);
	writel(readl(addr) & ~bits, addr);
	local_irq_restore(flags);
}

/*
 *  Interrupt/event handling
 */


static struct mvnet_harr_device *event_queue = NULL;
static spinlock_t event_lock = SPIN_LOCK_UNLOCKED;

#ifdef USE_TASKLET
static void
mvnet_harr_handle_events(unsigned long data)
#else
static void
mvnet_harr_handle_events(void *data)
#endif
{
    struct mvnet_harr_device *dev = NULL;
    unsigned long flags;
    u32 events = 0;
    struct mvnet_event evnt;

    Debug((PRINT_PREFIX "mvnet_harr_handle_events\n"));

    while (event_queue != NULL) {
	spin_lock_irqsave(&event_lock, flags);
	if (event_queue != NULL) {
	    dev = event_queue;
	    event_queue = dev->next_event;
	    events = dev->events;
	    dev->events = 0;
	}
	spin_unlock_irqrestore(&event_lock, flags);

	/*
	 * "Accept" events received by master only.
	 */
	if (events & MVNET_HARR_EVENT_ACCEPT) {
	    u32 signature = readl(dev->csr +
			          HARRIER_MP_PMEP(HARRIER_MGOM0_OFF));

	    Debug((PRINT_PREFIX "MVNET_HARR_EVENT_ACCEPT\n"));

	    writel(MVNET_HARR_INTR_ACCEPT, dev->irq_in);

	    if (signature == be32_to_cpu(mvnet_harr_signature.integer)) {
		u32 offset = readl(dev->csr +
				   HARRIER_MP_PMEP(HARRIER_MGOM1_OFF));
		void *vaddr = (void *) (((u8 *) dev->mem) + offset);

		dev->buf_paddr = dev->mem_paddr + offset;
		evnt.event = accept;
		evnt.p1 = dev->netdev;
		evnt.p2 = vaddr;
		evnt.p3 = (void *)dev->csr_paddr;
		evnt.p4 = (void *)dev->buf_paddr;

		if (mvnet_proc_event(&evnt) != 0)
		{
		    mvnet_harr_clear_bits(dev->irq_msk, MVNET_HARR_INTR_ACCEPT);
		}
	    } else {
		Debug((PRINT_PREFIX "sig addr = 0x%08lx\n",
			dev->csr + HARRIER_MGOM0_OFF));
		Debug((PRINT_PREFIX "invalid signature 0x%X\n", signature));
		mvnet_harr_set_bits(dev->irq_msk, MVNET_HARR_INTR_ACCEPT);
	    }

	    events &= ~MVNET_HARR_EVENT_ACCEPT;
	}

	/*
	 * "Connect" events received by target only.
	 */
	if (events & MVNET_HARR_EVENT_CONNECT) {
	    u32 sysbuf = readl(dev->csr +
			       HARRIER_MP_XCSR(HARRIER_MGIM0_OFF));
	    u32 base = sysbuf & MVNET_HARR_MEM_MASK;
	    u32 offset = sysbuf & ~MVNET_HARR_MEM_MASK;
	    void *sysmem = (void *) (((u8 *) dev->mem) + offset);
	    u32 devnum = readl(dev->csr +
			       HARRIER_MP_XCSR(HARRIER_MGIM1_OFF));
	    struct irq_spec irq;

	    Debug((PRINT_PREFIX "MVNET_HARR_EVENT_CONNECT\n"));

	    writel(MVNET_HARR_INTR_CONNECT, dev->irq_in);

	    dev->buf_paddr = sysbuf;

	    mvnet_harr_window_init(HARRIER_DEFAULT_XCSR_BASE, 2, dev->mem_size,
			           HARR_SYSMEM_PPC_BASE, base);

	    irq.addr = HARRIER_MP_PMEP(HARRIER_MGID_OFF);
	    irq.val = MVNET_HARR_INTR_RECV;
	    irq.len  = sizeof(u32);

	    evnt.event = connect;
	    evnt.p1 = dev->netdev;
	    evnt.p2 = (void *)sysmem;
	    evnt.p3 = (void *)devnum;
	    evnt.p4 = (void *) &irq;

	    if (mvnet_proc_event(&evnt) != 0)
		    mvnet_harr_clear_bits(dev->irq_msk,
				          MVNET_HARR_INTR_CONNECT);
	    events &= ~MVNET_HARR_EVENT_CONNECT;
	}

	/*
	 * "Start" events received by master only.
	 */
	if (events & MVNET_HARR_EVENT_START) {
	    Debug((PRINT_PREFIX "MVNET_HARR_EVENT_START\n"));

	    writel(MVNET_HARR_INTR_START, dev->irq_in);

	    evnt.event = start;
	    evnt.p1 = dev->netdev;

	    if (mvnet_proc_event(&evnt) == 0) {
		writel(MVNET_HARR_INTR_STOP | MVNET_HARR_INTR_RECV,
		       dev->irq_in);
		mvnet_harr_clear_bits(dev->irq_msk, (MVNET_HARR_INTR_STOP |
				      MVNET_HARR_INTR_RECV));
	    } else {
		mvnet_harr_clear_bits(dev->irq_msk, MVNET_HARR_INTR_START);
	    }

	    events &= ~MVNET_HARR_EVENT_START;
	}

	/*
	 * "Update" events received by target only.
	 */
	if (events & MVNET_HARR_EVENT_UPDATE) {
	    Debug((PRINT_PREFIX "MVNET_HARR_EVENT_UPDATE\n"));

	    writel(MVNET_HARR_INTR_UPDATE, dev->irq_in);

	    evnt.event = update;
	    evnt.p1 = dev->netdev;

	    (void) mvnet_proc_event(&evnt);

	    mvnet_harr_clear_bits(dev->irq_msk, MVNET_HARR_INTR_UPDATE);
	    events &= ~MVNET_HARR_EVENT_UPDATE;
	}

	/*
	 * "Receive" events received by both master and target.
	 */
	if (events & MVNET_HARR_EVENT_RECV) {
	    Debug((PRINT_PREFIX "MVNET_HARR_EVENT_RECV\n"));

	    writel(MVNET_HARR_INTR_RECV, dev->irq_in);

	    evnt.event = receive;
	    evnt.p1 = dev->netdev;

	    (void) mvnet_proc_event(&evnt);

	    mvnet_harr_clear_bits(dev->irq_msk, MVNET_HARR_INTR_RECV);
	    events &= ~MVNET_HARR_EVENT_RECV;
	}

	/*
	 * "Stop" events received by master only.
	 */
	if (events & MVNET_HARR_EVENT_STOP) {
	    Debug((PRINT_PREFIX "MVNET_HARR_EVENT_STOP\n"));

	    writel(MVNET_HARR_INTR_STOP, dev->irq_in);
	    mvnet_harr_set_bits(dev->irq_msk, MVNET_HARR_INTR_STOP);

	    evnt.event = stop;
	    evnt.p1 = dev->netdev;

	    (void) mvnet_proc_event(&evnt);

	    events &= ~MVNET_HARR_EVENT_STOP;
	}

	/*
	 * "Disconnect" events received by target only.
	 */
	if (events & MVNET_HARR_EVENT_DISCONNECT) {
	    Debug((PRINT_PREFIX "MVNET_HARR_EVENT_DISCONNECT\n"));

	    writel(MVNET_HARR_INTR_DISCONNECT, dev->irq_in);
	    mvnet_harr_set_bits(dev->irq_msk, MVNET_HARR_INTR_ALL);

	    evnt.event = disconnect;
	    evnt.p1 = dev->netdev;

	    (void) mvnet_proc_event(&evnt);

	    events &= ~MVNET_HARR_EVENT_DISCONNECT;
	}
    }
}

#ifdef USE_TASKLET
static DECLARE_TASKLET(mvnet_harr_event_tasklet, mvnet_harr_handle_events, 0);

#else
static struct tq_struct mvnet_harr_event_task = {
    routine:  mvnet_harr_handle_events,
};
#endif

static void
mvnet_harr_intr(
    struct mvnet_harr_device *dev,
    u32 intrs,
    u32 intr,
    unsigned int event
)
{
    unsigned long flags;

    if (intrs & intr) {
	Debug((PRINT_PREFIX "intrs = 0x%08x, intr = 0x%08x\n", intrs, intr));

	mvnet_harr_set_bits(dev->irq_msk, intr);

	spin_lock_irqsave(&event_lock, flags);
	if (dev->events == 0) {
	    dev->next_event = event_queue;
	    event_queue = dev;
	}
	dev->events |= event;
	spin_unlock_irqrestore(&event_lock, flags);

#ifdef USE_TASKLET
	tasklet_hi_schedule(&mvnet_harr_event_tasklet);
#else
#ifdef USE_SCHEDULE_TASK
	schedule_task(&mvnet_harr_event_task);
#else
	queue_task(&mvnet_harr_event_task, &tq_immediate);
	mark_bh(IMMEDIATE_BH);
#endif
#endif
    }
}


static void
mvnet_harr_intr_primary(int irq, void *dev_id, struct pt_regs *regs)
{
    struct mvnet_harr_device *dev = (struct mvnet_harr_device *) dev_id;
    u32 intrs = readl(dev->irq_in) & ~(readl(dev->irq_msk));

    Debug((PRINT_PREFIX "mvnet_harr_intr_primary: intrs = 0x%08x\n", intrs));

#ifndef NDEBUG
    /* DEBUG */
    mvnet_harr_dump_csr(dev, 0, 0x30);
#endif
    mvnet_harr_intr(dev,intrs,MVNET_HARR_INTR_ACCEPT,
	MVNET_HARR_EVENT_ACCEPT);
    mvnet_harr_intr(dev,intrs,MVNET_HARR_INTR_START,MVNET_HARR_EVENT_START);
    mvnet_harr_intr(dev,intrs,MVNET_HARR_INTR_RECV,MVNET_HARR_EVENT_RECV);
    mvnet_harr_intr(dev,intrs,MVNET_HARR_INTR_STOP,MVNET_HARR_EVENT_STOP);
}


static void
mvnet_harr_intr_secondary(int irq, void *dev_id, struct pt_regs *regs)
{
    struct mvnet_harr_device *dev = (struct mvnet_harr_device *) dev_id;
    u32 intrs = readl(dev->irq_in) & ~(readl(dev->irq_msk));

    Debug((PRINT_PREFIX "mvnet_harr_intr_secondary: intrs = 0x%08x\n", intrs));

#ifndef NDEBUG
    /* DEBUG */
    mvnet_harr_dump_csr(dev, 0, 0x30);
#endif
    mvnet_harr_intr(dev,intrs,MVNET_HARR_INTR_CONNECT,
	MVNET_HARR_EVENT_CONNECT);
    mvnet_harr_intr(dev,intrs,MVNET_HARR_INTR_UPDATE,
	MVNET_HARR_EVENT_UPDATE);
    mvnet_harr_intr(dev,intrs,MVNET_HARR_INTR_RECV,MVNET_HARR_EVENT_RECV);
    mvnet_harr_intr(dev,intrs,MVNET_HARR_INTR_DISCONNECT,
	MVNET_HARR_EVENT_DISCONNECT);
}


static void
mvnet_harr_send_8(struct irq_spec *irq)
{
    Debug((PRINT_PREFIX "mvnet_harr_send_8: %08x, %02x\n", irq->addr,
	  (u8)irq->val));
    writeb(irq->val, irq->addr);
}

static void
mvnet_harr_send_16(struct irq_spec *irq)
{
    Debug((PRINT_PREFIX "mvnet_harr_send_16: %08x, %04x\n", irq->addr,
	  (u16)irq->val));
    writew(irq->val, irq->addr);
}

static void
mvnet_harr_send_32(struct irq_spec *irq)
{
    Debug((PRINT_PREFIX "mvnet_harr_send_32: %08x, %08x\n", irq->addr,
	  (u32)irq->val));
    writel(irq->val, irq->addr);
};

/*
 *  mvnet_harr_functions
 */


/*
 * Called by master only. Un-masks (enables) the "accept" interrupt to come in
 * from the target.
 */
static int
mvnet_harr_listen(void *data)
{
    struct mvnet_harr_device *dev = (struct mvnet_harr_device *) data;

    Debug((PRINT_PREFIX "mvnet_harr_listen\n"));
    assert(dev->primary);

    writel(MVNET_HARR_INTR_ALL & ~MVNET_HARR_INTR_ACCEPT, dev->irq_in);
    mvnet_harr_clear_bits(dev->irq_msk, MVNET_HARR_INTR_ACCEPT);
#ifndef NDEBUG
    /* DEBUG */
    mvnet_harr_dump_csr(dev, 0, 0x30);
#endif
    return 0;
}


/*
 * Called by master only. Sends "connect" interrupt to target and passes
 * target's device number and address of buffer space in masters RAM.
 */
static int
mvnet_harr_accept(void *data, unsigned long sysbuf, int devnum)
{
    struct mvnet_harr_device *dev = (struct mvnet_harr_device *) data;

    Debug((PRINT_PREFIX "mvnet_harr_accept\n"));

    writel(sysbuf, dev->csr + HARRIER_MP_PMEP(HARRIER_MGIM0_OFF));
    writel(devnum, dev->csr + HARRIER_MP_PMEP(HARRIER_MGIM1_OFF));

    writel(MVNET_HARR_INTR_CONNECT, dev->irq_out);

    mvnet_harr_clear_bits(dev->irq_msk, MVNET_HARR_INTR_START);
#ifndef NDEBUG
    /* DEBUG */
    mvnet_harr_dump_csr(dev, 0, 0x30);
#endif
    return 0;
}


/*
 * Called by target only. Generates an "accept" interrupt to master if there
 * is no (inbound) "connect" interrupt pending. Also sets up address
 * translation of downstream memory 2 window to address local buffer address,
 * and passes the offset from the base of downstream memory 2 to the master
 * for eventual use by the master and other targets.
 */
static int
mvnet_harr_connect(void *data, void *buf)
{
    struct mvnet_harr_device *dev = (struct mvnet_harr_device *) data;
    u32 busaddr = virt_to_phys(buf);
    u32 base, temp;

    Debug((PRINT_PREFIX "mvnet_harr_connect\n"));

    Debug((PRINT_PREFIX "mvnet_harr_connect: buf = %08x, busaddr = %08x\n",
	  (unsigned int) buf, busaddr));

    base = busaddr & MVNET_HARR_MEM_MASK;
    temp = readl(dev->csr + HARRIER_XCSR_CONFIG(HARRIER_ITBAR0_OFF));

    Debug((PRINT_PREFIX "mvnet_harr_connect: base = %08x, pci = %08x\n",
	  (unsigned int) base, temp));

    temp = readl(dev->csr + HARRIER_XCSR_CONFIG(HARRIER_ITSZ0_OFF));
    temp = (temp & 0x0000ffff) | base;
    writel(temp, dev->csr + HARRIER_XCSR_CONFIG(HARRIER_ITSZ0_OFF));

    Debug((PRINT_PREFIX "mvnet_harr_connect: ITAIO_OFF = %08x\n", temp));

    temp = readl(dev->csr + HARRIER_XCSR_CONFIG(HARRIER_ITSZ0_OFF));
    Debug((PRINT_PREFIX "temp = 0x%08x\n", temp));

    Debug((PRINT_PREFIX "mvnet_harr_connect: MGOM0 = 0x%08lx, MGOM1 = 0x%08lx\n", 
	   dev->csr + HARRIER_MP_XCSR(HARRIER_MGOM0_OFF),
	   dev->csr + HARRIER_MP_XCSR(HARRIER_MGOM1_OFF)));

    writel(mvnet_harr_signature.integer, dev->csr +
		   HARRIER_MP_XCSR(HARRIER_MGOM0_OFF));
    writel(busaddr & ~MVNET_HARR_MEM_MASK, dev->csr +
	   HARRIER_MP_XCSR(HARRIER_MGOM1_OFF));

    /* clear all but connect interrupt. */
    writel(MVNET_HARR_INTR_ALL & ~MVNET_HARR_INTR_CONNECT, dev->irq_in);

    /*
     * if a connect interrupt is not pending, generate an outboud accept
     * to get the ball rolling.
     */
    if ((readl(dev->irq_in) & MVNET_HARR_INTR_CONNECT) == 0)
	    writel(MVNET_HARR_INTR_ACCEPT, dev->irq_out);

    mvnet_harr_clear_bits(dev->irq_msk, MVNET_HARR_INTR_CONNECT);
#ifndef NDEBUG
    /* DEBUG */
    mvnet_harr_dump_csr(dev, 0, 0x30);
#endif
    return 0;
}

/*
 * Called by target only. Sends "start" interrupt to master indicating that
 * target is up and ready to receive packets.
 */
static int
mvnet_harr_start(void *data, unsigned long rtmemstart, unsigned long rtmemend)
{
    struct mvnet_harr_device *dev = (struct mvnet_harr_device *) data;

    Debug((PRINT_PREFIX "mvnet_harr_start %08x, %08x\n", rtmemstart, rtmemend));

    (void) mvnet_harr_window_move(dev, HARRIER_DEFAULT_XCSR_BASE, 0,
			    	  rtmemend + 1 - rtmemstart, rtmemstart);

    writel(MVNET_HARR_INTR_UPDATE | MVNET_HARR_INTR_RECV |
	           MVNET_HARR_INTR_DISCONNECT, dev->irq_in);
    mvnet_harr_clear_bits(dev->irq_msk, (MVNET_HARR_INTR_UPDATE |
		          MVNET_HARR_INTR_RECV | MVNET_HARR_INTR_DISCONNECT));
    writel(MVNET_HARR_INTR_START, dev->irq_out);

#ifndef NDEBUG
    /* DEBUG */
    mvnet_harr_dump_csr(dev, 0, 0x30);
#endif

    return 0;
}


/* Called by master only. Alerts target to a change in system data by 
 * sending an "update" interrupt to the target.
 */
static int
mvnet_harr_update(void *data)
{
    struct mvnet_harr_device *dev = (struct mvnet_harr_device *) data;

    Debug((PRINT_PREFIX "mvnet_harr_update\n"));

    writel(MVNET_HARR_INTR_UPDATE, dev->irq_out);

#ifndef NDEBUG
    /* DEBUG */
    mvnet_harr_dump_csr(dev, 0, 0x30);
#endif
    return 0;
}

struct mvnet_device_conn {
	struct mvnet_harr_device *dev;
	unsigned long csr_vaddr;
	unsigned long csr;
	u8 *mem;
	void (*irq_send)(struct irq_spec *irq);
	struct irq_spec irq;
	int num;
	int indirect;
};

static void *
mvnet_harr_open_conn(
    void *data,
    int num,
    int indirect,
    volatile struct nodeinfo *node,
    void **devconn
)
{
    struct mvnet_harr_device *dev = (struct mvnet_harr_device *) data;
    struct mvnet_device_conn *conn;
    u32 mem_paddr;
    u32 csr_paddr;
    unsigned long base;
    void *membase;

    Debug((PRINT_PREFIX "mvnet_harr_open_conn: %d\n", num));

    if (node == NULL) {
	    printk("NULL node pointer\n");
	    return NULL;
    }

    mem_paddr = readl(&node->mem_paddr);
    csr_paddr = readl(&node->csr_paddr);

    Debug((PRINT_PREFIX "node = %08x, mem_paddr = %08x, csr_paddr = %08x\n",
	   node, mem_paddr, csr_paddr));

    conn = kmalloc(sizeof(struct mvnet_device_conn), GFP_ATOMIC);
    if (conn == NULL) {
	printk(PRINT_PREFIX "Cannot allocate memory for device connection\n");
	return NULL;
    }
    memset(conn, 0, sizeof(struct mvnet_device_conn));

    if (indirect) {
	assert(dev->primary == 0);

	base = (csr_paddr & MVNET_HARR_MEM_MASK) - dev->pci_base +
		dev->ppc_base;
	conn->csr_vaddr = (unsigned long) ioremap_nocache(base,
						    MVNET_HARR_MEM_SIZE);
	if (conn->csr_vaddr == (unsigned long) NULL) {
	    printk(PRINT_PREFIX "Cannot map CSR for connection\n");
	    kfree(conn);
	    return NULL;
	}

	conn->csr = conn->csr_vaddr + (csr_paddr & ~MVNET_HARR_MEM_MASK);

	Debug((PRINT_PREFIX "csr = %08x, csr_vaddr = %08x csr_paddr = %08x\n",
	       conn->csr, conn->csr_vaddr, csr_paddr));

	base = (mem_paddr & MVNET_HARR_MEM_MASK) - dev->pci_base +
	       dev->ppc_base;
#ifdef CONFIG_MVNET_STORE_GATHERING
	conn->mem = __ioremap(base, MVNET_HARR_MEM_SIZE, _PAGE_NO_CACHE);
#else
	conn->mem = ioremap(base, MVNET_HARR_MEM_SIZE);
#endif
	if (conn->mem == NULL) {
	    printk(PRINT_PREFIX "Cannot map memory buffer for connection\n");
	    iounmap((void*)conn->csr_vaddr);
	    kfree(conn);
	    return NULL;
	}

	membase = conn->mem + (mem_paddr & ~MVNET_HARR_MEM_MASK);

	Debug((PRINT_PREFIX "mem = %08x, mem_paddr = %08x, membase = %08x\n",
	       conn->mem, mem_paddr, membase));

	conn->irq.addr = (u32)conn->csr + readl(&node->irq.addr);
	conn->irq.len = readl(&node->irq.len);
	conn->irq.val = readl(&node->irq.val);

	switch (conn->irq.len) {
		case sizeof(u8):
			conn->irq_send = mvnet_harr_send_8;
			break;
		case sizeof(u16):
			conn->irq_send = mvnet_harr_send_16;
			break;
		case sizeof(u32):
			conn->irq_send = mvnet_harr_send_32;
			break;
		default:
			printk(PRINT_PREFIX "Unknown IRQ size %d\n",
			       conn->irq.len);
			iounmap((void*)conn->mem);
			iounmap((void*)conn->csr_vaddr);
			kfree(conn);
			return NULL;
	}

	Debug((PRINT_PREFIX "conn->irq.addr = %08x\n", conn->irq.addr));

    } else {
	assert(mem_paddr == dev->buf_paddr);

	Debug((PRINT_PREFIX "mem_paddr = %08x, buf_paddr = %08x\n", mem_paddr,
		dev->buf_paddr));

	if (!dev->primary) {
	    assert(num == 0);
	}

	conn->irq.addr = (u32)dev->irq_out;
	conn->irq.len  = sizeof(u32);
	conn->irq.val = MVNET_HARR_INTR_RECV;
	conn->irq_send = mvnet_harr_send_32;

	membase = ((u8 *) dev->mem) + (dev->buf_paddr & ~MVNET_HARR_MEM_MASK);
    }

    conn->dev = dev;
    conn->num = num;
    conn->indirect = indirect;

    *devconn = (void *) conn;
    return membase;
}


static int
mvnet_harr_send(void *devconn)
{
    struct mvnet_device_conn *conn = (struct mvnet_device_conn *) devconn;

    Debug((PRINT_PREFIX "mvnet_harr_send\n"));

    conn->irq_send(&conn->irq);

    return 0;
}


static void
mvnet_harr_close_conn(void *devconn)
{
    struct mvnet_device_conn *conn = (struct mvnet_device_conn *) devconn;
#ifndef NDEBUG
    struct mvnet_harr_device *dev = conn->dev;
#endif
    Debug((PRINT_PREFIX "mvnet_harr_close_conn\n"));

    if (conn->indirect) {
	assert(dev->primary == 0);

	if (conn->csr_vaddr)
		iounmap((void *)conn->csr_vaddr);
	if (conn->mem)
		iounmap(conn->mem);
    }

    kfree(conn);
}


static void
mvnet_harr_stop(void *data)
{
    struct mvnet_harr_device *dev = (struct mvnet_harr_device *) data;

    Debug((PRINT_PREFIX "mvnet_harr_stop\n"));

    mvnet_harr_set_bits(dev->irq_msk, MVNET_HARR_INTR_ALL);
    writel(MVNET_HARR_INTR_STOP, dev->irq_out);
#ifndef NDEBUG
    /* DEBUG */
    mvnet_harr_dump_csr(dev, 0, 0x30);
#endif
}


static void
mvnet_harr_disconnect(void *data)
{
    struct mvnet_harr_device *dev = (struct mvnet_harr_device *) data;

    Debug((PRINT_PREFIX "mvnet_harr_disconnect\n"));

    mvnet_harr_set_bits(dev->irq_msk, MVNET_HARR_INTR_ALL);
    writel(MVNET_HARR_INTR_DISCONNECT, dev->irq_out);
#ifndef NDEBUG
    /* DEBUG */
    mvnet_harr_dump_csr(dev, 0, 0x30);
#endif
}

static void
mvnet_harr_open(void) {

    MOD_INC_USE_COUNT;

}

static void
mvnet_harr_close(void) {

    MOD_DEC_USE_COUNT;

}

struct mvnet_device_funcs mvnet_harr_functions = {
    mvnet_harr_open,
    mvnet_harr_listen,
    mvnet_harr_accept,
    mvnet_harr_connect,
    mvnet_harr_start,
    mvnet_harr_update,
    mvnet_harr_open_conn,
    mvnet_harr_send,
    mvnet_harr_close_conn,
    mvnet_harr_stop,
    mvnet_harr_disconnect,
    mvnet_harr_close
};

/*
 * Read harrier internal vector number from MPIC
 */
static int
mvnet_harr_get_irq(u32 ppc_reg_base)
{
	u32 mpic_bar;

	mpic_bar = le32_to_cpu(readl(ppc_reg_base + HARRIER_MBAR_OFF)) &
			   HARRIER_MBAR_MSK;

	return le32_to_cpu(readl(mpic_bar + HARRIER_MPIC_IFEVP_OFF)) &
	       	       HARRIER_MPIC_IFEVP_VECT_MSK;
}

static int
mvnet_harr_validate_configuration(struct pci_dev *pdev)
{
    int result;
    u32 setup;
    /*
     *  Make sure the harrier configuration registers are set correctly
     */

    result = pci_read_config_dword(pdev, HARRIER_ITSZ0_OFF, &setup);
    if ( (result != PCIBIOS_SUCCESSFUL)
	|| ((setup & HARRIER_ITSZ_MSK) != HARRIER_ITSZ_1MB) ) {
	mvnet_harr_print_error("Inbound translation 0 configuration invalid.",
				pdev);
	return 0;
    }

    result = pci_read_config_dword(pdev, HARRIER_MPAT_OFF, &setup);
    if ( (result != PCIBIOS_SUCCESSFUL)
	|| ((setup & HARRIER_ITAT_ENA) != HARRIER_ITAT_ENA) ) {
	mvnet_harr_print_error("MPAT configuration invalid.", pdev);
	return 0;
    }
    return 1;
}


static int
mvnet_harr_check_interface(struct pci_dev *pdev)
{
    int result;
    u32 lba;

    Debug((PRINT_PREFIX "mvnet_harr_check_interface\n"));

    /*
     * LBA bit is set if target Harrier == initiating Harrier
     * (i.e. if we are reading our own PCI header).
     */
    result = pci_read_config_dword(pdev, HARRIER_LBA_OFF, &lba);
    if (result != PCIBIOS_SUCCESSFUL) {
	Debug((PRINT_PREFIX "Error reading PCI configuration\n"));
	return -EIO;
    }

    /*
     * Return 1 for primary interface (PCI side), 0 for secondary interface
     * (PowerPC side)
     */
    return (lba & HARRIER_LBA_MSK) ? 0 : 1;
}

/* Called by master to setup the PCI side of a Harrier */
static int
mvnet_harr_setup_pci(struct pci_dev *pdev, 
		   struct mvnet_harr_device *dev)
{
	/*
	 *  Harrier must be able to respond to and generate memory
	 *  accesses.
	 */
	if (pci_enable_device(pdev) < 0) {
		mvnet_harr_print_error("Cannot enable device.", pdev);
		return 0;
	}

	/*
	 *  Map Message Passing registers
	 */
	Debug((PRINT_PREFIX "csr res = %08lx\n",
				pci_resource_start(pdev, 0)));
	if (!request_mem_region(pci_resource_start(pdev, 0),
				pci_resource_len(pdev, 0), HARR_NAME)) {
		mvnet_harr_print_error("Cannot reserve PMEP region", pdev);
		return 0;
	}

#ifdef CONFIG_PPC
	dev->csr_paddr = pci_resource_to_bus(pdev, &pdev->resource[0]);
#else
	dev->csr_paddr = pci_resource_start(pdev, 0);
#endif
	dev->csr_size  = pci_resource_len(pdev, 0);

	dev->csr =
		(unsigned long) ioremap_nocache(pci_resource_start(pdev, 0),
						pci_resource_len(pdev, 0));

	if (dev->csr == (u32)NULL) {
		mvnet_harr_print_error("Cannot map PMEP for device", pdev);
		goto err_out_release_csr;
	}

	/*
	 *  Map shared memory buffer
	 */
	if (!request_mem_region(pci_resource_start(pdev, 1),
				pci_resource_len(pdev, 1), HARR_NAME))
	{
		mvnet_harr_print_error("Cannot reserve shared memory buffer",
					pdev);
		goto err_out_unmap_csr;
	}

#ifdef CONFIG_PPC
	Debug((PRINT_PREFIX "mem res = %08lx\n",
	      pci_resource_start(pdev, 1)));
	dev->mem_paddr = pci_resource_to_bus(pdev, &pdev->resource[1]);
#else
	dev->mem_paddr = pci_resource_start(pdev, 1);
#endif

	Debug((PRINT_PREFIX "mem_paddr = %08lx\n", dev->mem_paddr));

#ifdef CONFIG_MVNET_STORE_GATHERING
	dev->mem = __ioremap(pci_resource_start(pdev, 1),
			    pci_resource_len(pdev, 1), _PAGE_NO_CACHE);
#else
	dev->mem = ioremap(pci_resource_start(pdev, 1),
			    pci_resource_len(pdev, 1));
#endif


	if (dev->mem == NULL) {
		mvnet_harr_print_error("Cannot map memory buffer for device",
					pdev);
		goto err_out_release_mmio;
	}

	Debug((PRINT_PREFIX "mem virt = %08lx\n", (unsigned long) dev->mem));

	/* Set IRQ offsets */
	dev->irq_in = (u32*)HARRIER_MP_PMEP(HARRIER_MGOD_OFF);
	dev->irq_msk = (u32*)HARRIER_MP_PMEP(HARRIER_PMEP_MGODM_OFF);
	dev->irq_out = (u32*)HARRIER_MP_PMEP(HARRIER_MGID_OFF);

	dev->irq = pdev->irq;

	dev->irq_handler = mvnet_harr_intr_primary;

	pci_set_master(pdev);

	mvnet_find_root_resources(pdev, &pdev->resource[1], &dev->rtmemstart,
			/* compensate for possible PReP map */
				  &dev->rtmemend);
	return 1;

err_out_release_mmio:
	release_mem_region(pci_resource_start(pdev, 1),
			pci_resource_len(pdev, 1));

err_out_unmap_csr:
	iounmap((void *)dev->csr);

err_out_release_csr:
	release_mem_region(pci_resource_start(pdev, 0),
			pci_resource_len(pdev, 0));

	return 0;
}

static void
mvnet_harr_release_pci(struct mvnet_harr_device *dev)
{
	struct pci_dev *pdev = dev->pdev;

	iounmap(dev->mem);

	release_mem_region(pci_resource_start(pdev, 1),
			pci_resource_len(pdev, 1));

	iounmap((void *)dev->csr);

	release_mem_region(pci_resource_start(pdev, 0),
			pci_resource_len(pdev, 0));
}

/* Called by target to setup the PPC side of its Harrier */
static int
mvnet_harr_setup_ppc(struct pci_dev *pdev, 
		     struct mvnet_harr_device *dev)
{

	/*
	 *  Map Message Passing registers
	 */
	dev->csr_paddr = HARRIER_DEFAULT_XCSR_BASE;
	dev->csr_size = HARRIER_XCSR_SIZE;

	dev->csr = (u32)ioremap(dev->csr_paddr, dev->csr_size);

	if (dev->csr == (u32)NULL) {
		mvnet_harr_print_error("Cannot map PMEP for device", pdev);
		return 0;
	}

	/*
	 *  Map shared memory buffer
	 */
	/* FIXME
	 * We have a CPU->PCI memory window that covers PCI devices.
	 * We need to create a CPU->PCI window that covers system memory.
	 * By default we place the window at 0xe0000000 in our physical
	 * address space and hope nobody's using that address.
	 */
	if (mvnet_harr_window_init(HARRIER_DEFAULT_XCSR_BASE, 2,
				   MVNET_HARR_MEM_SIZE,
				   HARR_SYSMEM_PPC_BASE,
				   HARR_SYSMEM_PCI_BASE)) {
		mvnet_harr_print_error("Cannot init system memory window",
				       pdev);
		goto err_out_unmap_csr;
	}

	dev->mem_paddr = HARR_SYSMEM_PPC_BASE;
	dev->mem_size = MVNET_HARR_MEM_SIZE;
	/* end FIXME */

#ifdef CONFIG_MVNET_STORE_GATHERING
	dev->mem = __ioremap(dev->mem_paddr, dev->mem_size, _PAGE_NO_CACHE);
#else
	dev->mem = ioremap(dev->mem_paddr, dev->mem_size);
#endif

	if (dev->mem == (u32)NULL) {
		mvnet_harr_print_error("Cannot map system memory window",
				       pdev);
		goto err_out_unmap_csr;
	}

	/* disable generic mailbox functions and interrupts */
	mvnet_harr_clear_bits((volatile u32 *)(dev->csr + HARRIER_FEEN_OFF),
			      HARRIER_FE_MIDB | HARRIER_FE_MIM0 |
			      HARRIER_FE_MIM1);
	mvnet_harr_set_bits((volatile u32 *)(dev->csr + HARRIER_FEMA_OFF),
			     HARRIER_FE_MIDB | HARRIER_FE_MIM0 |
			     HARRIER_FE_MIM1);

	dev->irq = mvnet_harr_get_irq(HARRIER_DEFAULT_XCSR_BASE);

	/* Set IRQ offsets */
	dev->irq_in = (u32*)HARRIER_MP_XCSR(HARRIER_MGID_OFF);
	dev->irq_msk = (u32*)HARRIER_MP_XCSR(HARRIER_MGIDM_OFF);
	dev->irq_out = (u32*)HARRIER_MP_XCSR(HARRIER_MGOD_OFF);

	dev->irq_handler = mvnet_harr_intr_secondary;

	return 1;

err_out_unmap_csr:
	iounmap((void*)dev->csr);

	return 0;
}

static void
mvnet_harr_release_ppc(struct mvnet_harr_device *dev)
{

	/* Close the outbound window to system memory */
	(void) mvnet_harr_window_init(HARRIER_DEFAULT_XCSR_BASE, 2, 0, 0, 0);

	iounmap(dev->mem);

	iounmap((void*)dev->csr);
}

static struct mvnet_harr_device *
mvnet_harr_open_dev(struct pci_dev *pdev)
{
    int result, primary;
    struct mvnet_harr_device *dev;

    Debug((PRINT_PREFIX "mvnet_harr_open_dev\n"));

    /*
     *  Make sure the harrier configuration registers are set correctly
     */
    if (!mvnet_harr_validate_configuration(pdev))
	    goto err_out;

    /*
     *  Determine whether connected to primary or secondary interface
     */
    primary = mvnet_harr_check_interface(pdev);

    if (primary < 0)
	goto err_out;
    Debug((PRINT_PREFIX "%s interface\n", primary ? "primary" : "secondary"));

    /*
     *  Allocate and initialize Harrier device information
     */
    dev = kmalloc(sizeof(struct mvnet_harr_device), GFP_KERNEL);
    if (dev == NULL) {
	mvnet_harr_print_error("Cannot allocate memory for device", pdev);
	goto err_out;
    }
    memset(dev, 0, sizeof(struct mvnet_harr_device));

    dev->primary = primary;
    dev->pdev = pdev;

    if (primary) {

	    if (!mvnet_harr_setup_pci(pdev, dev))
		    goto err_out_free_dev;

    }
    else {
	    if (!mvnet_harr_setup_ppc(pdev, dev))
		    goto err_out_free_dev;
    }

    Debug((PRINT_PREFIX "csr, irq in, msk, out = 0x%08lx, 0x%08x, 0x%08x, 0x%08x\n",
	    dev->csr, (u32)dev->irq_in, (u32)dev->irq_msk, (u32)dev->irq_out));

    /* Complete IRQ addresses */
    dev->irq_in = (u32*)(dev->csr + (u32)dev->irq_in);
    dev->irq_msk = (u32*)(dev->csr + (u32)dev->irq_msk);
    dev->irq_out = (u32*)(dev->csr + (u32)dev->irq_out);

    Debug((PRINT_PREFIX "csr, irq in, msk, out = 0x%08lx, 0x%08x, 0x%08x, 0x%08x\n",
	    dev->csr, (u32)dev->irq_in, (u32)dev->irq_msk, (u32)dev->irq_out));

    /*
     * Link pdev struct to dev struct for later.
     */

    pci_set_drvdata(pdev, dev);

    /*
     *  Mask all incoming interrupts
     */
    mvnet_harr_set_bits(dev->irq_msk, MVNET_HARR_INTR_ALL);

    Debug((PRINT_PREFIX "requesting IRQ %d\n", dev->irq));

    result = request_irq(dev->irq, dev->irq_handler,
		         SA_SAMPLE_RANDOM | SA_SHIRQ, "mvnet_harr", dev);

    if (result == 0) {
	    if (!primary) {
		    /* enable and un-mask the doorbell interrupt source */
		    mvnet_harr_set_bits((volatile u32 *)(dev->csr +
					 HARRIER_FEEN_OFF), HARRIER_FE_MIDB);
		    mvnet_harr_clear_bits((volatile u32 *)(dev->csr +
					   HARRIER_FEMA_OFF), HARRIER_FE_MIDB);
	    }
	    return dev;
    }
    else {
	    if (primary)
		    mvnet_harr_release_pci(dev);
	    else
		    mvnet_harr_release_ppc(dev);
    }

    printk(PRINT_PREFIX "Cannot get IRQ. IRQ number = %d\n", dev->irq);

    pci_set_drvdata(pdev, NULL);

err_out_free_dev:
    kfree(dev);

err_out:
    return NULL;
}

static void
mvnet_harr_close_dev(struct mvnet_harr_device *dev)
{
    unsigned long flags;

    Debug((PRINT_PREFIX "mvnet_harr_close_dev\n"));

    /* Disable doorbell interrupts */
    mvnet_harr_set_bits(dev->irq_msk, MVNET_HARR_INTR_ALL);

    spin_lock_irqsave(&event_lock, flags);
    if (dev->events != 0) {
	struct mvnet_harr_device **eqptr = &event_queue;

	while (*eqptr != NULL) {
	    if (*eqptr == dev) {
		*eqptr = dev->next_event;
		break;
	    }
	    eqptr = &((*eqptr)->next_event);
	}
	dev->events = 0;
    }
    spin_unlock_irqrestore(&event_lock, flags);

    free_irq(dev->irq, dev);

    if (dev->primary)
	    mvnet_harr_release_pci(dev);
    else
	    mvnet_harr_release_ppc(dev);

    pci_set_drvdata(dev->pdev, NULL);

    kfree(dev);

}

static int
mvnet_harr_init_one(struct pci_dev *pdev,
		     const struct pci_device_id *ent)
{
    struct mvnet_harr_device *dev;
    int result = -ENODEV;

    Debug((PRINT_PREFIX "mvnet_harr_init_one\n"));

    if ((pdev->bus->number == 0) && (pdev->devfn == PCI_DEVFN(0,0)))
	    return result;

    dev = mvnet_harr_open_dev(pdev);
    if (dev != NULL) {
	    dev->netfn = NULL;
	    result = mvnet_add_device(&mvnet_harr_functions, dev, 
			    		&dev->netdev, dev->primary,
					dev->rtmemstart, dev->rtmemend);
	    if (result)
		    mvnet_harr_close_dev(dev);
    }

    return result;
}

static void
mvnet_harr_remove_one(struct pci_dev *pdev) {

    struct mvnet_harr_device *dev = pci_get_drvdata(pdev);

    Debug((PRINT_PREFIX "mvnet_harr_remove_one\n"));

    if (dev->netdev != NULL)
	mvnet_remove_device(dev->netdev);

    mvnet_harr_close_dev(dev);
}

static struct pci_device_id mvnet_harr_tbl[] __devinitdata = {
	{ PCI_VENDOR_ID_MOTOROLA, PCI_DEVICE_ID_MOTOROLA_HARRIER,
		PCI_ANY_ID, PCI_ANY_ID, },
	{ 0,}
};

static struct pci_driver mvnet_harr_driver = {
	name:		"mvnet_harr",
	id_table:	mvnet_harr_tbl,
	probe:		mvnet_harr_init_one,
	remove:		mvnet_harr_remove_one,
};

static int
mvnet_harr_notify_shutdown( struct notifier_block *, unsigned long, void *);
static struct notifier_block mvnet_harr_reboot_notifier = {
    mvnet_harr_notify_shutdown, NULL, 0
};

static int
mvnet_harr_notify_shutdown(
    struct notifier_block *self,
    unsigned long ldata,
    void *vdata
)
{
    Debug((PRINT_PREFIX "mvnet_harr_notify_shutdown\n"));
    pci_unregister_driver(&mvnet_harr_driver);
    unregister_reboot_notifier(&mvnet_harr_reboot_notifier);
    return NOTIFY_OK;
}

int
__init mvnet_harr_init_module(void)
{
    int ret;
    Debug((PRINT_PREFIX "init_module\n"));

    ret = pci_module_init(&mvnet_harr_driver);

    if (ret >= 0)
        register_reboot_notifier(&mvnet_harr_reboot_notifier);
    return ret;
}

void
__exit mvnet_harr_cleanup_module(void)
{
    Debug((PRINT_PREFIX "cleanup_module\n"));

    unregister_reboot_notifier(&mvnet_harr_reboot_notifier);
    pci_unregister_driver(&mvnet_harr_driver);

}

module_init(mvnet_harr_init_module);
module_exit(mvnet_harr_cleanup_module);
