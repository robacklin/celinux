/*
 *
 *  mvnet/driver/mvnet_drawb.c, version 2.0
 *
 *  Copyright 2000-2003, MontaVista Software, Inc.
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

#include <linux/reboot.h>

#include "mvnet_drawb.h"
#include "mvnet.h"

MODULE_AUTHOR("MontaVista Software <source@mvista.com>");
MODULE_DESCRIPTION("MontaVista Net Backpanel Driver (21554 driver)");
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

#define MVNET_NAME	"mvnet_drawb"
#define PRINT_PREFIX	"mvnet_drawb:  "

#define NOT_BREAK	1


/* The device id for the 21554 (Drawbridge) is not in the PCI headers yet */
#ifndef PCI_DEVICE_ID_DEC_21554
#define PCI_DEVICE_ID_DEC_21554	0x46
#endif /* PCI_DEVICE_ID_DEC_21554 */

#ifndef PCI_DEVICE_ID_INTEL21555
#define PCI_DEVICE_ID_INTEL_21555	0xb555
#endif /* PCI_DEVICE_ID_INTEL_21555 */

/* Drawbridge configuration space register addresses */
#define DRAWB_CFG_ADDR_UCA		0x88
#define DRAWB_CFG_SETUP_DS2		0xB4
#define DRAWB_CFG_CHIP_CONTROL_1	0xCE

/* Use 1MB mappings for memory accesses */
#define MVNET_DRAWB_MEM_SIZE		(1024 * 1024)
#define MVNET_DRAWB_MEM_MASK		(~(MVNET_DRAWB_MEM_SIZE - 1))
#define DRAWB_CC1_PAGE_SIZE_MASK	0x0F00
#define DRAWB_CC1_PAGE_SIZE_1MB		0x0D00
#define DRAWB_LUT_VALID			1

/* Secondary->Primary interrupts */
#define MVNET_DRAWB_INTR_ACCEPT		(1<<0)
#define MVNET_DRAWB_INTR_START		(1<<1)
#define MVNET_DRAWB_INTR_STOP		(1<<2)

/* Primary->Secondary interrupts */
#define MVNET_DRAWB_INTR_CONNECT	(1<<0)
#define MVNET_DRAWB_INTR_UPDATE		(1<<1)
#define MVNET_DRAWB_INTR_DISCONNECT	(1<<2)

/* Common interrupts */
#define MVNET_DRAWB_INTR_RECV		(1<<3)
#define MVNET_DRAWB_INTR_ALL		(MVNET_DRAWB_INTR_ACCEPT | \
					 MVNET_DRAWB_INTR_START	 | \
					 MVNET_DRAWB_INTR_STOP   | \
					 MVNET_DRAWB_INTR_RECV)

/* Events */
#define MVNET_DRAWB_EVENT_ACCEPT	(1<<0)
#define MVNET_DRAWB_EVENT_CONNECT	(1<<1)
#define MVNET_DRAWB_EVENT_START		(1<<2)
#define MVNET_DRAWB_EVENT_UPDATE	(1<<3)
#define MVNET_DRAWB_EVENT_RECV		(1<<4)
#define MVNET_DRAWB_EVENT_STOP		(1<<5)
#define MVNET_DRAWB_EVENT_DISCONNECT	(1<<6)

/* The memory-mapped Control and Status Registers */
struct drawb_csr {
    u8 resv1[0x68];

    volatile u32 xlat_base_ds0;
    volatile u32 xlat_base_ds1;
    volatile u32 xlat_base_ds2;
    volatile u32 xlat_base_ds3;
    volatile u32 xlat_base_us0;
    volatile u32 xlat_base_us1;

    u8 resv2[0x18];

    volatile u16 p_clr_irq;
    volatile u16 s_clr_irq;
    volatile u16 p_set_irq;
    volatile u16 s_set_irq;
    volatile u16 p_clr_irqm;
    volatile u16 s_clr_irqm;
    volatile u16 p_set_irqm;
    volatile u16 s_set_irqm;

    volatile u32 scratchpad0;
    volatile u32 scratchpad1;
    volatile u32 scratchpad2;
    volatile u32 scratchpad3;
    volatile u32 scratchpad4;
    volatile u32 scratchpad5;
    volatile u32 scratchpad6;
    volatile u32 scratchpad7;

    u8 resv3[0x38];

    volatile u32 m2lut[64];
};
#define DRAWB_CSR_SIZE	4096

/* Per-device information */
struct mvnet_drawb_device {
    /* Control and Status Registers */
    struct drawb_csr *csr;
    unsigned long csr_paddr;

    /* Mapped memory */
    void *mem;
    unsigned long mem_paddr;
    unsigned long buf_paddr;
    int		  bar;

    int primary;

    unsigned int irq;
    volatile u16 *irq_out;
    volatile u16 *irq_in;
    volatile u16 *irq_clrm;
    volatile u16 *irq_setm;

    unsigned short events;
    struct mvnet_drawb_device *next_event;

    struct mvnet_device *netdev;
    struct mvnet_funcs *netfn;
    struct pci_dev *pdev;
    u32 rtmemstart;
    u32 rtmemend;
};

static union {
    char string[4];
    u32 integer;
} mvnet_drawb_signature = { { 'N', 'E', 'T', /* version */ 2 } };


/* DEBUG */
#ifndef NDEBUG
static void
mvnet_drawb_dump_csr(struct mvnet_drawb_device *dev,
		     unsigned long start, unsigned long end)
{
	printk(PRINT_PREFIX "mvnet_drawb_dump_csr\n");

	start += (unsigned long) dev->csr;
	end   += (unsigned long) dev->csr;

	for (; start < end; start++)
	{
		if (!((start - end) % 8))
				printk("\n0x%08lx ", start);
		printk("%2.2x ", *(unsigned char *) start);
			      
	}
	printk("\n");

}
#endif


/*
 *  Interrupt/event handling
 */


static struct mvnet_drawb_device *event_queue = NULL;
static spinlock_t event_lock = SPIN_LOCK_UNLOCKED;

#ifdef USE_TASKLET
static void
mvnet_drawb_handle_events(unsigned long data)
#else
static void
mvnet_drawb_handle_events(void *data)
#endif
{
    struct mvnet_drawb_device *dev = NULL;
    unsigned long flags;
    unsigned short events = 0;
    struct mvnet_event evnt;

    Debug((PRINT_PREFIX "mvnet_drawb_handle_events\n"));

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
	if (events & MVNET_DRAWB_EVENT_ACCEPT) {
	    u32 signature = readl(&dev->csr->scratchpad0);

	    Debug((PRINT_PREFIX "MVNET_DRAWB_EVENT_ACCEPT\n"));

	    writew(MVNET_DRAWB_INTR_ACCEPT, dev->irq_in);

	    if (signature == be32_to_cpu(mvnet_drawb_signature.integer)) {
		u32 offset = readl(&dev->csr->scratchpad1);
		void *vaddr = (void *) (((u8 *) dev->mem) + offset);

		dev->buf_paddr = dev->mem_paddr + offset;
		evnt.event = accept;
		evnt.p1 = dev->netdev;
		evnt.p2 = vaddr;
		evnt.p3 = (void *)dev->csr_paddr;
		evnt.p4 = (void *)dev->buf_paddr;

		if (mvnet_proc_event(&evnt) != 0)
		{
		    writew(MVNET_DRAWB_INTR_ACCEPT, dev->irq_clrm);
		}
	    } else {
		printk(PRINT_PREFIX "invalid signature 0x%X\n", signature);
		writew(MVNET_DRAWB_INTR_ACCEPT, dev->irq_clrm);
	    }

	    events &= ~MVNET_DRAWB_EVENT_ACCEPT;
	}

	/*
	 * "Connect" events received by target only.
	 */
	if (events & MVNET_DRAWB_EVENT_CONNECT) {
	    u32 sysbuf = readl(&dev->csr->scratchpad2);
	    u32 base = sysbuf & MVNET_DRAWB_MEM_MASK;
	    u32 offset = sysbuf & ~MVNET_DRAWB_MEM_MASK;
	    void *sysmem = (void *) (((u8 *) dev->mem) + offset);
	    u32 devnum = readl(&dev->csr->scratchpad3);
	    struct irq_spec irq;

	    Debug((PRINT_PREFIX "MVNET_DRAWB_EVENT_CONNECT\n"));

	    writew(MVNET_DRAWB_INTR_CONNECT, dev->irq_in);

	    dev->buf_paddr = sysbuf;
	    writel(base | DRAWB_LUT_VALID, &dev->csr->m2lut[0]);

	
	    irq.addr = (u32)&((struct drawb_csr *)0)->s_set_irq;
	    irq.val = (u32)MVNET_DRAWB_INTR_RECV;
	    irq.len = sizeof(u16);

	    evnt.event = connect;
	    evnt.p1 = dev->netdev;
	    evnt.p2 = sysmem;
	    evnt.p3 = (void *)devnum;
	    evnt.p4 = (void *)&irq;

	    if (mvnet_proc_event(&evnt) != 0)
		writew(MVNET_DRAWB_INTR_CONNECT, dev->irq_clrm);

	    events &= ~MVNET_DRAWB_EVENT_CONNECT;
	}

	/*
	 * "Start" events received by master only.
	 */
	if (events & MVNET_DRAWB_EVENT_START) {
	    Debug((PRINT_PREFIX "MVNET_DRAWB_EVENT_START\n"));

	    writew(MVNET_DRAWB_INTR_START, dev->irq_in);

	    evnt.event = start;
	    evnt.p1 = dev->netdev;

	    if (mvnet_proc_event(&evnt) == 0) {
		writew(MVNET_DRAWB_INTR_STOP | MVNET_DRAWB_INTR_RECV,
		       dev->irq_in);
		writew(MVNET_DRAWB_INTR_STOP | MVNET_DRAWB_INTR_RECV,
		       dev->irq_clrm);
	    } else {
		writew(MVNET_DRAWB_INTR_START, dev->irq_clrm);
	    }

	    events &= ~MVNET_DRAWB_EVENT_START;
	}

	/*
	 * "Update" events received by target only.
	 */
	if (events & MVNET_DRAWB_EVENT_UPDATE) {
	    Debug((PRINT_PREFIX "MVNET_DRAWB_EVENT_UPDATE\n"));

	    writew(MVNET_DRAWB_INTR_UPDATE, dev->irq_in);

	    evnt.event = update;
	    evnt.p1 = dev->netdev;

	    (void) mvnet_proc_event(&evnt);

	    writew(MVNET_DRAWB_INTR_UPDATE, dev->irq_clrm);

	    events &= ~MVNET_DRAWB_EVENT_UPDATE;
	}

	/*
	 * "Receive" events received by both master and target.
	 */
	if (events & MVNET_DRAWB_EVENT_RECV) {
	    Debug((PRINT_PREFIX "MVNET_DRAWB_EVENT_RECV\n"));

	    writew(MVNET_DRAWB_INTR_RECV, dev->irq_in);

	    evnt.event = receive;
	    evnt.p1 = dev->netdev;

	    (void) mvnet_proc_event(&evnt);

	    writew(MVNET_DRAWB_INTR_RECV, dev->irq_clrm);

	    events &= ~MVNET_DRAWB_EVENT_RECV;
	}

	/*
	 * "Stop" events received by master only.
	 */
	if (events & MVNET_DRAWB_EVENT_STOP) {
	    Debug((PRINT_PREFIX "MVNET_DRAWB_EVENT_STOP\n"));

	    writew(MVNET_DRAWB_INTR_STOP, dev->irq_in);
	    writew(MVNET_DRAWB_INTR_STOP, dev->irq_setm);

	    evnt.event = stop;
	    evnt.p1 = dev->netdev;

	    (void) mvnet_proc_event(&evnt);

	    events &= ~MVNET_DRAWB_EVENT_STOP;
	}

	/*
	 * "Disconnect" events received by target only.
	 */
	if (events & MVNET_DRAWB_EVENT_DISCONNECT) {
	    Debug((PRINT_PREFIX "MVNET_DRAWB_EVENT_DISCONNECT\n"));

	    writew(MVNET_DRAWB_INTR_DISCONNECT, dev->irq_in);
	    writew(MVNET_DRAWB_INTR_ALL, dev->irq_setm);

	    evnt.event = disconnect;
	    evnt.p1 = dev->netdev;

	    (void) mvnet_proc_event(&evnt);

	    events &= ~MVNET_DRAWB_EVENT_DISCONNECT;
	}
    }
}

#ifdef USE_TASKLET
static DECLARE_TASKLET(mvnet_drawb_event_tasklet, mvnet_drawb_handle_events, 0);

#else
static struct tq_struct mvnet_drawb_event_task = {
    routine:  mvnet_drawb_handle_events,
};
#endif

static void
mvnet_drawb_intr(
    struct mvnet_drawb_device *dev,
    unsigned short intrs,
    unsigned short intr,
    unsigned int event
)
{
    unsigned long flags;

    if (intrs & intr) {
	Debug((PRINT_PREFIX "interrupt %d\n", intr));

	writew(intr, dev->irq_setm);

	spin_lock_irqsave(&event_lock, flags);
	if (dev->events == 0) {
	    dev->next_event = event_queue;
	    event_queue = dev;
	}
	dev->events |= event;
	spin_unlock_irqrestore(&event_lock, flags);

#ifdef USE_TASKLET
	tasklet_hi_schedule(&mvnet_drawb_event_tasklet);
#else
#ifdef USE_SCHEDULE_TASK
	schedule_task(&mvnet_drawb_event_task);
#else
	queue_task(&mvnet_drawb_event_task, &tq_immediate);
	mark_bh(IMMEDIATE_BH);
#endif
#endif
    }
}


static void
mvnet_drawb_intr_primary(int irq, void *dev_id, struct pt_regs *regs)
{
    struct mvnet_drawb_device *dev = (struct mvnet_drawb_device *) dev_id;
    u16 intrs = (readw(dev->irq_in) & ~(readw(dev->irq_setm)));

    Debug((PRINT_PREFIX "mvnet_drawb_intr_primary\n"));

#ifndef NDEBUG
    /* DEBUG */
    mvnet_drawb_dump_csr(dev, 0x98, 0xa8);
#endif
    mvnet_drawb_intr(dev,intrs,MVNET_DRAWB_INTR_ACCEPT,
	MVNET_DRAWB_EVENT_ACCEPT);
    mvnet_drawb_intr(dev,intrs,MVNET_DRAWB_INTR_START,MVNET_DRAWB_EVENT_START);
    mvnet_drawb_intr(dev,intrs,MVNET_DRAWB_INTR_RECV,MVNET_DRAWB_EVENT_RECV);
    mvnet_drawb_intr(dev,intrs,MVNET_DRAWB_INTR_STOP,MVNET_DRAWB_EVENT_STOP);
}


static void
mvnet_drawb_intr_secondary(int irq, void *dev_id, struct pt_regs *regs)
{
    struct mvnet_drawb_device *dev = (struct mvnet_drawb_device *) dev_id;
    u16 intrs = (readw(dev->irq_in) & ~(readw(dev->irq_setm)));

    Debug((PRINT_PREFIX "mvnet_drawb_intr_secondary\n"));

#ifndef NDEBUG
    /* DEBUG */
    mvnet_drawb_dump_csr(dev, 0x98, 0xa8);
#endif
    mvnet_drawb_intr(dev,intrs,MVNET_DRAWB_INTR_CONNECT,
	MVNET_DRAWB_EVENT_CONNECT);
    mvnet_drawb_intr(dev,intrs,MVNET_DRAWB_INTR_UPDATE,
	MVNET_DRAWB_EVENT_UPDATE);
    mvnet_drawb_intr(dev,intrs,MVNET_DRAWB_INTR_RECV,MVNET_DRAWB_EVENT_RECV);
    mvnet_drawb_intr(dev,intrs,MVNET_DRAWB_INTR_DISCONNECT,
	MVNET_DRAWB_EVENT_DISCONNECT);
}

static void
mvnet_drawb_send_8(struct irq_spec *irq)
{
    Debug((PRINT_PREFIX "mvnet_harr_send_8: %08x, %02x\n", irq->addr,
	  (u8)irq->val));
    writeb(irq->val, irq->addr);
}

static void
mvnet_drawb_send_16(struct irq_spec *irq)
{
    Debug((PRINT_PREFIX "mvnet_harr_send_16: %08x, %04x\n", irq->addr,
          (u16)irq->val));
    writew(irq->val, irq->addr);
}

static void
mvnet_drawb_send_32(struct irq_spec *irq)
{
    Debug((PRINT_PREFIX "mvnet_harr_send_32: %08x, %08x\n", irq->addr,
          (u32)irq->val));
    writel(irq->val, irq->addr);
};

/*
 *  mvnet_drawb_functions
 */


/*
 * Called by master only. Un-masks (enables) the "accept" interrupt to come in
 * from the target.
 */
static int
mvnet_drawb_listen(void *data)
{
    struct mvnet_drawb_device *dev = (struct mvnet_drawb_device *) data;

    Debug((PRINT_PREFIX "mvnet_drawb_listen\n"));
    assert(dev->primary);

    writew(MVNET_DRAWB_INTR_ALL & ~MVNET_DRAWB_INTR_ACCEPT, dev->irq_in);
    writew(MVNET_DRAWB_INTR_ACCEPT, dev->irq_clrm);

#ifndef NDEBUG
    /* DEBUG */
    mvnet_drawb_dump_csr(dev, 0x98, 0xa8);
#endif
    return 0;
}


/*
 * Called by master only. Sends "connect" interrupt to target and passes
 * target's device number and address of buffer space in masters RAM.
 */
static int
mvnet_drawb_accept(void *data, unsigned long sysbuf, int devnum)
{
    struct mvnet_drawb_device *dev = (struct mvnet_drawb_device *) data;

    Debug((PRINT_PREFIX "mvnet_drawb_accept\n"));

    writel(sysbuf, &dev->csr->scratchpad2);
    writel(devnum, &dev->csr->scratchpad3);

    writew(MVNET_DRAWB_INTR_CONNECT, dev->irq_out);
    writew(MVNET_DRAWB_INTR_START, dev->irq_clrm);

#ifndef NDEBUG
    /* DEBUG */
    mvnet_drawb_dump_csr(dev, 0x98, 0xa8);
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
mvnet_drawb_connect(void *data, void *buf)
{
    struct mvnet_drawb_device *dev = (struct mvnet_drawb_device *) data;
    u32 busaddr = virt_to_bus(buf);

    Debug((PRINT_PREFIX "mvnet_drawb_connect\n"));

    writel(busaddr, &dev->csr->xlat_base_ds2);
    writel(be32_to_cpu(mvnet_drawb_signature.integer), &dev->csr->scratchpad0);
    writel(busaddr & ~MVNET_DRAWB_MEM_MASK, &dev->csr->scratchpad1);

    /* clear all but connect interrupt. */
    writew(MVNET_DRAWB_INTR_ALL & ~MVNET_DRAWB_INTR_CONNECT, dev->irq_in);

    /*
     * if a connect interrupt is not pending, generate an outboud accept
     * to get the ball rolling.
     */
    if ((readw(dev->irq_in) & MVNET_DRAWB_INTR_CONNECT) == 0)
	    writew(MVNET_DRAWB_INTR_ACCEPT, dev->irq_out);

    writew(MVNET_DRAWB_INTR_CONNECT, dev->irq_clrm);

#ifndef NDEBUG
    /* DEBUG */
    mvnet_drawb_dump_csr(dev, 0x98, 0xa8);
#endif
    return 0;
}

/*
 * Called by target only. Sends "start" interrupt to master indicating that
 * target is up and ready to receive packets.
 */
static int
mvnet_drawb_start(void *data, unsigned long rtmemstart, unsigned long rtmemend)
{
    struct mvnet_drawb_device *dev = (struct mvnet_drawb_device *) data;

    Debug((PRINT_PREFIX "mvnet_drawb_start\n"));

    writew(MVNET_DRAWB_INTR_UPDATE | MVNET_DRAWB_INTR_RECV |
	   MVNET_DRAWB_INTR_DISCONNECT, dev->irq_in);
    writew(MVNET_DRAWB_INTR_UPDATE | MVNET_DRAWB_INTR_RECV |
	   MVNET_DRAWB_INTR_DISCONNECT, dev->irq_clrm);
    writew(MVNET_DRAWB_INTR_START, dev->irq_out);

#ifndef NDEBUG
    /* DEBUG */
    mvnet_drawb_dump_csr(dev, 0x98, 0xa8);
#endif

    return 0;
}


/* Called by master only. Alerts target to a change in system data by 
 * sending an "update" interrupt to the target.
 */
static int
mvnet_drawb_update(void *data)
{
    struct mvnet_drawb_device *dev = (struct mvnet_drawb_device *) data;

    Debug((PRINT_PREFIX "mvnet_drawb_update\n"));

    writew(MVNET_DRAWB_INTR_UPDATE, dev->irq_out);

#ifndef NDEBUG
    /* DEBUG */
    mvnet_drawb_dump_csr(dev, 0x98, 0xa8);
#endif
    return 0;
}

struct mvnet_device_conn {
	struct mvnet_drawb_device *dev;
	struct drawb_csr *csr;
	u8 *mem;
	void (*irq_send)(struct irq_spec *irq);
	struct irq_spec irq;
	int num;
	int indirect;
};

static void *
mvnet_drawb_open_conn(
    void *data,
    int num,
    int indirect,
    volatile struct nodeinfo *node,
    void **devconn
)
{
    struct mvnet_drawb_device *dev = (struct mvnet_drawb_device *) data;
    struct mvnet_device_conn *conn;
    u32 csr_paddr = readl(&node->csr_paddr);
    u32	mem_paddr = readl(&node->mem_paddr);
    unsigned long base;
    void *membase;

    Debug((PRINT_PREFIX "mvnet_drawb_open_conn\n"));

    conn = kmalloc(sizeof(struct mvnet_device_conn), GFP_ATOMIC);
    if (conn == NULL) {
	printk(PRINT_PREFIX "Cannot allocate memory for device connection\n");
	return NULL;
    }
    memset(conn, 0, sizeof(struct mvnet_device_conn));

    if (indirect) {
	assert(dev->primary == 0);

	conn->csr = dev->mem + ((num * 2 + 1) * MVNET_DRAWB_MEM_SIZE)
	    + (csr_paddr & ~MVNET_DRAWB_MEM_MASK);

	base = csr_paddr & MVNET_DRAWB_MEM_MASK;
	writel(base | DRAWB_LUT_VALID, &dev->csr->m2lut[num * 2 + 1]);

	conn->mem = dev->mem + ((num * 2) * MVNET_DRAWB_MEM_SIZE);

	base = mem_paddr & MVNET_DRAWB_MEM_MASK;
	writel(base | DRAWB_LUT_VALID, &dev->csr->m2lut[num * 2]);
	membase = conn->mem + (mem_paddr & ~MVNET_DRAWB_MEM_MASK);

	conn->irq.addr = (u32)conn->csr + readl(&node->irq.addr);
	conn->irq.len  = readl(&node->irq.len);
	conn->irq.val  = readl(&node->irq.val);

    switch (conn->irq.len) {
	    case sizeof(u8):
		conn->irq_send = mvnet_drawb_send_8;
		break;
	    case sizeof(u16):
		conn->irq_send = mvnet_drawb_send_16;
		break;
	    case sizeof(u32):
		conn->irq_send = mvnet_drawb_send_32;
		break;
	    default:
		printk(PRINT_PREFIX "Unknown IRQ size %d\n", conn->irq.len);
		kfree(conn);
		return NULL;
    }

    } else {
	assert(mem_paddr == dev->buf_paddr);

	if (!dev->primary) {
	    assert(num == 0);
	}

	conn->irq.addr = (u32)dev->irq_out;
	conn->irq.len  = sizeof(u16);
	conn->irq.val  = MVNET_DRAWB_INTR_RECV;
	conn->irq_send = mvnet_drawb_send_16;

	membase = ((u8 *) dev->mem) + (dev->buf_paddr & ~MVNET_DRAWB_MEM_MASK);
    }


    conn->dev = dev;
    conn->num = num;
    conn->indirect = indirect;

    *devconn = (void *) conn;
    return membase;
}



static int
mvnet_drawb_send(void *devconn)
{
    struct mvnet_device_conn *conn = (struct mvnet_device_conn *) devconn;

    Debug((PRINT_PREFIX "mvnet_drawb_send\n"));

    conn->irq_send(&conn->irq);
    return 0;
}


static void
mvnet_drawb_close_conn(void *devconn)
{
    struct mvnet_device_conn *conn = (struct mvnet_device_conn *) devconn;
    struct mvnet_drawb_device *dev = conn->dev;

    Debug((PRINT_PREFIX "mvnet_drawb_close_conn\n"));

    if (conn->indirect) {
	assert(dev->primary == 0);

	writel(~DRAWB_LUT_VALID, &dev->csr->m2lut[conn->num * 2]);

	writel(~DRAWB_LUT_VALID, &dev->csr->m2lut[conn->num * 2 + 1]);
    }

    kfree(conn);
}


static void
mvnet_drawb_stop(void *data)
{
    struct mvnet_drawb_device *dev = (struct mvnet_drawb_device *) data;

    Debug((PRINT_PREFIX "mvnet_drawb_stop\n"));

    writew(MVNET_DRAWB_INTR_ALL, dev->irq_setm);
    writew(MVNET_DRAWB_INTR_STOP, dev->irq_out);
#ifndef NDEBUG
    /* DEBUG */
    mvnet_drawb_dump_csr(dev, 0x98, 0xa8);
#endif
}


static void
mvnet_drawb_disconnect(void *data)
{
    struct mvnet_drawb_device *dev = (struct mvnet_drawb_device *) data;

    Debug((PRINT_PREFIX "mvnet_drawb_disconnect\n"));

    writew(MVNET_DRAWB_INTR_ALL, dev->irq_setm);
    writew(MVNET_DRAWB_INTR_DISCONNECT, dev->irq_out);
#ifndef NDEBUG
    /* DEBUG */
    mvnet_drawb_dump_csr(dev, 0x98, 0xa8);
#endif
}

static void
mvnet_drawb_open(void) {

    MOD_INC_USE_COUNT;

}

static void
mvnet_drawb_close(void) {

    MOD_DEC_USE_COUNT;

}

struct mvnet_device_funcs mvnet_drawb_functions = {
    mvnet_drawb_open,
    mvnet_drawb_listen,
    mvnet_drawb_accept,
    mvnet_drawb_connect,
    mvnet_drawb_start,
    mvnet_drawb_update,
    mvnet_drawb_open_conn,
    mvnet_drawb_send,
    mvnet_drawb_close_conn,
    mvnet_drawb_stop,
    mvnet_drawb_disconnect,
    mvnet_drawb_close
};


/*
 *  Initialization
 */


static int
mvnet_drawb_check_interface(struct pci_dev *pdev)
{
    u32 readword = 0, readback = 0;
    int result;

    Debug((PRINT_PREFIX "mvnet_drawb_check_interface\n"));

    /*
     *  The upstream configuration address register can't be written
     *  from the primary interface.  Try to modify this register to
     *  determine which interface we are connected to.
     */

    result = pci_read_config_dword(pdev, DRAWB_CFG_ADDR_UCA, &readword);
    if (result != PCIBIOS_SUCCESSFUL) {
	Debug((PRINT_PREFIX "Error reading PCI configuration\n"));
	return -EIO;
    }

    result = pci_write_config_dword(pdev, DRAWB_CFG_ADDR_UCA, ~readword);
    if (result != PCIBIOS_SUCCESSFUL) {
	Debug((PRINT_PREFIX "Error writing PCI configuration\n"));
	return -EIO;
    }

    result = pci_read_config_dword(pdev, DRAWB_CFG_ADDR_UCA, &readback);
    if (result != PCIBIOS_SUCCESSFUL) {
	Debug((PRINT_PREFIX "Error reading back PCI configuration\n"));
	return -EIO;
    }

    /* Return 1 for primary, 0 for secondary interface */
    return (readword == readback) ? 1 : 0;
}

static void
mvnet_drawb_print_error(char *str, struct pci_dev *pdev)
{
	printk("%s %s (b:d:f = %d:%d:%d)\n", PRINT_PREFIX, str,
	       pdev->bus->number, PCI_SLOT(pdev->devfn), PCI_FUNC(pdev->devfn));
}


static struct mvnet_drawb_device *
mvnet_drawb_open_dev(struct pci_dev *pdev)
{
    int result, primary;
    u32 setup;
    u16 control;
    struct mvnet_drawb_device *dev;
    void (*irq_handler)(int irq, void *dev_id, struct pt_regs *regs);
    int bar;

    Debug((PRINT_PREFIX "mvnet_drawb_open_dev\n"));

    /*
     *  Make sure the drawbridge configuration registers are set correctly
     */

    result = pci_read_config_dword(pdev, DRAWB_CFG_SETUP_DS2, &setup);
    if ( (result != PCIBIOS_SUCCESSFUL)
	|| (setup & PCI_BASE_ADDRESS_SPACE_IO)
	|| ((setup & PCI_BASE_ADDRESS_MEM_MASK) != MVNET_DRAWB_MEM_MASK) )
    {
	mvnet_drawb_print_error("Downstream setup 2 configuration invalid.",
				pdev);
	goto err_out;
    }

    result = pci_read_config_word(pdev, DRAWB_CFG_CHIP_CONTROL_1, &control);
    if ( (result != PCIBIOS_SUCCESSFUL)
	|| ((control & DRAWB_CC1_PAGE_SIZE_MASK) != DRAWB_CC1_PAGE_SIZE_1MB) )
    {
	mvnet_drawb_print_error("Chip control 1 configuration invalid.", pdev);
	goto err_out;
    }

    /*
     *  Determine whether connected to primary or secondary interface
     */
    primary = mvnet_drawb_check_interface(pdev);
    if (primary < 0)
	goto err_out;
    Debug((PRINT_PREFIX "%s interface\n", primary ? "primary" : "secondary"));

    /*
     *  Drawbridge must be able to respond to and generate memory accesses
     */
    if (pci_enable_device(pdev) < 0)
    {
	    mvnet_drawb_print_error("Cannot enable device.", pdev);
	    goto err_out;
    }

    pci_set_master(pdev);

    /*
     *  Allocate and initialize Drawbridge device information
     */
    dev = kmalloc(sizeof(struct mvnet_drawb_device), GFP_KERNEL);
    if (dev == NULL) {
	mvnet_drawb_print_error("Cannot allocate memory for device", pdev);
	goto err_out;
    }
    memset(dev, 0, sizeof(struct mvnet_drawb_device));

    dev->primary = primary;
    dev->irq = pdev->irq;
    dev->pdev = pdev;

    /*
     *  Map Control and Status Registers
     */

    if (!request_mem_region(pci_resource_start(pdev, 0),
			    pci_resource_len(pdev, 0), MVNET_NAME))
    {
	    mvnet_drawb_print_error("Cannot reserve CSR region", pdev);
	    goto err_out_free_dev;
    }

#ifdef CONFIG_PPC
    if (primary)
    {
      Debug((PRINT_PREFIX "csr res = %08lx\n", pci_resource_start(pdev, 0)));
      dev->csr_paddr = pci_resource_to_bus(pdev, &pdev->resource[0]);
    }
    else
      dev->csr_paddr = pci_resource_start(pdev, 0);
#else
    dev->csr_paddr = pci_resource_start(pdev, 0);
#endif

    Debug((PRINT_PREFIX "csr_paddr = %08lx\n", dev->csr_paddr));
    dev->csr =
	(struct drawb_csr *) ioremap_nocache(pci_resource_start(pdev, 0),
					     DRAWB_CSR_SIZE);
    Debug((PRINT_PREFIX "csr virt = %08lx\n", (unsigned long) dev->csr));

    if (dev->csr == NULL)
    {
	mvnet_drawb_print_error("Cannot map CSR for device", pdev);
	goto err_out_release_csr;
    }

#ifndef NDEBUG
    /* DEBUG */
    mvnet_drawb_dump_csr(dev, 0x98, 0xa8);
#endif

    if (primary) {
	dev->irq_in = &dev->csr->p_clr_irq;
	dev->irq_clrm = &dev->csr->p_clr_irqm;
	dev->irq_setm = &dev->csr->p_set_irqm;
	dev->irq_out = &dev->csr->s_set_irq;

	bar = 3;
	irq_handler = mvnet_drawb_intr_primary;
    } else {
	dev->irq_in = &dev->csr->s_clr_irq;
	dev->irq_clrm = &dev->csr->s_clr_irqm;
	dev->irq_setm = &dev->csr->s_set_irqm;
	dev->irq_out = &dev->csr->p_set_irq;

	bar = 4;
	irq_handler = mvnet_drawb_intr_secondary;
    }

    /*
     *  Map shared memory buffer
     */
    if (!request_mem_region(pci_resource_start(pdev, bar),
			    pci_resource_len(pdev, bar), MVNET_NAME))
    {
	mvnet_drawb_print_error("Cannot reserve shared memory buffer", pdev);
	goto err_out_unmap_csr;
    }

#ifdef CONFIG_PPC
    if (primary)
    {
      Debug((PRINT_PREFIX "mem res = %08lx\n", pci_resource_start(pdev, bar)));
      dev->mem_paddr = pci_resource_to_bus(pdev, &pdev->resource[bar]);
    }
    else
      dev->mem_paddr = pci_resource_start(pdev, bar);
#else
    dev->mem_paddr = pci_resource_start(pdev, bar);
#endif

    Debug((PRINT_PREFIX "mem_paddr = %08lx\n", dev->mem_paddr));

    if (primary) {
#ifdef CONFIG_MVNET_STORE_GATHERING
	    dev->mem = __ioremap(pci_resource_start(pdev, bar),
			    pci_resource_len(pdev, bar), _PAGE_NO_CACHE);
#else
	    dev->mem = ioremap(pci_resource_start(pdev, bar),
			    pci_resource_len(pdev, bar));
#endif
    }
    else
#ifdef CONFIG_MVNET_STORE_GATHERING
	    dev->mem = __ioremap(pci_resource_start(pdev, bar),
			       2 * MVNET_MAX_DEVICES * MVNET_DRAWB_MEM_SIZE,
			       _PAGE_NO_CACHE);
#else
	    dev->mem = ioremap(pci_resource_start(pdev, bar),
			       2 * MVNET_MAX_DEVICES * MVNET_DRAWB_MEM_SIZE);
#endif
    Debug((PRINT_PREFIX "mem virt = %08lx\n", (unsigned long) dev->mem));

    dev->bar = bar;

    if (dev->mem == NULL) {
	mvnet_drawb_print_error("Cannot map memory buffer for device", pdev);
	goto err_out_release_mmio;
    }

    mvnet_find_root_resources(pdev, &pdev->resource[bar], &dev->rtmemstart,
		    	      &dev->rtmemend);

    /*
     * Link pdev struct to dev struct for later.
     */

    pci_set_drvdata(pdev, dev);

    /*
     *  Mask all incoming interrupts
     */
    writew(MVNET_DRAWB_INTR_ALL, dev->irq_setm);

    result = request_irq(dev->irq, irq_handler, SA_SAMPLE_RANDOM | SA_SHIRQ,
		         "mvnet_drawb", dev);

    if (result == 0)
	return dev;

    printk(PRINT_PREFIX "Cannot get IRQ. IRQ number = %d\n", dev->irq);

    pci_set_drvdata(pdev, NULL);

    iounmap(dev->mem);

err_out_release_mmio:
    release_mem_region(pci_resource_start(pdev, bar),
		       pci_resource_len(pdev, bar));

err_out_unmap_csr:
    iounmap(dev->csr);

err_out_release_csr:
    release_mem_region(pci_resource_start(pdev, 0),
		       pci_resource_len(pdev, 0));
err_out_free_dev:
    kfree(dev);

err_out:
    return NULL;
}

static void
mvnet_drawb_close_dev(struct mvnet_drawb_device *dev)
{
    unsigned long flags;

    Debug((PRINT_PREFIX "mvnet_drawb_close_dev\n"));

    writew(MVNET_DRAWB_INTR_ALL, dev->irq_setm);

    spin_lock_irqsave(&event_lock, flags);
    if (dev->events != 0) {
	struct mvnet_drawb_device **eqptr = &event_queue;

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
    iounmap(dev->mem);
    release_mem_region(pci_resource_start(dev->pdev, dev->bar),
		    pci_resource_len(dev->pdev, dev->bar));
    iounmap(dev->csr);
    release_mem_region(pci_resource_start(dev->pdev, 0),
		    pci_resource_len(dev->pdev, 0));
    kfree(dev);

}

static int
mvnet_drawb_init_one(struct pci_dev *pdev,
		     const struct pci_device_id *ent)
{
    struct mvnet_drawb_device *dev;
    int result = -ENODEV;

    Debug((PRINT_PREFIX "mvnet_drawb_init_one\n"));

    dev = mvnet_drawb_open_dev(pdev);
    if (dev != NULL) {
	    dev->netfn = NULL;
	    result = mvnet_add_device(&mvnet_drawb_functions, dev, &dev->netdev,
					dev->primary, dev->rtmemstart,
					dev->rtmemend);
	    if (result)
		    mvnet_drawb_close_dev(dev);
    }

    return result;
}

static void
mvnet_drawb_remove_one(struct pci_dev *pdev) {

    struct mvnet_drawb_device *dev = pci_get_drvdata(pdev);

    Debug((PRINT_PREFIX "mvnet_drawb_remove_one\n"));

    if (dev->netdev != NULL)
	mvnet_remove_device(dev->netdev);

    mvnet_drawb_close_dev(dev);
}

static struct pci_device_id mvnet_drawb_tbl[] __devinitdata = {
	{ PCI_VENDOR_ID_DEC, PCI_DEVICE_ID_DEC_21554,
		PCI_ANY_ID, PCI_ANY_ID, },
	{ PCI_VENDOR_ID_INTEL, PCI_DEVICE_ID_INTEL_21555,
		PCI_ANY_ID, PCI_ANY_ID, },
	{ 0,}
};

static struct pci_driver mvnet_drawb_driver = {
	name:		"mvnet_drawb",
	id_table:	mvnet_drawb_tbl,
	probe:		mvnet_drawb_init_one,
	remove:		mvnet_drawb_remove_one,
};

static int
mvnet_drawb_notify_shutdown( struct notifier_block *, unsigned long, void *);

static struct notifier_block mvnet_drawb_reboot_notifier = {
    mvnet_drawb_notify_shutdown, NULL, 0
};

static int
mvnet_drawb_notify_shutdown(
    struct notifier_block *self,
    unsigned long ldata,
    void *vdata
)
{
    Debug((PRINT_PREFIX "mvnet_drawb_notify_shutdown\n"));
    pci_unregister_driver(&mvnet_drawb_driver);
    unregister_reboot_notifier(&mvnet_drawb_reboot_notifier);
    return NOTIFY_OK;
}

int
__init mvnet_drawb_init_module(void)
{
    int ret;

    Debug((PRINT_PREFIX "init_module\n"));

    ret = pci_module_init(&mvnet_drawb_driver);

    if (ret >= 0)
        register_reboot_notifier(&mvnet_drawb_reboot_notifier);

    return ret;
}

void
__exit mvnet_drawb_cleanup_module(void)
{
    Debug((PRINT_PREFIX "cleanup_module\n"));

    unregister_reboot_notifier(&mvnet_drawb_reboot_notifier);
    pci_unregister_driver(&mvnet_drawb_driver);

}

module_init(mvnet_drawb_init_module);
module_exit(mvnet_drawb_cleanup_module);
