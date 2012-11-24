/*
 *
 *  mvnet/driver/mvnet.c, version 2.0
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
#include <linux/netdevice.h>
#include <linux/pci.h>
#include <linux/sched.h>
#include <linux/stddef.h>
#include <linux/string.h>
#include <linux/tqueue.h>
#include <linux/types.h>
#include <asm/bitops.h>
#include <asm/io.h>
#ifdef CONFIG_PPC
#include <asm/pci-bridge.h>
#endif

#include "mvnet.h"
#include "mvnet_eth.h"
#include "mvnet_drawb.h"
#include "mvnet_dma.h"

MODULE_AUTHOR("MontaVista Software <source@mvista.com>");
MODULE_DESCRIPTION("MontaVista Net Backpanel Driver (routing layer)");
MODULE_LICENSE("GPL");

#if defined(CONFIG_MVNET_DMA) || defined(CONFIG_MVNET_DMA_MODULE)
#define MVNET_USE_DMA
#endif
#undef SHARE_DMA

#ifndef NDEBUG
#define Debug(message)  printk message
#define assert(expr)    \
    if (!(expr))        \
        printk("Assertion failure:  (%s) in %s, %s:%d\n", \
                #expr, __FUNCTION__, __FILE__, __LINE__); else
#else /* NDEBUG */
#define Debug(message)
#define assert(expr)
#endif /* NDEBUG */

#define PRINT_PREFIX    "mvnet:  "

#define MVNET_MEM_PUT32(data)	cpu_to_le32(data)
#define MVNET_MEM_GET32(data)	le32_to_cpu(data)

#define MVNET_ALIGN		(sizeof(u32)*2)

#define MVNET_ALIGN_MASK	(MVNET_ALIGN - 1)
#define MVNET_ROUNDDOWN(value)	((value) & ~MVNET_ALIGN_MASK)
#define MVNET_ROUNDUP(value)	MVNET_ROUNDDOWN((value) + MVNET_ALIGN_MASK)

extern void * mvnet_memcpy(void *dest, const void *src, size_t n);

/*
 *  Buffer data structures
 */

struct queue {
    u32 qstart;
    u32 qend;
    u32 qread;
    u32 qwrite;
};

struct bufhdr {
    volatile unsigned long rtmemstart;
    volatile unsigned long rtmemend;
    volatile struct nodeinfo nodes[MVNET_MAX_DEVICES];
    volatile struct queue queues[MVNET_MAX_DEVICES];
};


/*
 *  Local data structures
 */

struct connection {
    struct mvnet_device *netdev;
    void *devconn;

    int connect_in;
    u8 *qin_start;
    u8 *qin_end;
    u8 *qin_read;
    volatile u32 *qin_read_return;

    int connect_out;
    u8 *qout_base;
    u8 *qout_start;
    u8 *qout_end;
    u8 *qout_write;
    volatile u32 *qout_write_return;

#ifdef MVNET_USE_DMA
    int status_out;
#endif
};

struct mvnet_device {
    struct mvnet_pool *pool;
    struct mvnet_device_funcs *devfn;
    void *dev;
    int num;
    int started;
    int closing;
    struct bufhdr *mem;
};

struct mvnet_pool {
	struct list_head list;
	int idx;
	struct mvnet_device devices[MVNET_MAX_DEVICES];
	u8 addr[MVNET_ETHERNET_ADDRLEN];
	int manager;
	u32 rtmemstart;
	u32 rtmemend;
	int nodenum;
	int closing;
	int device_count;
	void *ethdev;
	struct connection connections[MVNET_MAX_DEVICES];
	mvnet_receive_t recv_fn;
	mvnet_bufalloc_t bufalloc_fn;
	void *mvnet_recv_arg;
	unsigned long memory_block;
	struct bufhdr *localmem;
	struct bufhdr *sysmem;
};

static LIST_HEAD(pool_head);

#ifdef MVNET_USE_DMA
struct mvnet_usr {
	struct connection *conn;
	u8 *qwrite;
	u32 last_desc;
	mvnet_iodone_fn iodone;
	void * arg;
};

static void *dma_ch;
static LIST_HEAD(dma_head);
#endif



/*
 *  Buffer handling
 */

#define MVNET_BUFFER_ORDER	5
#define MVNET_BUFFER_SIZE	(4096 << MVNET_BUFFER_ORDER)


static void
mvnet_init_queues(struct mvnet_pool *pool, int devnum)
{
    struct bufhdr *buf = pool->localmem;
    int num;
    unsigned long qspace, qsize;
    u8 *qptr;

    Debug((PRINT_PREFIX "mvnet_init_queues\n"));

    qspace = MVNET_BUFFER_SIZE - MVNET_ROUNDUP(sizeof(struct bufhdr));
    qsize = MVNET_ROUNDDOWN(qspace / (MVNET_MAX_DEVICES - 1));
    Debug((PRINT_PREFIX "queue space = %08x\n", (unsigned int) qspace));
    Debug((PRINT_PREFIX "queue size  = %08x\n", (unsigned int) qsize));
    qptr = ((u8 *) buf) + MVNET_ROUNDUP(sizeof(struct bufhdr));

    for (num = 0; num < MVNET_MAX_DEVICES; num++) {
	buf->nodes[num].active = 0;

	if (num == devnum)
	    continue;

	pool->connections[num].qin_start = qptr;
	Debug((PRINT_PREFIX "qin_start[%d] = %08x\n", num,
		(unsigned int) pool->connections[num].qin_start));
	qptr += qsize;
	pool->connections[num].qin_end = qptr;
    }
}


static int
mvnet_init_memory(struct mvnet_pool *pool)
{
    /*
     *  Allocate physically contiguous memory for remote mapping
     */
    pool->memory_block = __get_free_pages(GFP_KERNEL, MVNET_BUFFER_ORDER);
    if (pool->memory_block == 0)
	return -ENOMEM;

    pool->localmem = (struct bufhdr *) pool->memory_block;
    memset(pool->localmem, 0, sizeof(struct bufhdr));

    return 0;
}


static int
mvnet_free_memory(struct mvnet_pool *pool)
{
    assert(pool->memory_block != 0);

    free_pages(pool->memory_block, MVNET_BUFFER_ORDER);
    pool->memory_block = 0;

    return 0;
}


/*
 *  Connection handling
 */


static void
mvnet_open_connection(struct mvnet_pool *pool, int num)
{
    struct connection *conn = &pool->connections[num];
    struct mvnet_device *netdev;
    int indirect;

    Debug((PRINT_PREFIX "mvnet_open_connection: %d\n", num));

    if (pool->devices[num].dev != NULL) {
	/* direct connection */
	netdev = &pool->devices[num];
	indirect = 0;
    } else {
	/* indirect connection */
	netdev = &pool->devices[0];
	indirect = 1;
    }

    conn->qout_base = netdev->devfn->open_conn(
	netdev->dev, num, indirect, &pool->sysmem->nodes[num],
	&conn->devconn);

    if (conn->qout_base != NULL) {
	struct bufhdr *localmem = pool->localmem;
	struct bufhdr *buf = (struct bufhdr *) conn->qout_base;
	volatile struct queue *qptr = &(buf->queues[pool->nodenum]);

	localmem->queues[num].qwrite =
	    cpu_to_le32(conn->qin_start - ((u8 *)localmem));
	
	writel(conn->qin_start - ((u8 *)localmem), &qptr->qread);
	writel(conn->qin_start - ((u8 *)localmem), &qptr->qstart);
	writel(conn->qin_end - ((u8 *) localmem), &qptr->qend);

	conn->qin_read_return = &qptr->qread;
	conn->qout_write_return = &qptr->qwrite;
	conn->qin_read = conn->qin_start;

	conn->netdev = netdev;
	conn->connect_in = 1;
    }

#ifdef MVNET_USE_DMA
    conn->status_out = 0;
#endif
}


static void
mvnet_close_connection(struct mvnet_pool *pool, int num)
{
    struct connection *conn = &pool->connections[num];
    struct bufhdr *buf = (struct bufhdr *) conn->qout_base;
    volatile struct queue *qptr = &(buf->queues[pool->nodenum]);

    Debug((PRINT_PREFIX "mvnet_close_connection: %d\n", num));

    conn->connect_in = 0;
    conn->connect_out = 0;

    qptr->qend = 0;

    conn->netdev->devfn->close_conn(conn->devconn);
}

/*
 *  Misc functions
 */

/*
 * Called by master only. Sends updates to all active targets to announce the
 * addition or deletion of a node, then updates master's connections.
 */
static void
mvnet_do_updates(struct mvnet_pool *pool)
{
    int num;

    Debug((PRINT_PREFIX "mvnet_do_updates\n"));

    for (num = 1; num < MVNET_MAX_DEVICES; num++) {
	struct mvnet_device *netdev = &pool->devices[num];

	Debug((PRINT_PREFIX "devices[%d].started = %d\n",
			    num, pool->devices[num].started));
	if (netdev->started) {
	    Debug((PRINT_PREFIX "update device %d\n", num));
	    memcpy((void *) netdev->mem->nodes, (void *) pool->localmem->nodes,
		    sizeof(struct nodeinfo) * MVNET_MAX_DEVICES);
	    netdev->devfn->update(netdev->dev);
	}
    }

    if (pool->recv_fn != NULL) {
	void mvnet_update(struct mvnet_pool *);

	/* Update system master connections */
	mvnet_update(pool);
    }
}


#ifdef MVNET_USE_DMA
#ifdef SHARE_DMA
static void mvnet_callback_desc(void *ptr)
{
	struct dma_pub *pub = (struct dma_pub *) ptr;
	struct mvnet_usr *usr = (struct mvnet_usr *)&pub->usr;
	struct connection *conn = usr->conn;
	int status;

	Debug((PRINT_PREFIX "mvnet_callback_list\n"));

	if (!usr->last_desc) {
		conn->status_out = pub->status;
		Debug((PRINT_PREFIX "skipping entry %x\n", usr));
		return;
	}

	status = conn->status_out;

	Debug((PRINT_PREFIX "processing entry %x\n", usr));

	if (status == 0)
		status = pub->status;

	Debug((PRINT_PREFIX "entry status %d\n", status));
	if (status == 0) {

		Debug((PRINT_PREFIX "updating queue %x\n", usr->qwrite));

		/* write the qwrite value */
		writel(usr->qwrite - conn->qout_base, conn->qout_write_return);
		status = conn->netdev->devfn->send(conn->devconn);
	} else
		printk(PRINT_PREFIX "Error %d\n", status);

	if (usr->iodone)
		usr->iodone(status, usr->arg);

	conn->status_out = 0;
}
#else
static void mvnet_callback_list(void *ptr)
{
	struct list_head *head = (struct list_head *) ptr;
	struct list_head *pos;
	int status = 0;

	Debug((PRINT_PREFIX "mvnet_callback_list\n"));

	list_for_each(pos, head) {
		struct dma_pub *pub = list_entry(pos, struct dma_pub, list);
		struct mvnet_usr *usr = (struct mvnet_usr *)&pub->usr;

		if (!usr->last_desc) {
			status = pub->status;
			Debug((PRINT_PREFIX "skipping entry %x\n", usr));
			continue;
		}

		Debug((PRINT_PREFIX "processing entry %x\n", usr));

		if (status == 0)
			status = pub->status;

		Debug((PRINT_PREFIX "entry status %d\n", status));
		if (status == 0) {
			struct connection *conn = usr->conn;

			Debug((PRINT_PREFIX "updating queue %x\n", usr->qwrite));

			/* write the qwrite value */
			writel(usr->qwrite - conn->qout_base,
			       conn->qout_write_return);
			status = conn->netdev->devfn->send(conn->devconn);
		} else 
/* TEMP */
			printk(PRINT_PREFIX "Error %d\n", status);

		if (usr->iodone)
			usr->iodone(status, usr->arg);
		status = 0;
	}
}
#endif
static struct mvnet_usr * mvnet_set_desc(void * dst, void *src, size_t
						  size)
{
	struct dma_pub *pub = NULL;

	Debug((PRINT_PREFIX "mvnet_set_desc\n"));

	if (mvnet_dma_alloc_desc(&pub, dma_ch) != 0)
		return NULL;

	pub->dst = dst;
	pub->src = src;
	pub->size = size;
	pub->direction = PCI_DMA_TODEVICE;
#ifdef SHARE_DMA
	pub->callback = mvnet_callback_desc;
#else
	pub->callback = mvnet_callback_list;
#endif
	list_add_tail(&pub->list, &dma_head);
	return (struct mvnet_usr *) &pub->usr;
}
#endif

static int
mvnet_xmit(struct mvnet_pool *pool, int num, void *data, unsigned int len,
	   mvnet_iodone_fn iodone, void * arg)
{
    struct connection *conn = &pool->connections[num];
    volatile struct queue *q = &pool->localmem->queues[num];
    u32 qend;
#ifdef MVNET_USE_DMA
    struct mvnet_usr *usr;
#else
    int retval;
#endif

    Debug((PRINT_PREFIX "mvnet_xmit\n"));
    assert(conn->connect_in == 1);

    Debug((PRINT_PREFIX "conn->connect_out = %d\n", conn->connect_out));
    if (!conn->connect_out) {
	qend = le32_to_cpu(q->qend);
	if (qend != 0) {
	    conn->qout_start = conn->qout_write =
		conn->qout_base + le32_to_cpu(q->qstart);
	    conn->qout_end = conn->qout_base + qend;
	    conn->connect_out = 1;

	}
    }

    if (conn->connect_out) {
	u8 *qread = conn->qout_base + le32_to_cpu(q->qread);
	u8 *qwrite = conn->qout_write;
	u32 size1, size2;

	if (qread > qwrite) {
	    size1 = qread - qwrite - MVNET_ALIGN;
	    size2 = 0;
	} else if (qread == conn->qout_start) {
	    size1 = conn->qout_end - qwrite - MVNET_ALIGN;
	    size2 = 0;
	} else {
	    size1 = conn->qout_end - qwrite;
	    size2 = qread - conn->qout_start - MVNET_ALIGN;
	}

	if (MVNET_ROUNDUP(len + MVNET_ALIGN) > (size1 + size2))
	    return -ENOSPC;
	
	writel(len, qwrite);
	qwrite += MVNET_ALIGN;
	size1 -= MVNET_ALIGN;

	Debug((PRINT_PREFIX "XMIT len = %d\n", len));
	if (size1 > 0) {
	    if (size1 > len)
		size1 = len;

	    Debug((PRINT_PREFIX "XMIT qwrite = 0x%lx (0x%x)\n",
		(unsigned long) qwrite, qwrite - conn->qout_base));
	    Debug((PRINT_PREFIX "size1 = %d\n", size1));

#ifdef MVNET_USE_DMA
	    usr = mvnet_set_desc((void *) qwrite, data, size1);
	    if (usr == NULL)
		    return -ENOSPC;

	    usr->conn = conn;
	    usr->last_desc = 0;
	    usr->iodone = iodone;
	    usr->arg = arg;
#else
#ifndef NDEBUG
	    /* DEBUG */
	    printk("1: %08x, %08x, %08x\n", (unsigned int) qwrite,
		   (unsigned int) data, (unsigned int) size1);
#endif
	    memcpy((void *) qwrite, data, size1);
#endif
	    len -= size1;
	    qwrite += MVNET_ROUNDUP(size1);
	}

	if (len > 0) {
	    qwrite = conn->qout_start;

	    if (size2 > len)
		size2 = len;

	    Debug((PRINT_PREFIX "XMIT qwrite = 0x%lx (0x%x)\n",
		(unsigned long) qwrite, qwrite - conn->qout_base));
	    Debug((PRINT_PREFIX "size2 = %d\n", size2));

#ifdef MVNET_USE_DMA
	    usr = mvnet_set_desc((void *) qwrite, ((u8 *)data) + size1, size2);
	    if (usr == NULL)
		    return -ENOSPC;

	    usr->conn = conn;
	    usr->last_desc = 0;
	    usr->iodone = iodone;
	    usr->arg = arg;
#else
#ifndef NDEBUG
	    /* DEBUG */
	    printk("2: %08x, %08x, %08x\n", (unsigned int) qwrite,
		   (unsigned int) (((u8 *)data) + size1), (unsigned int) size2);
#endif
	    memcpy((void *) qwrite, ((u8 *)data) + size1, size2);
#endif
	    qwrite += MVNET_ROUNDUP(size2);
	}

	if (qwrite == conn->qout_end)
	    qwrite = conn->qout_start;

	conn->qout_write = qwrite;

#ifdef MVNET_USE_DMA
	usr->qwrite = qwrite;
	usr->last_desc = 1;
	return 0;
#else
	/* write the qwrite value */
	writel(qwrite - conn->qout_base, conn->qout_write_return);

	retval = conn->netdev->devfn->send(conn->devconn);
	if (retval != 0)
		return retval;

	iodone(0, arg);
	return 0;
#endif

    } else {
	return -EAGAIN;
    }
}


int
mvnet_open(
    struct mvnet_pool *pool,
    void *addr,
    mvnet_receive_t recv,
    mvnet_bufalloc_t alloc,
    void *arg
)
{
    struct bufhdr *localmem = pool->localmem;
    int num;

    Debug((PRINT_PREFIX "mvnet_open\n"));

    if (addr != NULL)
    	memcpy(pool->addr, addr, MVNET_ETHERNET_ADDRLEN);

    if (pool->manager) {

	pool->recv_fn = recv;
	memcpy((void *) localmem->nodes[pool->nodenum].addr,
		pool->addr, MVNET_ETHERNET_ADDRLEN);
	localmem->nodes[pool->nodenum].active = 1;
	mvnet_do_updates(pool);
    } else {
	struct mvnet_device *netdev = &pool->devices[0];

	if (netdev->dev == NULL)
	    return -EIO;

	netdev->mem = localmem;

	if (netdev->devfn->connect(netdev->dev, (void *) localmem) != 0)
	    return -EIO;

	pool->recv_fn = recv;
    }

    pool->bufalloc_fn = alloc;
    pool->mvnet_recv_arg = arg;

    for (num = 0; num <MVNET_MAX_DEVICES; num++) {
	    if (pool->devices[num].dev != NULL) {
		    pool->devices[num].devfn->open();
		    break;
	    }
    }

    MOD_INC_USE_COUNT;

    return 0;
}


void
mvnet_close(struct mvnet_pool *pool)
{
    int num;

    Debug((PRINT_PREFIX "mvnet_close\n"));

    if (pool->manager == 1) {
	pool->localmem->nodes[pool->nodenum].active = 0;
	mvnet_do_updates(pool);
    } else {
	struct mvnet_device *netdev = &pool->devices[0];

	/* Send stop request */
	netdev->devfn->stop(netdev->dev);
    }

    /* Close all connections */
    for (num = 0; num < MVNET_MAX_DEVICES; num++) {
	if (pool->connections[num].connect_in)
	    mvnet_close_connection(pool, num);
    }

    Debug((PRINT_PREFIX "mvnet_close_complete\n"));

    pool->recv_fn = NULL;
    pool->bufalloc_fn = NULL;
    pool->mvnet_recv_arg = NULL;

    for (num = 0; num <MVNET_MAX_DEVICES; num++) {
	if (pool->devices[num].dev != NULL) {
	    pool->devices[num].devfn->close();
	    break;
	}
    }

    MOD_DEC_USE_COUNT;
}

int
mvnet_send(
    struct mvnet_pool *pool,
    void *addr,
    void *data,
    unsigned int len,
    mvnet_iodone_fn iodone,
    struct sk_buff *skb
)
{
    struct bufhdr *localmem = pool->localmem;
    int num;

    Debug((PRINT_PREFIX "mvnet_send\n"));
    assert(localmem != NULL);

    for (num = 0; num < MVNET_MAX_DEVICES; num++) {
	if (pool->connections[num].connect_in) {
	    volatile struct nodeinfo *node = &localmem->nodes[num];
	    u8 *dst = (u8 *) addr;
	    int result;

	    /* check address */
	    if ( (dst[MVNET_ETHERNET_ADDRLEN - 1] != node->addr[MVNET_ETHERNET_ADDRLEN - 1])
	    && (memcmp((void *) node->addr, dst, MVNET_ETHERNET_ADDRLEN) != 0) )
		continue;

	    result = mvnet_xmit(pool, num, data, len, iodone, (void *)skb);

#ifdef MVNET_USE_DMA
	    if (!list_empty(&dma_head)) {
		    if (result != 0)
			    (void) mvnet_dma_free_list(&dma_head, dma_ch);
		    else
			    result = mvnet_dma_queue_list(&dma_head, dma_ch);
		    INIT_LIST_HEAD(&dma_head);
	    }
#endif
	    return result;
	}
    }

    /* no address match, drop packet silently */
    dev_kfree_skb_any(skb);
    return 0;
}

int
mvnet_broadcast(
    struct mvnet_pool *pool,
    void *data,
    unsigned int len,
    mvnet_iodone_fn iodone,
    struct sk_buff *skb
)
{
    int num;

    Debug((PRINT_PREFIX "mvnet_broadcast\n"));
    assert(pool->localmem != NULL);

    for (num = 0; num < MVNET_MAX_DEVICES; num++) {
	if (pool->connections[num].connect_in)
	    (void) mvnet_xmit(pool, num, data, len, iodone,
			      (void *)skb_get(skb));
    }

#ifdef MVNET_USE_DMA
    if (!list_empty(&dma_head)) {
	(void) mvnet_dma_queue_list(&dma_head, dma_ch);
	INIT_LIST_HEAD(&dma_head);
    }
#endif

    dev_kfree_skb_any(skb);
    return 0;
}


/*
 *  mvnet functions
 */

/*
 * Called by master only. Captures virtual address of target's memory along
 * with physical addresses of target's CSR and memory areas. Physical addresses
 * will be sent to other targets to allow them to map the CSR and memory spaces
 * of the current device.
 */
static int
mvnet_accept(
    struct mvnet_device *netdev,
    void *vaddr,
    unsigned long csr_paddr,
    unsigned long mem_paddr)
{
    struct mvnet_pool *pool = netdev->pool;
    struct bufhdr *localmem = pool->localmem;

    Debug((PRINT_PREFIX "mvnet_accept\n"));
    assert(pool->manager == 1);

    if (pool->closing)
	return 0;

    netdev->mem = (struct bufhdr *) vaddr;

    localmem->nodes[netdev->num].csr_paddr = cpu_to_le32(csr_paddr);
    localmem->nodes[netdev->num].mem_paddr = cpu_to_le32(mem_paddr);

    return netdev->devfn->accept(netdev->dev,virt_to_bus(localmem),
		    		 netdev->num);
}


/* Called by target only. Captures assigned node number and copies target's
 * MAC address to system memory. Initializes local queues and sends "start"
 * back to master.
 */
static int
mvnet_connect(struct mvnet_device *netdev, void *sys, int devnum,
	      struct irq_spec *irq)
{
    struct mvnet_pool *pool = netdev->pool;
    struct bufhdr *sysmem = (struct bufhdr *) sys;

    Debug((PRINT_PREFIX "mvnet_connect\n"));
    assert(pool->manager == 0);

    pool->nodenum = devnum;
    pool->sysmem = sysmem;

    Debug((PRINT_PREFIX "node = %d, sysmem = %08x\n", devnum,
			    (unsigned int)sysmem));

    memcpy((void *) sysmem->nodes[devnum].addr, pool->addr,
	    MVNET_ETHERNET_ADDRLEN);
    writel(irq->addr, &sysmem->nodes[devnum].irq.addr);
    writel(irq->len, &sysmem->nodes[devnum].irq.len);
    writel(irq->val, &sysmem->nodes[devnum].irq.val);
    mvnet_init_queues(pool, devnum);

    return netdev->devfn->start(netdev->dev,
		    		readl(&sysmem->rtmemstart),
				readl(&sysmem->rtmemend));
}


/*
 * Called by master only. Marks target active and announces new target to 
 * other nodes.
 */
static int
mvnet_start(struct mvnet_device *netdev)
{
    struct mvnet_pool *pool = netdev->pool;

    Debug((PRINT_PREFIX "mvnet_start\n"));
    assert(pool->manager == 1);

    netdev->started = 1;
    pool->sysmem->nodes[netdev->num].active = 1;

    mvnet_do_updates(pool);

    return 0;
}


void
mvnet_update(struct mvnet_pool *pool)
{
    struct bufhdr *localmem = pool->localmem;
    int num;

    Debug((PRINT_PREFIX "mvnet_update\n"));

    for (num = 0; num < MVNET_MAX_DEVICES; num++) {
	struct connection *conn = &pool->connections[num];

	if (num == pool->nodenum)
	    continue;

	Debug((PRINT_PREFIX "localmem->nodes[%d].active = %d\n",
		num, localmem->nodes[num].active));
	if (localmem->nodes[num].active) {
	    Debug((PRINT_PREFIX "conn->connect_in = %d\n", conn->connect_in));
	    if (!conn->connect_in)
		mvnet_open_connection(pool, num);
	}
	if (!(localmem->nodes[num].active)) {
	    if (conn->connect_in)
		mvnet_close_connection(pool, num);
	}
    }
}


static void
mvnet_receive(struct mvnet_device *netdev)
{
    struct mvnet_pool *pool = netdev->pool;
    struct bufhdr *localmem = pool->localmem;
    int num;

    Debug((PRINT_PREFIX "mvnet_receive\n"));

    for (num = 0; num < MVNET_MAX_DEVICES; num++) {
	struct connection *conn = &pool->connections[num];

	if ((conn->connect_in) && (conn->netdev == netdev)) {
	    u8 *qwrite = ((u8 *) localmem) +
		le32_to_cpu(localmem->queues[num].qwrite);
	    u8 *qread = conn->qin_read;

	    /* Maybe should close connection if out of range? */
	    if ( (qwrite < conn->qin_start) || (qwrite >= conn->qin_end)
	    || ((unsigned long)qwrite != MVNET_ROUNDUP((unsigned long)qwrite)) )
		continue;

	    while (qwrite != qread) {
		u32 len = le32_to_cpu(*((u32 *) qread));
		unsigned int size1, size2;
		u8 *data;
		void *buf;

		qread += MVNET_ALIGN;
		if (qread == conn->qin_end)
		    qread = conn->qin_start;

		if (qwrite >= qread) {
		    size1 = qwrite - qread;
		    size2 = 0;
		} else {
		    size1 = conn->qin_end - qread;
		    size2 = qwrite - conn->qin_start;
		}

		if (len > (size1 + size2)) {
		    printk(PRINT_PREFIX "Invalid message received\n");

		    /* Discard all messages */
		    conn->qin_read = qread = qwrite;
		    writel(qread - ((u8 *) localmem), conn->qin_read_return);
		    break;
		}

		data = pool->bufalloc_fn(pool->mvnet_recv_arg, len, &buf);

		if (size1 > len)
		    size1 = len;

		Debug((PRINT_PREFIX "RECV qread = 0x%lx (0x%x), size1 = %d\n",
			(unsigned long)qread, qread - ((u8 *)localmem),
			size1));
		if (data != NULL)
		    memcpy(data, qread, size1);
		qread += MVNET_ROUNDUP(size1);
		len -= size1;

		if (len > 0) {
		    qread = conn->qin_start;

		    if (size2 > len)
			size2 = len;

		    if (data != NULL) {
			data += size1;
			Debug((PRINT_PREFIX "RECV qread = 0x%lx (0x%x)\n",
				(unsigned long)qread,
				qread-((u8 *)localmem)));
			Debug((PRINT_PREFIX "RECV size2 = %d\n", size2));
			memcpy(data, qread, size2);
		    }

		    qread += MVNET_ROUNDUP(size2);
		}

		if (data != NULL)
		    pool->recv_fn(pool->mvnet_recv_arg, buf);

		if (qread == conn->qin_end)
		    qread = conn->qin_start;
		conn->qin_read = qread;
		writel(qread - ((u8 *) localmem), conn->qin_read_return);
	    }
	}
    }
}


static void
mvnet_stop(struct mvnet_device *netdev)
{
    struct mvnet_pool *pool = netdev->pool;

    Debug((PRINT_PREFIX "mvnet_stop\n"));
    assert(pool->manager == 1);

    netdev->started = 0;
    pool->sysmem->nodes[netdev->num].active = 0;

    mvnet_do_updates(pool);

    if (!netdev->closing)
	(void) netdev->devfn->listen(netdev->dev);
}


static void
mvnet_disconnect(struct mvnet_device *netdev)
{
    struct mvnet_pool *pool = netdev->pool;
    struct bufhdr *localmem = pool->localmem;
    int num;

    Debug((PRINT_PREFIX "mvnet_disconnect\n"));
    assert(pool->manager == 0);

    /* Break all connections */
    for (num = 0; num < MVNET_MAX_DEVICES; num++) {
	if (pool->connections[num].connect_in) {
	    mvnet_close_connection(pool, num);
	    localmem->queues[num].qend = 0;
	}
    }

    if (pool->recv_fn != NULL) {
	/* Go back into connect mode */
	netdev->devfn->connect(netdev->dev, (void *) localmem);
    }
}

struct mvnet_funcs mvnet_functions = {
    mvnet_open,
    mvnet_close,
    mvnet_send,
    mvnet_broadcast
};

/*
 *  Externally accessible functions
 */

EXPORT_SYMBOL(mvnet_find_root_resources);
void
mvnet_find_root_resources(struct pci_dev *pdev, struct resource *memres,
			  u32 *rtmemstart, u32 *rtmemend)

{
	struct pci_dev *lpdev;

	if ((pdev == NULL) ||
	    (memres == NULL) ||
	    (rtmemstart == NULL) ||
	    (rtmemend == NULL)) {
		printk("mvnet_find_root_resources called with NULL pointer\n");
		return;
	    }

	while (pdev) {
		struct resource *res;
		if ((res = pci_find_parent_resource(pdev, memres))
								!= NULL) {
			memres = res;
		}
		lpdev = pdev;
		pdev = pdev->bus->self;
	}
#ifdef CONFIG_PPC
	/* compensate for possible PReP map */
	*rtmemstart = memres->start - pci_bus_mem_base_phys(lpdev->bus->number);
	*rtmemend = memres->end - pci_bus_mem_base_phys(lpdev->bus->number);
#else
	*rtmemstart = memres->start;
	*rtmemend = memres->end;

#endif
}

static struct mvnet_pool *
mvnet_locate_pool(int manager, u32 rtmemstart, u32 rtmemend)
{
	struct list_head *pos;

	Debug((PRINT_PREFIX "mvnet_locate_pool\n"));

	if (!list_empty(&pool_head)) {
		list_for_each(pos, &pool_head) {
			struct mvnet_pool *pool;
			pool = list_entry(pos, struct mvnet_pool, list);
			if ((pool->manager == manager) &&
					(pool->rtmemstart == rtmemstart) &&
					(pool->rtmemend == rtmemend))
				return pool;
		}
	}

	return NULL;
}

static struct mvnet_pool *
mvnet_add_pool(u32 rtmemstart, u32 rtmemend)
{
	struct mvnet_pool *pool;

	Debug((PRINT_PREFIX "mvnet_add_pool\n"));

	pool = kmalloc(sizeof(struct mvnet_pool), GFP_KERNEL);
	if (pool == NULL) {
		printk(PRINT_PREFIX "Unable to reserve memory for new pool\n");
		return NULL;
	}

	memset(pool, 0, sizeof(struct mvnet_pool));

	if (mvnet_init_memory(pool) != 0) {
		printk(PRINT_PREFIX "Unable to reserver buffer space.\n");
		kfree(pool);
		return NULL;
	}

	pool->rtmemstart = rtmemstart;
	pool->rtmemend = rtmemend;
	list_add(&pool->list, &pool_head);
	return pool;
}
static void
mvnet_remove_pool(struct mvnet_pool *pool)
{
	Debug((PRINT_PREFIX "mvnet_remove_pool\n"));

	list_del(&pool->list);
	mvnet_free_memory(pool);
	kfree(pool);
}

EXPORT_SYMBOL(mvnet_add_device);
int
mvnet_add_device(
    struct mvnet_device_funcs *devfn,
    void *dev,
    struct mvnet_device **ndev,
    int manager,
    u32 rtmemstart,
    u32 rtmemend
)
{
    struct mvnet_pool *pool;
    struct mvnet_device *netdev = NULL;
    int num;

    Debug((PRINT_PREFIX "mvnet_add_device\n"));

    pool = mvnet_locate_pool(manager, rtmemstart, rtmemend);

    if (pool == NULL)
	    pool = mvnet_add_pool(rtmemstart, rtmemend);

    if (pool == NULL)
	    return -ENOMEM;

    if (manager) {

	struct bufhdr *localmem = pool->localmem;

	/* Manager reserves device 0 for itself */
	if (pool->devices[0].dev != NULL)
	    return -EINVAL;

	if (pool->manager == 0) {

	    pool->nodenum = 0;
	    mvnet_init_queues(pool, pool->nodenum);
	    localmem->nodes[0].mem_paddr = cpu_to_le32(virt_to_bus(localmem));
	    localmem->rtmemstart = cpu_to_le32(rtmemstart);
	    localmem->rtmemend = cpu_to_le32(rtmemend);
	}

	pool->manager = 1;
	pool->sysmem = localmem;

	/* Find available device entry */
	for (num = 1; num < MVNET_MAX_DEVICES; num++) {
	    if (pool->devices[num].dev == NULL) {
		netdev = &pool->devices[num];
		break;
	    }
	}
	if (netdev == NULL)
	    return -ENOSPC; 

    } else {
	/* Refuse if we're already declared the manager */
	if (pool->manager == 1)
	    return -EINVAL;

	/* Non-manager always uses device 0 for manager */
	netdev = &pool->devices[0];
	num = 0;

	/* Make sure device 0 is not already in use */
	if (netdev->dev != NULL)
	    return -ENOSPC;
    }

    netdev->pool = pool;
    netdev->dev = dev;
    netdev->devfn = devfn;
    netdev->num = num;
    netdev->closing = 0;
    *ndev = netdev;

    if (manager) {
	int result;

	/* Listen for device to complete connection */
	result = devfn->listen(dev);
	if (result != 0)
	    return result;
    }

    /* If this is the first device, add the ethernet interface */

    if (pool->device_count == 0)
	    pool->ethdev = mvnet_eth_add_device((void *)pool,
			    			      &mvnet_functions);

    if (pool->ethdev != NULL)
        pool->device_count++;

    return 0;
}

EXPORT_SYMBOL(mvnet_remove_device);
void
mvnet_remove_device(struct mvnet_device *netdev) {

    struct mvnet_pool *pool = netdev->pool;

    netdev->closing = 1;

    /* tell target to disconnect */

    if (pool->manager) {
	netdev->devfn->disconnect(netdev->dev);
	mvnet_stop(netdev);
    }

    /* if last device, remove ethernet interface */

    if (--pool->device_count == 0) {
	    mvnet_eth_remove_device(pool->ethdev);
	    mvnet_remove_pool(pool);
    }

    netdev->dev = NULL;

}

EXPORT_SYMBOL(mvnet_proc_event);
int
mvnet_proc_event(struct mvnet_event *evnt) {

    Debug((PRINT_PREFIX "mvnet_proc_event\n"));

    switch (evnt->event) {
	case accept:
	    return mvnet_accept(evnt->p1, evnt->p2, (unsigned long) evnt->p3,
			        (unsigned long) evnt->p4);

	case connect:
	    return mvnet_connect(evnt->p1, evnt->p2, (int) evnt->p3,
			         (struct irq_spec *) evnt->p4);

	case start:
	    return mvnet_start(evnt->p1);

	case update:
	    mvnet_update(((struct mvnet_device *)(evnt->p1))->pool);
	    return 0;

	case receive:
	    mvnet_receive(evnt->p1);
	    return 0;

	case stop:
	    mvnet_stop(evnt->p1);
	    return 0;

	case disconnect:
	    mvnet_disconnect(evnt->p1);
	    return 0;
    }
    return 0;
}


/*
 *  init/cleanup
 */

int
__init mvnet_init_module(void)
{
    int result = 0;

    Debug((PRINT_PREFIX "init_module\n"));

#ifdef MVNET_USE_DMA
#ifdef SHARE_DMA
    result = mvnet_dma_open(0, DMA_MODE_SHARED, &dma_ch);
#else
    result = mvnet_dma_open(0, DMA_MODE_EXCLUSIVE, &dma_ch);
#endif
#endif

    return result;
}


void
__exit mvnet_cleanup_module(void)
{
    Debug((PRINT_PREFIX "cleanup_module\n"));

#ifdef MVNET_USE_DMA
    mvnet_dma_close(dma_ch);
#endif
}

module_init(mvnet_init_module);
module_exit(mvnet_cleanup_module);
