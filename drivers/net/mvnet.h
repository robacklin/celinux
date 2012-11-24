/*
 *
 *  mvnet/driver/mvnet.h, version 2.0
 *
 *  Copyright 2000-2003, MontaVista Software, Inc.
 *
 *  This software may be used and distributed according to the terms of
 *  the GNU Public License, Version 2, incorporated herein by reference.
 *
 *  Contact:  <source@mvista.com>
 *
 */

#ifndef _MVNET_H_
#define _MVNET_H_

#include <linux/skbuff.h>
#include <linux/ioport.h>

#define NDEBUG
#define MVNET_ETHERNET_ADDRLEN	6

#define MVNET_MAX_DEVICES       8

struct irq_spec {
	u32 addr;
	u32 val;
	u32 len;
};

struct nodeinfo {
    u8 addr[MVNET_ETHERNET_ADDRLEN];
    u8 active;
    u8 valid;
    u32 mem_paddr;
    u32 csr_paddr;
    struct irq_spec irq;
};

typedef void (*mvnet_receive_t)(void *arg, void *buf);
typedef void * (*mvnet_bufalloc_t)(void *arg, unsigned int len, void **buf);

struct mvnet_pool;

extern int
mvnet_open(
    struct mvnet_pool *pool,
    void *addr,
    mvnet_receive_t recv,
    mvnet_bufalloc_t alloc,
    void *arg);

extern void mvnet_close(struct mvnet_pool *pool);

typedef void (*mvnet_iodone_fn)(int result, void *arg);

extern int
mvnet_send(
    struct mvnet_pool *pool,
    void *addr,
    void *data,
    unsigned int len,
    mvnet_iodone_fn iodone,
    struct sk_buff *skb);

extern int
mvnet_broadcast(
    struct mvnet_pool *pool,
    void *data,
    unsigned int len,
    mvnet_iodone_fn iodone,
    struct sk_buff *skb);

struct mvnet_device;

struct mvnet_device_funcs {
    void (*open)(void);
    int (*listen)(void *dev);
    int (*accept)(void *dev, unsigned long sysbuf, int devnum);
    int (*connect)(void *dev, void *buf);
    int (*start)(void *dev, unsigned long memstart, unsigned long memend);
    int (*update)(void *dev);
    void * (*open_conn)(
	void *dev,
	int num,
	int indirect,
	volatile struct nodeinfo *node,
	void **devconn);
    int (*send)(void *dev);
    void (*close_conn)(void *devconn);
    void (*stop)(void *dev);
    void (*disconnect)(void *dev);
    void (*close)(void);
};

enum events {accept = 0,
	     connect,
	     start,
	     update,
	     receive,
	     stop,
	     disconnect};

struct mvnet_event {
	enum events event;
	void *p1;
	void *p2;
	void *p3;
	void *p4;
};

extern int mvnet_proc_event(struct mvnet_event *evnt);

extern void mvnet_find_root_resources(struct pci_dev *pdev,
	     			      struct resource *memres,
				      u32 *rtmemstart, u32 *rtmemend);

extern int mvnet_add_device(struct mvnet_device_funcs *devfn,
			    void *dev, struct mvnet_device **netdev,
			    int manager, u32 rtmemstart,
			    u32 rtmemend);

extern void mvnet_remove_device(struct mvnet_device *netdev);

struct mvnet_funcs {
    int (*open)(struct mvnet_pool *pool, void *addr, mvnet_receive_t recv,
		mvnet_bufalloc_t alloc, void *arg);
    void (*close)(struct mvnet_pool *pool);
    int  (*send)(struct mvnet_pool *pool, void *addr, void *data,
		 unsigned int len, mvnet_iodone_fn iodone, struct sk_buff *skb);
    int  (*broadcast)(struct mvnet_pool *pool, void *data, unsigned int len,
		      mvnet_iodone_fn iodone, struct sk_buff *skb);
};

#endif /* _MVNET_H_ */
