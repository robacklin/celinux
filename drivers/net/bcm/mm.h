
/******************************************************************************/
/*                                                                            */
/* Broadcom BCM5700 Linux Network Driver, Copyright (c) 2000 Broadcom         */
/* Corporation.                                                               */
/* All rights reserved.                                                       */
/*                                                                            */
/* This program is free software; you can redistribute it and/or modify       */
/* it under the terms of the GNU General Public License as published by       */
/* the Free Software Foundation, located in the file LICENSE.                 */
/*                                                                            */
/******************************************************************************/

#ifndef MM_H
#define MM_H

#include <linux/config.h>
#if defined(CONFIG_SMP) && ! defined(__SMP__)
#define __SMP__
#endif
#if defined(CONFIG_MODVERSIONS) && defined(MODULE) && ! defined(MODVERSIONS)
#define MODVERSIONS
#endif

#ifndef B57UM
#define __NO_VERSION__
#endif
#include <linux/version.h>
#ifdef MODULE
#ifdef MODVERSIONS
#include <linux/modversions.h>
#endif
#include <linux/module.h>
#else
#define MOD_INC_USE_COUNT
#define MOD_DEC_USE_COUNT
#define SET_MODULE_OWNER(dev)
#define MODULE_DEVICE_TABLE(pci, pci_tbl)
#endif

#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/string.h>
#include <linux/timer.h>
#include <linux/errno.h>
#include <linux/ioport.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/pci.h>
#include <linux/init.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/skbuff.h>
#include <asm/processor.h>		/* Processor type for cache alignment. */
#include <asm/bitops.h>
#include <asm/io.h>
#include <asm/unaligned.h>
#include <linux/delay.h>
#include <asm/byteorder.h>
#include <linux/time.h>
#if (LINUX_VERSION_CODE >= 0x020400)
#include <linux/ethtool.h>
#include <asm/uaccess.h>
#endif
#ifdef CONFIG_PROC_FS
#include <linux/smp_lock.h>
#include <linux/proc_fs.h>
#define BCM_PROC_FS 1
#endif
#ifdef NETIF_F_HW_VLAN_TX
#include <linux/if_vlan.h>
#define BCM_VLAN 1
#endif

#if (LINUX_VERSION_CODE >= 0x020400)
#ifndef ETHTOOL_GEEPROM

#define ETHTOOL_GEEPROM		0x0000000b /* Get EEPROM data */
#define ETHTOOL_SEEPROM		0x0000000c /* Set EEPROM data */

/* for passing EEPROM chunks */
struct ethtool_eeprom {
	u32	cmd;
	u32	magic;
	u32	offset; /* in bytes */
	u32	len; /* in bytes */
	u8	data[0];
};
#define BCM_EEDUMP_LEN(info_p, size) *((u32 *) &((info_p)->reserved1[24]))=size

#else

#define BCM_EEDUMP_LEN(info_p, size) (info_p)->eedump_len=size

#endif
#endif

#define BCM_INT_COAL 1
#define BCM_NIC_SEND_BD 1
#define BCM_ASF 1
#define BCM_WOL 1
#define BCM_TASKLET 1

#ifdef BCM_SMALL_DRV
#undef BCM_PROC_FS
#undef ETHTOOL_GEEPROM
#undef ETHTOOL_SEEPROM
#undef BCM_INT_COAL
#undef BCM_NIC_SEND_BD
#undef BCM_ASF
#undef BCM_WOL
#undef NICE_SUPPORT
#undef BCM_TASKLET
#endif

#ifdef __BIG_ENDIAN
#define BIG_ENDIAN_HOST 1
#endif

#if (LINUX_VERSION_CODE < 0x020327)
#define __raw_readl readl
#define __raw_writel writel
#endif

#include "lm.h"
#include "queue.h"
#include "tigon3.h"

#if DBG
#define STATIC
#else
#define STATIC static
#endif

extern int MM_Packet_Desc_Size;

#define MM_PACKET_DESC_SIZE MM_Packet_Desc_Size

DECLARE_QUEUE_TYPE(UM_RX_PACKET_Q, MAX_RX_PACKET_DESC_COUNT+1);

#define MAX_MEM 16

#if (LINUX_VERSION_CODE < 0x020211)
typedef u32 dma_addr_t;
#endif

#if (LINUX_VERSION_CODE < 0x02032a)
#define pci_map_single(dev, address, size, dir) virt_to_bus(address)
#define pci_unmap_single(dev, dma_addr, size, dir)
#endif

#if MAX_SKB_FRAGS
#if (LINUX_VERSION_CODE >= 0x02040d)

typedef dma_addr_t dmaaddr_high_t;

#else

#if defined(CONFIG_HIGHMEM) && defined(CONFIG_X86) && ! defined(CONFIG_X86_64)

#if defined(CONFIG_HIGHMEM64G)
typedef unsigned long long dmaaddr_high_t;
#else
typedef dma_addr_t dmaaddr_high_t;
#endif

#ifndef pci_map_page
#define pci_map_page bcm_pci_map_page
#endif

static inline dmaaddr_high_t
bcm_pci_map_page(struct pci_dev *dev, struct page *page,
		    int offset, size_t size, int dir)
{
	dmaaddr_high_t phys;

	phys = (page-mem_map) *	(dmaaddr_high_t) PAGE_SIZE + offset;

	return phys;
}

#ifndef pci_unmap_page
#define pci_unmap_page(dev, map, size, dir)
#endif

#else /* #if defined(CONFIG_HIGHMEM) && defined(CONFIG_X86) && ! defined(CONFIG_X86_64) */

typedef dma_addr_t dmaaddr_high_t;

/* Warning - This may not work for all architectures if HIGHMEM is defined */

#ifndef pci_map_page
#define pci_map_page(dev, page, offset, size, dir) \
	pci_map_single(dev, page_address(page) + (offset), size, dir)
#endif
#ifndef pci_unmap_page
#define pci_unmap_page(dev, map, size, dir) \
	pci_unmap_single(dev, map, size, dir)
#endif

#endif /* #if defined(CONFIG_HIGHMEM) && defined(CONFIG_X86) && ! defined(CONFIG_X86_64)*/

#endif /* #if (LINUX_VERSION_CODE >= 0x02040d)*/
#endif /* #if MAX_SKB_FRAGS*/

#if defined (CONFIG_X86) && ! defined (CONFIG_X86_64)
#define NO_PCI_UNMAP 1
#endif

#if (LINUX_VERSION_CODE < 0x020412)
#if ! defined (NO_PCI_UNMAP)
#define DECLARE_PCI_UNMAP_ADDR(ADDR_NAME) dma_addr_t ADDR_NAME;
#define DECLARE_PCI_UNMAP_LEN(LEN_NAME) __u32 LEN_NAME;

#define pci_unmap_addr(PTR, ADDR_NAME)	\
	((PTR)->ADDR_NAME)

#define pci_unmap_len(PTR, LEN_NAME)	\
	((PTR)->LEN_NAME)

#define pci_unmap_addr_set(PTR, ADDR_NAME, VAL)	\
	(((PTR)->ADDR_NAME) = (VAL))

#define pci_unmap_len_set(PTR, LEN_NAME, VAL)	\
	(((PTR)->LEN_NAME) = (VAL))
#else
#define DECLARE_PCI_UNMAP_ADDR(ADDR_NAME)
#define DECLARE_PCI_UNMAP_LEN(ADDR_NAME)

#define pci_unmap_addr(PTR, ADDR_NAME)	0
#define pci_unmap_len(PTR, LEN_NAME)	0
#define pci_unmap_addr_set(PTR, ADDR_NAME, VAL) do { } while (0)
#define pci_unmap_len_set(PTR, LEN_NAME, VAL) do { } while (0)
#endif
#endif

#if (LINUX_VERSION_CODE < 0x02030e)
#define net_device device
#define netif_carrier_on(dev)
#define netif_carrier_off(dev)
#endif

#if (LINUX_VERSION_CODE < 0x02032b)
#define tasklet_struct			tq_struct
#endif

typedef struct _UM_DEVICE_BLOCK {
	LM_DEVICE_BLOCK lm_dev;
	struct net_device *dev;
	struct pci_dev *pdev;
	struct net_device *next_module;
	char *name;
#ifdef BCM_PROC_FS
	struct proc_dir_entry *pfs_entry;
	char pfs_name[32];
#endif
	void *mem_list[MAX_MEM];
	dma_addr_t dma_list[MAX_MEM];
	int mem_size_list[MAX_MEM];
	int mem_list_num;
	int index;
	int opened;
	int suspended;
	int delayed_link_ind; /* Delay link status during initial load */
	int adapter_just_inited; /* the first few seconds after init. */
	int timer_interval;
	int adaptive_expiry;
	int crc_counter_expiry;
	int poll_tbi_expiry;
	int asf_heartbeat;
	int tx_full;
	int tx_queued;
	int line_speed;		/* in Mbps, 0 if link is down */
	UM_RX_PACKET_Q rx_out_of_buf_q;
	int rx_out_of_buf;
	int rx_buf_repl_thresh;
	int rx_buf_repl_panic_thresh;
	int rx_buf_align;
	struct timer_list timer;
	int do_global_lock;
	spinlock_t global_lock;
	spinlock_t undi_lock;
	spinlock_t phy_lock;
	unsigned long undi_flags;
	volatile unsigned long interrupt;
	atomic_t intr_sem;
	int tasklet_pending;
	volatile unsigned long tasklet_busy;
	struct tasklet_struct tasklet;
	struct net_device_stats stats;
#ifdef NICE_SUPPORT
	void (*nice_rx)( struct sk_buff*, void* );
	void* nice_ctx;
#endif /* NICE_SUPPORT */
#ifdef NETIF_F_HW_VLAN_TX
	struct vlan_group *vlgrp;
#endif
	int adaptive_coalesce;
	uint rx_last_cnt;
	uint tx_last_cnt;
	uint rx_curr_coalesce_frames;
	uint rx_curr_coalesce_ticks;
	uint tx_curr_coalesce_frames;
#if TIGON3_DEBUG
	uint tx_zc_count;
	uint tx_chksum_count;
	uint tx_himem_count;
	uint rx_good_chksum_count;
#endif
	uint rx_bad_chksum_count;
	uint rx_misc_errors;
} UM_DEVICE_BLOCK, *PUM_DEVICE_BLOCK;

typedef struct _UM_PACKET {
	LM_PACKET lm_packet;
	struct sk_buff *skbuff;
#if MAX_SKB_FRAGS
	DECLARE_PCI_UNMAP_ADDR(map[MAX_SKB_FRAGS + 1])
	DECLARE_PCI_UNMAP_LEN(map_len[MAX_SKB_FRAGS + 1])
#else
	DECLARE_PCI_UNMAP_ADDR(map[1])
	DECLARE_PCI_UNMAP_LEN(map_len[1])
#endif
} UM_PACKET, *PUM_PACKET;

static inline void MM_SetAddr(LM_PHYSICAL_ADDRESS *paddr, dma_addr_t addr)
{
#if (BITS_PER_LONG == 64)
	paddr->High = ((unsigned long) addr) >> 32;
	paddr->Low = ((unsigned long) addr) & 0xffffffff;
#else
	paddr->High = 0;
	paddr->Low = (unsigned long) addr;
#endif
}

static inline void MM_SetT3Addr(T3_64BIT_HOST_ADDR *paddr, dma_addr_t addr)
{
#if (BITS_PER_LONG == 64)
	paddr->High = ((unsigned long) addr) >> 32;
	paddr->Low = ((unsigned long) addr) & 0xffffffff;
#else
	paddr->High = 0;
	paddr->Low = (unsigned long) addr;
#endif
}

#if MAX_SKB_FRAGS
static inline void MM_SetT3AddrHigh(T3_64BIT_HOST_ADDR *paddr,
	dmaaddr_high_t addr)
{
#if defined(CONFIG_HIGHMEM64G) && defined(CONFIG_X86) && ! defined (CONFIG_X86_64)
	paddr->High = (unsigned long) (addr >> 32);
	paddr->Low = (unsigned long) (addr & 0xffffffff);
#else
	MM_SetT3Addr(paddr, (dma_addr_t) addr);
#endif
}
#endif

static inline void MM_MapRxDma(PLM_DEVICE_BLOCK pDevice,
	struct _LM_PACKET *pPacket,
	T3_64BIT_HOST_ADDR *paddr)
{
	dma_addr_t map;
	struct sk_buff *skb = ((struct _UM_PACKET *) pPacket)->skbuff;

	map = pci_map_single(((struct _UM_DEVICE_BLOCK *)pDevice)->pdev,
			skb->tail,
			pPacket->u.Rx.RxBufferSize,
			PCI_DMA_FROMDEVICE);
	pci_unmap_addr_set(((struct _UM_PACKET *) pPacket), map[0], map);
	MM_SetT3Addr(paddr, map);
}

static inline void MM_MapTxDma(PLM_DEVICE_BLOCK pDevice,
	struct _LM_PACKET *pPacket,
	T3_64BIT_HOST_ADDR *paddr,
	LM_UINT32 *len,
	int frag)
{
	dma_addr_t map;
	struct sk_buff *skb = ((struct _UM_PACKET *) pPacket)->skbuff;
	unsigned int length;

	if (frag == 0) {
#if MAX_SKB_FRAGS
		if (skb_shinfo(skb)->nr_frags)
			length = skb->len - skb->data_len;
		else
#endif
			length = skb->len;
		map = pci_map_single(((struct _UM_DEVICE_BLOCK *)pDevice)->pdev,
			skb->data, length, PCI_DMA_TODEVICE);
		MM_SetT3Addr(paddr, map);
		pci_unmap_addr_set(((struct _UM_PACKET *)pPacket), map[0], map);
		pci_unmap_len_set(((struct _UM_PACKET *) pPacket), map_len[0],
			length);
		*len = length;
	}
#if MAX_SKB_FRAGS
	else {
		skb_frag_t *sk_frag;
		dmaaddr_high_t hi_map;

		sk_frag = &skb_shinfo(skb)->frags[frag - 1];
			
		hi_map = pci_map_page(
				((struct _UM_DEVICE_BLOCK *)pDevice)->pdev,
				sk_frag->page,
				sk_frag->page_offset,
				sk_frag->size, PCI_DMA_TODEVICE);

		MM_SetT3AddrHigh(paddr, hi_map);
		pci_unmap_addr_set(((struct _UM_PACKET *) pPacket), map[frag],
			hi_map);
		pci_unmap_len_set(((struct _UM_PACKET *) pPacket),
			map_len[frag], sk_frag->size);
		*len = sk_frag->size;
	}
#endif
}

#define MM_ACQUIRE_UNDI_LOCK(_pDevice) \
	if (!(((PUM_DEVICE_BLOCK)(_pDevice))->do_global_lock)) {	\
		unsigned long flags;					\
		spin_lock_irqsave(&((PUM_DEVICE_BLOCK)(_pDevice))->undi_lock, flags);	\
		((PUM_DEVICE_BLOCK)(_pDevice))->undi_flags = flags; \
	}

#define MM_RELEASE_UNDI_LOCK(_pDevice) \
	if (!(((PUM_DEVICE_BLOCK)(_pDevice))->do_global_lock)) {	\
		unsigned long flags = ((PUM_DEVICE_BLOCK) (_pDevice))->undi_flags; \
		spin_unlock_irqrestore(&((PUM_DEVICE_BLOCK)(_pDevice))->undi_lock, flags); \
	}

#define MM_ACQUIRE_PHY_LOCK_IN_IRQ(_pDevice) \
	if (!(((PUM_DEVICE_BLOCK)(_pDevice))->do_global_lock)) {	\
		spin_lock(&((PUM_DEVICE_BLOCK)(_pDevice))->phy_lock);	\
	}

#define MM_RELEASE_PHY_LOCK_IN_IRQ(_pDevice) \
	if (!(((PUM_DEVICE_BLOCK)(_pDevice))->do_global_lock)) {	\
		spin_unlock(&((PUM_DEVICE_BLOCK)(_pDevice))->phy_lock); \
	}

#define MM_UINT_PTR(_ptr)   ((unsigned long) (_ptr))

#if (BITS_PER_LONG == 64)
#define MM_GETSTATS(_Ctr) \
	(unsigned long) (_Ctr).Low + ((unsigned long) (_Ctr).High << 32)
#else
#define MM_GETSTATS(_Ctr) \
	(unsigned long) (_Ctr).Low
#endif


#define DbgPrint(fmt, arg...) printk(KERN_WARNING fmt, ##arg)
#if defined(CONFIG_X86)
#define DbgBreakPoint() __asm__("int $129")
#else
#define DbgBreakPoint()
#endif
#define MM_Wait(time) udelay(time)

#endif
