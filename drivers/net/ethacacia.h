/*
 *
 * BRIEF MODULE DESCRIPTION
 *      Helpfile for banyan.c
 *
 * Author: Steve Longerbeam <stevel@mvista.com, or source@mvista.com>
 *
 * Heavily modified from original version by:
 *
 * (C) 2001, IDT Inc.
 *
 * ########################################################################
 *
 * 2002 (c) MontaVista Software, Inc. This file is licensed under
 * the terms of the GNU General Public License version 2. This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 */

#ifndef ETHACACIA_H
#define ETHACACIA_H

#include <linux/config.h>
#include  <asm/rc32438/dma_v.h>
#include  <asm/rc32438/eth_v.h>

#ifdef ACACIA_DEBUG
#define dbg(lvl, format, arg...) \
    if (acacia_debug > lvl) \
        printk(KERN_INFO "%s: %s:" format, dev->name , __func__ , ## arg)
#else
#define dbg(lvl, format, arg...) do {} while (0)
#endif
#define err(format, arg...) \
    printk(KERN_ERR "%s: " format, dev->name , ## arg)
#define info(format, arg...) \
    printk(KERN_INFO "%s: " format, dev->name , ## arg)
#define warn(format, arg...) \
    printk(KERN_WARNING "%s: " format, dev->name , ## arg)
#if 0
#define GROUP3_IRQ_BASE   23
#define GROUP5_IRQ_BASE   39

#define ETH0_DMA_RX_IRQ   GROUP3_IRQ_BASE + 2
#define ETH0_DMA_TX_IRQ   GROUP3_IRQ_BASE + 3
#define ETH0_RX_OVR_IRQ   GROUP5_IRQ_BASE + 12
#define ETH1_DMA_RX_IRQ   GROUP3_IRQ_BASE + 4
#define ETH1_DMA_TX_IRQ   GROUP3_IRQ_BASE + 5
#define ETH1_RX_OVR_IRQ   GROUP5_IRQ_BASE + 15
#endif

#define ETH0_DMA_RX_IRQ   GROUP1_IRQ_BASE + 2
#define ETH0_DMA_TX_IRQ   GROUP1_IRQ_BASE + 3
#define ETH0_RX_OVR_IRQ   GROUP3_IRQ_BASE + 12
#define ETH1_DMA_RX_IRQ   GROUP1_IRQ_BASE + 4
#define ETH1_DMA_TX_IRQ   GROUP1_IRQ_BASE + 5
#define ETH1_RX_OVR_IRQ   GROUP3_IRQ_BASE + 15

/* Index to functions, as function prototypes. */

static int acacia_open(struct net_device *dev);
static int acacia_send_packet(struct sk_buff *skb, struct net_device *dev);
static void acacia_ovr_interrupt(int irq, void *dev_id,
			      struct pt_regs * regs);
static void acacia_rx_dma_interrupt(int irq, void *dev_id,
				 struct pt_regs * regs);
static void acacia_tx_dma_interrupt(int irq, void *dev_id,
				 struct pt_regs * regs);
static void acacia_rx(struct net_device *dev);
static void acacia_tx(struct net_device *dev);
static int  acacia_close(struct net_device *dev);
static struct net_device_stats *acacia_get_stats(struct net_device *dev);
static void acacia_multicast_list(struct net_device *dev);
static int  acacia_init(struct net_device *dev);
static void acacia_tx_timeout(struct net_device *dev);

/* cgg - the following must be powers of two */
#define ACACIA_NUM_RDS    32    /* number of receive descriptors */
#define ACACIA_NUM_TDS    32    /* number of transmit descriptors */

#define ACACIA_RBSIZE     1536  /* size of one resource buffer = Ether MTU */
#define ACACIA_RDS_MASK   (ACACIA_NUM_RDS-1)
#define ACACIA_TDS_MASK   (ACACIA_NUM_TDS-1)
#define RD_RING_SIZE (ACACIA_NUM_RDS * sizeof(struct DMAD_s))
#define TD_RING_SIZE (ACACIA_NUM_TDS * sizeof(struct DMAD_s))

#define ACACIA_TX_TIMEOUT HZ/4

#define rc32438_eth0_regs ((ETH_t)(ETH0_VirtualAddress);
#define rc32438_eth1_regs ((ETH_t)(ETH1_VirtualAddress);

/* Information that need to be kept for each board. */
struct acacia_local {
	ETH_t  eth_regs;
	DMA_Chan_t  rx_dma_regs;
	DMA_Chan_t  tx_dma_regs;
 	volatile DMAD_t   td_ring;  /* transmit descriptor ring */ 
	volatile DMAD_t   rd_ring;  /* receive descriptor ring  */

	u8*     rba;               /* start of rx buffer areas */  
	struct sk_buff* tx_skb[ACACIA_NUM_TDS]; /* skbuffs for pkt to trans */
	struct sk_buff* rx_skb[ACACIA_NUM_RDS]; /* skbuffs for pkt to trans */

	int     rx_next_out;
	int     tx_next_in;    	   /* next trans packet */
	int     tx_next_out;       /* last packet processed by ISR */
	int     tx_count;          /* current # of pkts waiting to be sent */

	int     tx_full;

	int     rx_irq;
	int     tx_irq;
	int     ovr_irq;

	struct net_device_stats stats;
	spinlock_t lock; /* Serialise access to device */


	/* debug /proc entry */
	struct proc_dir_entry *ps;
	int dma_halt_cnt;    u32 halt_tx_count;
	int dma_collide_cnt; u32 collide_tx_count;
	int dma_run_cnt;     u32 run_tx_count;
	int dma_race_cnt;    u32 race_tx_count;
};



#endif /* ETHACACIA_H */













