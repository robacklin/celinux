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

#ifndef BANYAN_H
#define BANYAN_H

#include <linux/config.h>

#ifdef BANYAN_DEBUG
#define dbg(lvl, format, arg...) \
    if (banyan_debug > lvl) \
        printk(KERN_INFO "%s: " format, dev->name , ## arg)
#else
#define dbg(lvl, format, arg...) do {} while (0)
#endif
#define err(format, arg...) \
    printk(KERN_ERR "%s: " format, dev->name , ## arg)
#define info(format, arg...) \
    printk(KERN_INFO "%s: " format, dev->name , ## arg)
#define warn(format, arg...) \
    printk(KERN_WARNING "%s: " format, dev->name , ## arg)

#define ETH_DMA_RX_IRQ   GROUP1_IRQ_BASE + 9
#define ETH_DMA_TX_IRQ   GROUP1_IRQ_BASE + 10
#define ETH_RX_OVR_IRQ   GROUP3_IRQ_BASE + 22

/* Index to functions, as function prototypes. */

static int banyan_open(struct net_device *dev);
static int banyan_send_packet(struct sk_buff *skb, struct net_device *dev);
static void ban_ovr_interrupt(int irq, void *dev_id,
			      struct pt_regs * regs);
static void ban_rx_dma_interrupt(int irq, void *dev_id,
				 struct pt_regs * regs);
static void ban_tx_dma_interrupt(int irq, void *dev_id,
				 struct pt_regs * regs);
static void banyan_rx(struct net_device *dev);
static void banyan_tx(struct net_device *dev);
static int  banyan_close(struct net_device *dev);
static struct net_device_stats *banyan_get_stats(struct net_device *dev);
static void banyan_multicast_list(struct net_device *dev);
static int  banyan_init(struct net_device *dev);
static void banyan_tx_timeout(struct net_device *dev);

/* cgg - the following must be powers of two */
#define BANYAN_NUM_RDS    32    /* number of receive descriptors */
#define BANYAN_NUM_TDS    32    /* number of transmit descriptors */

#define BANYAN_RBSIZE     1536  /* size of one resource buffer = Ether MTU */
#define BANYAN_RDS_MASK   (BANYAN_NUM_RDS-1)
#define BANYAN_TDS_MASK   (BANYAN_NUM_TDS-1)
#define RD_RING_SIZE (BANYAN_NUM_RDS * sizeof(rc32355_dma_desc_t))
#define TD_RING_SIZE (BANYAN_NUM_TDS * sizeof(rc32355_dma_desc_t))

#define BANYAN_TX_TIMEOUT HZ/4

/* Information that need to be kept for each board. */
struct banyan_local {
	rc32355_eth_regs_t* eth_regs;
	rc32355_dma_ch_t* rx_dma_regs;
	rc32355_dma_ch_t* tx_dma_regs;
 	volatile rc32355_dma_desc_t * td_ring;  /* transmit descriptor ring */ 
	volatile rc32355_dma_desc_t * rd_ring;  /* receive descriptor ring  */

	u8*     rba;               /* start of rx buffer areas */  
	struct sk_buff* tx_skb[BANYAN_NUM_TDS]; /* skbuffs for pkt to trans */
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

#ifdef BANYAN_PROC_DEBUG
	/* debug /proc entry */
	struct proc_dir_entry *ps;
	int dma_halt_cnt;    u32 halt_tx_count;
	int dma_collide_cnt; u32 collide_tx_count;
	int dma_run_cnt;     u32 run_tx_count;
	int dma_race_cnt;    u32 race_tx_count;
#endif
};

#endif /* BANYAN_H */
