/*
 * acacia.c
 *
 * Author: Steve Longerbeam <stevel@mvista.com, or source@mvista.com>
 *
 * A driver for the IDT RC32438 on-chip ethernet controller.
 * 
 * This driver is modified from IDT:
 *
 * (C) 2001, IDT Inc.
 *
 * B Maruthanayagam  : Ported from sonic.c 
 * cgg               : Modified code.
 * P Sadik           : Applied filters.
 *   - Ported to MVL2.1 79EB355 lsp and brought forward to 2.4.17
 *     (Space.c obsolete, added module init/cleanup).
 *   - The entire struct banyan_local doesn't need to be in kseg1,
 *     just the descriptor rings should be dma coherent. The skb->data
 *     payloads and the receive buffers can be written-back and
 *     invalidated (respectively) at the appropriate times.
 *   - Added banyan_tx_timeout().
 *   - Added MAC address string module parameter and parsing.
 *   - The tx/rx index counters were not being wrapped around
 *     to the start of the rings, which was OK except when the
 *     index counters overflowed from 2^32-1 to 0.
 *   - No protection around critical code. Added spin_lock_irqsave/
 *     irqrestore around banyan_send_packet() and other places. This
 *     driver should be SMP-safe now.
 *   - Added a tx_count to make tx ring maintenance easier.
 *   - Added logic to stop/restart the tx queue when the tx ring
 *     becomes full/not full (tx_count >= BANYAN_NUM_TDS).
 *   - Original code was setting the link field of each new Tx descriptor
 *     to NULL, and was using NDPTR (chaining) to queue the new Tx packet.
 *     This limited the DMA list to at most only 2 linked descriptors. This
 *     also required a while(1) loop in send_packet() when the DMA was actively
 *     processing the desc pointed to by DPTR, and another desc was
 *     waiting in NDPTR (ie. the two-deep queue was full). A better solution
 *     is to initialize the Tx desc ring with the link pointers all NULL, but
 *     when adding a desc to the end of the Tx desc queue in send_packet(),
 *     the link pointer is set to point to the next (null) desc in the
 *     ring. When the DMA engine finishes processing the last desc in
 *     the queue, it jumps to a null desc, which has a count field of
 *     zero and link = NULL, so the DMA immediately finishes the null desc
 *     and halts. With this approach, the entire Tx descriptor ring is
 *     utilized to form a real h/w descriptor linked list. There are race
 *     conditions with the running DMA engine using this approach, but
 *     these have been characterized and are being handled in send_packet().
 *   - misc code cleanup.
 *   B Maruthanayagam : Ported it to acacia ethernet controller.
 *
 * ########################################################################
 *
 * 2002 (c) MontaVista Software, Inc. This file is licensed under
 * the terms of the GNU General Public License version 2. This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/ctype.h>
#include <linux/types.h>
#include <linux/fcntl.h>
#include <linux/interrupt.h>
#include <linux/ptrace.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/proc_fs.h>
#include <linux/in.h>
#include <linux/slab.h> 
#include <linux/string.h>
#include <linux/delay.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/skbuff.h>
#include <linux/errno.h>
#include <asm/bootinfo.h>
#include <asm/system.h>
#include <asm/bitops.h>
#include <asm/pgtable.h>
#include <asm/segment.h>
#include <asm/io.h>
#include <asm/dma.h>

#include "ethacacia.h"
#include  <asm/rc32438/rc32438.h> 

//#define ACACIA_DEBUG 3
#undef ACACIA_DEBUG
#undef ACACIA_PROC_DEBUG

/* use 0 for production, 1 for verification, >2 for debug */
#ifdef ACACIA_DEBUG
static int acacia_debug = ACACIA_DEBUG;
#endif



#define STATION_ADDRESS_HIGH(dev) (((dev)->dev_addr[0] << 8) | \
			           ((dev)->dev_addr[1]))
#define STATION_ADDRESS_LOW(dev)  (((dev)->dev_addr[2] << 24) | \
				   ((dev)->dev_addr[3] << 16) | \
				   ((dev)->dev_addr[4] << 8)  | \
				   ((dev)->dev_addr[5]))

#define MII_CLOCK 1250000 /* no more than 2.5MHz */
#define NINTFC		2 /* no of interface */
static char mac0[18] = "08:00:06:05:40:01"; 
static char mac1[18] = "08:00:06:05:50:01"; 
#if 0
unsigned char  Ether_Addr[6] = {0x08,0x00,0x06,0x05,0x40,0xff}; 
#endif
MODULE_PARM(mac0, "c18");
MODULE_PARM_DESC(mac0, "MAC address for RC32438 ethernet");

static struct acacia_if_t {
	struct net_device *dev;
	char* mac_str;
	u32 iobase;
	int rx_dma_irq;
	int tx_dma_irq;
	int rx_ovr_irq;
} acacia_iflist[] = {
	{NULL, mac0, ETH0_PhysicalAddress,
	 ETH0_DMA_RX_IRQ, ETH0_DMA_TX_IRQ, ETH0_RX_OVR_IRQ},
	{NULL, mac1, ETH1_PhysicalAddress,
	 ETH1_DMA_RX_IRQ, ETH1_DMA_TX_IRQ, ETH1_RX_OVR_IRQ},
	{NULL, 0, 0}
};

/* Index to functions, as function prototypes. */

int acacia_init_module(void);
static void acacia_cleanup_module(void);
int acacia_probe(int port_num);

static int
parse_mac_addr(struct net_device *dev, char* macstr)
{
	int i, j;
	unsigned char result, value;
		
	for (i=0; i<6; i++) {
		result = 0;
		if (i != 5 && *(macstr+2) != ':') {
			err(__FILE__ "invalid mac address format: %d %c\n",
			    i, *(macstr+2));
			return -EINVAL;
		}
				
		for (j=0; j<2; j++) {
			if (isxdigit(*macstr) &&
			    (value = isdigit(*macstr) ? *macstr-'0' : 
			     toupper(*macstr)-'A'+10) < 16) {
				result = result*16 + value;
				macstr++;
			} else {
				err(__FILE__ "invalid mac address "
				    "character: %c\n", *macstr);
				return -EINVAL;
			}
		}
				
		macstr++; // step over ':'
		dev->dev_addr[i] = result;
	}
		
	return 0;
}


static inline void acacia_halt_tx(struct net_device *dev)
{
	struct acacia_local *lp = (struct acacia_local *)dev->priv;
	if (rc32438_halt_dma(lp->tx_dma_regs))
		err("%s: timeout!\n", __func__);
}
static inline void acacia_halt_rx(struct net_device *dev)
{
	struct acacia_local *lp = (struct acacia_local *)dev->priv;
	if (rc32438_halt_dma(lp->rx_dma_regs))
		err("%s: timeout!\n", __func__);
}

static inline unsigned long kseg1_to_phys(volatile void * kseg1_addr)
{
	return virt_to_phys((void*)KSEG0ADDR(kseg1_addr));
}

static inline void acacia_start_tx(struct acacia_local *lp,
				   volatile DMAD_t td)
{
	rc32438_start_dma(lp->tx_dma_regs, kseg1_to_phys(td));
}
static inline void acacia_start_rx(struct acacia_local *lp,
				   volatile DMAD_t rd)
{
	rc32438_start_dma(lp->rx_dma_regs, kseg1_to_phys(rd));
}

static inline void acacia_chain_tx(struct acacia_local *lp,
				   volatile DMAD_t td)
{
	rc32438_chain_dma(lp->tx_dma_regs, kseg1_to_phys(td));
}
static inline void acacia_chain_rx(struct acacia_local *lp,
				   volatile DMAD_t rd)
{
	rc32438_chain_dma(lp->rx_dma_regs, kseg1_to_phys(rd));
}

#ifdef ACACIA_PROC_DEBUG
static int acacia_read_proc(char *buf, char **start, off_t fpos,
			    int length, int *eof, void *data)
{
	struct net_device *dev = (struct net_device *)data;
	struct acacia_local *lp = (struct acacia_local *)dev->priv;
	int len = 0;

	/* print out header */
	len += sprintf(buf + len, "\n\tRC32438 Ethernet Debug\n\n");

	len += sprintf (buf + len,
			"DMA halt count      = %10d, total pkt cnt = %10d\n",
			lp->dma_halt_cnt, lp->halt_tx_count);
	len += sprintf (buf + len,
			"DMA run count       = %10d, total pkt cnt = %10d\n",
			lp->dma_run_cnt, lp->run_tx_count);
	len += sprintf (buf + len,
			"DMA race count      = %10d, total pkt cnt = %10d\n",
			lp->dma_race_cnt, lp->race_tx_count);
	len += sprintf (buf + len,
			"DMA collision count = %10d, total pkt cnt = %10d\n",
			lp->dma_collide_cnt, lp->collide_tx_count);

	if (fpos >= len) {
		*start = buf;
		*eof = 1;
		return 0;
	}
	*start = buf + fpos;
	if ((len -= fpos) > length)
		return length;
	*eof = 1;
	return len;

}
#endif


// Returns 1 if the Tx counter and indeces don't gel
static int
acacia_check_tx_consistent(struct acacia_local *lp)
{
	int diff = lp->tx_next_in - lp->tx_next_out;

	diff = diff<0 ? ACACIA_NUM_TDS + diff : diff;
	diff = (lp->tx_count == ACACIA_NUM_TDS) ?
		diff + ACACIA_NUM_TDS : diff;
    
	return (diff != lp->tx_count);
}

/*
 * Restart the ACACIA ethernet controller. Hold a spin lock
 * before calling.
 */
static int acacia_restart(struct net_device *dev)
{
	struct acacia_local *lp = (struct acacia_local *)dev->priv;
	
	/*
	 * stop the BANYAN, disable interrupts
	 */
	disable_irq(lp->rx_irq);
	disable_irq(lp->tx_irq);
	disable_irq(lp->ovr_irq);
		
	/* Disable F bit in Tx DMA */
	writel(readl(&lp->tx_dma_regs->dmasm) | DMAS_f_m,
	       &lp->tx_dma_regs->dmasm);
     
	/* Disable D bit in Tx DMA */
	writel(readl(&lp->rx_dma_regs->dmasm) | DMAS_d_m,
	       &lp->rx_dma_regs->dmasm);

	acacia_init(dev);

	enable_irq(lp->ovr_irq);
	enable_irq(lp->tx_irq);
	enable_irq(lp->rx_irq);

	return 0;
}

int acacia_init_module(void)
{
	int i, retval=0;

	for (i = 0; acacia_iflist[i].iobase; i++) { 
		retval |= acacia_probe(i);
	} 

	return retval;
}

int acacia_probe(int port_num)
{
	struct acacia_local *lp = NULL;
	struct acacia_if_t *bif = NULL;
	struct net_device *dev = NULL;
	int i, retval;
	bif = &acacia_iflist[port_num];
        if (port_num == 0){
           
		request_region(bif->iobase, 0x24C, "ACACIA0");
        }
        else 
		if (port_num == 1)
		{
			request_region(bif->iobase, 0x24C, "ACACIA1");
		}
	/* Allocate a new 'dev' if needed */
	dev = init_etherdev(0, sizeof(struct acacia_local));
	bif->dev = dev;
	
	if (port_num == 0){
		info("RC32438 ethernet0 found at 0x%08x\n", bif->iobase);
        }
        else if (port_num == 1)
		info("RC32438 ethernet1 found at 0x%08x\n", bif->iobase);

	/* Fill in the 'dev' fields. */
	dev->base_addr = bif->iobase;
	dev->irq = bif->rx_dma_irq; /* just use the rx dma irq */

	if ((retval = parse_mac_addr(dev, bif->mac_str))) {
		err("%s: MAC address parse failed\n", __func__);
		retval = -EINVAL;
		goto probe1_err_out;
	}

	info("HW Address ");
	for (i = 0; i < 6; i++) {
		printk("%2.2x", dev->dev_addr[i]);
		if (i<5)
			printk(":");
	}
	printk("\n");
    
	info("Rx IRQ %d, Tx IRQ %d\n", bif->rx_dma_irq, bif->tx_dma_irq);
    
	/* Initialize the device structure. */
	if (dev->priv == NULL) {
		lp = (struct acacia_local *)kmalloc(sizeof(*lp), GFP_KERNEL);
		memset(lp, 0, sizeof(struct acacia_local));
	} else {
		lp = (struct acacia_local *)dev->priv;
	}

	dev->priv = lp;
		
	lp->rx_irq = bif->rx_dma_irq;
	lp->tx_irq = bif->tx_dma_irq;
	lp->ovr_irq = bif->rx_ovr_irq;
		
	lp->eth_regs = ioremap_nocache(bif->iobase,
				       sizeof(*lp->eth_regs));
	if (!lp->eth_regs) {
		err("Can't remap eth registers\n");
		retval = -ENXIO;
		goto probe1_err_out;
	}
	if (port_num == 0){	
		lp->rx_dma_regs =
			ioremap_nocache(DMA0_PhysicalAddress + 2*DMA_CHAN_OFFSET,
					sizeof(struct DMA_Chan_s));
		if (!lp->rx_dma_regs) {
			err("Can't remap Rx DMA registers\n");
			retval = -ENXIO;
			goto probe1_err_out;
		}
		
		lp->tx_dma_regs =
			ioremap_nocache(DMA0_PhysicalAddress  + 3*DMA_CHAN_OFFSET,
					sizeof(struct DMA_Chan_s));
		if (!lp->tx_dma_regs) {
			err("Can't remap Tx DMA registers\n");
			retval = -ENXIO;
			goto probe1_err_out;
		}
	}
        else if (port_num == 1){
		lp->rx_dma_regs =
			ioremap_nocache(DMA0_PhysicalAddress  + 4*DMA_CHAN_OFFSET,
					sizeof(struct DMA_Chan_s));
		if (!lp->rx_dma_regs) {
			err("Can't remap Rx DMA registers\n");
			retval = -ENXIO;
			goto probe1_err_out;
		}
		
		lp->tx_dma_regs =
			ioremap_nocache(DMA0_PhysicalAddress  + 5*DMA_CHAN_OFFSET,
					sizeof(struct DMA_Chan_s));
		if (!lp->tx_dma_regs) {
			err("Can't remap Tx DMA registers\n");
			retval = -ENXIO;
			goto probe1_err_out;
		}
       
        }	
	lp->td_ring =
		(DMAD_t)kmalloc(TD_RING_SIZE + RD_RING_SIZE,
				GFP_KERNEL);
	if (!lp->td_ring) {
		err("Can't allocate descriptors\n");
		retval = -ENOMEM;
		goto probe1_err_out;
	}

	dma_cache_inv((unsigned long)(lp->td_ring),
		      TD_RING_SIZE + RD_RING_SIZE);

	/* now convert TD_RING pointer to KSEG1 */
	lp->td_ring = (DMAD_t )KSEG1ADDR(lp->td_ring);
	lp->rd_ring = &lp->td_ring[ACACIA_NUM_TDS];
	
	/* allocate receive buffer area */
	/* FIXME, maybe we should use skbs */
	if ((lp->rba = (u8*)kmalloc(ACACIA_NUM_RDS * ACACIA_RBSIZE,
				    GFP_KERNEL)) == NULL) {
		err("couldn't allocate receive buffers\n");
		retval = -ENOMEM; 	          
		goto probe1_err_out;
	} 	 	/* get virtual dma address */ 
		
	dma_cache_inv((unsigned long)(lp->rba),
		      ACACIA_NUM_RDS * ACACIA_RBSIZE);
		
	spin_lock_init(&lp->lock);

	dev->open = acacia_open;
	dev->stop = acacia_close;
	dev->hard_start_xmit = acacia_send_packet;
	dev->get_stats	= acacia_get_stats;
	dev->set_multicast_list = &acacia_multicast_list;
	dev->tx_timeout = acacia_tx_timeout;
	dev->watchdog_timeo = ACACIA_TX_TIMEOUT;

#ifdef ACACIA_PROC_DEBUG
	lp->ps = create_proc_read_entry ("net/rc32438", 0, NULL,
					 acacia_read_proc, dev);
#endif
	/*
	 * clear tally counter
	 */

	/* Fill in the fields of the device structure with ethernet values. */
	ether_setup(dev);
	return 0;

 probe1_err_out:
	acacia_cleanup_module();
	err("%s failed.  Returns %d\n", __func__, retval);
	return retval;
}


/*
 * Open/initialize the ACACIA controller.
 *
 * This routine should set everything up anew at each open, even
 *  registers that "should" only need to be set once at boot, so that
 *  there is non-reboot way to recover if something goes wrong.
 */

static int acacia_open(struct net_device *dev)
{
	struct acacia_local *lp = (struct acacia_local *)dev->priv;

	MOD_INC_USE_COUNT;
	/*
	 * Initialize
	 */
	acacia_init(dev);
	
	/*
	 * Install the interrupt handler that handles the dma Done and
	 * Finished Events.
	 */
	
	if (request_irq(lp->rx_irq, &acacia_rx_dma_interrupt,
			SA_SHIRQ | SA_INTERRUPT,
			"banyan ethernet Rx", dev)) {
		err("%s: unable to get Rx DMA IRQ %d\n",
		    __func__, lp->rx_irq);
		MOD_DEC_USE_COUNT;
		return -EAGAIN;
	}
	if (request_irq(lp->tx_irq, &acacia_tx_dma_interrupt,
			SA_SHIRQ | SA_INTERRUPT,
			"banyan ethernet Tx", dev)) {
		err("%s: unable to get Tx DMA IRQ %d\n",
		    __func__, lp->tx_irq);
		free_irq(lp->rx_irq, dev);
		MOD_DEC_USE_COUNT;
		return -EAGAIN;
	}

	/* Install handler for overrun error. */
	if (request_irq(lp->ovr_irq, &acacia_ovr_interrupt,
			SA_SHIRQ | SA_INTERRUPT,
			"Ethernet Overflow", dev)) {
		err("%s: unable to get OVR IRQ %d\n",
		    __func__, lp->ovr_irq);
		free_irq(lp->rx_irq, dev);
		free_irq(lp->tx_irq, dev);
		MOD_DEC_USE_COUNT;
		return -EAGAIN;
	}
	


	return 0;
}


/*
 * Close the BANYAN device
 */
static int
acacia_close(struct net_device *dev)
{
	struct acacia_local *lp = (struct acacia_local *)dev->priv;
	u32 tmp;
    
	/*
	 * stop the ACACIA, disable interrupts
	 */
	disable_irq(lp->rx_irq);
	disable_irq(lp->tx_irq);
	disable_irq(lp->ovr_irq);
		
	tmp = readl(&lp->tx_dma_regs->dmasm);
	tmp = tmp | DMASM_f_m | DMASM_e_m;
	writel(tmp, &lp->tx_dma_regs->dmasm);
     
	tmp = readl(&lp->rx_dma_regs->dmasm);
	tmp = tmp | DMASM_d_m | DMASM_h_m | DMASM_e_m;
	writel(tmp, &lp->rx_dma_regs->dmasm);

	free_irq(lp->rx_irq, dev);
	free_irq(lp->tx_irq, dev);
	free_irq(lp->ovr_irq, dev);

	MOD_DEC_USE_COUNT;
	return 0;
}


/* transmit packet */
static int acacia_send_packet(struct sk_buff *skb, struct net_device *dev)
{
	struct acacia_local *lp = (struct acacia_local *)dev->priv;
	volatile DMAD_t td;
	struct DMAD_s local_td;
	unsigned long flags;
	int tx_next_in, empty_index;
	u32 laddr, length;
	
	spin_lock_irqsave(&lp->lock, flags);

	if (lp->tx_count >= ACACIA_NUM_TDS) {
		err("Tx ring full, packet dropped\n");
		lp->tx_full = 1;
		lp->stats.tx_dropped++;
		spin_unlock_irqrestore(&lp->lock, flags);
		return 1;
	}

	tx_next_in = lp->tx_next_in;
	td = &lp->td_ring[tx_next_in];
	empty_index = (tx_next_in + 1) & ACACIA_TDS_MASK;
	
	if (!IS_DMA_USED(td->control)) {
		err("%s: device owns descriptor, i/f reset\n", __func__);
		lp->stats.tx_errors++;
		lp->stats.tx_dropped++;
		acacia_restart(dev);     /* Restart interface */
		spin_unlock_irqrestore(&lp->lock, flags);
		return 1;
	}
	
	laddr = virt_to_phys(skb->data);  
	/* make sure payload gets written to memory */

	dma_cache_inv((unsigned long)skb->data, skb->len);

	if (lp->tx_skb[tx_next_in] != NULL)
		dev_kfree_skb_any(lp->tx_skb[tx_next_in]);
	lp->tx_skb[tx_next_in] = skb;
    
	length = (skb->len < ETH_ZLEN) ? ETH_ZLEN : skb->len;

	/*
	 * Setup the transmit descriptor.
	 */
	local_td.devcs = ETHTX_fd_m | ETHTX_ld_m;
	local_td.ca = laddr;    
	local_td.control = DMAD_iod_m | DMAD_iof_m | DMA_COUNT(length);
	local_td.link = kseg1_to_phys(&lp->td_ring[empty_index]);
	
	if (!(readl(&lp->tx_dma_regs->dmac) & DMAC_run_m)) {
		/*
		 * DMA is halted, just update the td and go. Note that
		 * the dptr will *always* be stopped at this td, so
		 * there won't be a linked list left (this has been
		 * verified too).
		 */
		*td = local_td;
		acacia_start_tx(lp, td);
#ifdef ACACIA_PROC_DEBUG
		lp->dma_halt_cnt++;
		lp->halt_tx_count += lp->tx_count;
#endif
	} else if (readl(&lp->tx_dma_regs->dmadptr) != kseg1_to_phys(td)) {
		/*
		 * DMA is running but not on this td presently. There
		 * is a race condition right here. The DMA may
		 * have moved to this td just after the above 'if'
		 * statement, and reads the td from memory just before
		 * we update it on the next line. So check if DMA
		 * has since moved to this td while we updated it.
		 */
		*td = local_td;
		if (readl(&lp->tx_dma_regs->dmadptr) == kseg1_to_phys(td)) {
			dbg(2, "DMA race detected\n");
			acacia_halt_tx(dev);
			*td = local_td;
			acacia_start_tx(lp, td);
#ifdef ACACIA_PROC_DEBUG
			lp->dma_race_cnt++;
			lp->race_tx_count += lp->tx_count;
		} else {
			lp->dma_run_cnt++;
			lp->run_tx_count += lp->tx_count;
#endif
		}
	} else {
		/*
		 * DMA is running (or was running) and is presently
		 * processing this td, so stop the DMA from what
		 * it's doing, update the td and start again.
		 */
		acacia_halt_tx(dev);
		*td = local_td;
		acacia_start_tx(lp, td);
#ifdef ACACIA_PROC_DEBUG
		lp->dma_collide_cnt++;
		lp->collide_tx_count += lp->tx_count;
#endif
	}

	dev->trans_start = jiffies;

	/* increment nextIn index */
 	lp->tx_next_in = empty_index;
	// increment count and stop queue if full
	if (++lp->tx_count == ACACIA_NUM_TDS) {
		lp->tx_full = 1;
		netif_stop_queue(dev);
		err("Tx Ring now full, queue stopped.\n");
	} 

	lp->stats.tx_bytes += length;

	spin_unlock_irqrestore(&lp->lock, flags);

	return 0;
}


/* Ethernet Rx Overflow interrupt */
static void acacia_ovr_interrupt(int irq, void *dev_id, struct pt_regs * regs)
{
	struct net_device *dev = (struct net_device *)dev_id;
	struct acacia_local *lp;

	if (dev == NULL) {
		err("%s: irq %d for unknown device.\n", __func__, irq);
		return;
	}


	lp = (struct acacia_local *)dev->priv;

	spin_lock(&lp->lock);
#if 0
	writel(0, &lp->eth_regs->ethfifos); /* Reset ethfifos register */
#endif
	acacia_restart(dev);     /* Restart interface */
	spin_unlock(&lp->lock);

	err("Rx overflow - i/f reset\n");
}


/* Ethernet Rx DMA interrupt */
static void
acacia_rx_dma_interrupt(int irq, void *dev_id, struct pt_regs * regs)
{
	struct net_device *dev = (struct net_device *)dev_id;
	struct acacia_local *lp;
	u32 dmas;
	
	if (dev == NULL) {
		err("%s: irq %d for unknown device.\n", __func__, irq);
		return;
	}

	lp = (struct acacia_local *)dev->priv;

	spin_lock(&lp->lock);

	dmas = readl(&lp->rx_dma_regs->dmas);

	if (dmas & DMAS_f_m) {
		err("%s: Rx DMA Finished? i/f reset\n", __func__);
		acacia_restart(dev);     /* Restart interface */
	} else if (dmas & DMAS_d_m) {
		writel(0, &lp->rx_dma_regs->dmas);  
		acacia_rx(dev);
	} else if (dmas & DMAS_e_m) {
		err("%s: DMA error\n", __func__);
		writel(0, &lp->rx_dma_regs->dmas);  
		lp->stats.rx_errors++;
	}

	spin_unlock(&lp->lock);
}

/* Ethernet Tx DMA interrupt */
static void
acacia_tx_dma_interrupt(int irq, void *dev_id, struct pt_regs * regs)
{
	struct net_device *dev = (struct net_device *)dev_id;
	struct acacia_local *lp;
	u32 dmas;
	
	if (dev == NULL) {
		err("%s: irq %d for unknown device.\n", __func__, irq);
		return;
	}

	lp = (struct acacia_local *)dev->priv;

	spin_lock(&lp->lock);

	dmas = readl(&lp->tx_dma_regs->dmas);
	writel(0, &lp->tx_dma_regs->dmas); 

	if (dmas & DMAS_e_m)
		err("%s: DMA error\n", __func__);
	if (dmas & DMAS_f_m)
		acacia_tx(dev);

	spin_unlock(&lp->lock);
}


static void
acacia_tx(struct net_device *dev)
{
	struct acacia_local* lp = (struct acacia_local *)dev->priv;
	volatile DMAD_t td = &lp->td_ring[lp->tx_next_out];
	struct DMAD_s ltd;

	/* cgg - see what has happened to each transmission in backlog */
	while (lp->tx_count && IS_DMA_USED(td->control)) {

		ltd = *td;
		td->devcs = 0;
		td->ca = 0;
		td->link = 0;
		
		if (acacia_check_tx_consistent(lp)) {
			err("%s: tx queue inconsistent, i/f reset\n",
			    __func__);
			err("%s:    nI=%d, nO=%d, cnt=%d\n", __func__,
			    lp->tx_next_in, lp->tx_next_out, lp->tx_count);
			lp->stats.tx_errors++;
			acacia_restart(dev);
			break;
		}
	
		if ((ltd.devcs & 0xffffffc0) == 0) {
			/* last transmission not complete? */
			dbg(0, "no devcs status\n");
			dbg(0, "    dptr=%08x, td=%08x\n",
			    readl(&lp->tx_dma_regs->dmadptr),
			    (u32)kseg1_to_phys(td));
			dbg(0, "    cmdstat=%08x, ca=%08x, "
			    "devcs=%08x, link=%08x\n",
			    ltd.cmdstat, ltd.curr_addr, ltd.devcs, ltd.link);
			dbg(0, "    nI=%d, nO=%d, cnt=%d\n",
			    lp->tx_next_in, lp->tx_next_out, lp->tx_count);
			lp->stats.tx_errors++;
		} else if ((ltd.devcs & (ETHTX_fd_m | ETHTX_ld_m)) !=
			   (ETHTX_fd_m | ETHTX_ld_m)) {
			lp->stats.tx_errors++;
			/* should never happen */
			dbg(1, "split tx ignored\n");
		} else if (IS_TX_TOK(ltd.devcs)) {
			/* transmit OK */
			lp->stats.tx_packets++;
		} else {
			dbg(0, "error, devcs=0x%08x\n",
			    ltd.devcs);

			lp->stats.tx_errors++;
						
			/* underflow */
			if (IS_TX_UND_ERR(ltd.devcs)) 
				lp->stats.tx_fifo_errors++;
						
			/* oversized frame */
			if (IS_TX_OF_ERR(ltd.devcs))
				lp->stats.tx_aborted_errors++;

			/* excessive deferrals */
			if (IS_TX_ED_ERR(ltd.devcs))
				lp->stats.tx_carrier_errors++;
						
			/* collisions: medium busy */
			if (IS_TX_EC_ERR(ltd.devcs))
				lp->stats.collisions++;

			/* late collision */
			if (IS_TX_LC_ERR(ltd.devcs))
				lp->stats.tx_window_errors++;
		}
				
		// Wake the queue if the ring was full
		if (lp->tx_full) {
			lp->tx_full = 0;
			netif_wake_queue(dev);
			err("%s: Tx Ring was full, queue waked\n", __func__);
		}
		
		/* We must always free the original skb */
		if (lp->tx_skb[lp->tx_next_out] != NULL) {
			dev_kfree_skb_irq(lp->tx_skb[lp->tx_next_out]);
			lp->tx_skb[lp->tx_next_out] = NULL;
		}

		// decrement tx ring buffer count
		lp->tx_count--;
		/* go on to next transmission */
		lp->tx_next_out = (lp->tx_next_out + 1) & ACACIA_TDS_MASK;
		td = &lp->td_ring[lp->tx_next_out];
	}
}

/*
 * We have a good packet(s), get it/them out of the buffers.
 *
 * cgg - this driver works by creating (once) a circular list of receiver
 *       DMA descriptors that will be used serially by the Banyan.
 *       Because the descriptors are never unlinked from the list _they
 *       are always live_.  We are counting on Linux (and the chosen number
 *	 of buffers) to keep ahead of the hardware otherwise the same
 *	 descriptor might be used for more than one reception.
 */
static void
acacia_rx(struct net_device *dev)
{
	struct acacia_local* lp = (struct acacia_local *)dev->priv;
	volatile DMAD_t  rd = &lp->rd_ring[lp->rx_next_out];
	struct sk_buff *skb;
	u8* pkt_buf;
	u32 devcs;
	u32 count, pkt_len;
	
	/* cgg - keep going while we have received into more descriptors */
	
	while (IS_DMA_USED(rd->control)) {
		
		devcs = rd->devcs;

		pkt_len = RCVPKT_LENGTH(devcs);

		pkt_buf = &lp->rba[lp->rx_next_out * ACACIA_RBSIZE];

		/*
		 * cgg - RESET the address pointer later - if we get a second
		 * reception it will occur in the remains of the current
		 * area of memory - protected by the diminished DMA count.
		 */

		/*
		 * Due to a bug in banyan processor, the packet length
		 * given by devcs field and count field sometimes differ.
		 * If that is the case, report Error.
		 */
		count = ACACIA_RBSIZE - (u32)DMA_COUNT(rd->control);
		if( count != pkt_len) {
			lp->stats.rx_errors++;
		} else if (count < 64) {
			lp->stats.rx_errors++;
		} else if ((devcs & (/*ETHERDMA_IN_FD |*/ ETHRX_ld_m)) !=
			   (/*ETHERDMA_IN_FD |*/ ETHRX_ld_m)) {
			/* cgg - check that this is a whole packet */
			/* WARNING: DMA_FD bit incorrectly set in Acacia
			   (errata ref #077) */
			lp->stats.rx_errors++;
			lp->stats.rx_over_errors++;
		} else if (devcs & ETHRX_rok_m) {
			/* must be the (first and) last descriptor then */

			/* Malloc up new buffer. */
			skb = dev_alloc_skb(pkt_len+2);
			if (skb == NULL) {
				err("no memory, dropping rx packet.\n");
				lp->stats.rx_dropped++;
			} else {
				/* else added by cgg - used to fall through! */
				/* invalidate the cache before copying
				   the buffer */
				dma_cache_inv((unsigned long)pkt_buf, pkt_len);

				skb->dev = dev;
				skb_reserve(skb, 2);	/* 16 bit align */
				skb_put(skb, pkt_len);	/* Make room */
				eth_copy_and_sum(skb, pkt_buf, pkt_len, 0);
				skb->protocol = eth_type_trans(skb, dev);
				/* pass the packet to upper layers */
				netif_rx(skb);
				dev->last_rx = jiffies;
				lp->stats.rx_packets++;
				lp->stats.rx_bytes += pkt_len;
								
				if (IS_RCV_MP(devcs))
					lp->stats.multicast++;
			}
						
		} else {
			/* This should only happen if we enable
			   accepting broken packets */
			lp->stats.rx_errors++;
						
			/* cgg - (re-)added statistics counters */
			if (IS_RCV_CRC_ERR(devcs)) {
				dbg(2, "RX CRC error\n");
				lp->stats.rx_crc_errors++;
			} else {	
				if (IS_RCV_LOR_ERR(devcs)) {
					dbg(2, "RX LOR error\n");
					lp->stats.rx_length_errors++;
				}
				
				if (IS_RCV_LE_ERR(devcs)) {
					dbg(2, "RX LE error\n");
					lp->stats.rx_length_errors++;
				}
			}
			
			if (IS_RCV_OVR_ERR(devcs)) {
				/*
				 * The overflow errors are handled through
				 * an interrupt handler.
				 */
				lp->stats.rx_over_errors++;
			}
			/* code violation */
			if (IS_RCV_CV_ERR(devcs)) {
				dbg(2, "RX CV error\n");
				lp->stats.rx_frame_errors++;
			}
			
			if (IS_RCV_CES_ERR(devcs)) {
				dbg(2, "RX Preamble error\n");
			}
		}
		

		/* reset descriptor's curr_addr */
		rd->ca = virt_to_phys(pkt_buf);

		/*
		 * cgg - clear the bits that let us see whether this
		 * descriptor has been used or not & reset reception
		 * length.
		 */
		rd->control = DMAD_iod_m | DMA_COUNT(ACACIA_RBSIZE);
		rd->devcs = 0;
		lp->rx_next_out = (lp->rx_next_out + 1) & ACACIA_RDS_MASK;
		rd = &lp->rd_ring[lp->rx_next_out];

		/*
		 * we'll deal with all possible interrupts up to the last
		 * used descriptor - so cancel any interrupts that may have
		 * arrisen while we've been processing.
		 */
		writel(0, &lp->rx_dma_regs->dmas);
	} 
		
	/*
	 * If any worth-while packets have been received, dev_rint()
	 * has done a mark_bh(NET_BH) for us and will work on them
	 * when we get to the bottom-half routine.
	 */
}


/*
 * Get the current statistics.
 * This may be called with the device open or closed.
 */
static struct net_device_stats *
acacia_get_stats(struct net_device *dev)
{
	struct acacia_local *lp = (struct acacia_local *)dev->priv;
	return &lp->stats;
}


/*
 * Set or clear the multicast filter for this adaptor.
 */
static void
acacia_multicast_list(struct net_device *dev)
{   /* cgg - changed to listen to broadcasts always and to treat 	*/
	/*       IFF bits independantly	*/
	struct acacia_local *lp = (struct acacia_local *)dev->priv;
	unsigned long flags;
	u32 recognise = ETHARC_ab_m; /* always accept broadcasts */

	if (dev->flags & IFF_PROMISC)         /* set promiscuous mode */
		recognise |= ETHARC_pro_m;

	if ((dev->flags & IFF_ALLMULTI) || (dev->mc_count > 15))
		recognise |= ETHARC_am_m;    	  /* all multicast & bcast */
	
	else if (dev->mc_count > 0) {
		dbg(2, "mc_count %d\n", dev->mc_count);
		
		recognise |= ETHARC_am_m;    	  /* for the time being */
		/* recognise |= ETHARC_afm_m; */
		/* cgg - need to set up multicast list here, surely? */
	}
	
	spin_lock_irqsave(&lp->lock, flags);
	writel(recognise, &lp->eth_regs->etharc);
	spin_unlock_irqrestore(&lp->lock, flags);
}


static void
acacia_tx_timeout(struct net_device *dev)
{
	struct acacia_local *lp = (struct acacia_local *)dev->priv;
	unsigned long flags;
	
	spin_lock_irqsave(&lp->lock, flags);
	acacia_restart(dev);
	spin_unlock_irqrestore(&lp->lock, flags);

	err("%s: i/f reset\n", __func__);
	
	netif_wake_queue(dev);
}


/*
 * Initialize the BANYAN ethernet controller.
 */
static int acacia_init(struct net_device *dev)
{
	struct acacia_local *lp = (struct acacia_local *)dev->priv;
	int i;

	/* Disable DMA */
	acacia_halt_tx(dev);
	acacia_halt_rx(dev);

	/* reset ethernet logic */ 
	writel(0, &lp->eth_regs->ethintfc);
 
	i = readl(&lp->eth_regs->ethintfc); 
	for(i = 0xfffff; i>0 ;i--) { 
		if (!(readl(&lp->eth_regs->ethintfc) & ETHINTFC_rip_m)) 
			break; 
	} 
	/* Enable Ethernet Interface */ 
	writel(ETHINTFC_en_m, &lp->eth_regs->ethintfc); 
	/* Fifo Tx Threshold level */ 

	/* Accept only packets destined for this Ethernet device address */
	/* cgg - and broadcasts */
	writel(ETHARC_ab_m, &lp->eth_regs->etharc); 
 
	/* Set all Ether station address registers to their initial values */ 
	writel(STATION_ADDRESS_LOW(dev), &lp->eth_regs->ethsal0); 
	writel(STATION_ADDRESS_HIGH(dev), &lp->eth_regs->ethsah0);
	
	writel(STATION_ADDRESS_LOW(dev), &lp->eth_regs->ethsal1); 
	writel(STATION_ADDRESS_HIGH(dev), &lp->eth_regs->ethsah1);
	
	writel(STATION_ADDRESS_LOW(dev), &lp->eth_regs->ethsal2); 
	writel(STATION_ADDRESS_HIGH(dev), &lp->eth_regs->ethsah2);
	
	writel(STATION_ADDRESS_LOW(dev), &lp->eth_regs->ethsal3); 
	writel(STATION_ADDRESS_HIGH(dev), &lp->eth_regs->ethsah3); 
 
	/* Input Ready threshold = 16, output Ready threshold = 16 */ 
#if 0
	writel((0x10 << 16) + 16, &lp->eth_regs->ethfifost); 
#endif
	/* Frame Length Checking, Pad Enable, CRC Enable, Full Duplex set */ 
	writel(ETHMAC2_flc_m | ETHMAC2_pe_m | ETHMAC2_cen_m | ETHMAC2_fd_m,
	       &lp->eth_regs->ethmac2);  

	/* Back to back inter-packet-gap */ 
	writel(0x15, &lp->eth_regs->ethipgt); 
	/* Non - Back to back inter-packet-gap */ 
	writel(0x12, &lp->eth_regs->ethipgr); 
     
	/* Management Clock Prescaler Divisor */
	/* cgg - changed from clock independent setting:
	   writel(40, &lp->eth_regs->ethmcp); */

	writel(((IDT_BUS_FREQ * 1000 * 1000)/MII_CLOCK+1) & ~1,
	       &lp->eth_regs->ethmcp);

	/* Clear Stat. Registers by reading them */ 
#if 0 
	tmp = readl(&lp->eth_regs->ethrbc); 
	tmp = readl(&lp->eth_regs->ethrpc); 
	tmp = readl(&lp->eth_regs->ethrupc); 
	tmp = readl(&lp->eth_regs->ethrfc); 
	tmp = readl(&lp->eth_regs->ethtbc); 
#endif
	
	/* don't transmit until fifo contains 48b */
	writel(48, &lp->eth_regs->ethfifott);

	writel(ETHMAC1_re_m, &lp->eth_regs->ethmac1);  

	/* Initialize the transmit Descriptors */
	for (i = 0; i < ACACIA_NUM_TDS; i++) {
		lp->td_ring[i].control = DMAD_f_m;
		lp->td_ring[i].devcs = 0;
		lp->td_ring[i].ca = 0;
		lp->td_ring[i].link = 0;
		if (lp->tx_skb[i] != NULL) {
			/* free dangling skb */
			dev_kfree_skb_any(lp->tx_skb[i]);
			lp->tx_skb[i] = NULL;
		}
	}
	
	lp->tx_next_in = lp->tx_next_out = lp->tx_count = 0;

	/*
	 * Initialize the receive descriptors so that they
	 * become a circular linked list, ie. let the last
	 * descriptor point to the first again.
	 */
	for (i=0; i<ACACIA_NUM_RDS; i++) {
		lp->rd_ring[i].control =
			DMAD_iod_m | DMA_COUNT(ACACIA_RBSIZE);
		lp->rd_ring[i].devcs = 0;
		lp->rd_ring[i].ca =
			virt_to_phys(&lp->rba[i * ACACIA_RBSIZE]);
		lp->rd_ring[i].link = kseg1_to_phys(&lp->rd_ring[i+1]);
	}
	/* loop back */
	lp->rd_ring[ACACIA_NUM_RDS-1].link = kseg1_to_phys(&lp->rd_ring[0]);

	lp->rx_next_out = 0;         
	writel(0, &lp->rx_dma_regs->dmas);

	/* Start Rx DMA */
	acacia_start_rx(lp, &lp->rd_ring[0]);
    
	writel(readl(&lp->tx_dma_regs->dmasm) & ~(DMAS_f_m | DMAS_e_m),
	       &lp->tx_dma_regs->dmasm); 

	writel(readl(&lp->rx_dma_regs->dmasm) & ~(DMAS_d_m | DMAS_h_m | DMAS_e_m),
	       &lp->rx_dma_regs->dmasm);
	
	netif_start_queue(dev);

	return 0; 
}


static void acacia_cleanup_module(void)
{
	int i;

	for (i = 0; acacia_iflist[i].iobase; i++) {
		struct acacia_if_t * bif = &acacia_iflist[i];
		if (bif->dev != NULL) {
			struct acacia_local *lp =
				(struct acacia_local *)bif->dev->priv;
			if (lp != NULL) {
				if (lp->eth_regs)
					iounmap(lp->eth_regs);
				if (lp->rx_dma_regs)
					iounmap(lp->rx_dma_regs);
				if (lp->tx_dma_regs)
					iounmap(lp->tx_dma_regs);
				if (lp->td_ring)
					kfree((void*)KSEG0ADDR(lp->td_ring));
				if (lp->rba)
					kfree(lp->rba);
#ifdef ACACIA_PROC_DEBUG
				if (lp->ps)
					remove_proc_entry("net/rc32355",
							  NULL);
#endif
				kfree(lp);
			}

			unregister_netdev(bif->dev);
			kfree(bif->dev);
			release_region(bif->iobase, 0x24C);
		}
	}
}


#ifndef MODULE

static int __init acacia_setup(char *options)
{
	/* no options yet */
	return 1;
}

static int __init acacia_setup_ethaddr(char *options)
{
	memcpy(mac0, options, 17);
	mac0[17]= '\0';
	return 1;
}

__setup("rc32438eth=", acacia_setup);

__setup("ethaddr=", acacia_setup_ethaddr);

#endif /* !MODULE */

module_init(acacia_init_module);
module_exit(acacia_cleanup_module);














