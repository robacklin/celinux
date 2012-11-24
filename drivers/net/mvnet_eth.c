/*
 *
 *  mvnet/driver/mvnet_eth.c, version 2.0
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
#include <linux/etherdevice.h>
#include <linux/stddef.h>
#include <linux/string.h>
#include <linux/types.h>
#include <linux/ctype.h>

#include "mvnet.h"
#include "mvnet_eth.h"

MODULE_AUTHOR("MontaVista Software <source@mvista.com>");
MODULE_DESCRIPTION("MontaVista Net Backpanel Driver (Ethernet layer)");
MODULE_LICENSE("GPL");

#define TOGGLE_QUEUE
#define START_QUEUE


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

#define PRINT_PREFIX	"mvnet_eth:  "
#define MVNET_DEVICE_NAME	"pci%d"
#define MVNET_DEVICE_ALT_NAME	"eth%d"

struct eth_priv {
    struct mvnet_funcs *netfn;
    struct mvnet_pool *pool;
    struct net_device_stats stats;
};

/* Default ethernet hardware address */
static u8 mac_addr[6] = { 0x42, 0x00, 0x00, 0x00, 0x00, 0x01 }; 

MODULE_PARM(mac_addr, "6b");
MODULE_PARM_DESC(mac_addr, "MAC address.");

/* If non-zero, have kernel assign "ethN" device name */
static int eth = 0;

MODULE_PARM(eth, "i");
MODULE_PARM_DESC(eth, "Set non-zero to use ethN device names vs pciN style.");

#ifndef MODULE
static int __init set_mac_lsb(char *str)
{
	int temp;

	temp = simple_strtoul(str, NULL, 0);
	mac_addr[5] = (u8)(temp & 0xff);
	return 1;
}

__setup("mvnet_mac_lsb=", set_mac_lsb);

static int __init set_eth(char *str)
{
	eth = simple_strtoul(str, NULL, 0);
	return 1;
}

__setup("mvnet_eth=", set_eth);
#endif

/*
 *  Network device functions
 */

static int
eth_set_config(struct net_device *ndev, struct ifmap *map)
{
    Debug((PRINT_PREFIX "eth_set_config\n"));

    if (ndev->flags & IFF_UP)
	return -EBUSY;

    return 0;
}


static struct net_device_stats *
eth_get_stats(struct net_device *ndev)
{
    struct eth_priv *priv = (struct eth_priv *) ndev->priv;

    Debug((PRINT_PREFIX "eth_get_stats\n"));

    return &(priv->stats);
}


static void
eth_set_multicast_list(struct net_device *ndev)
{
    Debug((PRINT_PREFIX "eth_set_multicast_list\n"));
}


static int
eth_set_mac_address(struct net_device *ndev, void *addr)
{
    struct sockaddr *newaddr = (struct sockaddr *) addr;

    Debug((PRINT_PREFIX "eth_set_mac_address\n"));

    if (ndev->flags & IFF_UP)
	return -EBUSY;
    memcpy(ndev->dev_addr, newaddr->sa_data, ndev->addr_len);

    return 0;
}


static void *
eth_buffer_alloc(void *arg, unsigned int len, void **buf)
{
    struct net_device *ndev = (struct net_device *) arg;
    struct eth_priv *priv = (struct eth_priv *) ndev->priv;
    struct sk_buff *skb;

    Debug((PRINT_PREFIX "eth_buffer_alloc\n"));

    skb = dev_alloc_skb(len + 2);
    if (skb == NULL) {
	printk(PRINT_PREFIX "Cannot allocate skb\n");
	priv->stats.rx_dropped += 1;
	return NULL;
    } else {
	*buf = (void *) skb;
	return (void *) skb_put(skb, len);
    }
}


static void
eth_recv(void *arg, void *buf)
{
    struct net_device *ndev = (struct net_device *) arg;
    struct eth_priv *priv = (struct eth_priv *) ndev->priv;
    struct sk_buff *skb = (struct sk_buff *) buf;

    Debug((PRINT_PREFIX "eth_recv\n"));

    skb->dev = ndev;
    skb->protocol = eth_type_trans(skb, ndev);
    /* skb->ip_summed = CHECKSUM_UNNECESSARY; */

    priv->stats.rx_packets += 1;
    priv->stats.rx_bytes += skb->len;
    ndev->last_rx = jiffies;

    netif_rx(skb);
}


static void
eth_xmit_done(int result, void *arg)
{
    struct sk_buff *skb = (struct sk_buff *) arg;
    struct eth_priv *priv = (struct eth_priv *) skb->dev->priv;
    unsigned int len = (ETH_ZLEN < skb->len) ? skb->len : ETH_ZLEN;
#ifdef TOGGLE_QUEUE
    struct net_device *ndev = skb->dev;
#endif

    Debug((PRINT_PREFIX "eth_xmit_done\n"));

    if (result == 0) {
	priv->stats.tx_packets += 1;
	priv->stats.tx_bytes += len;
    } else {
	priv->stats.tx_dropped += 1;
    }

    dev_kfree_skb_any(skb);

/* TEMP */
#ifdef TOGGLE_QUEUE
#ifdef START_QUEUE
    netif_start_queue(ndev);
#else
    netif_wake_queue(ndev);
#endif
#endif

}


/* Drop packets when output buffers full instead of returning them to queue */
#define MVNET_DROP_BAD_XMITS

static int
eth_hard_start_xmit(struct sk_buff *skb, struct net_device *ndev)
{
    unsigned int len;
    unsigned char addr1;
    int result;
    struct eth_priv *priv = (struct eth_priv *) ndev->priv;

    Debug((PRINT_PREFIX "eth_hard_start_xmit\n"));

    if (skb == NULL) {
	Debug((PRINT_PREFIX "eth_hard_start_xmit -- skb\n"));
	return 0;
    }

    if (netif_queue_stopped(ndev)) {
	Debug((PRINT_PREFIX "eth_hard_start_xmit -- queue stopped\n"));

/* TEMP */
	printk("BUSY\n");
	return -EBUSY;
    }

/* TEMP */
#ifdef TOGGLE_QUEUE
    netif_stop_queue(ndev);
#endif

    ndev->trans_start = jiffies;

    len = (ETH_ZLEN < skb->len) ? skb->len : ETH_ZLEN;
    skb->dev = ndev;
    addr1 = *((unsigned char *) skb->data);

    if (addr1 & 0x01) {
	/* broadcast or multicast */
	result = priv->netfn->broadcast(
		priv->pool,
		skb->data,
	       	len,
	       	eth_xmit_done,
		(void *) skb);
    } else {
	result = priv->netfn->send(
		priv->pool,
		skb->data,
	       	skb->data,
	       	len,
	       	eth_xmit_done,
		(void *) skb);
    }

#ifdef MVNET_DROP_BAD_XMITS

    if (result != 0) {
	    dev_kfree_skb_any(skb);
	    priv->stats.tx_dropped += 1;
	    result = 0;
    }
#endif /* MVNET_DROP_BAD_XMITS */

#ifdef TOGGLE_QUEUE
#ifdef START_QUEUE
    netif_start_queue(ndev);
#else
    netif_wake_queue(ndev);
#endif
#endif

    return result;
}


/*
 *  Network device start/stop
 */


static int
eth_open(struct net_device *ndev)
{
    int result;
    struct eth_priv *priv = (struct eth_priv *) ndev->priv;

    Debug((PRINT_PREFIX "eth_open\n"));

    result = priv->netfn->open(priv->pool, ndev->dev_addr, eth_recv,
		    	       eth_buffer_alloc, (void *)ndev);
    if (result != 0) {
	printk(PRINT_PREFIX "Can't open network layer\n");
	return result;
    }

    netif_start_queue(ndev);

    MOD_INC_USE_COUNT;

    return 0;
}


static int
eth_stop(struct net_device *ndev)
{

    struct eth_priv *priv = (struct eth_priv *) ndev->priv;
    Debug((PRINT_PREFIX "eth_stop\n"));

    netif_stop_queue(ndev);
    
    priv->netfn->close(priv->pool);

    MOD_DEC_USE_COUNT;

    return 0;
}


/*
 *  Network device init/destroy
 */
static void
eth_assign_def_addr(struct net_device *ndev)
{
	unsigned long num;
	char *s;

	num = strlen(ndev->name);
	s = &ndev->name[num-1];
	while (isdigit(*s))
		--s;
	s++;
	num = simple_strtoul(s, NULL, 10);

	assert(ndev->addr_len == 6);
	memcpy(ndev->dev_addr, mac_addr, ndev->addr_len);

	ndev->dev_addr[4] = (ndev->name[4] + num) & 0xff;
}

static int
eth_init(struct net_device *ndev)
{
    Debug((PRINT_PREFIX "eth_init\n"));

    ether_setup(ndev);
    ndev->open = eth_open;
    ndev->stop = eth_stop;
    ndev->hard_start_xmit = eth_hard_start_xmit;
    ndev->set_config = eth_set_config;
    ndev->get_stats = eth_get_stats;
    ndev->set_multicast_list = eth_set_multicast_list;
    ndev->set_mac_address = eth_set_mac_address;

    /* Assign default hardware ethernet address */
    eth_assign_def_addr(ndev);

    return 0;
}


static void
eth_destructor(struct net_device *ndev)
{
    Debug((PRINT_PREFIX "eth_destructor\n"));
    kfree(ndev->priv);
}


/*
 *  Module init/cleanup
 */

EXPORT_SYMBOL(mvnet_eth_add_device);
void *
mvnet_eth_add_device(struct mvnet_pool *pool, struct mvnet_funcs *netfn)
{
    struct net_device *dev;
    struct eth_priv *priv;
    int result;

    Debug((PRINT_PREFIX "mvnet_eth_add_device\n"));

    dev = kmalloc(sizeof(struct net_device), GFP_KERNEL);
    if (dev == NULL) {
	Debug((PRINT_PREFIX "Cannot allocate memory for device.\n"));
	return NULL;
    }

    priv  = kmalloc(sizeof(struct eth_priv), GFP_KERNEL);
    if (priv == NULL) {
	kfree(dev);
	Debug((PRINT_PREFIX "Cannot allocate memory for priv.\n"));
	return NULL;
    }

    memset(dev, 0, sizeof(struct net_device));
    memset(priv, 0, sizeof(struct eth_priv));

    priv->netfn = netfn;
    priv->pool = pool;

    dev->priv = priv;
    dev->init = eth_init;
    dev->destructor = eth_destructor;

    if (eth)
	    strcpy(dev->name, MVNET_DEVICE_ALT_NAME);
    else
	    strcpy(dev->name, MVNET_DEVICE_NAME);

    result = register_netdev(dev);
    if (result != 0) {
	    printk(PRINT_PREFIX "error %d registering device \"%s\"\n",
			    result, dev->name);
	    kfree(priv);
	    kfree(dev);
	    return NULL;
    }

    return dev;
}

EXPORT_SYMBOL(mvnet_eth_remove_device);
int
mvnet_eth_remove_device(void *dev)
{
    Debug((PRINT_PREFIX "mvnet_eth_remove_device\n"));
    unregister_netdev((struct net_device *)dev);
    return 0; /* FIXME */
}

int
__init mvnet_eth_init_module(void)
{
    Debug((PRINT_PREFIX "init_module\n"));

    EXPORT_NO_SYMBOLS;

    return 0;
}


void
__exit mvnet_eth_cleanup_module(void)
{
    Debug((PRINT_PREFIX "cleanup_module\n"));
}

module_init(mvnet_eth_init_module);
module_exit(mvnet_eth_cleanup_module);
