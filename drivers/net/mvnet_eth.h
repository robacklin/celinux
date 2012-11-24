/*
 *
 *  mvnet/driver/mvnet_eth.h, version 2.1
 *
 *  Copyright 2000-2003, MontaVista Software, Inc.
 *
 *  This software may be used and distributed according to the terms of
 *  the GNU Public License, Version 2, incorporated herein by reference.
 *
 *  Contact:  <source@mvista.com>
 *
 */

#ifndef _MVNET_ETH_H_
#define _MVNET_ETH_H_

#include "mvnet.h"

extern void * mvnet_eth_add_device(struct mvnet_pool *pool,
				   struct mvnet_funcs *netfn);
extern int mvnet_eth_remove_device(void *dev);

#endif /* _MVNET_ETH_H_ */
