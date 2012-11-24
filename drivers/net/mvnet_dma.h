/*
 *
 *  mvnet/driver/mvnet_dma.h, version 2.0
 *
 *  Copyright 2000-2003, MontaVista Software, Inc.
 *
 *  This software may be used and distributed according to the terms of
 *  the GNU Public License, Version 2, incorporated herein by reference.
 *
 *  Contact:  <source@mvista.com>
 *
 */

#ifndef _MVNETDMA_H_
#define _MVNETDMA_H_

#define DMA_MODE_SHARED		0
#define DMA_MODE_EXCLUSIVE	1

struct dma_pub {
	struct list_head list;
	void *src;
	void *dst;
	size_t size;
	int direction;
	void (*callback)(void *);
	int status;
	char usr[32];
};

extern int	mvnet_dma_open(int num, int mode, void **dma_ch);
extern void	mvnet_dma_close(void *dma_ch);

extern int mvnet_dma_alloc_desc(struct dma_pub **desc, void *dma_ch);
extern int mvnet_dma_alloc_list(unsigned int count, struct list_head *head,
				 void *dma_ch);

extern int mvnet_dma_free_desc(struct dma_pub *desc, void *dma_ch);
extern int mvnet_dma_free_list(struct list_head *head, void *dma_ch);

extern int mvnet_dma_queue_list(struct list_head *head , void *dma_ch);

static __inline__ int mvnet_dma_queue_desc(struct dma_pub *pub, void *dma_ch)
{
	LIST_HEAD(head);

	if (pub == NULL)
		return -EINVAL;
	list_add(&pub->list, &head);
	return mvnet_dma_queue_list(&head, dma_ch);
}                                                                               
#endif /* _MVNETDMA_H_ */
