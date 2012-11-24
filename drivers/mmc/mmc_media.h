/*
 * Header for MultiMediaCard (MMC)
 *
 * Copyright 2002 Hewlett-Packard Company
 *
 * Use consistent with the GNU GPL is permitted,
 * provided that this copyright notice is
 * preserved in its entirety in all copies and derived works.
 *
 * HEWLETT-PACKARD COMPANY MAKES NO WARRANTIES, EXPRESSED OR IMPLIED,
 * AS TO THE USEFULNESS OR CORRECTNESS OF THIS CODE OR ITS
 * FITNESS FOR ANY PARTICULAR PURPOSE.
 *
 * Many thanks to Alessandro Rubini and Jonathan Corbet!
 *
 * Based strongly on code by:
 *
 * Author: Yong-iL Joh <tolkien@mizi.com>
 * Date  : $Date: 2002/10/03 05:21:14 $ 
 *
 * Author:  Andrew Christian
 *          15 May 2002
 */

#ifndef MMC_MMC_MEDIA_H
#define MMC_MMC_MEDIA_H

#include <linux/interrupt.h>
#include <linux/list.h>

#include <linux/mmc/mmc_protocol.h>

/* Set an upper bound for how many cards we'll support */
/* This is used only for static array initialization */
#define MMC_MAX_SLOTS   2

#define MMC_SLOT_FLAG_INSERT  (1<<0)
#define MMC_SLOT_FLAG_EJECT   (1<<1)

struct mmc_media_driver;

struct mmc_slot {
	int             id;     /* Card index */
	/* Card specific information */
	struct mmc_cid  cid;
	struct mmc_csd  csd;

	enum card_state state;  /* empty, ident, ready, whatever */
	int             flags;  /* Ejected, inserted */
        int             sd;     /* MMC or SD card */
        int             rca;    /* RCA */
        u32             scr;    /* SCR 63:32*/        
	/* Assigned media driver */
	struct mmc_media_driver *media_driver;
};

struct mmc_io_request {
	int            id;         /* Card index     */
	int            cmd;        /* READ or WRITE  */
	unsigned long  sector;     /* Start address  */
	unsigned long  nr_sectors; /* Length of read */
	unsigned long  block_len;  /* Size of sector (sanity check) */
	char          *buffer;     /* Data buffer    */
};

/* Media driver (e.g., Flash card, I/O card...) */
struct mmc_media_driver {
	struct list_head   node;
	char              *name;
	void (*load)(struct mmc_slot *);
	void (*unload)(struct mmc_slot *);
	int  (*probe)(struct mmc_slot *);
	void (*io_request_done)(struct mmc_io_request *, int result);
};

struct mmc_media_module {
	int (*init)(void);
	void (*cleanup)(void);
};

/* Calls made by the media driver */
extern int  mmc_register_media_driver( struct mmc_media_driver * );
extern void mmc_unregister_media_driver( struct mmc_media_driver * );
extern void mmc_handle_io_request( struct mmc_io_request * );

#endif  /* MMC_MMC_MEDIA_H */

