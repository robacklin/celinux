/*
 * JFFS2 -- Journalling Flash File System, Version 2.
 *
 * Copyright (C) 2001 Red Hat, Inc.
 *
 * Created by David Woodhouse <dwmw2@cambridge.redhat.com>
 *
 * For licensing information, see the file 'LICENCE' in this directory.
 *
 * $Id: ioctl.c,v 1.5 2001/03/15 15:38:24 dwmw2 Exp $
 *
 */

#include <linux/fs.h>

int jffs2_ioctl(struct inode *inode, struct file *filp, unsigned int cmd, 
		unsigned long arg)
{
	/* Later, this will provide for lsattr.jffs2 and chattr.jffs2, which
	   will include compression support etc. */
	return -EINVAL;
}
	
