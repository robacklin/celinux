/*
 *   Copyright (c) International Business Machines Corp., 2000-2002
 *   Portions Copyright (c) Christoph Hellwig, 2001-2002
 *
 *   This program is free software;  you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation; either version 2 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY;  without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See
 *   the GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program;  if not, write to the Free Software
 *   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */
#ifndef _H_JFS_COMPAT
#define	_H_JFS_COMPAT

/*
 *	jfs_compat.h:
 *
 * Definitions to allow JFS to build on older kernels.
 *
 * This file should be removed when JFS is merged with linux kernel
 *
 */

#include <linux/module.h>
#include <linux/version.h>
#include <linux/slab.h>

#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,4,20))
#include "endian24.h"
#endif

#ifndef __weak
#define __weak	__attribute__((weak));
#endif

#ifndef MODULE_LICENSE
#define MODULE_LICENSE(x)
#endif

#ifndef GFP_NOFS
#define GFP_NOFS GFP_BUFFER
#endif

#if !defined(KERNEL_HAS_O_DIRECT)
#define fsync_inode_data_buffers fsync_inode_buffers
#endif

/*
 * Linux 2.4.9 has broken min/max macros.
 * Linux < 2.4.9 doesn't have min/max at all.
 */
#if (LINUX_VERSION_CODE == KERNEL_VERSION(2,4,9))
#undef min
#undef max
#endif

/*
 * Completions are new in 2.4.7.
 */
#if (LINUX_VERSION_CODE <= KERNEL_VERSION(2,4,6))
#define DECLARE_COMPLETION(c)	DECLARE_MUTEX_LOCKED(c)
#define complete(c)		up(c)
#define wait_for_completion(c)	down(c)
/* must be last to not mess up the namespace */
#define completion		semaphore
#else
#include <linux/completion.h>
#endif

#if (LINUX_VERSION_CODE <= KERNEL_VERSION(2,4,9))
#define min(x,y) ({			\
	const typeof(x) _x = (x);	\
	const typeof(y) _y = (y);	\
	(void) (&_x == &_y);		\
	_x < _y ? _x : _y; })

#define max(x,y) ({			\
	const typeof(x) _x = (x);	\
	const typeof(y) _y = (y);	\
	(void) (&_x == &_y);		\
	_x > _y ? _x : _y; })
#endif

#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,4,18))
static inline struct buffer_head * sb_bread(struct super_block *sb, int block)
{
	return bread(sb->s_dev, block, sb->s_blocksize);
}
#endif
#endif				/* !_H_JFS_COMPAT */
