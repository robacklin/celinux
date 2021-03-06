/*
 * Copyright (c) 2001-2002 Silicon Graphics, Inc.  All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of version 2 of the GNU General Public License as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it would be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Further, this software is distributed without any warranty that it is
 * free of the rightful claim of any third person regarding infringement
 * or the like.	 Any license provided herein, whether implied or
 * otherwise, applies only to this software file.  Patent licenses, if
 * any, provided herein do not apply to combinations of this program with
 * other software, or any other product whatsoever.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write the Free Software Foundation, Inc., 59
 * Temple Place - Suite 330, Boston MA 02111-1307, USA.
 *
 * Contact information: Silicon Graphics, Inc., 1600 Amphitheatre Pkwy,
 * Mountain View, CA  94043, or:
 *
 * http://www.sgi.com
 *
 * For further information regarding this notice, see:
 *
 * http://oss.sgi.com/projects/GenInfo/SGIGPLNoticeExplan/
 */

#ifndef __XFS_SYSCTL_H__
#define __XFS_SYSCTL_H__

#include <linux/sysctl.h>

/*
 * Tunable xfs parameters
 */

#define XFS_PARAM	(sizeof(struct xfs_param) / sizeof(ulong))

typedef struct xfs_param {
	ulong	refcache_size;	/* Size of NFS reference cache.          */
	ulong	refcache_purge;	/* # of entries to purge each time.      */
	ulong	stats_clear;	/* Reset all XFS statistics to zero.     */
	ulong	restrict_chown;	/* Root/non-root can give away files.    */
	ulong	sgid_inherit;	/* Inherit ISGID bit if process' GID is  */
				/*  not a member of the parent dir GID.  */
	ulong	symlink_mode;	/* Symlink creat mode affected by umask. */
} xfs_param_t;

enum {
	XFS_REFCACHE_SIZE = 1,
	XFS_REFCACHE_PURGE = 2,
	XFS_STATS_CLEAR = 3,
	XFS_RESTRICT_CHOWN = 4,
	XFS_SGID_INHERIT = 5,
	XFS_SYMLINK_MODE = 6,
};

extern xfs_param_t	xfs_params;

#ifdef CONFIG_SYSCTL
extern void xfs_sysctl_register(void);
extern void xfs_sysctl_unregister(void);
#else
static __inline void xfs_sysctl_register(void) { };
static __inline void xfs_sysctl_unregister(void) { };
#endif

#endif /* __XFS_SYSCTL_H__ */
