#ifndef _LINUX_ERR_H
#define _LINUX_ERR_H

#include <asm/errno.h>

#if 1 /* linux-pm */
#include <linux/fs.h>
#else /* linux-pm */
/*
 * Kernel pointers have redundant information, so we can use a
 * scheme where we can return either an error code or a dentry
 * pointer with the same return value.
 *
 * This should be a per-architecture thing, to allow different
 * error and pointer decisions.
 */
static inline void *ERR_PTR(long error)
{
	return (void *) error;
}

static inline long PTR_ERR(const void *ptr)
{
	return (long) ptr;
}

static inline long IS_ERR(const void *ptr)
{
	return (unsigned long)ptr > (unsigned long)-1000L;
}
#endif /* linux-pm */

#endif /* _LINUX_ERR_H */
