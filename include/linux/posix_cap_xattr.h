/*
  File: linux/posix_cap_xattr.h

  Extended attribute representation of capabilities
*/
#ifndef _POSIX_CAP_XATTR_H
#define _POSIX_CAP_XATTR_H

#define POSIX_CAP_XATTR		"system.posix_capabilities"
#define POSIX_CAP_XATTR_VERSION	0x0001

typedef __u64 posix_cap_xattr_value;

typedef struct {
	__u32			c_version;
	__u32			c_abiversion;
	posix_cap_xattr_value	c_effective;
	posix_cap_xattr_value	c_permitted;
	posix_cap_xattr_value	c_inheritable;
} posix_cap_xattr;

static inline size_t posix_cap_xattr_size(void)
{
	return (sizeof(posix_cap_xattr));
}

#endif	/* _POSIX_CAP_XATTR_H */
