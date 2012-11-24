#ifndef __ASM_XTENSA_IPCBUF_H
#define __ASM_XTENSA_IPCBUF_H

/*
 * include/asm-xtensa/ipcbuf.h
 *
 * The ipc64_perm structure for the Xtensa architecture.
 * Note extra padding because this structure is passed back and forth
 * between kernel and user space.
 *
 * Pad space is left for 2 miscellaneous 32-bit values.
 *
 * This file is subject to the terms and conditions of the GNU General
 * Public License.  See the file "COPYING" in the main directory of
 * this archive for more details.
 */


struct ipc64_perm
{
	__kernel_key_t		key;
	__kernel_uid32_t	uid;
	__kernel_gid32_t	gid;
	__kernel_uid32_t	cuid;
	__kernel_gid32_t	cgid;
	__kernel_mode_t		mode;
	unsigned long int	seq;
	unsigned long int	__unused1;
	unsigned long int	__unused2;
};

#endif /* __ASM_XTENSA_IPCBUF_H */
