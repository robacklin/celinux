#ifndef __ASM_XTENSA_USER_H
#define __ASM_XTENSA_USER_H

/*
 * include/asm-xtensa/user.h
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2001 - 2003 Tensilica Inc.
 */


/* 'struct user' is only for a.out file formats.  Xtensa doesn't
 * support the a.out format.  We provide a dummy definition to resolve
 * compiler warnings in a generic kernel file. */

struct user {
	int __dont_use_this_struct;
	int __aout_not_supported_on_xtensa;
};

#endif /* __ASM_XTENSA_USER_H */
