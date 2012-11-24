/*
 * linux/arch/arm/boot/compressed/head-s3c2440.S
 *
 * Copyright (C) 2003 Samsung Electronics
 *
 */

#include <asm/mach-types.h>
		.section	".start", "ax"
		mov	r7, #MACH_TYPE_S3C2440

