/* 
 * xtensa/config/tie.h -- HAL definitions that are dependent on CORE and TIE configuration
 *
 *  This header file is sometimes referred to as the "compile-time HAL" or CHAL.
 *  It was generated for a specific Xtensa processor configuration,
 *  and furthermore for a specific set of TIE source files that extend
 *  basic core functionality.
 *
 *  Source for configuration-independent binaries (which link in a
 *  configuration-specific HAL library) must NEVER include this file.
 *  It is perfectly normal, however, for the HAL source itself to include this file.
 */

/*
 * Copyright (c) 2003 Tensilica, Inc.  All Rights Reserved.
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of version 2.1 of the GNU Lesser General Public
 * License as published by the Free Software Foundation.
 * 
 * This program is distributed in the hope that it would be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * 
 * Further, this software is distributed without any warranty that it is
 * free of the rightful claim of any third person regarding infringement
 * or the like.  Any license provided herein, whether implied or
 * otherwise, applies only to this software file.  Patent licenses, if
 * any, provided herein do not apply to combinations of this program with
 * other software, or any other product whatsoever.
 * 
 * You should have received a copy of the GNU Lesser General Public
 * License along with this program; if not, write the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston MA 02111-1307,
 * USA.
 */


#ifndef XTENSA_CONFIG_TIE_H
#define XTENSA_CONFIG_TIE_H

#include <xtensa/hal.h>


/*----------------------------------------------------------------------
				GENERAL
  ----------------------------------------------------------------------*/

/*
 *  Separators for macros that expand into arrays.
 *  These can be predefined by files that #include this one,
 *  when different separators are required.
 */
/*  Element separator for macros that expand into 1-dimensional arrays:  */
#ifndef XCHAL_SEP
#define XCHAL_SEP			,
#endif
/*  Array separator for macros that expand into 2-dimensional arrays:  */
#ifndef XCHAL_SEP2
#define XCHAL_SEP2			},{
#endif






/*----------------------------------------------------------------------
			COPROCESSORS and EXTRA STATE
  ----------------------------------------------------------------------*/

#define XCHAL_CP_NUM			4	/* number of coprocessors */
#define XCHAL_CP_MAX			7	/* max coprocessor id plus one (0 if none) */
#define XCHAL_CP_MASK			0x65	/* bitmask of coprocessors by id */

/*  Space for coprocessors' state save areas:  */
#define XCHAL_CP0_SA_SIZE		72
#define XCHAL_CP1_SA_SIZE		0
#define XCHAL_CP2_SA_SIZE		68
#define XCHAL_CP3_SA_SIZE		0
#define XCHAL_CP4_SA_SIZE		0
#define XCHAL_CP5_SA_SIZE		136
#define XCHAL_CP6_SA_SIZE		36
#define XCHAL_CP7_SA_SIZE		0
/*  Minimum required alignments of CP state save areas:  */
#define XCHAL_CP0_SA_ALIGN		4
#define XCHAL_CP1_SA_ALIGN		1
#define XCHAL_CP2_SA_ALIGN		4
#define XCHAL_CP3_SA_ALIGN		1
#define XCHAL_CP4_SA_ALIGN		1
#define XCHAL_CP5_SA_ALIGN		8
#define XCHAL_CP6_SA_ALIGN		4
#define XCHAL_CP7_SA_ALIGN		1

/*  Indexing macros:  */
#define _XCHAL_CP_SA_SIZE(n)		XCHAL_CP ## n ## _SA_SIZE
#define XCHAL_CP_SA_SIZE(n)		_XCHAL_CP_SA_SIZE(n)	/* n = 0 .. 7 */
#define _XCHAL_CP_SA_ALIGN(n)		XCHAL_CP ## n ## _SA_ALIGN
#define XCHAL_CP_SA_ALIGN(n)		_XCHAL_CP_SA_ALIGN(n)	/* n = 0 .. 7 */


/*  Space for "extra" state (user special registers and non-cp TIE) save area:  */
#define XCHAL_EXTRA_SA_SIZE		28
#define XCHAL_EXTRA_SA_ALIGN		4

/*  Total save area size (extra + all coprocessors)  */
/*  (not useful until xthal_{save,restore}_all_extra() is implemented,  */
/*   but included for Tor2 beta; doesn't account for alignment!):  */
#define XCHAL_CPEXTRA_SA_SIZE_TOR2	340	/* Tor2Beta temporary definition -- do not use */

/*  Combined required alignment for all CP and EXTRA state save areas  */
/*  (does not include required alignment for any base config registers):  */
#define XCHAL_CPEXTRA_SA_ALIGN		8

/* ... */


#ifdef _ASMLANGUAGE
/*
 *  Assembly-language specific definitions (assembly macros, etc.).
 */
#include <xtensa/config/specreg.h>

/********************
 *  Macros to save and restore the non-coprocessor TIE portion of EXTRA state.
 */

/* (none) */


/********************
 *  Macros to create functions that save and restore all EXTRA (non-coprocessor) state
 *  (does not include zero-overhead loop registers and non-optional registers).
 */

	/*
	 *  Macro that expands to the body of a function that
	 *  stores the extra (non-coprocessor) optional/custom state.
	 *	Entry:	a2 = ptr to save area in which to save extra state
	 *	Exit:	any register a2-a15 (?) may have been clobbered.
	 */
	.macro	xchal_extra_store_funcbody
	// Start saving state for MAC16
	rsr	a3, ACCLO
	s32i	a3, a2, 0
	rsr	a3, ACCHI
	s32i	a3, a2, 4
	rsr	a3, MR_0
	s32i	a3, a2, 8
	rsr	a3, MR_1
	s32i	a3, a2, 12
	rsr	a3, MR_2
	s32i	a3, a2, 16
	rsr	a3, MR_3
	s32i	a3, a2, 20
	// End of saving state for MAC16
	// Start saving state for Booleans
	rsr	a3, BR
	s32i	a3, a2, 24
	// End of saving state for Booleans
	.endm


	/*
	 *  Macro that expands to the body of a function that
	 *  loads the extra (non-coprocessor) optional/custom state.
	 *	Entry:	a2 = ptr to save area from which to restore extra state
	 *	Exit:	any register a2-a15 (?) may have been clobbered.
	 */
	.macro	xchal_extra_load_funcbody
	// begin restore state for MAC16
	l32i	a3, a2, 0
	wsr	a3, ACCLO
	l32i	a3, a2, 4
	wsr	a3, ACCHI
	l32i	a3, a2, 8
	wsr	a3, MR_0
	l32i	a3, a2, 12
	wsr	a3, MR_1
	l32i	a3, a2, 16
	wsr	a3, MR_2
	l32i	a3, a2, 20
	wsr	a3, MR_3
	// end restore state for MAC16
	// begin restore state for Booleans
	l32i	a3, a2, 24
	wsr	a3, BR
	// end restore state for Booleans
	.endm


/********************
 *  Macros to save and restore the state of each TIE coprocessor.
 */

#define xchal_cp_cp0_store	xchal_cp0_store
#define xchal_cp_cp0_store_a2	xchal_cp0_store_a2
#define xchal_cp0_store_a2	xchal_cp0_store	a2, a3
// Name: xchal_cp0_store
// This macro saves the states of cp0
// Prototype: xchal_cp0_store a_0 a_1 
// Pointer to memory: a_0, aligned to 4 bytes
// Scratch register needed:  a_1 
// Example use:
//
// Clobbers:
//    a_0 a_1 

.macro xchal_cp0_store a_0 a_1 
rur232	\a_1
s32i	\a_1, \a_0, 0
rur233	\a_1
s32i	\a_1, \a_0, 4
addi	\a_0, \a_0, 8
ssi f0, \a_0,  0
ssi f1, \a_0,  4
ssi f2, \a_0,  8
ssi f3, \a_0,  12
ssi f4, \a_0,  16
ssi f5, \a_0,  20
ssi f6, \a_0,  24
ssi f7, \a_0,  28
ssi f8, \a_0,  32
ssi f9, \a_0,  36
ssi f10, \a_0,  40
ssi f11, \a_0,  44
ssi f12, \a_0,  48
ssi f13, \a_0,  52
ssi f14, \a_0,  56
ssi f15, \a_0,  60

.endm // xchal_cp0_store


#define xchal_cp_cp0_load	xchal_cp0_load
#define xchal_cp_cp0_load_a2	xchal_cp0_load_a2
#define xchal_cp0_load_a2	xchal_cp0_load	a2, a3
// Name: xchal_cp0_load
// This macro restores the states of cp0
// Prototype: xchal_cp0_load a_0 a_1 
// Pointer to memory: a_0, aligned to 4 bytes
// Scratch register needed:  a_1 
// Example use:
//
// Clobbers:
//    a_0 a_1 

.macro xchal_cp0_load a_0 a_1 
l32i	\a_1, \a_0, 0
wur232	\a_1
l32i	\a_1, \a_0, 4
wur233	\a_1
addi	\a_0, \a_0, 8
lsi f0, \a_0,  0
lsi f1, \a_0,  4
lsi f2, \a_0,  8
lsi f3, \a_0,  12
lsi f4, \a_0,  16
lsi f5, \a_0,  20
lsi f6, \a_0,  24
lsi f7, \a_0,  28
lsi f8, \a_0,  32
lsi f9, \a_0,  36
lsi f10, \a_0,  40
lsi f11, \a_0,  44
lsi f12, \a_0,  48
lsi f13, \a_0,  52
lsi f14, \a_0,  56
lsi f15, \a_0,  60

.endm // xchal_cp0_load


#define xchal_cp_cp6_store	xchal_cp6_store
#define xchal_cp_cp6_store_a2	xchal_cp6_store_a2
#define xchal_cp6_store_a2	xchal_cp6_store	a2, a3
// Name: xchal_cp6_store
// This macro saves the states of cp6
// Prototype: xchal_cp6_store a_0 a_1 
// Pointer to memory: a_0, aligned to 4 bytes
// Scratch register needed:  a_1 
// Example use:
//
// Clobbers:
//    a_0 a_1 

.macro xchal_cp6_store a_0 a_1 
rur0	\a_1
s32i	\a_1, \a_0, 0
addi	\a_0, \a_0, 4
i16_si i160, \a_0,  0
i16_si i161, \a_0,  2
i16_si i162, \a_0,  4
i16_si i163, \a_0,  6
i16_si i164, \a_0,  8
i16_si i165, \a_0,  10
i16_si i166, \a_0,  12
i16_si i167, \a_0,  14
i16_si i168, \a_0,  16
i16_si i169, \a_0,  18
i16_si i1610, \a_0,  20
i16_si i1611, \a_0,  22
i16_si i1612, \a_0,  24
i16_si i1613, \a_0,  26
i16_si i1614, \a_0,  28
i16_si i1615, \a_0,  30

.endm // xchal_cp6_store


#define xchal_cp_cp6_load	xchal_cp6_load
#define xchal_cp_cp6_load_a2	xchal_cp6_load_a2
#define xchal_cp6_load_a2	xchal_cp6_load	a2, a3, a4, a5
// Name: xchal_cp6_load
// This macro restores the states of cp6
// Prototype: xchal_cp6_load a_0 a_1 a_2 a_3 
// Pointer to memory: a_0, aligned to 4 bytes
// Scratch register needed:  a_1 a_2 a_3 
// Example use:
//
// Clobbers:
//    a_0 a_1 a_2 a_3 

.macro xchal_cp6_load a_0 a_1 a_2 a_3 
l32i	\a_1, \a_0, 0
rur0	\a_2
movi	\a_3,  0x0000ffff
xor	\a_1, \a_1, \a_2
and	\a_1, \a_1, \a_3
xor	\a_1, \a_1, \a_2
wur0	\a_1
addi	\a_0, \a_0, 4
i16_li i160, \a_0,  0
i16_li i161, \a_0,  2
i16_li i162, \a_0,  4
i16_li i163, \a_0,  6
i16_li i164, \a_0,  8
i16_li i165, \a_0,  10
i16_li i166, \a_0,  12
i16_li i167, \a_0,  14
i16_li i168, \a_0,  16
i16_li i169, \a_0,  18
i16_li i1610, \a_0,  20
i16_li i1611, \a_0,  22
i16_li i1612, \a_0,  24
i16_li i1613, \a_0,  26
i16_li i1614, \a_0,  28
i16_li i1615, \a_0,  30

.endm // xchal_cp6_load


#define xchal_cp_cp2_store	xchal_cp2_store
#define xchal_cp_cp2_store_a2	xchal_cp2_store_a2
#define xchal_cp2_store_a2	xchal_cp2_store	a2, a3
// Name: xchal_cp2_store
// This macro saves the states of cp2
// Prototype: xchal_cp2_store a_0 a_1 
// Pointer to memory: a_0, aligned to 4 bytes
// Scratch register needed:  a_1 
// Example use:
//
// Clobbers:
//    a_0 a_1 

.macro xchal_cp2_store a_0 a_1 
rur1	\a_1
s32i	\a_1, \a_0, 0
addi	\a_0, \a_0, 4
i32_si i320, \a_0,  0
i32_si i321, \a_0,  4
i32_si i322, \a_0,  8
i32_si i323, \a_0,  12
i32_si i324, \a_0,  16
i32_si i325, \a_0,  20
i32_si i326, \a_0,  24
i32_si i327, \a_0,  28
i32_si i328, \a_0,  32
i32_si i329, \a_0,  36
i32_si i3210, \a_0,  40
i32_si i3211, \a_0,  44
i32_si i3212, \a_0,  48
i32_si i3213, \a_0,  52
i32_si i3214, \a_0,  56
i32_si i3215, \a_0,  60

.endm // xchal_cp2_store


#define xchal_cp_cp2_load	xchal_cp2_load
#define xchal_cp_cp2_load_a2	xchal_cp2_load_a2
#define xchal_cp2_load_a2	xchal_cp2_load	a2, a3
// Name: xchal_cp2_load
// This macro restores the states of cp2
// Prototype: xchal_cp2_load a_0 a_1 
// Pointer to memory: a_0, aligned to 4 bytes
// Scratch register needed:  a_1 
// Example use:
//
// Clobbers:
//    a_0 a_1 

.macro xchal_cp2_load a_0 a_1 
l32i	\a_1, \a_0, 0
wur1	\a_1
addi	\a_0, \a_0, 4
i32_li i320, \a_0,  0
i32_li i321, \a_0,  4
i32_li i322, \a_0,  8
i32_li i323, \a_0,  12
i32_li i324, \a_0,  16
i32_li i325, \a_0,  20
i32_li i326, \a_0,  24
i32_li i327, \a_0,  28
i32_li i328, \a_0,  32
i32_li i329, \a_0,  36
i32_li i3210, \a_0,  40
i32_li i3211, \a_0,  44
i32_li i3212, \a_0,  48
i32_li i3213, \a_0,  52
i32_li i3214, \a_0,  56
i32_li i3215, \a_0,  60

.endm // xchal_cp2_load


#define xchal_cp_cp5_store	xchal_cp5_store
#define xchal_cp_cp5_store_a2	xchal_cp5_store_a2
#define xchal_cp5_store_a2	xchal_cp5_store	a2, a3
// Name: xchal_cp5_store
// This macro saves the states of cp5
// Prototype: xchal_cp5_store a_0 a_1 
// Pointer to memory: a_0, aligned to 8 bytes
// Scratch register needed:  a_1 
// Example use:
//
// Clobbers:
//    a_0 a_1 

.macro xchal_cp5_store a_0 a_1 
rur2	\a_1
s32i	\a_1, \a_0, 0
rur3	\a_1
s32i	\a_1, \a_0, 4
addi	\a_0, \a_0, 8
i64_si i640, \a_0,  0
i64_si i641, \a_0,  8
i64_si i642, \a_0,  16
i64_si i643, \a_0,  24
i64_si i644, \a_0,  32
i64_si i645, \a_0,  40
i64_si i646, \a_0,  48
i64_si i647, \a_0,  56
i64_si i648, \a_0,  64
i64_si i649, \a_0,  72
i64_si i6410, \a_0,  80
i64_si i6411, \a_0,  88
i64_si i6412, \a_0,  96
i64_si i6413, \a_0,  104
i64_si i6414, \a_0,  112
i64_si i6415, \a_0,  120

.endm // xchal_cp5_store


#define xchal_cp_cp5_load	xchal_cp5_load
#define xchal_cp_cp5_load_a2	xchal_cp5_load_a2
#define xchal_cp5_load_a2	xchal_cp5_load	a2, a3
// Name: xchal_cp5_load
// This macro restores the states of cp5
// Prototype: xchal_cp5_load a_0 a_1 
// Pointer to memory: a_0, aligned to 8 bytes
// Scratch register needed:  a_1 
// Example use:
//
// Clobbers:
//    a_0 a_1 

.macro xchal_cp5_load a_0 a_1 
l32i	\a_1, \a_0, 0
wur2	\a_1
l32i	\a_1, \a_0, 4
wur3	\a_1
addi	\a_0, \a_0, 8
i64_li i640, \a_0,  0
i64_li i641, \a_0,  8
i64_li i642, \a_0,  16
i64_li i643, \a_0,  24
i64_li i644, \a_0,  32
i64_li i645, \a_0,  40
i64_li i646, \a_0,  48
i64_li i647, \a_0,  56
i64_li i648, \a_0,  64
i64_li i649, \a_0,  72
i64_li i6410, \a_0,  80
i64_li i6411, \a_0,  88
i64_li i6412, \a_0,  96
i64_li i6413, \a_0,  104
i64_li i6414, \a_0,  112
i64_li i6415, \a_0,  120

.endm // xchal_cp5_load




/********************
 *  Macros to create functions that save and restore the state of *any* TIE coprocessor.
 */

	/*
	 *  Macro that expands to the body of a function
	 *  that stores the selected coprocessor's state (registers etc).
	 *	Entry:	a2 = ptr to save area in which to save cp state
	 *		a3 = coprocessor number
	 *	Exit:	any register a2-a15 (?) may have been clobbered.
	 */
	.macro	xchal_cpi_store_funcbody
	bnez	a3, 99f
	xchal_cp_cp0_store_a2
	j	.Lcpi_store_end_\@
99:
	bnei	a3, 6, 99f
	xchal_cp_cp6_store_a2
	j	.Lcpi_store_end_\@
99:
	bnei	a3, 2, 99f
	xchal_cp_cp2_store_a2
	j	.Lcpi_store_end_\@
99:
	bnei	a3, 5, 99f
	xchal_cp_cp5_store_a2
	j	.Lcpi_store_end_\@
99:
.Lcpi_store_end_\@:
	.endm


	/*
	 *  Macro that expands to the body of a function
	 *  that loads the selected coprocessor's state (registers etc).
	 *	Entry:	a2 = ptr to save area from which to restore cp state
	 *		a3 = coprocessor number
	 *	Exit:	any register a2-a15 (?) may have been clobbered.
	 */
	.macro	xchal_cpi_load_funcbody
	bnez	a3, 99f
	xchal_cp_cp0_load_a2
	j	.Lcpi_load_end_\@
99:
	bnei	a3, 6, 99f
	xchal_cp_cp6_load_a2
	j	.Lcpi_load_end_\@
99:
	bnei	a3, 2, 99f
	xchal_cp_cp2_load_a2
	j	.Lcpi_load_end_\@
99:
	bnei	a3, 5, 99f
	xchal_cp_cp5_load_a2
	j	.Lcpi_load_end_\@
99:
.Lcpi_load_end_\@:
	.endm

#endif /*_ASMLANGUAGE*/


/*
 *  Contents of save areas in terms of libdb register numbers.
 *  NOTE:  CONTENTS_LIBDB_{UREG,REGF} macros are not defined in this file;
 *  it is up to the user of this header file to define these macros
 *  usefully before each expansion of the CONTENTS_LIBDB macros.
 *  (Fields rsv[123] are reserved for future additions; they are currently
 *   set to zero but may be set to some useful values in the future.)
 *
 *	CONTENTS_LIBDB_SREG(libdbnum, offset, size, align, rsv1, name, sregnum, bitmask, rsv2, rsv3)
 *	CONTENTS_LIBDB_UREG(libdbnum, offset, size, align, rsv1, name, uregnum, bitmask, rsv2, rsv3)
 *	CONTENTS_LIBDB_REGF(libdbnum, offset, size, align, rsv1, name, index, numentries, contentsize, regname_base, regfile_name, rsv2, rsv3)
 */

#define XCHAL_EXTRA_SA_CONTENTS_LIBDB_NUM	7
#define XCHAL_EXTRA_SA_CONTENTS_LIBDB	\
	CONTENTS_LIBDB_SREG(0x08000010,   0,  4,  4, 0,   "ACCLO",  16, 0xFFFFFFFF, 0,0) \
	CONTENTS_LIBDB_SREG(0x08000011,   4,  4,  4, 0,   "ACCHI",  17, 0x000000FF, 0,0) \
	CONTENTS_LIBDB_SREG(0x08000020,   8,  4,  4, 0,     "MR0",  32, 0xFFFFFFFF, 0,0) \
	CONTENTS_LIBDB_SREG(0x08000021,  12,  4,  4, 0,     "MR1",  33, 0xFFFFFFFF, 0,0) \
	CONTENTS_LIBDB_SREG(0x08000022,  16,  4,  4, 0,     "MR2",  34, 0xFFFFFFFF, 0,0) \
	CONTENTS_LIBDB_SREG(0x08000023,  20,  4,  4, 0,     "MR3",  35, 0xFFFFFFFF, 0,0) \
	CONTENTS_LIBDB_SREG(0x08000004,  24,  4,  4, 0,      "BR",   4, 0x0000FFFF, 0,0) \
	/* end */

#define XCHAL_CP0_SA_CONTENTS_LIBDB_NUM	18
#define XCHAL_CP0_SA_CONTENTS_LIBDB	\
	CONTENTS_LIBDB_UREG(0x0C0000E8,   0,  4,  4, 0,     "FCR", 232, 0xFFFFFFFF, 0,0) \
	CONTENTS_LIBDB_UREG(0x0C0000E9,   4,  4,  4, 0,     "FSR", 233, 0xFFFFFFFF, 0,0) \
	CONTENTS_LIBDB_REGF(0x10030000,   8,  4,  4, 0,      "f0",  0, 16,  4, "f", "FR", 0,0) \
	CONTENTS_LIBDB_REGF(0x10030001,  12,  4,  4, 0,      "f1",  1, 16,  4, "f", "FR", 0,0) \
	CONTENTS_LIBDB_REGF(0x10030002,  16,  4,  4, 0,      "f2",  2, 16,  4, "f", "FR", 0,0) \
	CONTENTS_LIBDB_REGF(0x10030003,  20,  4,  4, 0,      "f3",  3, 16,  4, "f", "FR", 0,0) \
	CONTENTS_LIBDB_REGF(0x10030004,  24,  4,  4, 0,      "f4",  4, 16,  4, "f", "FR", 0,0) \
	CONTENTS_LIBDB_REGF(0x10030005,  28,  4,  4, 0,      "f5",  5, 16,  4, "f", "FR", 0,0) \
	CONTENTS_LIBDB_REGF(0x10030006,  32,  4,  4, 0,      "f6",  6, 16,  4, "f", "FR", 0,0) \
	CONTENTS_LIBDB_REGF(0x10030007,  36,  4,  4, 0,      "f7",  7, 16,  4, "f", "FR", 0,0) \
	CONTENTS_LIBDB_REGF(0x10030008,  40,  4,  4, 0,      "f8",  8, 16,  4, "f", "FR", 0,0) \
	CONTENTS_LIBDB_REGF(0x10030009,  44,  4,  4, 0,      "f9",  9, 16,  4, "f", "FR", 0,0) \
	CONTENTS_LIBDB_REGF(0x1003000A,  48,  4,  4, 0,     "f10", 10, 16,  4, "f", "FR", 0,0) \
	CONTENTS_LIBDB_REGF(0x1003000B,  52,  4,  4, 0,     "f11", 11, 16,  4, "f", "FR", 0,0) \
	CONTENTS_LIBDB_REGF(0x1003000C,  56,  4,  4, 0,     "f12", 12, 16,  4, "f", "FR", 0,0) \
	CONTENTS_LIBDB_REGF(0x1003000D,  60,  4,  4, 0,     "f13", 13, 16,  4, "f", "FR", 0,0) \
	CONTENTS_LIBDB_REGF(0x1003000E,  64,  4,  4, 0,     "f14", 14, 16,  4, "f", "FR", 0,0) \
	CONTENTS_LIBDB_REGF(0x1003000F,  68,  4,  4, 0,     "f15", 15, 16,  4, "f", "FR", 0,0) \
	/* end */

#define XCHAL_CP1_SA_CONTENTS_LIBDB_NUM	0
#define XCHAL_CP1_SA_CONTENTS_LIBDB	/* empty */

#define XCHAL_CP2_SA_CONTENTS_LIBDB_NUM	17
#define XCHAL_CP2_SA_CONTENTS_LIBDB	\
	CONTENTS_LIBDB_UREG(0x0C000001,   0,  4,  4, 0,     "UR1",   1, 0xFFFFFFFF, 0,0) \
	CONTENTS_LIBDB_REGF(0x10050000,   4,  4,  4, 0,    "i320",  0, 16,  4, "i32", "i32", 0,0) \
	CONTENTS_LIBDB_REGF(0x10050001,   8,  4,  4, 0,    "i321",  1, 16,  4, "i32", "i32", 0,0) \
	CONTENTS_LIBDB_REGF(0x10050002,  12,  4,  4, 0,    "i322",  2, 16,  4, "i32", "i32", 0,0) \
	CONTENTS_LIBDB_REGF(0x10050003,  16,  4,  4, 0,    "i323",  3, 16,  4, "i32", "i32", 0,0) \
	CONTENTS_LIBDB_REGF(0x10050004,  20,  4,  4, 0,    "i324",  4, 16,  4, "i32", "i32", 0,0) \
	CONTENTS_LIBDB_REGF(0x10050005,  24,  4,  4, 0,    "i325",  5, 16,  4, "i32", "i32", 0,0) \
	CONTENTS_LIBDB_REGF(0x10050006,  28,  4,  4, 0,    "i326",  6, 16,  4, "i32", "i32", 0,0) \
	CONTENTS_LIBDB_REGF(0x10050007,  32,  4,  4, 0,    "i327",  7, 16,  4, "i32", "i32", 0,0) \
	CONTENTS_LIBDB_REGF(0x10050008,  36,  4,  4, 0,    "i328",  8, 16,  4, "i32", "i32", 0,0) \
	CONTENTS_LIBDB_REGF(0x10050009,  40,  4,  4, 0,    "i329",  9, 16,  4, "i32", "i32", 0,0) \
	CONTENTS_LIBDB_REGF(0x1005000A,  44,  4,  4, 0,   "i3210", 10, 16,  4, "i32", "i32", 0,0) \
	CONTENTS_LIBDB_REGF(0x1005000B,  48,  4,  4, 0,   "i3211", 11, 16,  4, "i32", "i32", 0,0) \
	CONTENTS_LIBDB_REGF(0x1005000C,  52,  4,  4, 0,   "i3212", 12, 16,  4, "i32", "i32", 0,0) \
	CONTENTS_LIBDB_REGF(0x1005000D,  56,  4,  4, 0,   "i3213", 13, 16,  4, "i32", "i32", 0,0) \
	CONTENTS_LIBDB_REGF(0x1005000E,  60,  4,  4, 0,   "i3214", 14, 16,  4, "i32", "i32", 0,0) \
	CONTENTS_LIBDB_REGF(0x1005000F,  64,  4,  4, 0,   "i3215", 15, 16,  4, "i32", "i32", 0,0) \
	/* end */

#define XCHAL_CP3_SA_CONTENTS_LIBDB_NUM	0
#define XCHAL_CP3_SA_CONTENTS_LIBDB	/* empty */

#define XCHAL_CP4_SA_CONTENTS_LIBDB_NUM	0
#define XCHAL_CP4_SA_CONTENTS_LIBDB	/* empty */

#define XCHAL_CP5_SA_CONTENTS_LIBDB_NUM	18
#define XCHAL_CP5_SA_CONTENTS_LIBDB	\
	CONTENTS_LIBDB_UREG(0x0C000002,   0,  4,  4, 0,   "S64HI",   2, 0xFFFFFFFF, 0,0) \
	CONTENTS_LIBDB_UREG(0x0C000003,   4,  4,  4, 0,   "S64LO",   3, 0xFFFFFFFF, 0,0) \
	CONTENTS_LIBDB_REGF(0x10040000,   8,  8,  8, 0,    "i640",  0, 16,  8, "i64", "i64", 0,0) \
	CONTENTS_LIBDB_REGF(0x10040001,  16,  8,  8, 0,    "i641",  1, 16,  8, "i64", "i64", 0,0) \
	CONTENTS_LIBDB_REGF(0x10040002,  24,  8,  8, 0,    "i642",  2, 16,  8, "i64", "i64", 0,0) \
	CONTENTS_LIBDB_REGF(0x10040003,  32,  8,  8, 0,    "i643",  3, 16,  8, "i64", "i64", 0,0) \
	CONTENTS_LIBDB_REGF(0x10040004,  40,  8,  8, 0,    "i644",  4, 16,  8, "i64", "i64", 0,0) \
	CONTENTS_LIBDB_REGF(0x10040005,  48,  8,  8, 0,    "i645",  5, 16,  8, "i64", "i64", 0,0) \
	CONTENTS_LIBDB_REGF(0x10040006,  56,  8,  8, 0,    "i646",  6, 16,  8, "i64", "i64", 0,0) \
	CONTENTS_LIBDB_REGF(0x10040007,  64,  8,  8, 0,    "i647",  7, 16,  8, "i64", "i64", 0,0) \
	CONTENTS_LIBDB_REGF(0x10040008,  72,  8,  8, 0,    "i648",  8, 16,  8, "i64", "i64", 0,0) \
	CONTENTS_LIBDB_REGF(0x10040009,  80,  8,  8, 0,    "i649",  9, 16,  8, "i64", "i64", 0,0) \
	CONTENTS_LIBDB_REGF(0x1004000A,  88,  8,  8, 0,   "i6410", 10, 16,  8, "i64", "i64", 0,0) \
	CONTENTS_LIBDB_REGF(0x1004000B,  96,  8,  8, 0,   "i6411", 11, 16,  8, "i64", "i64", 0,0) \
	CONTENTS_LIBDB_REGF(0x1004000C, 104,  8,  8, 0,   "i6412", 12, 16,  8, "i64", "i64", 0,0) \
	CONTENTS_LIBDB_REGF(0x1004000D, 112,  8,  8, 0,   "i6413", 13, 16,  8, "i64", "i64", 0,0) \
	CONTENTS_LIBDB_REGF(0x1004000E, 120,  8,  8, 0,   "i6414", 14, 16,  8, "i64", "i64", 0,0) \
	CONTENTS_LIBDB_REGF(0x1004000F, 128,  8,  8, 0,   "i6415", 15, 16,  8, "i64", "i64", 0,0) \
	/* end */

#define XCHAL_CP6_SA_CONTENTS_LIBDB_NUM	17
#define XCHAL_CP6_SA_CONTENTS_LIBDB	\
	CONTENTS_LIBDB_UREG(0x0C000000,   0,  4,  4, 0,     "UR0",   0, 0x0000FFFF, 0,0) \
	CONTENTS_LIBDB_REGF(0x10060000,   4,  2,  2, 0,    "i160",  0, 16,  2, "i16", "i16", 0,0) \
	CONTENTS_LIBDB_REGF(0x10060001,   6,  2,  2, 0,    "i161",  1, 16,  2, "i16", "i16", 0,0) \
	CONTENTS_LIBDB_REGF(0x10060002,   8,  2,  2, 0,    "i162",  2, 16,  2, "i16", "i16", 0,0) \
	CONTENTS_LIBDB_REGF(0x10060003,  10,  2,  2, 0,    "i163",  3, 16,  2, "i16", "i16", 0,0) \
	CONTENTS_LIBDB_REGF(0x10060004,  12,  2,  2, 0,    "i164",  4, 16,  2, "i16", "i16", 0,0) \
	CONTENTS_LIBDB_REGF(0x10060005,  14,  2,  2, 0,    "i165",  5, 16,  2, "i16", "i16", 0,0) \
	CONTENTS_LIBDB_REGF(0x10060006,  16,  2,  2, 0,    "i166",  6, 16,  2, "i16", "i16", 0,0) \
	CONTENTS_LIBDB_REGF(0x10060007,  18,  2,  2, 0,    "i167",  7, 16,  2, "i16", "i16", 0,0) \
	CONTENTS_LIBDB_REGF(0x10060008,  20,  2,  2, 0,    "i168",  8, 16,  2, "i16", "i16", 0,0) \
	CONTENTS_LIBDB_REGF(0x10060009,  22,  2,  2, 0,    "i169",  9, 16,  2, "i16", "i16", 0,0) \
	CONTENTS_LIBDB_REGF(0x1006000A,  24,  2,  2, 0,   "i1610", 10, 16,  2, "i16", "i16", 0,0) \
	CONTENTS_LIBDB_REGF(0x1006000B,  26,  2,  2, 0,   "i1611", 11, 16,  2, "i16", "i16", 0,0) \
	CONTENTS_LIBDB_REGF(0x1006000C,  28,  2,  2, 0,   "i1612", 12, 16,  2, "i16", "i16", 0,0) \
	CONTENTS_LIBDB_REGF(0x1006000D,  30,  2,  2, 0,   "i1613", 13, 16,  2, "i16", "i16", 0,0) \
	CONTENTS_LIBDB_REGF(0x1006000E,  32,  2,  2, 0,   "i1614", 14, 16,  2, "i16", "i16", 0,0) \
	CONTENTS_LIBDB_REGF(0x1006000F,  34,  2,  2, 0,   "i1615", 15, 16,  2, "i16", "i16", 0,0) \
	/* end */

#define XCHAL_CP7_SA_CONTENTS_LIBDB_NUM	0
#define XCHAL_CP7_SA_CONTENTS_LIBDB	/* empty */






/*----------------------------------------------------------------------
				MISC
  ----------------------------------------------------------------------*/

#if 0	/* is there something equivalent for user TIE? */
#define XCHAL_CORE_ID			"linux_test"	/* configuration's alphanumeric core identifier
							   (CoreID) set in the Xtensa Processor Generator */

#define XCHAL_BUILD_UNIQUE_ID		0x000063EB	/* software build-unique ID (22-bit) */

/*  These definitions describe the hardware targeted by this software:  */
#define XCHAL_HW_CONFIGID0		0xC1FFDFFE	/* config ID reg 0 value (upper 32 of 64 bits) */
#define XCHAL_HW_CONFIGID1		0x008063EB	/* config ID reg 1 value (lower 32 of 64 bits) */
#define XCHAL_CONFIGID0			XCHAL_HW_CONFIGID0	/* for backward compatibility only -- don't use! */
#define XCHAL_CONFIGID1			XCHAL_HW_CONFIGID1	/* for backward compatibility only -- don't use! */
#define XCHAL_HW_RELEASE_MAJOR		1050	/* major release of targeted hardware */
#define XCHAL_HW_RELEASE_MINOR		1	/* minor release of targeted hardware */
#define XCHAL_HW_RELEASE_NAME		"T1050.1"	/* full release name of targeted hardware */
#define XTHAL_HW_REL_T1050	1
#define XTHAL_HW_REL_T1050_1	1
#define XCHAL_HW_CONFIGID_RELIABLE	1
#endif /*0*/



/*----------------------------------------------------------------------
				ISA
  ----------------------------------------------------------------------*/

#if 0	/* these probably don't belong here, but are related to or implemented using TIE */
#define XCHAL_HAVE_BOOLEANS		1	/* 1 if booleans option configured, 0 otherwise */
/*  Misc instructions:  */
#define XCHAL_HAVE_MUL32		0	/* 1 if 32-bit integer multiply option configured, 0 otherwise */
#define XCHAL_HAVE_MUL32_HIGH		0	/* 1 if MUL32 option includes MULUH and MULSH, 0 otherwise */

#define XCHAL_HAVE_FP			1	/* 1 if floating point option configured, 0 otherwise */
#endif /*0*/


#endif /*XTENSA_CONFIG_TIE_H*/

