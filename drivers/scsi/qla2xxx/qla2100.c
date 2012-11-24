/******************************************************************************
 *                  QLOGIC LINUX SOFTWARE
 *
 * QLogic ISP2x00 device driver for Linux 2.4.x
 * Copyright (C) 2003 Qlogic Corporation
 * (www.qlogic.com)
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2, or (at your option) any
 * later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 ******************************************************************************/

/*
 * This file set some defines that are required to compile the 
 * command source for 2100 module
 */
#define ISP2100

#if !defined(LINUX)
#define LINUX
#endif  /* LINUX not defined */
#if !defined(linux)
#define linux
#endif  /* linux not defined */
#if !defined(INTAPI)
#define INTAPI
#endif  /* INTAPI not defined */
/*
 * Include common setting 
 */
#include "qla_settings.h"

/*
 * Include common source 
 */
#include "qla2x00.c"
