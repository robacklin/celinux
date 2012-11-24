/*
 *	$Id: ulinux-sh.patch,v 1.6 2003/12/19 01:04:57 seh Exp $
 *	I/O routines for SEGA Dreamcast
 */

#include <asm/io.h>
#include <asm/machvec.h>

unsigned long dreamcast_isa_port2addr(unsigned long offset)
{
	return offset + 0xa0000000;
}
