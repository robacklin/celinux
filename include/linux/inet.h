/* $USAGI: inet.h,v 1.13 2002/08/04 02:57:44 yoshfuji Exp $ */

/*
 *		Swansea University Computer Society NET3
 *
 *	This work is derived from NET2Debugged, which is in turn derived
 *	from NET2D which was written by:
 * 		Fred N. van Kempen, <waltje@uWalt.NL.Mugnet.ORG>
 *
 *		This work was derived from Ross Biro's inspirational work
 *		for the LINUX operating system.  His version numbers were:
 *
 *		$Id: Space.c,v     0.8.4.5  1992/12/12 19:25:04 bir7 Exp $
 *		$Id: arp.c,v       0.8.4.6  1993/01/28 22:30:00 bir7 Exp $
 *		$Id: arp.h,v       0.8.4.6  1993/01/28 22:30:00 bir7 Exp $
 *		$Id: dev.c,v       0.8.4.13 1993/01/23 18:00:11 bir7 Exp $
 *		$Id: dev.h,v       0.8.4.7  1993/01/23 18:00:11 bir7 Exp $
 *		$Id: eth.c,v       0.8.4.4  1993/01/22 23:21:38 bir7 Exp $
 *		$Id: eth.h,v       0.8.4.1  1992/11/10 00:17:18 bir7 Exp $
 *		$Id: icmp.c,v      0.8.4.9  1993/01/23 18:00:11 bir7 Exp $
 *		$Id: icmp.h,v      0.8.4.2  1992/11/15 14:55:30 bir7 Exp $
 * 		$Id: ip.c,v        0.8.4.8  1992/12/12 19:25:04 bir7 Exp $
 * 		$Id: ip.h,v        0.8.4.2  1993/01/23 18:00:11 bir7 Exp $
 * 		$Id: loopback.c,v  0.8.4.8  1993/01/23 18:00:11 bir7 Exp $
 * 		$Id: packet.c,v    0.8.4.7  1993/01/26 22:04:00 bir7 Exp $
 *		$Id: protocols.c,v 0.8.4.3  1992/11/15 14:55:30 bir7 Exp $
 *		$Id: raw.c,v       0.8.4.12 1993/01/26 22:04:00 bir7 Exp $
 *		$Id: sock.c,v      0.8.4.6  1993/01/28 22:30:00 bir7 Exp $
 *		$Id: sock.h,v      0.8.4.7  1993/01/26 22:04:00 bir7 Exp $
 *		$Id: tcp.c,v       0.8.4.16 1993/01/26 22:04:00 bir7 Exp $
 *		$Id: tcp.h,v       0.8.4.7  1993/01/22 22:58:08 bir7 Exp $
 *		$Id: timer.c,v     0.8.4.8  1993/01/23 18:00:11 bir7 Exp $
 *		$Id: timer.h,v     0.8.4.2  1993/01/23 18:00:11 bir7 Exp $
 *		$Id: udp.c,v       0.8.4.12 1993/01/26 22:04:00 bir7 Exp $
 *		$Id: udp.h,v       0.8.4.1  1992/11/10 00:17:18 bir7 Exp $
 *		$Id: we.c,v        0.8.4.10 1993/01/23 18:00:11 bir7 Exp $
 *		$Id: wereg.h,v     0.8.4.1  1992/11/10 00:17:18 bir7 Exp $
 *
 *		This program is free software; you can redistribute it and/or
 *		modify it under the terms of the GNU General Public License
 *		as published by the Free Software Foundation; either version
 *		2 of the License, or (at your option) any later version.
 */
#ifndef _LINUX_INET_H
#define _LINUX_INET_H

#ifdef __KERNEL__

struct net_proto;
struct in_addr;

extern void		inet_proto_init(struct net_proto *pro);
extern char		*in_ntop(struct in_addr *in, char *buf);
extern __u32		in_aton(const char *str);
#if defined(CONFIG_IPV6)
struct in6_addr;
extern char  *in6_ntop(const struct in6_addr *in6, char *buf);
#elif defined(CONFIG_IPV6_MODULE)
#include <linux/in6.h>
extern __inline__ char	*in6_ntop(const struct in6_addr *in6, char *buf){
	if (!buf)
		return NULL;
	sprintf(buf,
		"%04x:%04x:%04x:%04x:%04x:%04x:%04x:%04x",
		ntohs(in6->s6_addr16[0]), ntohs(in6->s6_addr16[1]),
		ntohs(in6->s6_addr16[2]), ntohs(in6->s6_addr16[3]),
		ntohs(in6->s6_addr16[4]), ntohs(in6->s6_addr16[5]),
		ntohs(in6->s6_addr16[6]), ntohs(in6->s6_addr16[7]));
	return buf;
}
#endif

#endif
#endif	/* _LINUX_INET_H */
