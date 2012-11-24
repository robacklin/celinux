
#ifndef __VLINK_VLINK_H__
#define __VLINK_VLINK_H__

#define	BUF_MAX	512

#include <vlink/osemu.h>

static char device_name[] = "/dev/vlink";

#define	KMC_VLINK_NULL	0
#define	KMC_VLINK_SEND	1
#define	KMC_VLINK_RECV	2



#define	KMC_PTSVR_READ	10
#define	KMC_PTSVR_WRITE	11
#define	KMC_PTSVR_STAT	12
#define	KMC_PTSVR_LOCK	13

typedef struct {
	char * buf;
	int size;
	int ret;
}PTSVR_STR,*PPTSVR_STR;

#define	PTSVR_UNLOCK_SCHEDULE		0x00
#define	PTSVR_LOCK_SCHEDULE		0x01
#define	PTSVR_LOCK1CHAR_SCHEDULE	0x02


/*
#define	PTSVR_UNLOCK_INTERRUPT		0x20
#define	PTSVR_LOCK_INTERRUPT		0x21
#define	PTSVR_LOCK1CHAR_INTERRUPT	0x22
#define	PTSVR_SCHEDULE			0x10
#define	PTSVR_INTERRUPT			0x20
#define	PTSVR_UNLOCK			0x00
#define	PTSVR_LOCK			0x01
#define	PTSVR_LOCK1CHAR			0x02

#define	PTSVR_LOCK_STAT			0x0f
#define	PTSVR_LOCK_FNC			0xf0
*/

#endif /* __VLINK_VLINK_H__ */

