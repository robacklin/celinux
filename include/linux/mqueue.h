#ifndef _LINUX_MQUEUE_H
#define _LINUX_MQUEUE_H

#include <asm/types.h>

#define MQ_MAX		64	/* max number of message queues */
#define MQ_MSGSIZE 	16384	/* max message size */
#define MQ_MAXSYSSIZE	1048576	/* max size that all m.q. can have together */
#define MQ_PRIO_MAX 	32768	/* max priority */
/* MQ_MAXMSG is defined in mqueue_fs_i.h */

typedef int mqd_t;

/* the same for send & receive */
struct ioctl_mq_sndrcv {
	__u64	msg_ptr;
	__u32	msg_len;
	__u64	msg_prio;	/* it is long or long* */
	__u64	timeout;
};


#define MQ_IOC_CREATE	_IOW(0xB2, 0, struct kern_mq_attr)
#define MQ_IOC_GETATTR	_IOR(0xB2, 1, struct kern_mq_attr)
#define MQ_IOC_SEND	_IOW(0xB2, 2, struct ioctl_mq_sndrcv)
#define MQ_IOC_RECEIVE	_IOR(0xB2, 3, struct ioctl_mq_sndrcv)
#define MQ_IOC_NOTIFY	_IOW(0xB2, 4, struct sigevent)
#define MQ_IOC_CLOSE	_IOW(0xB2, 5, void)
#define MQ_IOC_SETATTR	_IOW(0xB2, 6, void)

#endif
