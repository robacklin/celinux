#ifndef _LINUX_MQUEUE_FS_I_H
#define _LINUX_MQUEUE_FS_I_H

#include <asm/siginfo.h>

#define MQ_MAXMSG	40	/* max number of messages in each queue */
				
struct kern_mq_attr {
	__u32	mq_flags;	/* message queue flags */
	__u32	mq_maxmsg;	/* maximum number of messages */
	__u32	mq_msgsize;	/* maximum message size */
	__u32	mq_curmsgs;	/* number of messages currently queued */
};

struct ext_wait_queue {		/* queue of sleeping processes */
	struct task_struct *task;
	struct list_head list;
};

/* this stores extra data for inode - queue specific data */
struct mqueue_inode_info {
	struct kern_mq_attr attr;
	struct msg *messages[MQ_MAXMSG];

	pid_t notify_pid;
	struct sigevent notify;
	
	/* for processes waiting for free space or message (respectively) */
	/* this is left mainly because of poll */
	wait_queue_head_t wait_q[2];
	/* avoids extra invocations of wake_up */
	wait_queue_head_t wait_q2[2];
	struct ext_wait_queue e_wait_q[2];	/* 0=free space   1=message */
	
	__u32 qsize; /* size of queue in memory (msgs & struct) */	
	spinlock_t lock;
};


#endif
