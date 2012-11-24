#ifndef _LINUX_ELEVATOR_H
#define _LINUX_ELEVATOR_H

typedef int (elevator_merge_fn)(request_queue_t *, struct request **,
				struct list_head *, struct buffer_head *bh,
				int rw, int max_sectors);

typedef void (elevator_merge_cleanup_fn) (request_queue_t *, struct request *, int);

typedef void (elevator_merge_req_fn) (struct request *, struct request *);

struct elevator_s
{
	int read_latency;
	int write_latency;
	int max_bomb_segments;

	elevator_merge_fn *elevator_merge_fn;
	elevator_merge_req_fn *elevator_merge_req_fn;

	unsigned int queue_ID;
};

elevator_merge_fn		elevator_noop_merge;
elevator_merge_cleanup_fn	elevator_noop_merge_cleanup;
elevator_merge_req_fn		elevator_noop_merge_req;

elevator_merge_fn		elevator_linus_merge;
elevator_merge_cleanup_fn	elevator_linus_merge_cleanup;
elevator_merge_req_fn		elevator_linus_merge_req;

typedef struct blkelv_ioctl_arg_s {
	int queue_ID;
	int read_latency;
	int write_latency;
	int max_bomb_segments;
} blkelv_ioctl_arg_t;

#define BLKELVGET   _IOR(0x12,106,sizeof(blkelv_ioctl_arg_t))
#define BLKELVSET   _IOW(0x12,107,sizeof(blkelv_ioctl_arg_t))

extern int blkelvget_ioctl(elevator_t *, blkelv_ioctl_arg_t *);
extern int blkelvset_ioctl(elevator_t *, const blkelv_ioctl_arg_t *);

extern void elevator_init(elevator_t *, elevator_t);

/*
 * Return values from elevator merger
 */
#define ELEVATOR_NO_MERGE	0
#define ELEVATOR_FRONT_MERGE	1
#define ELEVATOR_BACK_MERGE	2

static inline int elevator_request_latency(elevator_t * elevator, int rw)
{
	int latency;

	latency = elevator->read_latency;
	if (rw != READ)
		latency = elevator->write_latency;

	return latency;
}

#define ELV_LINUS_SEEK_COST	16

#define ELEVATOR_NOOP							\
((elevator_t) {								\
	0,				/* read_latency */		\
	0,				/* write_latency */		\
	0,				/* max_bomb_segments */		\
	elevator_noop_merge,		/* elevator_merge_fn */		\
	elevator_noop_merge_req,	/* elevator_merge_req_fn */	\
	})

#define ELEVATOR_LINUS							\
((elevator_t) {								\
	2048,				/* read passovers */		\
	8192,				/* write passovers */		\
	6,								\
	elevator_linus_merge,		/* elevator_merge_fn */		\
	elevator_linus_merge_req,	/* elevator_merge_req_fn */	\
	})

#endif
