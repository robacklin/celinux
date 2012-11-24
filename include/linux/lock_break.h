#ifndef _LINUX_LOCK_BREAK_H
#define _LINUX_LOCK_BREAK_H

#if defined(CONFIG_LOCK_BREAK)

#define conditional_schedule_needed()	(unlikely(current->need_resched))

#define unconditional_schedule()					\
	do {								\
		if (unlikely(current->state != TASK_RUNNING))		\
			__set_current_state(TASK_RUNNING);		\
		schedule();						\
	} while(0)

#define conditional_schedule()						\
	do {								\
		if (conditional_schedule_needed())			\
			unconditional_schedule();			\
	} while (0)

#define DEFINE_RESCHED_COUNT		int __resched_count = 0
#define TEST_RESCHED_COUNT(n)		(++__resched_count > (n))
#define RESET_RESCHED_COUNT()		__resched_count = 0

#else	/* !CONFIG_LOCK_BREAK */

#define conditional_schedule_needed()	0
#define conditional_schedule()		do { } while(0)
#define unconditional_schedule()	do { } while(0)
#define DEFINE_RESCHED_COUNT		do { } while(0)
#define TEST_RESCHED_COUNT(n)		0
#define RESET_RESCHED_COUNT()		do { } while(0)

#endif	/* CONFIG_LOCK_BREAK */

#endif	/* _LINUX_LOCK_BREAK_H */
