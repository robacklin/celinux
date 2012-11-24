/*
 * $Id: compatmac.h,v 1.56 2003/05/29 09:36:47 dwmw2 Exp $
 *
 * Extensions and omissions from the normal 'linux/compatmac.h'
 * files. hopefully this will end up empty as the 'real' one 
 * becomes fully-featured.
 */

#ifndef __LINUX_MTD_COMPATMAC_H__
#define __LINUX_MTD_COMPATMAC_H__

#include <linux/version.h>

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,4,10)
#error "This kernel is too old: not supported by this file"
#endif

	/* O(1) scheduler stuff. */

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,5,5) && !defined(__rh_config_h__)
#include <linux/sched.h>
static inline void __recalc_sigpending(void)
{
	recalc_sigpending(current);
}
#undef recalc_sigpending
#define recalc_sigpending() __recalc_sigpending ()

#if 0
/* MontaVista ships with the O(1) scheduler. */
#define set_user_nice(tsk, n) do { (tsk)->nice = n; } while(0)
#endif

#endif



#if LINUX_VERSION_CODE < KERNEL_VERSION(2,4,20)

#ifndef yield
#define yield() do { set_current_state(TASK_RUNNING); schedule(); } while(0)
#endif

#ifndef minor
#define major(d) (MAJOR(to_kdev_t(d)))
#define minor(d) (MINOR(to_kdev_t(d)))
#endif

#ifndef mk_kdev
#define mk_kdev(ma,mi) MKDEV(ma,mi)
#define kdev_t_to_nr(x)	(x)
#endif

#define need_resched() (current->need_resched)
#define cond_resched() do { if need_resched() { yield(); } } while(0)

#endif /* < 2.4.20 */

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,5,61)
#include <linux/sched.h>
static inline void __daemonize_modvers(void)
{
	daemonize();
}
#undef daemonize
#define daemonize(fmt, ...) do {						\
	snprintf(current->comm, sizeof(current->comm), fmt ,##__VA_ARGS__);	\
	__daemonize_modvers();							\
	} while(0)

static inline int __dequeue_signal25(struct task_struct *tsk, sigset_t *mask, siginfo_t *info)
{
	return dequeue_signal(mask, info);
}
#undef dequeue_signal
#define dequeue_signal __dequeue_signal25
#endif

	/* Module bits */

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,5,60)
#define try_module_get(m) try_inc_mod_count(m)
#define __module_get(m) do { if (!try_inc_mod_count(m)) BUG(); } while(0)
#define module_put(m) do { if (m) __MOD_DEC_USE_COUNT((struct module *)(m)); } while(0)
#define set_module_owner(x) do { x->owner = THIS_MODULE; } while(0)
#endif


	/* Random filesystem stuff, only for JFFS2 really */

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,5,5)
#define parent_ino(d) ((d)->d_parent->d_inode->i_ino)
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,5,12)
#define PageUptodate(x) Page_Uptodate(x)
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,5,48)
#define get_seconds() CURRENT_TIME
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,5,53)
#define generic_file_readonly_mmap generic_file_mmap
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,5,70)

#include <linux/kmod.h>
#include <linux/string.h>

/*static inline char *strlcpy(char *dest, const char *src, int len)
{
	dest[len-1] = 0;
	return strncpy(dest, src, len-1);
}*/

static inline int do_old_request_module(const char *mod)
{
	return request_module(mod);
}
#undef request_module
#define request_module(fmt, ...) \
 ({ char modname[32]; snprintf(modname, 31, fmt ,##__VA_ARGS__); do_old_request_module(modname); })

#endif /* 2.5.70 */

#endif /* __LINUX_MTD_COMPATMAC_H__ */
