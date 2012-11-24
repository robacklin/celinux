/*
 *	Generic watchdog defines. Derived from..
 *
 * Berkshire PC Watchdog Defines
 * by Ken Hollis <khollis@bitgate.com>
 *
 */

#ifndef _LINUX_WATCHDOG_H
#define _LINUX_WATCHDOG_H

#include <linux/ioctl.h>

#define	WATCHDOG_IOCTL_BASE	'W'

struct watchdog_info {
	__u32 options;		/* Options the card/driver supports, see
				   WDIOF_xxx below. */
	__u32 firmware_version;	/* Firmware version of the card */
	__u8  identity[32];	/* Identity of the board */
};

/* Get information about what the watchdog timer supports. */
#define	WDIOC_GETSUPPORT     _IOR(WATCHDOG_IOCTL_BASE, 0, struct watchdog_info)

/* Return the current status of the watchdog. */
#define	WDIOC_GETSTATUS	     _IOR(WATCHDOG_IOCTL_BASE, 1, int)

/* Return the status of the watchdog when the driver was started.
   This will have set WFIOF_CARDRESET if the last reset was due to a
   watchdog, and if the watchdog had an alternate reason (besides
   timeout) for the reset, then one of the reason bits from the
   WDIOF_xxx will be set. */
#define	WDIOC_GETBOOTSTATUS  _IOR(WATCHDOG_IOCTL_BASE, 2, int)

/* Return the current temperature (only for cards that have a
   temperature probe). */
#define	WDIOC_GETTEMP	     _IOR(WATCHDOG_IOCTL_BASE, 3, int)

/* Set options (see WDIOS_xxx for options). */
#define	WDIOC_SETOPTIONS     _IOR(WATCHDOG_IOCTL_BASE, 4, int)

/* Ping the keepalive to keep us alive for another timeout. */
#define	WDIOC_KEEPALIVE	     _IOR(WATCHDOG_IOCTL_BASE, 5, int)

/* Set the timeout for the watchdog.  The parameter is an integer
   number of seconds, the watchdog timer will be set to the first
   value larger than the given time.  It will return EINVAL if the
   timeout is beyond the capability of the watchdog timer.  For some
   boards, it is a custom timeout that is board dependent.  Note that
   this function will automatically does the keepalive for the
   watchdog. */
#define	WDIOC_SETTIMEOUT     _IOWR(WATCHDOG_IOCTL_BASE, 6, int)

#define	WDIOC_GETTIMEOUT     _IOR(WATCHDOG_IOCTL_BASE, 7, int)

#define	WDIOF_UNKNOWN		-1	/* Unknown flag error */
#define	WDIOS_UNKNOWN		-1	/* Unknown status error */

/* Various options a card may support returned by GETSUPPORT, and also
   status returned by GETSTATUS.  If the bit is set in the options
   field returned by GETSUPPORT, then the value may be returned by
   GETSTATUS.  The SETTIMEOUT is the only real exception, it is returned
   by GETSUPPORT, indicating the card is capable, but set by separate
   SETTIMEOUT ioctl. */
#define	WDIOF_OVERHEAT		0x0001	/* Reset due to CPU overheat */
#define	WDIOF_FANFAULT		0x0002	/* Fan failed */
#define	WDIOF_EXTERN1		0x0004	/* External relay 1 */
#define	WDIOF_EXTERN2		0x0008	/* External relay 2 */
#define	WDIOF_POWERUNDER	0x0010	/* Power bad/power fault */
#define	WDIOF_CARDRESET		0x0020	/* Card previously reset the CPU */
#define WDIOF_POWEROVER		0x0040	/* Power over voltage */
#define WDIOF_SETTIMEOUT	0x0080	/* Set timeout (in seconds) */
#define WDIOF_MAGICCLOSE	0x0100	/* Supports magic close char */
#define	WDIOF_KEEPALIVEPING	0x8000	/* Keep alive ping reply */

/* The following are options that may be set by WDIOC_SETOPTIONS. */
#define	WDIOS_DISABLECARD	0x0001	/* Turn off the watchdog
					   timer. If the watchdog is
					   set to go off even if the
					   device gets closed, the
					   application can disable the
					   watchdog to prevent it from
					   resetting the card if the
					   reboot is not wanted.  Note
					   that setting the timeout or
					   re-opening the driver will
					   re-enable the watchdog. */
#define	WDIOS_ENABLECARD	0x0002	/* Turn on the watchdog timer */
#define	WDIOS_TEMPPANIC		0x0004	/* Kernel panic on temperature trip */

#endif  /* ifndef _LINUX_WATCHDOG_H */
