/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 */
#ifndef __ASM_TX4927_PMON_H
#define __ASM_TX4927_PMON_H

struct pmon_vector {
	int (*read)(int fd, char *buf, int size);
	int (*write)(int fd, char *buf, int size);
	int (*open)(char *name, int mode);
	int (*close)(int fd);
	int (*ioctl)(int fd, int request, void* arg);
	int (*printf)(char *fmstr, ...);
	int (*vsprintf)(char *dst, char *fmtstr, ...);
	int (*ttctl)(int fd, int op, int ap1, int ap2);
	void (*exit)(int status);
	char *(*getenv)(char *name);
	void (*onintr)(int code, int *dat);
	void (*flush_cache)(int cache);
	void (*_exception)(void);
	int (*_fpstatesz)(void);
	void (*_fpinit)(void);
	void *(*_fpstate)(void);
	void (*cop1)(void);
	int (*adr2symoff)(char *dst, int value, int width);
	int (*sym2adr)(int *v, char *label);
	int (*getclkfreq)(void);
	void (*_clkinit)(void);
	int (*interpret)(const char *cmd, char *outbuf, int len);
};

extern struct pmon_vector *pmon_vector;

/* from pmonlib.c */
extern void prom_printf(char *fmt, ...);
/* pmonlib support routine */
extern void prom_putchar(char c);

#define pmon_printf prom_printf

extern void pmon_halt(void) __attribute__((noreturn));
extern void set_pmon_debug_traps(void);
extern void pmon_breakpoint(void);

#define PMON_VECTOR	0xbfc00200
#define PMON_VECTOR_R4K	0xbfc00500 /* with R4KEXCEPTIONS */
#define PMON_VECTOR_TX39	0xbfc00500 /* with TX39_DEBUG_EXCEPTIONS */

#endif /* __ASM_TX4927_PMON_H */
