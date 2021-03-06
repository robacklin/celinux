#ifndef _LINUX_PTRACE_H
#define _LINUX_PTRACE_H
/* ptrace.h */
/* structs and defines to help the user use the ptrace system call. */

/* has the defines to get at the registers. */

#define PTRACE_TRACEME		   0
#define PTRACE_PEEKTEXT		   1
#define PTRACE_PEEKDATA		   2
#define PTRACE_PEEKUSR		   3
#define PTRACE_POKETEXT		   4
#define PTRACE_POKEDATA		   5
#define PTRACE_POKEUSR		   6
#define PTRACE_CONT		   7
#define PTRACE_KILL		   8
#define PTRACE_SINGLESTEP	   9

#define PTRACE_ATTACH		0x10
#define PTRACE_DETACH		0x11

#define PTRACE_SYSCALL		  24

/* 0x4200-0x4300 are reserved for architecture-independent additions.  */
#define PTRACE_SETOPTIONS      0x4200
#define PTRACE_GETEVENTMSG     0x4201

/* options set using PTRACE_SETOPTIONS */
#define PTRACE_O_TRACESYSGOOD  0x00000001
#define PTRACE_O_TRACEFORK     0x00000002
#define PTRACE_O_TRACEVFORK    0x00000004
#define PTRACE_O_TRACECLONE    0x00000008
#define PTRACE_O_TRACEEXEC     0x00000010

/* Wait extended result codes for the above trace options.  */
#define PTRACE_EVENT_FORK      1
#define PTRACE_EVENT_VFORK     2
#define PTRACE_EVENT_CLONE     3
#define PTRACE_EVENT_EXEC      4

struct task_struct;
extern int ptrace_request(struct task_struct *child, long request, long addr, long data);
extern void ptrace_notify(int exit_code);

#include <asm/ptrace.h>

#endif
