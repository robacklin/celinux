#ifndef __ASM_XTENSA_IPC_H
#define __ASM_XTENSA_IPC_H

/*
 * include/asm-xtensa/ipc.h
 *
 * These are used to wrap system calls on Xtensa.  See
 * arch/xtensa/kernel/ipc.c for ugly details..
 *
 * This file is subject to the terms and conditions of the GNU General
 * Public License.  See the file "COPYING" in the main directory of
 * this archive for more details.
 */

struct ipc_kludge {
	struct msgbuf *msgp;
	long msgtyp;
};

#define SEMOP		 1
#define SEMGET		 2
#define SEMCTL		 3
#define MSGSND		11
#define MSGRCV		12
#define MSGGET		13
#define MSGCTL		14
#define SHMAT		21
#define SHMDT		22
#define SHMGET		23
#define SHMCTL		24

/* Used by the DIPC package, try and avoid reusing it */
#define DIPC            25

#define IPCCALL(version,op)	((version)<<16 | (op))

#endif /* __ASM_XTENSA_IPC_H */
