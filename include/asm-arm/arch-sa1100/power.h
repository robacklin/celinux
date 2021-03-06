/*
 * suspend defines
 *
 */


#define SLEEP_PARAM_USER_R0		0
#define SLEEP_PARAM_USER_R1		1
#define SLEEP_PARAM_USER_R2		2
#define SLEEP_PARAM_USER_R3		3
#define SLEEP_PARAM_USER_R4		4
#define SLEEP_PARAM_USER_R5		5
#define SLEEP_PARAM_USER_R6		6
#define SLEEP_PARAM_USER_R7		7
#define SLEEP_PARAM_USER_R8		8
#define SLEEP_PARAM_USER_R9		9
#define SLEEP_PARAM_USER_R10		10		
#define SLEEP_PARAM_USER_R11		11
#define SLEEP_PARAM_USER_R12		12
#define SLEEP_PARAM_USER_R13		13
#define SLEEP_PARAM_USER_R14		14

#define SLEEP_PARAM_PC			15

#define SLEEP_PARAM_SVC_R13		16
#define SLEEP_PARAM_SVC_R14		17

#define SLEEP_PARAM_ABORT_R13		18
#define SLEEP_PARAM_ABORT_R14		19

#define SLEEP_PARAM_UNDEF_R13		20
#define SLEEP_PARAM_UNDEF_R14		21

#define SLEEP_PARAM_IRQ_R13		22
#define SLEEP_PARAM_IRQ_R14		23

#define SLEEP_PARAM_FIQ_R8		24
#define SLEEP_PARAM_FIQ_R9		25
#define SLEEP_PARAM_FIQ_R10		26
#define SLEEP_PARAM_FIQ_R11		27
#define SLEEP_PARAM_FIQ_R12		28
#define SLEEP_PARAM_FIQ_R13		29
#define SLEEP_PARAM_FIQ_R14		30

#define SLEEP_PARAM_CPSR		31
#define SLEEP_PARAM_SVC_SPSR		32
#define SLEEP_PARAM_ABORT_SPSR		33
#define SLEEP_PARAM_UNDEF_SPSR		34
#define SLEEP_PARAM_IRQ_SPSR		35
#define SLEEP_PARAM_FIQ_SPSR		36

#define SLEEP_PARAM_CP15_R1		37
#define SLEEP_PARAM_CP15_R2		38
#define SLEEP_PARAM_CP15_R3		39
#define SLEEP_PARAM_CP15_R5		40
#define SLEEP_PARAM_CP15_R6		41
#define SLEEP_PARAM_CP15_R13		42

#define SLEEP_PARAM_SIZE		(SLEEP_PARAM_CP15_R13 + 1)


#ifndef __ASSEMBLY__

int sa1110_suspend(void);

#endif

