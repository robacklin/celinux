/*
 * m1543.h - registers definitions for the M1543C B1 Super I/O
 */
#define	CFG_PORT	1

#if CFG_PORT
#define	CONFIG_PORT	0x3f0
#define	INDEX_PORT	0x3f0
#define	DATA_PORT	0x3f1
#else
#define	CONFIG_PORT	0x370
#define	INDEX_PORT	0x370
#define	DATA_PORT	0x371
#endif

#define	CFG_KEY1	0x51
#define	CFG_KEY2	0x23
#define	EXIT_KEY	0xbb

#define	INDEX_0x02	0x02
#define	INDEX_0x07	0x07	/* Logical device number	*/
#define	LOG_DEV_0	0x00
#define	LOG_DEV_1	0x01
#define	LOG_DEV_2	0x02
#define	LOG_DEV_3	0x03
#define	LOG_DEV_4	0x04
#define	LOG_DEV_5	0x05
#define	LOG_DEV_6	0x06
#define	LOG_DEV_7	0x07
#define	LOG_DEV_8	0x08
#define	LOG_DEV_9	0x09
#define	LOG_DEV_A	0x0a
#define	LOG_DEV_B	0x0b
#define	LOG_DEV_C	0x0c
#define	LOG_DEV_D	0x0d

#define LOG_DEV_FDC     LOG_DEV_0
#define LOG_DEV_IDE1    LOG_DEV_1
#define LOG_DEV_IDE2    LOG_DEV_2
#define LOG_DEV_PRT     LOG_DEV_3
#define LOG_DEV_COM1    LOG_DEV_4
#define LOG_DEV_COM2    LOG_DEV_5
#define LOG_DEV_KBC     LOG_DEV_7

#define	INDEX_0x1F	0x1f
#define	INDEX_0x20	0x20
#define	INDEX_0x21	0x21
#define	INDEX_0x22	0x22
#define	INDEX_0x23	0x23
#define	INDEX_0x2C	0x2c
#define	INDEX_0x2D	0x2d
#define	INDEX_0x30	0x30	/* Logical device enable	*/
#define	DEV_EN		0x01

#define	INDEX_0x60	0x60
#define	INDEX_0x61	0x61
#define	INDEX_0x70	0x70
#define	INDEX_0x72	0x72
#define	INDEX_0x74	0x74
#define	INDEX_0xF0	0xf0
#define	INDEX_0xF1	0xf1
#define	INDEX_0xF2	0xf2
#define	INDEX_0xF4	0xf4

#define INDEX_DEV_SEL       INDEX_0x07
#define INDEX_ACTIVE        INDEX_0x30
#define INDEX_SEL_IRQ       INDEX_0x70
#define INDEX_SEL_IO_HI     INDEX_0x60
#define INDEX_SEL_IO_LO     INDEX_0x61
#define INDEX_SEL_DMA       INDEX_0x74



#define	FDC_BASE	0x03f0
#define	PRN_BASE	0x0378
#define	UART1_BASE	0x03f8
#define	UART2_BASE	0x03e8
#define	UART3_BASE	0x02f8

#define I8259_M_CR	0x20
#define I8259_M_MR	0x21
#define I8259_S_CR	0xa0
#define I8259_S_MR	0xa1
