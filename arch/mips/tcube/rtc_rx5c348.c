/*
 *      arch/mips/tcube/rtc_rx5c348.c
 *
 *      Real Time Clock interface for Linux on T-Cube.
 *      (RICOH Co., Ltd., Rx5C348B)
 *
 *      Copyright (c) 2003 Lineo Solutions, Inc.
 *
 *      Based on rtc_ricoh.c
 *
 *      This program is free software; you can redistribute it and/or
 *      modify it under the terms of the GNU General Public License
 *      as published by the Free Software Foundation; either version
 *      2 of the License, or (at your option) any later version.
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/time.h>
#include <linux/kernel.h>

#include <asm/time.h>
#include <asm/addrspace.h>
#include <asm/delay.h>
#include <asm/debug.h>
#include <asm/tcube.h>

#undef  DEBUG
////#define  DEBUG 1
#define RTC_DELAY

////#define  DEBUG_PRINT 1
#ifdef  DEBUG_PRINT
#define DBG(x...)       printk(x)
#else
#define DBG(x...)
#endif

#define SET_32_BIT	0xffffffff
#define CLR_32_BIT	0x00000000

#define GPIO_3_INTR	(0x1 <<  3)
#define GPIO_4_CE	(0x1 <<  4)
#define GPIO_25_S1CLK	(0x1 << 25)
#define GPIO_26_S1DO	(0x1 << 26)
#define GPIO_27_S1DI	(0x1 << 27)
#define GPIO_CSI1_PIN	(GPIO_25_S1CLK | GPIO_26_S1DO | GPIO_27_S1DI)

#define CSIn_MODE_CKP		(0x1 << 12)
#define CSIn_MODE_DAP		(0x1 << 11)
#define CSIn_MODE_CKS_MASK	(0x7 <<  8)
#define CSIn_MODE_CKS_833333MHZ	(0x1 <<  8)
#define CSIn_MODE_CKS_416667MHZ	(0x2 <<  8)
#define CSIn_MODE_CKS_208333MHZ	(0x3 <<  8)
#define CSIn_MODE_CKS_104167MHZ	(0x4 <<  8)
#define CSIn_MODE_CKS_052083MHZ	(0x5 <<  8)
#define CSIn_MODE_CKS_0260417HZ	(0x6 <<  8)	/* Default */
#define CSIn_MODE_CSIE		(0x1 <<  7)
#define CSIn_MODE_TRMD		(0x1 <<  6)
#define CSIn_MODE_CCL_16	(0x1 <<  5)
#define CSIn_MODE_DIR_LSB	(0x1 <<  4)
#define CSIn_MODE_AUTO		(0x1 <<  2)
#define CSIn_MODE_CSOT		(0x1 <<  0)

#define CSIn_INT_CSIEND		(0x1 << 15)
#define CSIn_INT_T_EMP		(0x1 <<  8)
#define CSIn_INT_R_OVER		(0x1 <<  0)

#if defined(CONFIG_MIPS_TCUBE_RTC)
extern int (*rtc_get_alm_time)(struct rtc_time *);
extern int (*rtc_set_alm_time)(struct rtc_time *);
extern int (*rtc_get_ctrl_reg)(u8 *);
extern int (*rtc_set_ctrl_reg)(u8 *);
#endif

static inline void reg_set32(u32 offset, u32 mask, u32 val)
{
	u32 val0 = io_in32(offset);
	io_out32(offset, (val & mask) | (val0 & ~mask));
}

static inline void csi1_reset(void)
{
	/* CSI1 reset */
	reg_set32(CSI1_CNT, 0x00008000, SET_32_BIT);	/* set CSIRST bit */
	__delay(100000);
	reg_set32(CSI1_CNT, 0x00008000, CLR_32_BIT);	/* clear CSIRST bit */
	/* set clock phase  */
	while(io_in32(CSI1_MODE) & 1);
	reg_set32(CSI1_MODE, CSIn_MODE_CSIE, CLR_32_BIT);
	reg_set32(CSI1_MODE, CSIn_MODE_CKP,  SET_32_BIT);
////	reg_set32(CSI1_MODE, CSIn_MODE_CKS_208333MHZ, SET_32_BIT);
	reg_set32(CSI1_MODE, CSIn_MODE_CKS_104167MHZ, SET_32_BIT);
	reg_set32(CSI1_MODE, CSIn_MODE_CSIE, SET_32_BIT);
	while(io_in32(CSI1_MODE) & CSIn_MODE_CSOT);
}

void static rtc_set_ce(u32 val)
{
#ifdef DEBUG
	printk("rtc_set_ce(%d)\n",val);
#endif
	reg_set32(GIU_PIO0, GPIO_4_CE, val ? SET_32_BIT : 0);
#ifdef RTC_DELAY
	__delay(100000);
#endif
}

void static rtc_write_burst(int adr, unsigned char *data, int dataLen)
{
	int i;
#ifdef DEBUG
	for (i = 0; i < dataLen; i++)
		printk(" rtc_write_burst : data=%08x\n", data[i]);
#endif
	DBG(" rtc_write_burst : adr=0x%02x\n", adr);
	csi1_reset();
	while(io_in32(CSI1_MODE) & CSIn_MODE_CSOT);
	reg_set32(CSI1_MODE, CSIn_MODE_AUTO, SET_32_BIT);
	reg_set32(CSI1_MODE, CSIn_MODE_TRMD, SET_32_BIT);
	io_out32(CSI1_INT, CSIn_INT_CSIEND);
	rtc_set_ce(1);

	DBG(" rtc_write_burst : CSI1_MODE=%08x\n", io_in32(CSI1_MODE));
	DBG(" rtc_write_burst : CSI1_CNT=%08x\n", io_in32(CSI1_CNT));
	io_out32(CSI1_SOTBF, ((adr << 4) | 0x00));

	for (i = 0; i < dataLen; i++) {
		io_out32(CSI1_SOTB, data[i]);
		while(!(io_in32(CSI1_INT) & CSIn_INT_CSIEND));
		io_out32(CSI1_INT, CSIn_INT_CSIEND);
	}
	while(io_in32(CSI1_MODE) & CSIn_MODE_CSOT);
	rtc_set_ce(0);
}

void static rtc_read_burst(int adr, unsigned char *data, int dataLen)
{
	int i;
	DBG(" rtc_read_burst : adr=0x%02x\n", adr);
	csi1_reset();
	while(io_in32(CSI1_MODE) & CSIn_MODE_CSOT);
	reg_set32(CSI1_MODE, CSIn_MODE_AUTO, CLR_32_BIT);
	reg_set32(CSI1_MODE, CSIn_MODE_TRMD, SET_32_BIT);
	io_out32(CSI1_INT, CSIn_INT_CSIEND);
	rtc_set_ce(1);

	DBG(" rtc_read_burst : CSI1_MODE=%08x\n", io_in32(CSI1_MODE));
	DBG(" rtc_read_burst : CSI1_CNT=%08x\n", io_in32(CSI1_CNT));
	io_out32(CSI1_SOTB, (((adr & 0xf) << 4) | 0x04));
	while(!(io_in32(CSI1_INT) & CSIn_INT_CSIEND));

	while(io_in32(CSI1_MODE) & CSIn_MODE_CSOT);
	reg_set32(CSI1_MODE, CSIn_MODE_AUTO, SET_32_BIT);
	reg_set32(CSI1_MODE, CSIn_MODE_TRMD, CLR_32_BIT);
	io_out32(CSI1_INT, CSIn_INT_CSIEND);

	udelay(50);
	DBG(" rtc_read_burst : CSI1_MODE=%08x\n", io_in32(CSI1_MODE));
	DBG(" rtc_read_burst : CSI1_CNT=%08x\n", io_in32(CSI1_CNT));
	io_in32(CSI1_SIRB);	/* dummy read */
////	io_out32(CSI1_INT, CSIn_INT_CSIEND);

	for (i = 0; i < dataLen; i++) {
		while(!(io_in32(CSI1_INT) & CSIn_INT_CSIEND));
		io_out32(CSI1_INT, CSIn_INT_CSIEND);
		data[i] = io_in32(CSI1_SIRB);
	}
	while(io_in32(CSI1_MODE) & CSIn_MODE_CSOT);
	rtc_set_ce(0);
#ifdef DEBUG
	for (i = 0; i < dataLen; i++)
		printk(" rtc_read_burst : data=%08x\n", data[i]);
#endif
}

#define BcdToDec(x) (((x)>>4)*10+((x)&0x0f))
#define DecToBcd(x)  (((x)/10<<4)+(x)%10)

static unsigned long
rtc_ricoh_rx5c348_get_time(void)
{	
	u8 date[7];
	unsigned int year, month, day, hour, minute, second;
  
	rtc_read_burst(0, date, sizeof(date));
  
	year   = BcdToDec(date[6]) + (date[5] & 0x80 ? 2000 : 1900);
	month  = BcdToDec(date[5] & 0x1f);
	day    = BcdToDec(date[4]);
	hour   = BcdToDec(date[2]);
	minute = BcdToDec(date[1]);
	second = BcdToDec(date[0]);

#ifdef DEBUG
	printk(KERN_INFO "rtc_ricoh_rx5c348_get_time: %d/%02d/%02d %02d:%02d:%02d\n",
	year, month, day, hour, minute, second);
#endif
	return mktime(year, month, day, hour, minute, second);
}

static int 
rtc_ricoh_rx5c348_set_time(unsigned long t)
{
	u8 date[7];
	struct rtc_time tm;
  
	to_tm(t, &tm);
	date[0] = DecToBcd(tm.tm_sec);
	date[1] = DecToBcd(tm.tm_min);
	date[2] = DecToBcd(tm.tm_hour);
	date[4] = DecToBcd(tm.tm_mday);
	date[5] = DecToBcd(tm.tm_mon+1) + (tm.tm_year > 1999 ? 0x80 : 0);
	date[6] = DecToBcd(tm.tm_year > 1999 ? tm.tm_year-2000 : tm.tm_year-1900);
  
	rtc_write_burst(0, date, 3);
	rtc_write_burst(4, date+4, 3);

#ifdef DEBUG
	printk(KERN_INFO "rtc_ricoh_rx5c348_set_time:t=%ld %d/%02d/%02d %02d:%02d:%02d\n",
	 t,tm.tm_year,tm.tm_mon+1,tm.tm_mday,tm.tm_hour,tm.tm_min,tm.tm_sec);
#endif
	return 0;
}

static int
rtc_ricoh_rx5c348_get_alm_time(struct rtc_time *alm_tm)
{	
	u8 alm_time[2];
  
	rtc_read_burst(0xb, alm_time, sizeof(alm_time));
  
	alm_tm->tm_hour = BcdToDec(alm_time[1]);
	alm_tm->tm_min  = BcdToDec(alm_time[0]);

#ifdef DEBUG
	printk(KERN_INFO "rtc_ricoh_rx5c348_get_alm_time: %02d:%02d\n",
					alm_tm->tm_hour, alm_tm->tm_min);
#endif
	return 0;
}

static int 
rtc_ricoh_rx5c348_set_alm_time(struct rtc_time *alm_tm)
{
	u8 alm_time[2];
  
	alm_time[0] = DecToBcd(alm_tm->tm_min);
	alm_time[1] = DecToBcd(alm_tm->tm_hour);
  
	rtc_write_burst(0xb, alm_time, sizeof(alm_time));

#ifdef DEBUG
	printk(KERN_INFO "rtc_ricoh_rx5c348_set_alm_time: %02d:%02d\n",
					alm_time[0], alm_time[1]);
#endif
	return 0;
}

static int 
rtc_ricoh_rx5c348_get_ctrl_reg(u8 *ctrl_reg)
{
	rtc_read_burst(0xe, ctrl_reg, 2);

#ifdef DEBUG
	printk(KERN_INFO "rtc_ricoh_rx5c348_get_ctrl_reg: ctrl_reg1=%02x\n",
					ctrl_reg[0]);
	printk(KERN_INFO "rtc_ricoh_rx5c348_get_ctrl_reg: ctrl_reg2=%02x\n",
					ctrl_reg[1]);
#endif
	return 0;
}

static int 
rtc_ricoh_rx5c348_set_ctrl_reg(u8 *ctrl_reg)
{
	rtc_write_burst(0xe, ctrl_reg, 2);

#ifdef DEBUG
	printk(KERN_INFO "rtc_ricoh_rx5c348_set_ctrl_reg: ctrl_reg1=%02x\n",
					ctrl_reg[0]);
	printk(KERN_INFO "rtc_ricoh_rx5c348_set_ctrl_reg: ctrl_reg2=%02x\n",
					ctrl_reg[1]);
#endif
	return 0;
}

static int __init rtc_ricoh_rx5c348_init(void)
{
	unsigned char data;

	/* CSI1 reset  */
	io_set16(PIB_RESET, 0x40, 0xffff);
	__delay(10000);
	io_set16(PIB_RESET, 0x40, 0x0000);

	/* set GPIO3 , GPIO4 */
	reg_set32(GIU_FUNCSEL0, (GPIO_4_CE | GPIO_3_INTR), SET_32_BIT);
	/* clear GPIO25 , GPIO26 , GPIO27 */
	reg_set32(GIU_FUNCSEL0, GPIO_CSI1_PIN, CLR_32_BIT);
	/* make GPIO4 output */
	reg_set32(GIU_DIR0, GPIO_4_CE, SET_32_BIT);
	/* make GPIO3 input  */
	reg_set32(GIU_DIR0, GPIO_3_INTR, CLR_32_BIT);

	csi1_reset();

	rtc_read_burst(0x0e, &data, 1);
	if((data & 0x20) == 0) {	/* 24 hour */
		data |= 0x20;
		rtc_write_burst(0x0e, &data, 1);
#ifdef RTC_DELAY
		__delay(10000);
#endif
	}

	/* set the function pointers */
	rtc_get_time     = rtc_ricoh_rx5c348_get_time;
	rtc_set_time     = rtc_ricoh_rx5c348_set_time;
#if defined(CONFIG_MIPS_TCUBE_RTC)
	rtc_get_alm_time = rtc_ricoh_rx5c348_get_alm_time;
	rtc_set_alm_time = rtc_ricoh_rx5c348_set_alm_time;
	rtc_get_ctrl_reg = rtc_ricoh_rx5c348_get_ctrl_reg;
	rtc_set_ctrl_reg = rtc_ricoh_rx5c348_set_ctrl_reg;
#endif

	return 0;
}

module_init(rtc_ricoh_rx5c348_init);
