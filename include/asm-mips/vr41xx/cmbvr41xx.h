/*
 * include/asm-mips/vr41xx/cmbvr41xx.h
 *
 * Include file for NEC CMB-VR4122/VR4131.
 *
 * Author: Yoichi Yuasa <yyuasa@mvista.com, or source@mvista.com> and
 *         Jun Sun <jsun@mvista.com, or source@mvista.com>
 *
 * 2002,2003 (c) MontaVista, Software, Inc. This file is licensed under
 * the terms of the GNU General Public License version 2. This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 */
#ifndef __NEC_CMBVR41XX_H
#define __NEC_CMBVR41XX_H

#include <linux/config.h>

#include <asm/addrspace.h>
#include <asm/vr41xx/vr41xx.h>

/*
 * Board specific address mapping
 */
#define VR41XX_PCI_MEM1_BASE		0x10000000
#define VR41XX_PCI_MEM1_SIZE		0x04000000
#define VR41XX_PCI_MEM1_MASK		0x7c000000

#define VR41XX_PCI_MEM2_BASE		0x14000000
#define VR41XX_PCI_MEM2_SIZE		0x02000000
#define VR41XX_PCI_MEM2_MASK		0x7e000000

#define VR41XX_PCI_IO_BASE		0x16000000
#define VR41XX_PCI_IO_SIZE		0x02000000
#define VR41XX_PCI_IO_MASK		0x7e000000

#define VR41XX_PCI_IO_START		0x01000000
#define VR41XX_PCI_IO_END		0x01ffffff

#define VR41XX_PCI_MEM_START		0x12000000
#define VR41XX_PCI_MEM_END		0x15ffffff

#define IO_PORT_BASE			KSEG1ADDR(VR41XX_PCI_IO_BASE)
#define IO_PORT_RESOURCE_START		0
#define IO_PORT_RESOURCE_END		VR41XX_PCI_IO_SIZE
#define IO_MEM1_RESOURCE_START		VR41XX_PCI_MEM1_BASE
#define IO_MEM1_RESOURCE_END		(VR41XX_PCI_MEM1_BASE + VR41XX_PCI_MEM1_SIZE)
#define IO_MEM2_RESOURCE_START		VR41XX_PCI_MEM2_BASE
#define IO_MEM2_RESOURCE_END		(VR41XX_PCI_MEM2_BASE + VR41XX_PCI_MEM2_SIZE)

/*
 * General-Purpose I/O Pin Number
 */
#define CMBVR41XX_INTA_PIN		3
#define CMBVR41XX_INTB_PIN		4
#define CMBVR41XX_INTC_PIN		5
#define CMBVR41XX_INTD_PIN		7
#define CMBVR41XX_INTE_PIN		8

/*
 * Interrupt Number
 */
#define CMBVR41XX_INTA_IRQ		GIU_IRQ(CMBVR41XX_INTA_PIN)
#define CMBVR41XX_INTB_IRQ		GIU_IRQ(CMBVR41XX_INTB_PIN)
#define CMBVR41XX_INTC_IRQ		GIU_IRQ(CMBVR41XX_INTC_PIN)
#define CMBVR41XX_INTD_IRQ		GIU_IRQ(CMBVR41XX_INTD_PIN)
#define CMBVR41XX_INTE_IRQ		GIU_IRQ(CMBVR41XX_INTE_PIN)

#define I8259_IRQ_BASE			72
#define I8259_IRQ(x)			(I8259_IRQ_BASE + (x))
#define TIMER_IRQ			I8259_IRQ(0)
#define KEYBOARD_IRQ			I8259_IRQ(1)
#define I8259_SLAVE_IRQ			I8259_IRQ(2)
#define UART3_IRQ			I8259_IRQ(3)
#define UART1_IRQ			I8259_IRQ(4)
#define UART2_IRQ			I8259_IRQ(5)
#define FDC_IRQ				I8259_IRQ(6)
#define PARPORT_IRQ			I8259_IRQ(7)
#define RTC_IRQ				I8259_IRQ(8)
#define USB_IRQ				I8259_IRQ(9)
#define I8259_INTA_IRQ			I8259_IRQ(10)
#define AUDIO_IRQ			I8259_IRQ(11)
#define AUX_IRQ				I8259_IRQ(12)
#define IDE_PRIMARY_IRQ			I8259_IRQ(14)
#define IDE_SECONDARY_IRQ		I8259_IRQ(15)
#define I8259_IRQ_LAST			IDE_SECONDARY_IRQ

extern void cmbvr41xx_irq_init(void);

#endif /* __NEC_CMBVR41XX_H */
