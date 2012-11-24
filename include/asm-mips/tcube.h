#ifndef __TCUBE_H
#define __TCUBE_H

#include <asm/vr5701.h>

#define TCUBE_SDRAM_SIZE 0x04000000

#ifndef __ASSEMBLY__
#include <asm/delay.h>

/*
 *  PCI Master Registers
 */

#define DDB_PCICMD_IACK		0	/* PCI Interrupt Acknowledge */
#define DDB_PCICMD_IO		1	/* PCI I/O Space */
#define DDB_PCICMD_MEM		3	/* PCI Memory Space */
#define DDB_PCICMD_CFG		5	/* PCI Configuration Space */

/*
 * additional options for pci init reg (no shifting needed)
 */
#define DDB_PCI_CFGTYPE1     0x200   /* for pci init0/1 regs */
#define DDB_PCI_ACCESS_32    0x10    /* for pci init0/1 regs */

#define ddb_sync       io_sync
#define ddb_out32(x,y) io_out32(x,y)
#define ddb_out16(x,y) io_out16(x,y)
#define ddb_out8(x,y)  io_out8(x,y)
#define ddb_in32(x)    io_in32(x)
#define ddb_in16(x)    io_in16(x)
#define ddb_in8(x)     io_in8(x)

static inline void io_sync(void){    asm("sync");}

static inline void io_out32(u32 offset, u32 val)
{
    *(volatile u32 *)(VR5701_IO_BASE+offset) = val;
    io_sync();
}

static inline u32 io_in32(u32 offset)
{
    u32 val = *(volatile u32 *)(VR5701_IO_BASE+offset);
    io_sync();
    return val;
}

static inline void io_out16(u32 offset, u16 val)
{
    *(volatile u16 *)(VR5701_IO_BASE+offset) = val;
    io_sync();
}

static inline u16 io_in16(u32 offset)
{
    u16 val = *(volatile u16 *)(VR5701_IO_BASE+offset);
    io_sync();
    return val;
}

static inline void io_reset16(unsigned long adr,
			      unsigned short val1,
			      unsigned delay,
			      unsigned short val2)
{
  io_out16(adr, val1);
  __delay(delay);
  io_out16(adr, val2);
}

static inline void io_out8(u32 offset, u8 val)
{
    *(volatile u8 *)(VR5701_IO_BASE+offset) = val;
    io_sync();
}

static inline u8 io_in8(u32 offset)
{
    u8 val = *(volatile u8 *)(VR5701_IO_BASE+offset);
    io_sync();
    return val;
}

static inline void io_set16(u32 offset, u16 mask, u16 val)
{
  u16 val0 = io_in16(offset);
  io_out16(offset, (val&mask)|(val0&~mask));
}

// irq_vr5701.c
extern void ll_vr5701_irq_route(int vr5701_irq, int ip);
extern void ll_vr5701_irq_enable(int vr5701_irq);

// nile4.c
extern void ddb_set_pdar(u32, u32, u32, int, int, int);
extern void ddb_set_pmr(u32 pmr, u32 type, u32 addr, u32 options);
extern void ddb_set_bar(u32 bar, u32 phys, int prefetchable);

// pci.c
extern void ddb_pci_reset_bus(void);

// setup.c
extern void shima_tcube_setup(void);

#endif
#endif
