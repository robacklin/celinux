
#ifndef __SH_KERNEL_PCI_H__
#define __SH_KERNEL_PCI_H__

extern unsigned long isa_io_base;
extern unsigned long isa_mem_base;
extern unsigned long pci_dram_offset;

extern unsigned int  *pci_config_address;
extern unsigned char *pci_config_data;
/*
void fix_intr(struct device_node *node, struct pci_dev *dev);
*/

#define IOBASE_BRIDGE_NUMBER	0
#define IOBASE_MEMORY		1
#define	IOBASE_IO		2

#endif /* __SH_KERNEL_PCI_H__ */
