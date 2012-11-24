#
# drivers/scsi/qla2xxx/Makefile
#
# Makefile for the Linux Qlogic QLA 2XXX v6 FC SCSI support
#

list-multi := qla2300_drv.o

obj-$(CONFIG_SCSI_QLOGIC_23XX) := qla2300.o

EXTRA_CFLAGS += -I$(TOPDIR)/drivers/scsi

qla2300_drv.o: qla2300.o
	$(LD) -r -o $@ $<

include $(TOPDIR)/Rules.make
