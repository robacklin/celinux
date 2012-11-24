#
# Makefile for SHIMAFUJI T-Cube machine.
#
# Note! Dependencies are done automagically by 'make dep', which also
# removes any old dependencies. DON'T put your own dependencies here
# unless it's something special (ie not a .c file).
#

.S.s:
	$(CPP) $(CFLAGS) $< -o $*.s
.S.o:
	$(CC) $(CFLAGS) -c $< -o $*.o

O_TARGET:= tcube.o

obj-y	 += int-handler.o irq.o irq_vr5701.o setup.o irq0.o prom.o 
obj-y    += pci.o pci_ops.o nile4.o
###obj-y    += pci-dma.o # KOSE.ADD
###obj-y    += uart.o led.o rtc_ricoh.o

obj-$(CONFIG_REMOTE_DEBUG)	+= kgdb_io.o
obj-$(CONFIG_TCUBE_RTC)		+= rtc_rx5c348.o

include $(TOPDIR)/Rules.make
