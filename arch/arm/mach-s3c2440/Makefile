
# Makefile for the linux kernel.
#
# Note! Dependencies are done automagically by 'make dep', which also
# removes any old dependencies. DON'T put your own dependencies here
# unless it's something special (ie not a .c file).

USE_STANDARD_AS_RULE := true

O_TARGET		:= s3c2440.o

obj-y :=
obj-m :=
obj-n :=
obj-  :=

export-objs := smdk2440.o irq.o generic.o cpu.o s3c2440-ldm.o dma.o

# Common support (must be linked before board specific support)
obj-y += generic.o irq.o cpu.o dma.o

# Specific board support
obj-$(CONFIG_S3C2440_SMDK2440) += smdk2440.o s3c2440-pm.o s3c2440-sleep.o 
obj-$(CONFIG_S3C2440_SMDK2440) += s3c2440-ck.o s3c2440-idle.o 

# LEDs support
leds-y := leds.o
leds-$(CONFIG_S3C2440_SMDK2440) += leds-smdk2440.o
obj-$(CONFIG_LEDS) += $(leds-y)

# Dynamic Power Management 
obj-$(CONFIG_DPM) += s3c2440-dpm.o
obj-y += s3c2440-ldm.o

include $(TOPDIR)/Rules.make

