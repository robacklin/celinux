#
# Makefile for common code of Sony's PlayStation 2
#
# Note! Dependencies are done automagically by 'make dep', which also
# removes any old dependencies. DON'T put your own dependencies here
# unless it's something special (ie not a .c file).
#

.S.s:
	$(CPP) $(CFLAGS) $< -o $*.s
.S.o:
	$(CC) $(CFLAGS) -c $< -o $*.o

O_TARGET := ps2.o

obj-y    += setup.o reset.o prom.o time.o

include $(TOPDIR)/Rules.make

