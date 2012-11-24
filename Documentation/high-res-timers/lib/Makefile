SOURCES = syscall_timer.c 
OBJECTS = $(SOURCES:.c=.o)
SYS_INCLUDE := ../../../include/

SYSASM := -I$(SYS_INCLUDE) -I../usr_incl/

IS_I386 := $(shell ls -dl $(SYS_INCLUDE)asm | grep i386 |sed -e "s/^.*> //")

CPPFLAGS = -D_POSIX_TIMERS=1 -D_POSIX_C_SOURCE=199309L -D_XOPEN_SOURCE $(SYSASM)
CFLAGS = -g -Wall
ifdef SHARED
CFLAGS += -shared -fpic -fPIC -DSHARED
LIBPOSIXTIME = libposixtime.so
else
LIBPOSIXTIME = libposixtime.a
endif

all: $(LIBPOSIXTIME)($(OBJECTS))

clean:
	rm -f *.o *.a *.so *~ core .depend t--* tmp

.depend: $(SOURCES) test_itimerspec
	$(CC) $(CPPFLAGS)  -M $(SOURCES) | \
		sed -e '/:/s|^[^ :]*|$(LIBPOSIXTIME)(&)|' > .depend
	chmod +x test_itimerspec
	./test_itimerspec $(CC) $(CPPFLAGS) &>/dev/null
	make REMOTE_KERNEL=$(REMOTE_KERNEL)
	make SHARED=$(IS_I386) REMOTE_KERNEL=$(REMOTE_KERNEL)
# The above make insures that "all:" is done with the newly created .depend

include .depend
