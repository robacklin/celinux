celinux
=======

Consumer Electronics Linux (CELinux)


----------------------------------------------------------------
basic steps used to create
	wget http://tree.celinuxforum.org/source/linux-2.4.20.tar.bz2
	wget http://tree.celinuxforum.org/source/celinux-040503.patchset.tar.bz2
	tar xvjf celinux-040503.patchset.tar.bz2
	mv linux-2.4.20.tar.bz2 patches-040503.release/
	patches-040503.release/tpm -t linux-2.4.20.tar.bz2 -f patches-040503.release/patchlist -o celinux-040503

quick start to build (2012-11-24)
	make CROSS_COMPILE=/usr/bin/i686-linux-gnu- ARCH=i386 mrproper
	touch include/linux/autoconf.h
	make CROSS_COMPILE=/usr/bin/i686-linux-gnu- ARCH=i386 menuconfig
	#make CROSS_COMPILE=/usr/bin/i686-linux-gnu- ARCH=i386 defconfig
	make CROSS_COMPILE=/usr/bin/i686-linux-gnu- ARCH=i386 dep
	make CROSS_COMPILE=/usr/bin/i686-linux-gnu- ARCH=i386 bzImage

optional steps
	wget http://vanguard.ucmerced.edu/ccmips-20070927.tar.gz
----------------------------------------------------------------

blah blah blah
	#export ARCH=arm
	#export CROSS_COMPILE=/arm-2009q1/bin/arm-none-linux-gnueabi-
	make CROSS_COMPILE=/usr/bin/i686-linux-gnu- ARCH=i386 mrproper
	make CROSS_COMPILE=/usr/bin/i686-linux-gnu- ARCH=i386 include/linux/version.h
	make CROSS_COMPILE=/usr/bin/i686-linux-gnu- ARCH=i386 symlinks
	make CROSS_COMPILE=/usr/bin/i686-linux-gnu- ARCH=i386 defconfig
	make CROSS_COMPILE=/usr/bin/i686-linux-gnu- ARCH=i386 dep
	#make CROSS_COMPILE=/usr/bin/i686-linux-gnu- ARCH=i386 menuconfig
	make CROSS_COMPILE=/usr/bin/i686-linux-gnu- ARCH=i386 bzImage
	make CROSS_COMPILE=/usr/bin/i686-linux-gnu- ARCH=i386 modules

