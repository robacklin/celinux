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
	make CROSS_COMPILE=/usr/bin/i686-linux-gnu- ARCH=i386 menuconfig
	#make CROSS_COMPILE=/usr/bin/i686-linux-gnu- ARCH=i386 defconfig
	make CROSS_COMPILE=/usr/bin/i686-linux-gnu- ARCH=i386 dep
	make CROSS_COMPILE=/usr/bin/i686-linux-gnu- ARCH=i386 bzImage
	include/linux/autoconf.h will exist from git repo but if you update by running menuconfig 
		this will update too

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

Note about choice of compiler:
	At the time that Linux 2.4.26 was released (2004-04-14), the latest
	released version of gcc was 3.3.3, with 3.4.0 just around the corner
	(2004-04-18.)  gcc 4.1.1 wouldn't even exist for another two years
	(2006-05-24) so it's not surprising that this combination isn't
	compatible because gcc tends to get stricter over time.  Those errors
	seem to refer to a gcc extension that was deprecated starting in 3.4 and
	apparently removed by the time of 4.1.

	You can most likely work around this without too much fuss, but IMO
	you'd be better off using versions that were historically coincident. 
	That way you get free validation that it's a stable combination, whereas
	with time-machine mix-and-match you have to determine that yourself. 

