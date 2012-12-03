#!/bin/sh

#git clone git://git.kernel.org/pub/scm/.../linux-2.6
#git clone git://git.kernel.org/pub/scm/linux/kernel/git/torvalds/linux.git 
#git pull

cd /home/racklin/Projects/celinux
if [ ! -e "linux-2.4.20.tar.bz2" ] ; then
	wget http://tree.celinuxforum.org/source/linux-2.4.20.tar.bz2
	#wget http://tree.celinuxforum.org/source/celinux-040503.patchset.tar.bz2
fi

if [ ! -e "patches-040503.release/linux-2.4.20.tar.bz2" ] ; then
	cp linux-2.4.20.tar.bz2 patches-040503.release/
fi

if [ ! -d "celinux-040503" ] ; then
	patches-040503.release/tpm -t linux-2.4.20.tar.bz2 -f patches-040503.release/patchlist -o celinux-040503
fi

if [ -d "celinux-040503" ] ; then
	if [ ! -d "linux-2.4.20" ] ; then
		ln -s celinux-040503 linux-2.4.20
	fi
fi

#wget http://vanguard.ucmerced.edu/ccmips-20070927.tar.gz

cd linux-2.4.20
#export ARCH=arm
#export CROSS_COMPILE=/home/racklin/Programs/arm-2009q1/bin/arm-none-linux-gnueabi-
mkdir -p /home/racklin/Projects/builds/celinux/linux
make CROSS_COMPILE=/usr/bin/i686-linux-gnu- ARCH=i386 mrproper
touch include/linux/autoconf.h
#make CROSS_COMPILE=/usr/bin/i686-linux-gnu- ARCH=i386 menuconfig
make CROSS_COMPILE=/usr/bin/i686-linux-gnu- ARCH=i386 oldconfig
#make CROSS_COMPILE=/usr/bin/i686-linux-gnu- ARCH=i386 defconfig
make CROSS_COMPILE=/usr/bin/i686-linux-gnu- ARCH=i386 dep
make CROSS_COMPILE=/usr/bin/i686-linux-gnu- ARCH=i386 bzImage

exit 0
cd linux-2.4.20
export ARCH=arm
export CROSS_COMPILE=/home/racklin/Programs/arm-2009q1/bin/arm-none-linux-gnueabi-
mkdir -p /home/racklin/Projects/builds/celinux/linux
make O=/home/racklin/Projects/builds/celinux/linux mrproper
make O=/home/racklin/Projects/builds/celinux/linux menuconfig
#cp arch/arm/def-configs/frodo .config
make O=/home/racklin/Projects/builds/celinux/linux oldconfig
make O=/home/racklin/Projects/builds/celinux/linux dep
make O=/home/racklin/Projects/builds/celinux/linux 

