#!/bin/sh

make CROSS_COMPILE=/opt/toolchain/mipsel-linux/bin/mipsel-linux- ARCH=mips mrproper
make CROSS_COMPILE=/opt/toolchain/mipsel-linux/bin/mipsel-linux- ARCH=mips symlinks
#make CROSS_COMPILE=/opt/toolchain/mipsel-linux/bin/mipsel-linux- ARCH=mips dep
make CROSS_COMPILE=/opt/toolchain/mipsel-linux/bin/mipsel-linux- ARCH=mips defconfig
#make CROSS_COMPILE=/opt/toolchain/mipsel-linux/bin/mipsel-linux- ARCH=mips menuconfig
make CROSS_COMPILE=/opt/toolchain/mipsel-linux/bin/mipsel-linux- ARCH=mips
#make CROSS_COMPILE=/opt/toolchain/mipsel-linux/bin/mipsel-linux- ARCH=mips bzImage
#make CROSS_COMPILE=/opt/toolchain/mipsel-linux/bin/mipsel-linux- ARCH=mips modules

exit 0
#export ARCH=arm
#export CROSS_COMPILE=/home/racklin/Programs/arm-2009q1/bin/arm-none-linux-gnueabi-
make CROSS_COMPILE=/usr/bin/i686-linux-gnu- ARCH=i386 mrproper
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

