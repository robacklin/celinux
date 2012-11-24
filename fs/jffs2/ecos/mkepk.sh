#!/bin/sh
# $Id$

EPK_TMPDIR=/tmp/jffs2-epk-$$
MTDDIR=`dirname $0`/../../..

COREFILES="build.c compr_zlib.c LICENCE scan.c compr.c gc.c pushpull.h compr_rtime.c histo.h nodelist.c read.c write.c compr_rubin.c erase.c histo_mips.h nodelist.h readinode.c compr_rubin.h nodemgmt.c"
ECOSFILES="os-ecos.h file-ecos.c jffs2port.h dir-ecos.c  fs-ecos.c  malloc-ecos.c"
INCFILES="jffs2.h jffs2_fs_i.h jffs2_fs_sb.h"

mkdir $EPK_TMPDIR || exit 1

mkdir -p $EPK_TMPDIR/fs/jffs2/current/src
mkdir -p $EPK_TMPDIR/fs/jffs2/current/cdl
mkdir -p $EPK_TMPDIR/fs/jffs2/current/include/linux
mkdir -p $EPK_TMPDIR/fs/jffs2/current/tests

cat > $EPK_TMPDIR/pkgadd.db <<EOF
package CYGPKG_FS_JFFS2 {
        alias           { "JFFS2 Filesystem" jffs2 }
        directory       fs/jffs2
        script          jffs2.cdl
        description "
           This package contains the JFFS2 filesystem."
}
EOF
for a in $COREFILES; do
	cp -v $MTDDIR/fs/jffs2/$a $EPK_TMPDIR/fs/jffs2/current/src || exit 1
done
for a in $ECOSFILES; do
	cp -v $MTDDIR/fs/jffs2/ecos/src/$a $EPK_TMPDIR/fs/jffs2/current/src || exit 1
done
for a in $INCFILES; do
	cp -v $MTDDIR/include/linux/$a $EPK_TMPDIR/fs/jffs2/current/include/linux || exit 1
done
cp -v $MTDDIR/fs/jffs2/ecos/cdl/jffs2.cdl $EPK_TMPDIR/fs/jffs2/current/cdl
cp -v $MTDDIR/fs/jffs2/ecos/tests/*.c $EPK_TMPDIR/fs/jffs2/current/tests

tar cvfz jffs2-`date +"%Y%m%d%H%M"`.epk -C $EPK_TMPDIR pkgadd.db fs/jffs2

rm -rf $EPK_TMPDIR
