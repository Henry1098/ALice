#!/bin/bash

root_dir=`pwd`
cd ${root_dir} && cd ..
kernel_dir=`pwd` && cd ${root_dir}
echo "kernel_dir=$kernel_dir"

mkdir out
export TARGET_ARCH_VARIANT=armv7-a-neon
export CROSS_COMPILE=${kernel_dir}/prebuilts/gcc/linux-x86/arm/arm-linux-androideabi-4.9/bin/arm-linux-androideabi-
make ARCH=arm O=out P98995AA1_defconfig
status=${PIPESTATUS[0]}
if [ "$status" != "0" ]; then 
     echo "ERROR:  make defconfig  error:"
     exit
else
     echo "#### make defconfig successfully ####"
fi
make ARCH=arm -j8 O=out
re=${PIPESTATUS[0]}
if [ "$re" != "0" ];then 
     echo "ERROR:  build kernel error:"
     exit
else
    echo "#### build kernel completed successfully ####"
fi