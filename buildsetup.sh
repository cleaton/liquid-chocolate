#!/bin/bash
export KERNEL_DIR=$PWD
export ARCH=arm
export CROSS_COMPILE=arm-eabi-
#echo -n "Enter full path to arm-eabi- (ex, ~/<Android source dir>/prebuilt/linux-x86/toolchain/arm-eabi-4.4.0/bin ) [ENTER]: "
#read fullpath
export PATH=$PATH:~/Android/Cyanogenmod/prebuilt/linux-x86/toolchain/arm-eabi-4.4.3/bin/
echo KERNEL_DIR=$KERNEL_DIR
echo ARCH=$ARCH
echo CROSS_COMPILE=$CROSS_COMPILE
echo PATH=$PATH
make -j4
