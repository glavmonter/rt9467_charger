#!/bin/sh

ARCH=arm64 CROSS_COMPILE=aarch64-none-linux-gnu- make KDIR=../orangepi-build/kernel/orange-pi-5.10-rk35xx/ all

scp -r *.ko orangepi@orangepi:/home/orangepi/modules/

