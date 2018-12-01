#!/bin/bash

if [ ! -e ./build/advanced_1 ]; then
	./build.sh
fi
qemu-arm -L /usr/arm-linux-gnueabihf build/advanced_1
