#!/bin/bash
cd libopencm3
make lib

cd ../Bootloader
make
make -f Makefile.f4 TARGET=fmu flash-bootloader

cd ..
