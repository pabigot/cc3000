This directory contains the spi branch of https://github.com/pabigot/cc3000,
which holds the CC3000 SPI HAL sources from Texas Instruments, packaged into
a git repository.

The original source code can be found at
http://www.ti.com/tool/cc3000-fram-sw.  This version is based on the
contents of the CC3000Spi subdirectory of swrc246a, downloadable from
http://www.ti.com/lit/zip/swrc246.  Source code is copyright Texas
Instruments and released under BSD-3-Clause.  Modifications to the Texas
Instruments code in this directory are intended to be public domain.

The spi.c file should be compiled for your target platform and linked with
the libcc3000 produced from the master branch of this repository along with
your application code.

This version has configuration for the following MCUs:

* MSP430FR5739
* MSP430F5438A
* MSP430F5529
