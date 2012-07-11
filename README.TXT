This directory contains the master branch of
https://github.com/pabigot/cc3000, which holds the CC3000 Host Driver
sources from Texas Instruments, packaged into a git repository and augmented
with an autoconf-based build infrastructure.  It specifically supports
building the library using mspgcc (http://sourceforge.net/projects/mspgcc/).

The original source code can be found at
http://www.ti.com/tool/cc3000-fram-sw.  This version is based on the
contents of the CC3000HostDriver subdirectory of swrc246a, downloadable from
http://www.ti.com/lit/zip/swrc246.  Source code is copyright Texas
Instruments and released under BSD-3-Clause.  Modifications to the Texas
Instruments code in this directory are intended to be public domain.

Note that the SPI driver is not integrated into the library, as it contains
platform-specific configuration information.  The original driver from
swrc246's CC3000Spi subdirectory, targeting the MSP430FR5739 chip, is
available in the spi branch.  It has been updated to support additional
MSP430 chips.

Example of configuration and installation assuming mspgcc is installed:

  ./configure --prefix=/usr/local/msp430-cc3000 --host=msp430
  make
  make install
