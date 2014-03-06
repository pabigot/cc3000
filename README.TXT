Current CC3000 Service Pack: 1.12

This directory contains the master branch of
https://github.com/pabigot/cc3000, which holds the CC3000 Host Driver
sources from Texas Instruments packaged into a git repository, rearranged to
avoid conflicts between CC3000 and application headers, and augmented with
an autoconf-based build infrastructure.  It specifically supports building
the library using mspgcc (http://sourceforge.net/projects/mspgcc/).  It also
contains patches to some problems discovered when using the driver.

Source code is copyright Texas Instruments, released under BSD-3-Clause, and
available from the [download page of the CC3000
Wiki](http://processors.wiki.ti.com/index.php/CC3000_Wi-Fi_Downloads).
Modifications to the Texas Instruments code in this repository are intended
to be public domain.

This material is provided as a courtesy to interested developers, and it is
not supported.  The discussion forum for the CC3000 SimpleLink Wi-Fi
capability is at: http://e2e.ti.com/support/low_power_rf/f/851.aspx

Please note: I (pabigot) do not use any TI SPI drivers, and can't provide
support for using it independently or with TI example programs.  For basic
evaluation of the CC3000 I used the CC3000EM with a [driver for
BSP430](http://pabigot.github.com/bsp430/ex_rfem_cc3000.html).

Example of configuration and installation assuming mspgcc is installed:

  ./configure --prefix=/usr/local/msp430-cc3000 --host=msp430
  make
  make install

To configure for TivaWare for C Series using GCC:

  ./configure --prefix=/prj/arm/tm4c \
     --host=arm-none-eabi \
     TARGET_CFLAGS='-mthumb -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=softfp'

Versioning Information
----------------------

The version numbering policy for the CC3000 host driver is complex.  Some
information about it is at
http://processors.wiki.ti.com/index.php/CC3000_Release_Notes. Be aware that
service pack versioning is unrelated to application package or host driver
versioning, and cannot be identified at compile-time: it may only be
obtained from a powered module by using the nvmem_read_sp_version() API.

At this time the standard version identifier appears to be the Service Pack
version, which is currently 1.12.  For each Service Pack there is a
recommended version of the host driver application.  In this repository are
tags recording the exact contents of these TI source releases.

swrc282 : TI CC3000 Host Driver extracted from CC3000 1.12 SDK on Windows

swrc263c : TI CC3000 Host Driver release recommended for SP 1.11.1.

swrc263b : TI CC3000 Host Driver release recommended for SP 1.11.

swrc263a : TI CC3000 Host Driver release recommended for SP 1.10.  This
version was never directly integrated into the master branch.

Information on earlier releases present in the git repository can be found
in the corresponding README.TXT at each revision on the master branch.
