This directory contains the master branch of
http://www.ti.com/lit/zip/swrc246 https://github.com/pabigot/cc3000, which
holds the CC3000 Host Driver sources from Texas Instruments packaged into a
git repository, rearranged to avoid conflicts between CC3000 and application
headers, and augmented with an autoconf-based build infrastructure.  It
specifically supports building the library using mspgcc
(http://sourceforge.net/projects/mspgcc/).

The original source code can be found at
http://www.ti.com/tool/cc3000-fram-sw.

The discussion forum for the CC3000 SimpleLink Wi-Fi capability is at:
http://e2e.ti.com/support/low_power_rf/f/851.aspx

Source code is copyright Texas Instruments and released under BSD-3-Clause.

Modifications to the Texas Instruments code in this directory are intended
to be public domain.

The version numbering policy for the CC3000 host driver is "unique", and the
interested reader may enjoy the information presented at
http://processors.wiki.ti.com/index.php/CC3000_Release_Notes.  In this
repository are tags for these releases:

version_2_2 : The original release, based on the contents of the
CC3000HostDriver subdirectory of swrc246a, a simple webserver application,
originally downloaded from http://www.ti.com/lit/zip/swrc246.  This version
is apparently no longer available, having been replaced by swrc246b.  The
last tested version is tagged as version_2_2.

version_6_10 : The release for service pack 1.7.  It was found that the
generic host driver is available from http://www.ti.com/lit/zip/swrc233.
The second version of the driver is based on the contents of swrc233a.
swrc233a which adds several improvements and corrections and in most cases
require the WiFi module firmware to be updated to Service Pack 1.7.
Information about this service pack may be found at:
http://processors.wiki.ti.com/index.php/CC3000_Patch_Programmer

Note that the SPI driver is not integrated into the driver branches of this
repository, as it contains platform-specific configuration information.  The
original driver from swrc246's CC3000Spi subdirectory, targeting the
MSP430FR5739 chip, is available in the spi branch.  It has been updated to
support additional MSP430 chips.  The SPI driver is not part of swrc236a.
The version used in swrc233b has been examined and found to add MCU-specific
and application-specific material to the spi.h header, while not changing
the core board support interface.  Consequently these changes have not been
integrated into the repository.

Please note: I (pabigot) do not use this driver, and can't provide support
for using it independently or with TI example programs.  I use the CC3000
with a driver under BSP430: https://github.com/pabigot/bsp430

Example of configuration and installation assuming mspgcc is installed:

  ./configure --prefix=/usr/local/msp430-cc3000 --host=msp430
  make
  make install
