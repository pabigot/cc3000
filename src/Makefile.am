ACLOCAL_AMFLAGS = -I m4
AM_CPPFLAGS = -I${top_srcdir}/include

# For MSP430, optimize for size
@WITH_MSP430_TRUE@ AM_CFLAGS += -Os

# If GCC, support -gc-sections at link time
@WITH_GCC_TRUE@ AM_CFLAGS += -ffunction-sections -fdata-sections

cc3000dir = ${libdir}
cc3000_LTLIBRARIES = libcc3000.la
libcc3000_la_SOURCES = \
	cc3000_common.c	evnt_handler.c hci.c netapp.c nvmem.c os.c \
	socket.c wlan.c
