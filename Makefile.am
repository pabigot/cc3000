ACLOCAL_AMFLAGS = -I m4

# Public headers are put into a cc3000 subdirectory so they only
# conflict with other project namespaces if somebody includes distinct
# headers that use the same multiple-include symbol.
cc3000incdir = ${includedir}/cc3000
cc3000inc_HEADERS = \
	cc3000_common.h evnt_handler.h hci.h host_driver_version.h \
	netapp.h nvmem.h os.h socket.h wlan.h \
	spi.h

cc3000dir = ${libdir}
cc3000_LTLIBRARIES = libcc3000.la
libcc3000_la_SOURCES = \
	cc3000_common.c	evnt_handler.c hci.c netapp.c nvmem.c os.c \
	socket.c wlan.c
