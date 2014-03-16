ACLOCAL_AMFLAGS = -I m4

# Public headers are put into a cc3000 subdirectory so they only
# conflict with other project namespaces if somebody includes distinct
# headers that use the same multiple-include symbol.
cc3000incdir = ${includedir}/cc3000
cc3000inc_HEADERS = \
	config.h \
	cc3000_common.h data_types.h evnt_handler.h hci.h host_driver_version.h \
	netapp.h nvmem.h socket.h wlan.h \
	spi.h security.h
