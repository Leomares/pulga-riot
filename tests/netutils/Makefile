include ../Makefile.tests_common

USEMODULE += netutils
USEMODULE += netif
USEMODULE += embunit
USEMODULE += sock_util

# make sure we have an implementation of sock_types.h
USEMODULE += gnrc_sock_udp
USEMODULE += gnrc_ipv6

USEMODULE += ipv4_addr
USEMODULE += ipv6_addr

# pretend to include sock_dns
CFLAGS += -DMODULE_SOCK_DNS=1

include $(RIOTBASE)/Makefile.include
