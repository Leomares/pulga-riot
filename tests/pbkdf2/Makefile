include ../Makefile.tests_common

# This application uses getchar and thus expects input from stdio
USEMODULE += stdin
USEMODULE += hashes
USEMODULE += base64

# Use a terminal that does not introduce extra characters into the stream.
RIOT_TERMINAL ?= socat

include $(RIOTBASE)/Makefile.include

# Increase Stack size for AVR
ifneq (,$(filter avr8_common,$(USEMODULE)))
  CFLAGS += -DTHREAD_STACKSIZE_MAIN=THREAD_STACKSIZE_LARGE
endif
