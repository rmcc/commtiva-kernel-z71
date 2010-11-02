#
# Makefile for the input core drivers.
#

# Each configuration option enables a list of files.

#WillChen 2009.05.14 FIH_Fxx
obj-$(CONFIG_INPUT_LPSENSOR)	+= LPSCM3602.o


#WillChen 2009.05.14 FIH_Fxx
ifeq ($(CONFIG_LPSENSORS_DEBUG),y)
EXTRA_CFLAGS += -DDEBUG
endif
