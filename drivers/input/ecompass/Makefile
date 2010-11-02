#
# Makefile for the input core drivers.
#

# Each configuration option enables a list of files.

obj-$(CONFIG_FIH_ECOMPASS_SENSOR)	+= fihsensor.o


ifeq ($(CONFIG_FIH_ECOMPASS_SENSOR_DEBUG),y)
EXTRA_CFLAGS += -DDEBUG
endif
