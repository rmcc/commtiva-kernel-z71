#
# Makefile for miscellaneous I2C chip drivers.
#
# Do not add new drivers to this directory! It is DEPRECATED.
#
# Device drivers are better grouped according to the functionality they
# implement rather than to the bus they are connected to. In particular:
# * Hardware monitoring chip drivers go to drivers/hwmon
# * RTC chip drivers go to drivers/rtc
# * I/O expander drivers go to drivers/gpio
#

obj-$(CONFIG_BACKLIGHT_LED_MAX8831) += max8831.o
obj-$(CONFIG_BACKLIGHT_LED_TCA6507) += tca6507.o
obj-$(CONFIG_FLASH_DRIVER_IC_AAT1272) += aat1272.o


ifeq ($(CONFIG_I2C_DEBUG_CHIP),y)
EXTRA_CFLAGS += -DDEBUG
endif

