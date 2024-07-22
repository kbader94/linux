# SPDX-License-Identifier: GPL-2.0
#
#  Makefile for the Linux Local Interconnect Network drivers.
#

obj-$(CONFIG_LIN_SLLIN)		+= slcan/
#TODO: vlin.o

obj-y				+= dev/
obj-y				+= esd/
obj-y				+= rcar/
obj-y				+= spi/
obj-y				+= usb/
obj-y				+= softing/

subdir-ccflags-$(CONFIG_LIN_DEBUG_DEVICES) += -DDEBUG