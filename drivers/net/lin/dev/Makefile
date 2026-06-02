# SPDX-License-Identifier: GPL-2.0
#
# Makefile for the LIN bus driver helpers (lin-dev).
#
# Holds skb-construction and owner-tagged loopback-synthesis code
# every host-side LIN driver consumes - mirrors the role of
# drivers/net/can/dev/can-dev.ko in the SocketCAN stack.

obj-$(CONFIG_LIN_DEV_HELPERS)	+= lin-dev.o

lin-dev-y			:= skb.o
