# SPDX-License-Identifier: GPL-2.0
#
# Makefile for the LIN bus driver helpers (lin-dev).
#
# Holds protocol/state-machine code shared across every LIN driver that
# drives the bus from the host: skb construction, owner-tagged loopback
# synthesis, schedule engine, response cache, and (when
# CONFIG_LIN_DEV_HELPERS_UART is selected) the UART byte-protocol layer
# used by sllin and sdlin. Modeled on drivers/net/can/dev/.

obj-$(CONFIG_LIN_DEV_HELPERS)		+= lin-dev.o

lin-dev-y				:= skb.o sched.o resp.o
lin-dev-$(CONFIG_LIN_DEV_HELPERS_UART)	+= uart.o kthread.o
