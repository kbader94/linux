/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
#ifndef _UAPI_LINUX_SERIAL_FIFO_H
#define _UAPI_LINUX_SERIAL_FIFO_H

#include <linux/types.h>

/* Flag Masks */
#define UART_FIFO_CTRL_FLAG_ENABLE_FIFO     (1 << 0)
struct uart_fifo_control {
	__u64 flags;               /* Bitmask for enable/flush/options */
	__u16 rx_trigger_bytes;    /* RX FIFO level */
	__u16 tx_trigger_bytes;    /* TX FIFO level */
};

#endif /* _UAPI_LINUX_SERIAL_FIFO_H */
