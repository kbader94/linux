/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
#ifndef _UAPI_LINUX_SERIAL_FIFO_H
#define _UAPI_LINUX_SERIAL_FIFO_H

#include <linux/types.h>

/* Flag Masks */
#define UART_FIFO_CTRL_FLAG_ENABLE_RX     (1 << 0)
#define UART_FIFO_CTRL_FLAG_ENABLE_TX     (1 << 1)
#define UART_FIFO_CTRL_FLAG_FLUSH_RX      (1 << 2)
#define UART_FIFO_CTRL_FLAG_FLUSH_TX      (1 << 3)
#define UART_FIFO_CTRL_FLAG_DMA_MODE      (1 << 4)

struct uart_fifo_control {
	__u64 flags;               /* Bitmask for enable/flush/options */
	__u32 rx_trigger_bytes;    /* RX FIFO level */
	__u32 tx_trigger_bytes;    /* Reserved */
	__u32 rx_timeout_ms;       /* Reserved */
	__u32 reserved;            /* Reserved for future extensions */
};

#endif /* _UAPI_LINUX_SERIAL_FIFO_H */
