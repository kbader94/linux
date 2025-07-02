#ifndef _8250_FIFO_H
#define _8250_FIFO_H

#include <linux/serial_core.h>
#include <linux/serial_fifo.h>

int serial8250_dispatch_set_fifo_control(struct uart_port *up,
                                      const struct uart_fifo_control *ctl);
                     
#endif /* _8250_FIFO_H */
