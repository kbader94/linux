#ifndef _8250_FIFO_H
#define _8250_FIFO_H

#include <linux/serial_core.h>
#include <linux/serial_fifo.h>

int serial8250_default_set_fifo_control(struct uart_port *up,
                                      const struct uart_fifo_control *ctl);
int serial8250_default_get_fifo_control(struct uart_port *up,
                                      struct uart_fifo_control *ctl);
int serial16550a_set_fifo_control(struct uart_port *port, 
                                        const struct uart_fifo_control *ctl);

#endif /* _8250_FIFO_H */
