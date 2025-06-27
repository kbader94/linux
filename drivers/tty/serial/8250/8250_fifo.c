#include <linux/module.h>
#include <linux/io.h>
#include <linux/serial_reg.h>
#include "8250.h"
#include "8250_fifo.h"

/*
 * Here we define the default fifo config used for each type of UART.
 * NOTE:
 * 
 *  - rxtrig_bytes[] is order sensitive!
 * 	  Ensure the order matches the hardware encoding of UART_FCR_R_TRIG_BITS().
 * 
 *  - fifo_control is intended to become the canonical representation of port 
 *    FIFO status. FCR is intended to become deprecated,
 *	  BUT, for the time being, we must ensure the values set in fifo_control
 * 	  align with the existing values in FCR. For example, in the 16550a the 
 *    initial FCR value sets UART_FCR_R_TRIG_10, which is associated with 
 *    an 8 byte rx trigger level, so fifo_control.rx_trigger_bytes MUST be 8. 
 * 
 *  - UART_CAP_XFIFO enables txtrig_bytes and additional rxtrig_bytes (>4)
 *    See 16550A & 16650V2 for examples
 */

static int rx_trig_to_fcr(struct uart_8250_port *up, u32 level)
{
	const struct serial8250_config *conf;
	 
	conf = &serial8250_uart_config[up->port.type];

	for (int i = 0; i < UART_FCR_R_TRIG_MAX_STATE; i++) {
		if (conf->rxtrig_bytes[i] && conf->rxtrig_bytes[i] == level)
			return UART_FCR_R_FROM_TRIG_I(i);
	}

	return -EINVAL;
}

static __maybe_unused int fcr_to_rx_trig(struct uart_8250_port *up, u8 fcr)
{
	const struct serial8250_config *conf;
	int index;
	 
	conf = &serial8250_uart_config[up->port.type];
	index = UART_FCR_R_TRIG_BITS(fcr);

	if (index >= UART_FCR_R_TRIG_MAX_STATE)
		return -EINVAL;

	return conf->rxtrig_bytes[index] ? conf->rxtrig_bytes[index]
		: -EOPNOTSUPP;
}

/* The 16550A's FIFO is controlled by the FCR register. 
 * Specifically, FCR[6:7] are used to control the rx trigger level.
 * The 16550A was the first widely-used UART to have a configurable
 * FIFO. As such, many other UARTs with configurable FIFOs are compatible.
 * Note: the existing rxtrig_bytes sysfs interface exclusively uses this method.
 */
int serial16550a_set_fifo_control(struct uart_port *port, 
                                        const struct uart_fifo_control *ctl)
{
	struct uart_8250_port *up = up_to_u8250p(port);
	const struct serial8250_config *port_config;
	unsigned long sl_flags;
	u8 fcr = 0;
	int fcr_rx_trig;

	/* validate port & fifo control */
	if (port->type != PORT_16550A)
		return -EINVAL;

	if (!(up->capabilities & UART_CAP_FIFO) || port->fifosize <= 1)
		return -EOPNOTSUPP;

	port_config = &serial8250_uart_config[port->type];

	if (ctl->tx_trigger_bytes)
		return -EOPNOTSUPP; /* Tx unsupported on 16550 */

	if (ctl->flags & UART_FIFO_CTRL_FLAG_DMA_MODE)
		return -EOPNOTSUPP; /* DMA unavailable on default 16550-type UARTs */

	/* Set rx trigger level */
	fcr_rx_trig = rx_trig_to_fcr(up, ctl->rx_trigger_bytes);
	if (fcr_rx_trig < 0)
		return fcr_rx_trig;

	fcr |= fcr_rx_trig;

	/* Enable FIFO if requested */
	if (ctl->flags & (UART_FIFO_CTRL_FLAG_ENABLE_RX | UART_FIFO_CTRL_FLAG_ENABLE_TX))
		fcr |= UART_FCR_ENABLE_FIFO;

	/* Flush FIFOs if requested */
	if (ctl->flags & UART_FIFO_CTRL_FLAG_FLUSH_RX)
		fcr |= UART_FCR_CLEAR_RCVR;

	if (ctl->flags & UART_FIFO_CTRL_FLAG_FLUSH_TX)
		fcr |= UART_FCR_CLEAR_XMIT;

	/* Write to UART FCR register */
	spin_lock_irqsave(&port->lock, sl_flags);
	serial_out(up, UART_FCR, fcr);
	
	/* Clear flush flags so they don't persist */
	fcr &= ~(UART_FCR_CLEAR_RCVR | UART_FCR_CLEAR_XMIT); 
	up->fcr = fcr;
	spin_unlock_irqrestore(&port->lock, sl_flags);

	return 0;
}

//TODO: refactor and rename dispatch_set_fifo
/* Use set fifo callback stored in uart_config[] */
int serial8250_default_set_fifo_control(struct uart_port *port,
                                const struct uart_fifo_control *ctl)
{
	const struct serial8250_config *conf;

	if (!port || !ctl)
		return -EINVAL;

	conf = &serial8250_uart_config[port->type];
	if (conf->set_fifo_control)
		return conf->set_fifo_control(port, ctl);

	return -EOPNOTSUPP;
}



