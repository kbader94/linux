#include <linux/module.h>
#include <linux/io.h>
#include <linux/serial_reg.h>
#include "8250.h"
#include "8250_fifo.h"

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

static int tx_trig_to_fcr(struct uart_8250_port *up, u32 level)
{
	const struct serial8250_config *conf = 
				 &serial8250_uart_config[up->port.type];

	for (int i = 0; i < UART_FCR_T_TRIG_MAX_STATE; i++) {
		if (conf->txtrig_bytes[i] && conf->txtrig_bytes[i] == level)
			return UART_FCR_T_FROM_TRIG_I(i);
	}

	return -EINVAL;
}

static int __maybe_unused fcr_to_tx_trig(struct uart_8250_port *up, u8 fcr)
{
	const struct serial8250_config *conf = 
				 &serial8250_uart_config[up->port.type];
	int index = UART_FCR_T_TRIG_BITS(fcr);

	if (index >= UART_FCR_T_TRIG_MAX_STATE)
		return -EINVAL;

	return conf->txtrig_bytes[index] ? conf->txtrig_bytes[index]
		: -EOPNOTSUPP;
}

static int write_fcr_common(struct uart_8250_port *up, const struct uart_fifo_control *ctl, u8 fcr)
{
	unsigned long sl_flags;

	uart_port_lock_irqsave(&up->port, &sl_flags);

	if (ctl->flags & (UART_FIFO_CTRL_FLAG_ENABLE_RX | UART_FIFO_CTRL_FLAG_ENABLE_TX))
		fcr |= UART_FCR_ENABLE_FIFO;

	if (ctl->flags & UART_FIFO_CTRL_FLAG_FLUSH_RX)
		fcr |= UART_FCR_CLEAR_RCVR;

	if (ctl->flags & UART_FIFO_CTRL_FLAG_FLUSH_TX)
		fcr |= UART_FCR_CLEAR_XMIT;

	serial_out(up, UART_FCR, fcr);
	fcr &= ~(UART_FCR_CLEAR_RCVR | UART_FCR_CLEAR_XMIT);
	up->fcr = fcr;
	uart_port_unlock_irqrestore(&up->port, sl_flags);
	return 0;
}

static int port_16550A_set_fifo_control(struct uart_8250_port *up, 
                                  const struct uart_fifo_control *ctl)
{
	int fcr_rx_trig = 0;

	/* Validate RX trigger level */
	fcr_rx_trig = rx_trig_to_fcr(up, ctl->rx_trigger_bytes);
	if (fcr_rx_trig < 0)
		return fcr_rx_trig;

	/* Validate TX trigger if supported */
	if (ctl->tx_trigger_bytes) 
		return -EOPNOTSUPP; /* TX fifo levels unavail on 16550A */

	return write_fcr_common(up, ctl, fcr_rx_trig);
}

static int port_16650V2_set_fifo_control(struct uart_8250_port *up, 
                                  const struct uart_fifo_control *ctl)
{
	int fcr_rx_trig, fcr_tx_trig = 0;

	/* Validate RX trigger level */
	fcr_rx_trig = rx_trig_to_fcr(up, ctl->rx_trigger_bytes);
	if (fcr_rx_trig < 0)
		return fcr_rx_trig;

	/* Validate TX trigger level */
	fcr_tx_trig = tx_trig_to_fcr(up, ctl->tx_trigger_bytes);
	if (fcr_tx_trig < 0)
		return fcr_tx_trig;

	return write_fcr_common(up, ctl, fcr_rx_trig | fcr_tx_trig);
}

/* Use set fifo callback stored in uart_config[] */
int serial8250_dispatch_set_fifo_control(struct uart_port *port,
                                const struct uart_fifo_control *ctl)
{
	struct uart_8250_port *up = up_to_u8250p(port);

	if (!port || !ctl)
		return -EINVAL;

	if (!(up->capabilities & UART_CAP_FIFO) || port->fifosize <= 1)
		return -EOPNOTSUPP;

	switch (port->type) {

		case PORT_16650V2:
			return port_16650V2_set_fifo_control(up, ctl);

		case PORT_16550A:
		default:
			return port_16550A_set_fifo_control(up, ctl);
	}

	return -EOPNOTSUPP;
}



