#include <linux/module.h>
#include <linux/io.h>
#include <linux/serial_reg.h>
#include "8250.h"
#include "8250_fifo.h"

/* TODO: move all this to 8250_port.c and rmv below func */
static void serial8250_clear_fifos(struct uart_8250_port *p)
{
	if (p->capabilities & UART_CAP_FIFO) {
		serial_out(p, UART_FCR, UART_FCR_ENABLE_FIFO);
		serial_out(p, UART_FCR, UART_FCR_ENABLE_FIFO |
			       UART_FCR_CLEAR_RCVR | UART_FCR_CLEAR_XMIT);
		serial_out(p, UART_FCR, 0);
	}
}

/* Supports 1550A-15750 */
static int rx_trig_to_fcr(struct uart_8250_port *up, u32 level)
{
	const struct serial8250_config *conf;
	 
	conf = &serial8250_uart_config[up->port.type];

	for (int i = 0; i < UART_FCR_R_TRIG_MAX_STATE; i++) {
		if (conf->rxtrig_bytes[i] && conf->rxtrig_bytes[i] == level) {
			if (i >= 4)
				return UART_FCR7_64BYTE | UART_FCR_R_FROM_TRIG_I(i - 4); /* 16750 mode */
			else
				return UART_FCR_R_FROM_TRIG_I(i); /* 16550A mode */
		}
	}

	return -EINVAL;
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

static int write_fcr_common(struct uart_8250_port *up, 
							const struct uart_fifo_control *ctl, u8 fcr)
{
	unsigned long sl_flags;

	uart_port_lock_irqsave(&up->port, &sl_flags);

	serial8250_clear_fifos(up);

	if (ctl->flags & (UART_FIFO_CTRL_FLAG_ENABLE_RX))
		fcr |= UART_FCR_ENABLE_FIFO;

	serial_out(up, UART_FCR, fcr);
	up->fcr = fcr & ~(UART_FCR_CLEAR_RCVR | UART_FCR_CLEAR_XMIT);	
	uart_port_unlock_irqrestore(&up->port, sl_flags);
	return 0;
}

static int port_16750_set_fifo_control(struct uart_8250_port *up, 
								  const struct uart_fifo_control *ctl)
{
	int ret, fcr_rx_trig = 0;
	u8 lcr = 0;

	fcr_rx_trig = rx_trig_to_fcr(up, ctl->rx_trigger_bytes);
	if (fcr_rx_trig < 0)
		return fcr_rx_trig;

	/* set DLAB LCR[7] */
	if (fcr_rx_trig & UART_FCR7_64BYTE) {
		lcr = serial_in(up, UART_LCR);
		serial_out(up, UART_LCR, lcr | UART_LCR_DLAB);
	}

	ret = write_fcr_common(up, ctl, fcr_rx_trig);

	/* unset DLAB LCR[7] */
	if (fcr_rx_trig & UART_FCR7_64BYTE) {
		serial_out(up, UART_LCR, lcr); // Restore LCR
	}

	return ret;
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

	if (!(up->capabilities & UART_CAP_FIFO) || port->fifosize <= 1)
		return -EOPNOTSUPP;

	switch (port->type) {
		case PORT_CH38X:
			return port_16750_set_fifo_control(up, ctl);

		case PORT_16750:
			return port_16750_set_fifo_control(up, ctl);

		case PORT_16650V2:
			return port_16650V2_set_fifo_control(up, ctl);

		case PORT_16550A:
		default:
			return port_16550A_set_fifo_control(up, ctl);
	}

	return -EOPNOTSUPP;
}



