// SPDX-License-Identifier: GPL-2.0
/*
 * uart_probe.c - DebugFS interface for UART FIFO probing
 *
 * Copyright (C) 2025 Kyle L. Bader
 *
 * This module allows probing of UART FIFO sizes and levels
 * by utilizing internal loopback mode to physically test
 * the current configuration. It currently supports
 * serial devices compatible with the 8250 core and is intended for driver
 * development and diagnostics of fifo_control.
 */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/debugfs.h>
#include <linux/fs.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/serial_core.h>
#include <linux/serial_8250.h>
#include <linux/uaccess.h>
#include <linux/delay.h>

#define FIFO_SIZE_MAX 512

static struct dentry *dir_entry;
static struct dentry *dev_entry;
static struct dentry *tx_fifo_entry;
static struct dentry *tx_trig_entry;
static struct dentry *rx_fifo_entry;
static struct dentry *rx_trig_entry;
static char selected_dev[16] = "ttyS0";

struct port_context {
	struct tty_driver *driver;
	struct tty_port *tport;
	struct uart_state *state;
	struct uart_port *port;
	struct uart_8250_port *u8250p;
	int line;
};

struct port_state {
	u8 lcr;
	u8 ier;
	u8 mcr;
	u8 fcr;
};

/* uart_probe/select_dev
 * Select serial device for testing 
 * eg. ttyS1
 */
static ssize_t select_dev_write(struct file *file, const char __user *buf,
				size_t count, loff_t *ppos)
{
	if (!count || count >= sizeof(selected_dev))
		return -EINVAL;

	if (copy_from_user(selected_dev, buf, count))
		return -EFAULT;

	selected_dev[count] = '\0';
	selected_dev[strcspn(selected_dev, "\n")] = '\0';

	pr_info("8250_fifo_probe: selected TTY device is now: %s\n",
		selected_dev);
	return count;
}

static ssize_t select_dev_read(struct file *file, char __user *buf,
			       size_t count, loff_t *ppos)
{
	char tmp[32];
	int len = snprintf(tmp, sizeof(tmp), "%s\n", selected_dev);
	return simple_read_from_buffer(buf, count, ppos, tmp, len);
}

static const struct file_operations select_dev_fops = {
	.write = select_dev_write,
	.read = select_dev_read,
};

static int get_port(struct port_context *context)
{
	context->driver = tty_find_polling_driver(selected_dev, &context->line);

	if (!context->driver) {
		pr_err("8250_fifo_probe: tty_find_driver failed\n");
		return -ENODEV;
	}

	if (context->line < 0 || context->line >= context->driver->num || 
	    !context->driver->ports) {
		pr_err("8250_fifo_probe: Invalid port\n");
		tty_driver_kref_put(context->driver);
		return -ENODEV;
	}

	context->tport = context->driver->ports[context->line];
	context->state = container_of(context->tport, struct uart_state, port);
	context->port = context->state->uart_port;
	context->u8250p = up_to_u8250p(context->port);

	if (!context->tport) {
		pr_err("8250_fifo_probe: No tty_port\n");
		tty_driver_kref_put(context->driver);
		return -ENODEV;
	}

	if (!context->u8250p) {
		pr_err("8250_fifo_probe: Not an 8250-based UART\n");
		tty_driver_kref_put(context->driver);
		return -ENODEV;
	} 

	if (uart_console(context->port)) {
		pr_err("8250_fifo_probe: Port is console\n");
		tty_driver_kref_put(context->driver);
		return -EBUSY;
	}

	if (tty_port_initialized(context->tport) && tty_port_users(context->tport) > 0) {
		pr_err("8250_fifo_probe: TTY device %s is busy or opened by userspace\n",
		       selected_dev);
		tty_driver_kref_put(context->driver);
		return -EBUSY;
	}

	mutex_lock(&context->tport->mutex);
	return 0;
}

static void init_port(struct uart_port *port, struct port_state *state)
{
	struct uart_8250_port *u8250p = up_to_u8250p(port);

	state->lcr = port->serial_in(port, UART_LCR);
	state->fcr = u8250p->fcr;
	state->mcr = port->serial_in(port, UART_MCR);
	state->ier = port->serial_in(port, UART_IER);

	/* Enable and clear FIFO */
	port->serial_out(port, UART_FCR, state->fcr | UART_FCR_CLEAR_RCVR | UART_FCR_CLEAR_XMIT);

	/* Enable loopback */
	port->serial_out(port, UART_MCR, state->mcr | UART_MCR_LOOP);

	/* Drain RX */
	while (port->serial_in(port, UART_LSR) & UART_LSR_DR)
		(void)port->serial_in(port, UART_RX);

	/* Set baud to 115200 */
	port->serial_out(port, UART_LCR, UART_LCR_CONF_MODE_A);
	port->serial_out(port, UART_DLL, 1);
	port->serial_out(port, UART_DLM, 0);
	port->serial_out(port, UART_LCR, UART_LCR_WLEN8);
}

static void release_port(struct port_context *context)
{

	mutex_unlock(&context->tport->mutex);
	tty_driver_kref_put(context->driver);

}

static void restore_port(struct uart_port *port, struct port_state *state)
{
	port->serial_out(port, UART_FCR, state->fcr);
	port->serial_out(port, UART_MCR, state->mcr);
	port->serial_out(port, UART_LCR, state->lcr);
	port->serial_out(port, UART_IER, state->ier);
}

static int measure_tx_fifo_size(struct port_context *ctx)
{
	struct port_state st;
	struct uart_port *port = ctx->port;
	unsigned long deadline;
	int i, tx_count = 0, rx_count = 0;
	u8 lsr;

	init_port(port, &st);

	for (i = 0; i < FIFO_SIZE_MAX; i++) {
		port->serial_out(port, UART_TX, 0xFF);
		tx_count++;
	}

	deadline = jiffies + msecs_to_jiffies(500);
	while (time_before(jiffies, deadline) && rx_count < tx_count) {
		lsr = port->serial_in(port, UART_LSR);
		if (lsr & UART_LSR_DR) {
			if (port->serial_in(port, UART_RX) == 0xFF)
				rx_count++;
		} else {
			cpu_relax();
		}
	}

	restore_port(port, &st);
	return rx_count > 0 ? rx_count : -EIO;
}

/* uart_probe/rx_trig_test
 * Probe the serial devices RX FIFO trigger level
 * by setting internal loopback and sending data to itself,
 * one byte at a time,
 * until the rx interrupt is triggered 
 * @returns RX FIFO trigger level in number of bytes
 */
static ssize_t rx_trig_probe_read(struct file *file, char __user *buf,
				  size_t count, loff_t *ppos)
{
	char tmp[128];
	struct port_context context;
	struct port_state state;
	struct uart_port *port;
	int trig = 0;
	int rtn, strlen;
	u8 iir = 0;
	u8 iir_id;
	
	if (*ppos)
		return 0; /* EOF */

	pr_info("8250_fifo_probe: starting RX trigger probe\n");
	
	rtn = get_port(&context);
	if(rtn){
		strlen = snprintf(tmp, sizeof(tmp), "Failed to initialize port\n");
		return simple_read_from_buffer(buf, count, ppos, tmp, strlen);
	}

	port = context.port;
	init_port(port, &state);

	/* Enable RX interrupts */
	port->serial_out(port, UART_IER, UART_IER_RDI);

	/* Probe for trigger threshold */
	for (trig = 1; trig < 256; trig++) {
		port->serial_out(port, UART_TX, 0x55);

		/* Wait for byte transmission */
		udelay(100); /* 1 byte @ 115200 bps = ~87us */
		iir = port->serial_in(port, UART_IIR);
		iir_id = iir & UART_IIR_ID;

		if (!(iir & UART_IIR_NO_INT)) {
			if ((iir_id == UART_IIR_RDI))
				break;
		}
	}

	/* Disable interrupts */
	port->serial_out(port, UART_IER, 0x00);

	/* Drain RX FIFO */
	while (port->serial_in(port, UART_LSR) & UART_LSR_DR) 
		port->serial_in(port, UART_RX);

	restore_port(port, &state);
	release_port(&context);

	if (trig >= 256) {
		pr_err("8250_fifo_probe: RX trigger test failed — no interrupt detected\n");
		strlen = snprintf(tmp, sizeof(tmp), "RX trigger test failed\n");
	} else {
		strlen = snprintf(tmp, sizeof(tmp), "%d\n", trig);
	}

	return simple_read_from_buffer(buf, count, ppos, tmp, strlen);
}

static const struct file_operations rx_trig_fops = {
	.read = rx_trig_probe_read,
	.llseek = default_llseek,
};

/* uart_probe/rx_fifo_size
 * Probe the RX FIFO size by setting internal loopback,
 * transmitting data to itself, one byte at a time
 * and detecting rx overrun
 * @returns the size of th RX FIFO in number of bytes
 */
static ssize_t rx_fifo_size_read(struct file *file, char __user *buf,
				 size_t count, loff_t *ppos)
{
	char tmp[128];
	struct port_context context;
	struct port_state state;
	struct uart_port *port;
	int count_tx = 0;
	int rx_fifo_size = 0;
	u8 lsr = 0;
	int rtn, strlen;

	if (*ppos)
		return 0; /* EOF */

	rtn = get_port(&context);
	if(rtn){
		strlen = snprintf(tmp, sizeof(tmp), "Failed to initialize port\n");
		return simple_read_from_buffer(buf, count, ppos, tmp, strlen);
	}

	port = context.port;
	init_port(port, &state);

	/* Transmit one byte at a time and check for overrun */
	for (count_tx = 0; count_tx < FIFO_SIZE_MAX; count_tx++) {
		port->serial_out(port, UART_TX, 0xff);
		mdelay(1);

		lsr = port->serial_in(port, UART_LSR);
		if (lsr & UART_LSR_OE) {
			rx_fifo_size = count_tx;
			break;
		}
	}

	restore_port(port, &state);
	
	release_port(&context);

	strlen = (rx_fifo_size == 0) ? snprintf(tmp, sizeof(tmp), "RX overflow not detected\n") :
				    snprintf(tmp, sizeof(tmp), "%d\n", rx_fifo_size);

	return simple_read_from_buffer(buf, count, ppos, tmp, strlen);
}

static const struct file_operations rx_fifo_fops = {
	.read = rx_fifo_size_read,
	.llseek = default_llseek,
};

/* uart_probe/tx_fifo_size 
 * Probe the TX FIFO size by overrunning the THR
 * enabling loopback, and counting how many bytes
 * we received. This should match port->fifosize.
 * @returns TX FIFO size in number of bytes
 */
static ssize_t tx_fifo_size_read(struct file *file, char __user *buf,
				 size_t count, loff_t *ppos)
{
	char tmp[128];
	struct port_context context;
	struct port_state state;
	struct uart_port *port;
	unsigned long deadline;
	int i, rx_count = 0, tx_count = 0;
	int rtn, strlen;
	u8 lsr;

	if (*ppos)
		return 0; /* EOF */

	pr_info("8250_fifo_probe: starting TX size probe\n");

	rtn = get_port(&context);
	if(rtn){
		strlen = snprintf(tmp, sizeof(tmp), "Failed to initialize port\n");
		return simple_read_from_buffer(buf, count, ppos, tmp, strlen);
	}

	port = context.port;
	init_port(port, &state);

	/* Fill TX FIFO */
	for (i = 0; i < FIFO_SIZE_MAX; i++) {
		port->serial_out(port, UART_TX, 0xFF);
		tx_count++;
	}

	mdelay(50);

	/* Count how many bytes we received */
	deadline = jiffies + msecs_to_jiffies(500);
	while (time_before(jiffies, deadline) && rx_count < tx_count) {
		lsr = port->serial_in(port, UART_LSR);
		if (lsr & UART_LSR_DR) {
			unsigned char val = port->serial_in(port, UART_RX);
			if (val == 0xFF)
				rx_count++;
		} else {
			cpu_relax();
		}
	}

	restore_port(port, &state);

	release_port(&context);

	if (rx_count == 0)
		return simple_read_from_buffer(buf, count, ppos,
			"TX loopback failed or no data received\n", 42);
	else {
		strlen = scnprintf(tmp, sizeof(tmp), "%d\n", rx_count);
		return simple_read_from_buffer(buf, count, ppos, tmp, strlen);
	}
}

static const struct file_operations tx_fifo_fops = {
	.read = tx_fifo_size_read,
	.llseek = default_llseek,
};

/* uart_probe/tx_trig_level
 * Probe the trigger level of the TX FIFO
 * by enabling loopback, filling the TX FIFO, 
 * and counting how many bytes until THRI interrupt is set
 * NOTE: we explicitly cap TX size to port->fifosize
 * otherwise serial_out will drop & transmit
 * randomly while it's full.
 * @returns TX FIFO trigger level in number of bytes
 */
static ssize_t tx_trig_probe_read(struct file *file, char __user *buf,
				  size_t count, loff_t *ppos)
{
	char tmp[128];
	struct port_context context;
	struct port_state state;
	struct uart_port *port;
	unsigned long deadline;
	int measured_tx_fifo = 0;
	int trig = 0;
	int rx_count = 0;
	u8 lsr, iir;
	int i, rtn, strlen;

	if (*ppos)
		return 0; /* EOF */

	pr_info("8250_fifo_probe: starting TX size probe\n");

	rtn = get_port(&context);
	if(rtn){
		strlen = snprintf(tmp, sizeof(tmp), "Failed to initialize port\n");
		return simple_read_from_buffer(buf, count, ppos, tmp, strlen);
	}

	port = context.port;

	measured_tx_fifo = measure_tx_fifo_size(&context);

	if (measured_tx_fifo < 1) {
		char tmp[64];
		release_port(&context);
		strlen = scnprintf(tmp, sizeof(tmp),
				"TX loopback failed or no data received\n");
		return simple_read_from_buffer(buf, count, ppos, tmp, strlen);
	}

	init_port(port, &state);

	/* Enable Transmission Hold Register Empty Interrupt */
	port->serial_out(port, UART_IER, UART_IER_THRI);

	/* Fill THR, but don't overfill it!  */
	for (i = 0; i <= measured_tx_fifo; i++)
		port->serial_out(port, UART_TX, 0xFF);

	/* Count how many bytes we rx until THR is empty */
	deadline = jiffies + msecs_to_jiffies(1500);
	while (time_before(jiffies, deadline)) {
		lsr = port->serial_in(port, UART_LSR);
		if (lsr & UART_LSR_DR) {
			port->serial_in(port, UART_RX);
			rx_count++;
		}

		iir = port->serial_in(port, UART_IIR);
		if (!(iir & UART_IIR_NO_INT) && (iir & 0x0E) == UART_IIR_THRI) {
			trig = measured_tx_fifo + 1 - rx_count;
			break;
		}

		ndelay(10);
	}

	restore_port(port, &state);
	release_port(&context);

	if (trig == 0)
		return simple_read_from_buffer(
			buf, count, ppos,
			"TX loopback failed or no data received\n", 42);
	else {
		strlen = scnprintf(tmp, sizeof(tmp), "%d\n", trig);
		return simple_read_from_buffer(buf, count, ppos, tmp, strlen);
	}
}

static const struct file_operations tx_trig_fops = {
	.read = tx_trig_probe_read,
	.llseek = default_llseek,
};

static int __init serial8250_fifo_probe_debugfs_init(void)
{
	dir_entry = debugfs_create_dir("serial8250_fifo_probe", NULL);
	if (!dir_entry)
		return -ENOMEM;

	dev_entry = debugfs_create_file("select_dev", 0666, dir_entry, NULL,
					&select_dev_fops);
	tx_fifo_entry = debugfs_create_file("tx_fifo_size", 0444, dir_entry,
					    NULL, &tx_fifo_fops);
	tx_trig_entry = debugfs_create_file("tx_trig_level", 0444, dir_entry,
					    NULL, &tx_trig_fops);
	rx_fifo_entry = debugfs_create_file("rx_fifo_size", 0444, dir_entry,
					    NULL, &rx_fifo_fops);
	rx_trig_entry = debugfs_create_file("rx_trig_level", 0444, dir_entry,
					    NULL, &rx_trig_fops);

	if (!dev_entry || !rx_fifo_entry || !rx_trig_entry) {
		debugfs_remove_recursive(dir_entry);
		return -ENOMEM;
	}

	pr_info("8250_fifo_probe: loaded\n");
	return 0;
}

static void __exit serial8250_fifo_probe_debugfs_exit(void)
{
	debugfs_remove_recursive(dir_entry);
	pr_info("8250_fifo_probe: unloaded\n");
}

module_init(serial8250_fifo_probe_debugfs_init);
module_exit(serial8250_fifo_probe_debugfs_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Kyle L. Bader");
MODULE_DESCRIPTION("DebugFS interface for probing UART FIFO config");
