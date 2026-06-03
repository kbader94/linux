// SPDX-License-Identifier: GPL-2.0
/*
 * sdlin.c - LIN bus over a UART, attached via serdev.
 *
 * Sibling to sllin. Where sllin attaches at runtime from userspace via a
 * TTY line discipline (`ldattach 31 /dev/ttyXX`), sdlin binds at driver-
 * model probe time against a DT or ACPI description of a LIN node hanging
 * off a UART. That difference matters in embedded automotive scenarios
 * where the LIN bus must be answering headers within hundreds of ms of
 * power-on — too early for userspace, too early even for systemd-template
 * attach. sdlin lets the LIN netdev come up in the same wave as the rest
 * of the kernel device tree.
 *
 * Architecturally identical to sllin above the transport: embeds the
 * lin-dev helpers (struct lin_uart for the byte protocol, struct
 * lin_sched for the master schedule engine, struct lin_resp_table for
 * the publisher response cache), shares the lin_sched_kthread_fn driver
 * out of lin-dev, and registers the same lin_dev_ops vtable shape. The
 * only transport-specific code in this file is the four io_ops adapters
 * around serdev (write / break_ctl / set_baud / flush) plus the FIFO
 * probe via serdev_device_*_fifo_control.
 *
 * Author: Kyle Bader <kyle.bader94@gmail.com>
 * Copyright (c) 2026 Kyle Bader
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/kthread.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/of.h>
#include <linux/property.h>
#include <linux/serdev.h>
#include <linux/skbuff.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/string.h>
#include <linux/wait.h>

#include <linux/lin.h>
#include <linux/lin/dev.h>
#include <linux/lin/drv.h>
#include <linux/lin/uart.h>
#include <net/rtnetlink.h>

#include <uapi/linux/lin/error.h>
#include <uapi/linux/lin/raw.h>

#define DRV_NAME		"sdlin"

struct sdlin {
	struct serdev_device	*serdev;
	struct net_device	*dev;
	/* See sllin's @lock kerneldoc for the protected-state contract —
	 * identical here: wire-state cursor and per-slot @cur_* tags in
	 * @u, response cache @resp, and schedule-engine bookkeeping in
	 * @s. The pure-wakeup bits (LIN_UART_F_* / LIN_SCHED_F_*) are
	 * atomic and may be touched without holding @lock; coordination
	 * with @s.hdr_status uses the lock.
	 */
	spinlock_t		lock;

	struct lin_uart		u;
	struct lin_sched	s;

	struct task_struct	*kwthread;
	wait_queue_head_t	kwt_wq;

	struct lin_resp_table	resp;
};

/* ------------------------------------------------------------------ */
/* Forward declarations                                                */
/* ------------------------------------------------------------------ */

static const struct lin_dev_ops sdlin_lin_ops;
static const struct net_device_ops sdlin_netdev_ops;
static const struct lin_uart_io_ops sdlin_io_ops;
static const struct serdev_device_ops sdlin_serdev_ops;

/* ------------------------------------------------------------------ */
/* FIFO Control probe                                                  */
/* ------------------------------------------------------------------ */

/* Program the underlying UART for sub-frame RX interrupt latency. See
 * sllin_setup_fifo() for the rationale: the LIN spec requires a slave
 * publisher to put its response on the wire within ~40 character times
 * of the header arriving, which only works if the UART can interrupt
 * on the first received byte. Goes through serdev's set/get_fifo_control
 * ops — returns -EOPNOTSUPP gracefully when the underlying UART driver
 * has not adopted the FIFO Control framework or when the controller is
 * not TTY-backed, in which case we just skip LIN_CAP_PUB_SLAVE and the
 * link remains usable for master and observer roles.
 *
 * Returns true iff trigger == 1 (or FIFO disabled, functionally
 * equivalent on 8250-class hardware) was successfully programmed.
 */
static bool sdlin_setup_fifo(struct sdlin *sd)
{
	struct uart_fifo_control ctl, verify;
	int err;

	err = serdev_device_get_fifo_control(sd->serdev, &ctl);
	if (err) {
		netdev_warn(sd->dev,
			    "underlying UART driver does not support programmable FIFO trigger levels (serdev_device_get_fifo_control: %d); slave-publish role disabled. This is expected on UART drivers not yet ported to the FC framework.\n",
			    err);
		return false;
	}

	ctl.rx_trigger_bytes = 1;
	err = serdev_device_set_fifo_control(sd->serdev, &ctl,
					     UART_FIFO_ROUND_DOWN);
	if (!err && serdev_device_get_fifo_control(sd->serdev, &verify) == 0 &&
	    verify.rx_trigger_bytes == 1) {
		netdev_info(sd->dev, "FIFO rx trigger = 1 byte\n");
		return true;
	}

	ctl.flags &= ~UART_FIFO_CTRL_FLAG_ENABLE_FIFO;
	ctl.rx_trigger_bytes = 0;
	ctl.tx_trigger_bytes = 0;
	err = serdev_device_set_fifo_control(sd->serdev, &ctl,
					     UART_FIFO_ROUND_EXACT);
	if (!err) {
		netdev_info(sd->dev,
			    "FIFO disabled (rx trigger of 1 not supported)\n");
		return true;
	}

	netdev_warn(sd->dev,
		    "could not program FIFO for sub-frame latency (trigger=1 and disable both unsupported, err %d); slave-publish role disabled\n",
		    err);
	return false;
}

/* ------------------------------------------------------------------ */
/* serdev io_ops adapters                                              */
/* ------------------------------------------------------------------ */

static int sdlin_io_write(struct lin_uart *u, const u8 *buf, int len)
{
	struct serdev_device *serdev = u->io_priv;

	return serdev_device_write_buf(serdev, buf, len);
}

static int sdlin_io_break_ctl(struct lin_uart *u, int state)
{
	struct serdev_device *serdev = u->io_priv;

	return serdev_device_break_ctl(serdev, state);
}

static int sdlin_io_set_baud(struct lin_uart *u, u32 baud)
{
	struct serdev_device *serdev = u->io_priv;
	unsigned int actual;

	actual = serdev_device_set_baudrate(serdev, baud);
	if (!actual)
		return -EIO;
	return 0;
}

static void sdlin_io_flush_buffer(struct lin_uart *u)
{
	struct serdev_device *serdev = u->io_priv;

	serdev_device_write_flush(serdev);
}

static const struct lin_uart_io_ops sdlin_io_ops = {
	.write		= sdlin_io_write,
	.break_ctl	= sdlin_io_break_ctl,
	.set_baud	= sdlin_io_set_baud,
	.flush_buffer	= sdlin_io_flush_buffer,
	/* No tx_wakeup_arm / tx_wakeup_disarm: serdev_device_write_buf
	 * returns the byte count it accepted, and the controller fires
	 * write_wakeup unconditionally when room frees up. There is no
	 * per-transfer arm/disarm flag to flip.
	 */
};

/* ------------------------------------------------------------------ */
/* serdev_device_ops trampolines                                       */
/* ------------------------------------------------------------------ */

static size_t sdlin_receive_buf(struct serdev_device *serdev,
				const u8 *cp, size_t count)
{
	struct sdlin *sd = serdev_device_get_drvdata(serdev);

	if (!sd || !netif_running(sd->dev))
		return count;	/* consume to keep the controller buffer
				 * draining; nothing to dispatch.
				 */

	/* serdev does not pass per-byte framing-error flags. The slave
	 * parser tolerates fp == NULL via the 0x55-at-BREAK-position
	 * synthesis path; see lin_uart_slave_receive_buf().
	 */
	lin_uart_receive_buf(&sd->u, cp, NULL, count);
	return count;
}

static void sdlin_write_wakeup(struct serdev_device *serdev)
{
	struct sdlin *sd = serdev_device_get_drvdata(serdev);

	if (!sd || !netif_running(sd->dev))
		return;
	lin_uart_write_wakeup(&sd->u);
}

static const struct serdev_device_ops sdlin_serdev_ops = {
	.receive_buf	= sdlin_receive_buf,
	.write_wakeup	= sdlin_write_wakeup,
};

/* ------------------------------------------------------------------ */
/* lin_dev_ops                                                         */
/* ------------------------------------------------------------------ */

static int sdlin_op_master_start(struct lin_dev *ld)
{
	struct sdlin *sd = netdev_priv(ld->dev);

	set_bit(LIN_SCHED_F_MASTER_RUNNING, &sd->s.flags);
	return 0;
}

static int sdlin_op_master_stop(struct lin_dev *ld)
{
	struct sdlin *sd = netdev_priv(ld->dev);
	unsigned long flags;

	spin_lock_irqsave(&sd->lock, flags);
	clear_bit(LIN_SCHED_F_MASTER_RUNNING, &sd->s.flags);
	lin_sched_stop_locked(&sd->s);

	if (test_and_clear_bit(LIN_SCHED_F_HDR_REQ, &sd->s.flags) ||
	    sd->s.hdr_status == -EINPROGRESS) {
		sd->s.hdr_status = -ECANCELED;
		set_bit(LIN_SCHED_F_HDR_DONE, &sd->s.flags);
	}
	spin_unlock_irqrestore(&sd->lock, flags);

	wake_up(&sd->kwt_wq);
	return 0;
}

static int sdlin_op_set_response(struct lin_dev *ld, u8 lin_id,
				 const u8 *data, u8 len, bool enhanced)
{
	struct sdlin *sd = netdev_priv(ld->dev);
	unsigned long flags;

	spin_lock_irqsave(&sd->lock, flags);
	lin_resp_set_locked(&sd->resp, lin_id, data, len, enhanced);
	spin_unlock_irqrestore(&sd->lock, flags);
	return 0;
}

static int sdlin_op_clear_response(struct lin_dev *ld, u8 lin_id)
{
	struct sdlin *sd = netdev_priv(ld->dev);
	unsigned long flags;

	spin_lock_irqsave(&sd->lock, flags);
	lin_resp_clear_locked(&sd->resp, lin_id);
	spin_unlock_irqrestore(&sd->lock, flags);
	return 0;
}

static int sdlin_op_schedule_load(struct lin_dev *ld,
				  const struct lin_schedule *sched)
{
	struct sdlin *sd = netdev_priv(ld->dev);
	struct lin_schedule *copy, *old;
	unsigned long flags;

	copy = kmemdup(sched, struct_size(sched, entry, sched->entry_count),
		       GFP_KERNEL);
	if (!copy)
		return -ENOMEM;

	spin_lock_irqsave(&sd->lock, flags);
	old = lin_sched_load_locked(&sd->s, copy);
	spin_unlock_irqrestore(&sd->lock, flags);

	kfree(old);
	return 0;
}

static int sdlin_op_schedule_delete(struct lin_dev *ld, u8 handle)
{
	struct sdlin *sd = netdev_priv(ld->dev);
	struct lin_schedule *old;
	unsigned long flags;

	spin_lock_irqsave(&sd->lock, flags);
	if (test_bit(LIN_SCHED_F_RUNNING, &sd->s.flags) &&
	    sd->s.active == handle) {
		spin_unlock_irqrestore(&sd->lock, flags);
		return -EBUSY;
	}
	old = lin_sched_take_locked(&sd->s, handle);
	spin_unlock_irqrestore(&sd->lock, flags);

	kfree(old);
	return 0;
}

static int sdlin_op_schedule_activate(struct lin_dev *ld, u8 handle)
{
	struct sdlin *sd = netdev_priv(ld->dev);
	unsigned long flags;
	unsigned long timeout_j;

	spin_lock_irqsave(&sd->lock, flags);
	if (!test_bit(LIN_SCHED_F_RUNNING, &sd->s.flags)) {
		sd->s.active = handle;
		sd->s.slot = 0;
		sd->s.next_slot = ktime_get();
		set_bit(LIN_SCHED_F_RUNNING, &sd->s.flags);
		spin_unlock_irqrestore(&sd->lock, flags);
		wake_up(&sd->kwt_wq);
		return 0;
	}

	reinit_completion(&sd->s.activate_done);
	sd->s.activate_to = handle;
	sd->s.activate_req = true;
	timeout_j = usecs_to_jiffies(LIN_RAW_SCHEDULE_SLOT_MAX_US) +
		    msecs_to_jiffies(100);
	spin_unlock_irqrestore(&sd->lock, flags);

	wake_up(&sd->kwt_wq);

	if (!wait_for_completion_timeout(&sd->s.activate_done, timeout_j)) {
		int ret = -ETIMEDOUT;

		spin_lock_irqsave(&sd->lock, flags);
		if (!sd->s.activate_req && sd->s.active == handle)
			ret = 0;
		else
			sd->s.activate_req = false;
		spin_unlock_irqrestore(&sd->lock, flags);
		return ret;
	}
	return 0;
}

static int sdlin_op_schedule_stop(struct lin_dev *ld)
{
	struct sdlin *sd = netdev_priv(ld->dev);
	unsigned long flags;

	spin_lock_irqsave(&sd->lock, flags);
	lin_sched_stop_locked(&sd->s);
	spin_unlock_irqrestore(&sd->lock, flags);

	wake_up(&sd->kwt_wq);
	return 0;
}

static int sdlin_op_header_send(struct lin_dev *ld, u8 lin_id,
				const u8 *data, u8 len, bool enhanced)
{
	struct sdlin *sd = netdev_priv(ld->dev);
	unsigned long flags;
	unsigned long timeout_j;
	int ret;

	spin_lock_irqsave(&sd->lock, flags);
	if (sd->s.hdr_status == -EINPROGRESS) {
		spin_unlock_irqrestore(&sd->lock, flags);
		return -EBUSY;
	}
	sd->s.hdr_id = lin_id & LIN_ID_MASK;
	if (len) {
		memcpy(sd->s.hdr_data, data, len);
		sd->s.hdr_len = len;
	} else {
		sd->s.hdr_len = 0;
	}
	sd->s.hdr_enhanced = enhanced;
	sd->s.hdr_status = -EINPROGRESS;
	clear_bit(LIN_SCHED_F_HDR_DONE, &sd->s.flags);
	set_bit(LIN_SCHED_F_HDR_REQ, &sd->s.flags);
	spin_unlock_irqrestore(&sd->lock, flags);

	wake_up(&sd->kwt_wq);

	timeout_j = usecs_to_jiffies(LIN_RAW_SCHEDULE_SLOT_MAX_US) +
		    msecs_to_jiffies(100);
	if (!wait_event_timeout(sd->kwt_wq,
				test_bit(LIN_SCHED_F_HDR_DONE, &sd->s.flags),
				timeout_j)) {
		spin_lock_irqsave(&sd->lock, flags);
		sd->s.hdr_status = -ETIMEDOUT;
		spin_unlock_irqrestore(&sd->lock, flags);
	}

	spin_lock_irqsave(&sd->lock, flags);
	ret = sd->s.hdr_status;
	if (ret == -EINPROGRESS)
		ret = -EIO;
	sd->s.hdr_status = 0;
	clear_bit(LIN_SCHED_F_HDR_DONE, &sd->s.flags);
	spin_unlock_irqrestore(&sd->lock, flags);
	return ret < 0 ? ret : 0;
}

/* Wakeup pulse: drive the line dominant for ~5 ms. Per LIN 2.1+ the
 * pulse must be 250..5000 us; we sit at the upper end so even slow
 * receivers latch it.
 */
static int sdlin_op_wakeup_send(struct lin_dev *ld)
{
	struct sdlin *sd = netdev_priv(ld->dev);
	int ret;

	ret = serdev_device_break_ctl(sd->serdev, -1);
	if (ret) {
		netdev_warn(sd->dev, "wakeup break_ctl assert failed: %d\n",
			    ret);
		return ret;
	}
	usleep_range(2000, 5000);
	ret = serdev_device_break_ctl(sd->serdev, 0);
	if (ret) {
		netdev_warn(sd->dev,
			    "wakeup break_ctl release failed (%d); bus may be wedged low\n",
			    ret);
		return ret;
	}
	usleep_range(100, 200);

	lin_uart_emit_bus_event(&sd->u, LIN_ID_NONE, LIN_F_WAKEUP, 0);
	return 0;
}

static int sdlin_op_set_bitrate(struct lin_dev *ld, u32 bitrate)
{
	struct sdlin *sd = netdev_priv(ld->dev);

	if (bitrate < 1000 || bitrate > 20000)
		return -EINVAL;

	return lin_uart_set_baud(&sd->u, bitrate);
}

static const struct lin_dev_ops sdlin_lin_ops = {
	.master_start		= sdlin_op_master_start,
	.master_stop		= sdlin_op_master_stop,
	.set_response		= sdlin_op_set_response,
	.clear_response		= sdlin_op_clear_response,
	.schedule_load		= sdlin_op_schedule_load,
	.schedule_delete	= sdlin_op_schedule_delete,
	.schedule_activate	= sdlin_op_schedule_activate,
	.schedule_stop		= sdlin_op_schedule_stop,
	.header_send		= sdlin_op_header_send,
	.wakeup_send		= sdlin_op_wakeup_send,
	.set_bitrate		= sdlin_op_set_bitrate,
};

/* ------------------------------------------------------------------ */
/* netdev ops                                                          */
/* ------------------------------------------------------------------ */

static int sdlin_netdev_open(struct net_device *dev)
{
	netif_carrier_on(dev);
	netif_start_queue(dev);
	return 0;
}

static int sdlin_netdev_stop(struct net_device *dev)
{
	netif_stop_queue(dev);
	netif_carrier_off(dev);
	return 0;
}

static netdev_tx_t sdlin_netdev_xmit(struct sk_buff *skb,
				     struct net_device *dev)
{
	kfree_skb(skb);
	dev->stats.tx_dropped++;
	return NETDEV_TX_OK;
}

static const struct net_device_ops sdlin_netdev_ops = {
	.ndo_open	= sdlin_netdev_open,
	.ndo_stop	= sdlin_netdev_stop,
	.ndo_start_xmit	= sdlin_netdev_xmit,
};

/* ------------------------------------------------------------------ */
/* Bitrate resolution                                                  */
/* ------------------------------------------------------------------ */

/* Resolve the bus bitrate at probe time:
 *
 *   1. LIN-specific "bitrate" property on the sdlin node (preferred).
 *   2. Standard "current-speed" property on the UART parent (a useful
 *      fallback when the LIN bitrate happens to match the UART's
 *      configured speed, which it typically does).
 *   3. LIN_DEFAULT_BITRATE (19200) when neither is present.
 *
 * Range-checked against the LIN spec's [1000, 20000] bps window.
 */
static int sdlin_resolve_bitrate(struct serdev_device *serdev, u32 *out)
{
	struct device *parent = serdev->dev.parent;
	u32 bitrate;

	if (device_property_read_u32(&serdev->dev, "bitrate", &bitrate) == 0)
		goto check;

	if (parent &&
	    device_property_read_u32(parent, "current-speed", &bitrate) == 0)
		goto check;

	bitrate = LIN_DEFAULT_BITRATE;

check:
	if (bitrate < 1000 || bitrate > 20000) {
		dev_err(&serdev->dev,
			"bitrate %u out of range [1000, 20000]\n", bitrate);
		return -EINVAL;
	}
	*out = bitrate;
	return 0;
}

/* ------------------------------------------------------------------ */
/* probe / remove                                                      */
/* ------------------------------------------------------------------ */

static int sdlin_probe(struct serdev_device *serdev)
{
	struct net_device *dev;
	struct lin_dev *ld;
	struct sdlin *sd;
	unsigned int actual;
	u32 bitrate;
	int err;

	err = sdlin_resolve_bitrate(serdev, &bitrate);
	if (err)
		return err;

	dev = alloc_lindev(sizeof(struct sdlin), &sdlin_lin_ops);
	if (!dev)
		return -ENOMEM;

	dev->netdev_ops = &sdlin_netdev_ops;
	strscpy(dev->name, "sdlin%d", IFNAMSIZ);
	SET_NETDEV_DEV(dev, &serdev->dev);

	ld = lin_get_ml_priv(dev);
	ld->caps = LIN_CAP_SPORADIC | LIN_CAP_EVENT | LIN_CAP_DIAG |
		   LIN_CAP_CHK_ENH | LIN_CAP_WAKEUP;
	ld->bitrate = bitrate;

	sd = netdev_priv(dev);
	sd->serdev = serdev;
	sd->dev = dev;
	spin_lock_init(&sd->lock);
	lin_resp_table_init(&sd->resp);
	init_waitqueue_head(&sd->kwt_wq);
	lin_sched_init(&sd->s);

	lin_uart_init(&sd->u, dev, &sd->resp, &sd->s, &sd->lock, &sd->kwt_wq,
		      &sdlin_io_ops, serdev, bitrate);

	/* Set drvdata + client_ops before serdev_device_open: the
	 * receive_buf / write_wakeup trampolines look up @sd via
	 * serdev_device_get_drvdata, and the controller may start
	 * delivering RX bytes as soon as open returns.
	 */
	serdev_device_set_drvdata(serdev, sd);
	serdev_device_set_client_ops(serdev, &sdlin_serdev_ops);

	err = serdev_device_open(serdev);
	if (err) {
		dev_err(&serdev->dev, "serdev_device_open failed: %d\n", err);
		goto err_free;
	}

	actual = serdev_device_set_baudrate(serdev, bitrate);
	if (!actual) {
		dev_err(&serdev->dev, "serdev_device_set_baudrate(%u) failed\n",
			bitrate);
		err = -EIO;
		goto err_close;
	}
	sd->u.baud = actual;
	ld->bitrate = actual;

	serdev_device_set_flow_control(serdev, false);

	/* FIFO setup is best-effort: failure just means we cannot
	 * advertise LIN_CAP_PUB_SLAVE and the link works for master and
	 * observer roles. Operator can override via IFLA_LIN_FORCE_PUB_SLAVE
	 * if their cluster tolerates the higher RX latency.
	 */
	if (sdlin_setup_fifo(sd))
		ld->caps |= LIN_CAP_PUB_SLAVE;

	err = lin_register_netdev(dev);
	if (err) {
		dev_err(&serdev->dev, "lin_register_netdev failed: %d\n", err);
		goto err_close;
	}

	sd->kwthread = kthread_run(lin_sched_kthread_fn, &sd->u,
				   "sdlin/%s", dev->name);
	if (IS_ERR(sd->kwthread)) {
		err = PTR_ERR(sd->kwthread);
		sd->kwthread = NULL;
		lin_unregister_netdev(dev);
		goto err_close;
	}

	netdev_info(dev, "sdlin attached at %u bps\n", actual);
	return 0;

err_close:
	serdev_device_close(serdev);
err_free:
	free_lindev(dev);
	return err;
}

static void sdlin_remove(struct serdev_device *serdev)
{
	struct sdlin *sd = serdev_device_get_drvdata(serdev);

	/* Same teardown order as sllin_ldisc_close: fence sockopts via
	 * NETDEV_GOING_DOWN before stopping the kthread, so per-socket
	 * force-release ops (master_stop -> wake parked header_send)
	 * still find a live kthread to wake. See sllin.c for the full
	 * rationale.
	 */
	lin_unregister_netdev(sd->dev);

	if (sd->kwthread) {
		kthread_stop(sd->kwthread);
		sd->kwthread = NULL;
	}

	serdev_device_close(serdev);
	lin_sched_destroy(&sd->s);
	free_lindev(sd->dev);
}

/* ------------------------------------------------------------------ */
/* Driver registration                                                 */
/* ------------------------------------------------------------------ */

static const struct of_device_id sdlin_of_match[] = {
	{ .compatible = "linux,sdlin" },
	{}
};
MODULE_DEVICE_TABLE(of, sdlin_of_match);

static struct serdev_device_driver sdlin_driver = {
	.probe		= sdlin_probe,
	.remove		= sdlin_remove,
	.driver = {
		.name		= DRV_NAME,
		.of_match_table	= sdlin_of_match,
	},
};
module_serdev_device_driver(sdlin_driver);

MODULE_DESCRIPTION("Local Interconnect Network (LIN) interface over serdev");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Kyle Bader <kyle.bader94@gmail.com>");
MODULE_ALIAS("platform:" DRV_NAME);
