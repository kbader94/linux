// SPDX-License-Identifier: GPL-2.0
/*
 * sllin.c - serial line LIN interface (TTY line discipline)
 *
 * LIN bus on top of a UART/TTY. Attach with `ldattach 31 /dev/ttyXX`;
 * the discipline allocates a sllinN netdev and plumbs it into the
 * PF_LIN socket family via the LIN core's lin_dev_ops vtable. From a
 * userspace point of view the link behaves like any other LIN netdev:
 * `ip link set sllin0 type lin bitrate 19200`, then bind a PF_LIN
 * socket and exchange struct lin_frame.
 *
 * This driver is a rewrite of Pavel Pisa's out-of-tree sllin
 * (https://github.com/lin-bus/linux-lin) against the in-tree PF_LIN
 * subsystem. The original SocketCAN-based wire-format state machine
 * (break/sync/PID/data/checksum, both as a master driving a schedule
 * and as a slave answering an external master's headers) is faithfully
 * preserved; what changes is the surface above it. Where the original
 * encoded LIN frames into struct can_frame and routed config through
 * AF_CAN ioctls, this driver:
 *
 *   - allocates a struct lin_dev via alloc_lindev() and registers
 *     master + response + wakeup ops on the lin_dev_ops vtable, so the
 *     LIN core's master role / schedule / publisher / wakeup state is
 *     authoritative rather than being reinvented in the driver;
 *   - delivers received frames with alloc_lin_skb() + netif_rx() (bus-
 *     observed traffic) or lin_loopback_rx() (self-emitted master
 *     headers and locally-published responses, so owner-tagged
 *     loopback honours LIN_RAW_RECV_OWN_MSGS);
 *   - configures bit rate via rtnetlink IFLA_LIN_BITRATE through the
 *     LIN core's changelink path (lin_dev_ops.set_bitrate), seeded
 *     from the underlying TTY's termios baud at attach time;
 *   - generates the break low pulse via tty->ops->break_ctl plus a
 *     timed usleep_range. The underlying UART driver must implement
 *     break_ctl, which is universal on 8250, pl011, imx, amba, and
 *     effectively every other in-tree UART driver; attach fails
 *     cleanly on UARTs that do not. The original `break_by_baud`
 *     module parameter and the baud-switch fallback are gone.
 *
 * Credits:
 *
 *   The wire-format byte parser, master kthread scheduler shape, break
 *   timing, and the PID parity table are all directly inherited from
 *   Pavel Pisa's out-of-tree sllin, with assistance from Rostislav
 *   Lisovy and Michal Sojka (Czech Technical University in Prague,
 *   funded by Volkswagen Group Research). Without their decade of
 *   running serial-line LIN against real hardware this driver — and
 *   indeed the broader SocketLIN subsystem it builds on — would not
 *   exist.
 *
 * Author: Kyle Bader <kyle.bader94@gmail.com>
 * Copyright (c) 2026 Kyle Bader
 *
 * Based on the out-of-tree sllin driver:
 *   Copyright (c) 2011 Czech Technical University in Prague
 *   Copyright (c) 2011 Volkswagen Group Research
 *   Authors: Pavel Pisa <pisa@cmp.felk.cvut.cz>
 *            Rostislav Lisovy <lisovy@kormus.cz>
 *            Michal Sojka <sojkam1@fel.cvut.cz>
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/hrtimer.h>
#include <linux/if_arp.h>
#include <linux/init.h>
#include <linux/kthread.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/sched.h>
#include <linux/sched/types.h>
#include <linux/serial_core.h>
#include <linux/skbuff.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/string.h>
#include <linux/tty.h>
#include <linux/tty_driver.h>
#include <linux/tty_ldisc.h>
#include <linux/uaccess.h>
#include <linux/wait.h>
#include <asm/termbits.h>

#include <linux/lin.h>
#include <linux/lin/dev.h>
#include <linux/lin/drv.h>
#include <linux/lin/uart.h>
#include <net/rtnetlink.h>

#include <uapi/linux/lin/error.h>
#include <uapi/linux/lin/raw.h>
#include <uapi/linux/tty.h>

#define DRV_NAME		"sllin"

/* The per-ID publisher response cache and the wire-level byte-protocol
 * state both live in lin-dev helpers: struct lin_resp_table (see
 * <linux/lin/drv.h>) and struct lin_uart (see <linux/lin/uart.h>).
 * sllin embeds both directly in struct sllin and serialises access to
 * the response cache with its per-link spinlock.
 */

struct sllin {
	struct tty_struct	*tty;
	struct net_device	*dev;
	/* @lock protects state that must be coherent across the kthread,
	 * the TTY rx/tx callbacks, and the lin_dev_ops invoked from
	 * sockopt context: the wire state-machine cursor and counters
	 * (in @u), the per-slot @cur_* tags (in @u), the response cache
	 * (@resp), and the schedule-engine bookkeeping in @s. The pure-
	 * wakeup bits — LIN_UART_F_* (in @u.flags) for byte-machine
	 * events, LIN_SCHED_F_MASTER_RUNNING / LIN_SCHED_F_RUNNING /
	 * LIN_SCHED_F_HDR_REQ / LIN_SCHED_F_HDR_DONE (in @s.flags) for
	 * the schedule engine — are atomic and may be set/cleared/tested
	 * without holding @lock; coordination with the lock-protected
	 * @s.hdr_status uses the lock.
	 */
	spinlock_t		lock;

	/* UART byte-protocol state (lin-dev helper). Embeds rx/tx
	 * buffers, the BREAK_SENT->ID_SENT->RESPONSE_WAIT/SENT cursor,
	 * baud, the per-frame rx_timer and timeout, and the per-slot
	 * @cur_* loopback tags.
	 */
	struct lin_uart		u;

	/* Master schedule engine state (lin-dev helper). Embeds
	 * schedule storage, active-cursor + deadline, activate-deferred
	 * completion, event-divert bookkeeping, ad-hoc header_send
	 * slot, and the LIN_SCHED_F_* atomic flag bitmap.
	 */
	struct lin_sched	s;

	/* Kthread for the schedule engine + UART byte-machine. */
	struct task_struct	*kwthread;
	wait_queue_head_t	kwt_wq;

	/* Per-ID publisher response cache (lin-dev helper). Accessed
	 * under @lock; see <linux/lin/drv.h>.
	 */
	struct lin_resp_table	resp;
};

/* ------------------------------------------------------------------ */
/* Forward declarations                                                */
/* ------------------------------------------------------------------ */

static const struct lin_dev_ops sllin_lin_ops;
static const struct net_device_ops sllin_netdev_ops;
static struct tty_ldisc_ops sllin_ldisc;
static const struct lin_uart_io_ops sllin_io_ops;

/* ------------------------------------------------------------------ */
/* Helpers                                                             */
/* ------------------------------------------------------------------ */

/* Reprogram the underlying TTY to @speed bps using BOTHER (non-stdbaud)
 * + tty_encode_baud_rate; LIN runs at 1000..20000 bps which is not a
 * standard CBAUD value on most platforms, so BOTHER is mandatory. The
 * frame format is forced to 8N1 + CLOCAL + HUPCL (drop modem control
 * for LIN, hang up on close), parity / framing errors are flagged
 * through to receive_buf via BRKINT|INPCK so the wire-level state
 * machine can react to them. Output post-processing and canonical
 * input are off, since the byte stream is opaque.
 */
static int sltty_change_speed(struct tty_struct *tty, unsigned int speed)
{
	struct ktermios old_termios, termios;

	down_write(&tty->termios_rwsem);

	old_termios = termios = tty->termios;

	termios.c_cflag = CS8 | CREAD | CLOCAL | HUPCL;
	termios.c_cflag &= ~(CBAUD | CIBAUD);
	termios.c_cflag |= BOTHER;
	termios.c_oflag = 0;
	termios.c_lflag = 0;
	termios.c_iflag = BRKINT | INPCK;

	tty->termios = termios;

	tty_encode_baud_rate(tty, speed, speed);

	if (tty->ops->set_termios)
		tty->ops->set_termios(tty, &old_termios);

	up_write(&tty->termios_rwsem);

	return 0;
}

/* Program the UART RX FIFO for sub-frame interrupt latency so the
 * byte-level state machine is not stalled waiting for a half-full
 * FIFO to flush. LIN frames are at most twelve bytes (break, sync,
 * PID, eight data, checksum) — small enough that any RX trigger
 * larger than 1 forces the parser to wait on the rx-timeout or the
 * next character before it can react to a checksum byte, which on a
 * 10417 bps J2602 link is the difference between sub-frame and
 * supra-frame parsing latency.
 *
 * Returns true iff the FIFO was successfully programmed to interrupt
 * on every byte (trigger == 1, or FIFO disabled which is functionally
 * equivalent on 8250-class hardware). False otherwise — in which case
 * the caller MUST NOT advertise LIN_CAP_PUB_SLAVE, because the LIN
 * spec's header-RX → response-TX window cannot be met.
 *
 * Reaches across the TTY/UART boundary directly because no
 * tty_operations method exposes FIFO control; the cast is type-safe
 * because the TTY_DRIVER_TYPE_SERIAL check gates it. UART drivers
 * that do not implement @get_fifo_control / @set_fifo_control return
 * -EOPNOTSUPP from the uart_*_fifo_control helpers — that's the
 * canonical signal for "this transport cannot guarantee sub-frame RX
 * latency," covering USB-CDC bridges (ft232, ch340, pl2303, cp210x)
 * and any other UART driver that has not adopted the FC framework.
 * On those transports sllin still works for master and observer
 * roles, just not as a slave-only publisher.
 *
 * Every failure path emits a netdev_warn explaining why, so an
 * operator who hits -EOPNOTSUPP on LIN_RAW_PUBLISH can grep dmesg
 * and see the reason immediately rather than puzzling over an
 * analyzer trace.
 */
static bool sllin_setup_fifo(struct sllin *sl)
{
	struct tty_struct *tty = sl->tty;
	struct uart_state *state;
	struct uart_port *port;
	struct uart_fifo_control ctl, verify;
	int err;

	if (tty->driver->type != TTY_DRIVER_TYPE_SERIAL) {
		netdev_warn(sl->dev,
			    "TTY is not UART-backed (driver type %d); cannot guarantee sub-frame RX latency, slave-publish role disabled\n",
			    tty->driver->type);
		return false;
	}

	state = tty->driver_data;
	if (!state || !state->uart_port) {
		netdev_warn(sl->dev,
			    "TTY has no associated uart_port; cannot guarantee sub-frame RX latency, slave-publish role disabled\n");
		return false;
	}
	port = state->uart_port;

	err = uart_get_fifo_control(port, &ctl);
	if (err) {
		netdev_warn(sl->dev,
			    "underlying UART driver does not support programmable FIFO trigger levels (uart_get_fifo_control: %d); slave-publish role disabled. This is expected on USB-CDC bridges (ft232, ch340, pl2303, cp210x) and on UART drivers not yet ported to the FC framework.\n",
			    err);
		return false;
	}

	/* Try for trigger == 1 first. ROUND_DOWN picks the largest
	 * supported level <= 1, which is exactly 1 if supported and an
	 * error otherwise; either way the read-back below catches the
	 * "we didn't get 1" case so the disable fallback runs.
	 */
	ctl.rx_trigger_bytes = 1;
	err = uart_set_fifo_control(port, &ctl, UART_FIFO_ROUND_DOWN);
	if (!err && uart_get_fifo_control(port, &verify) == 0 &&
	    verify.rx_trigger_bytes == 1) {
		netdev_info(sl->dev, "FIFO rx trigger = 1 byte\n");
		return true;
	}

	/* Fallback: disable the FIFO entirely. */
	ctl.flags &= ~UART_FIFO_CTRL_FLAG_ENABLE_FIFO;
	ctl.rx_trigger_bytes = 0;
	ctl.tx_trigger_bytes = 0;
	err = uart_set_fifo_control(port, &ctl, UART_FIFO_ROUND_EXACT);
	if (!err) {
		netdev_info(sl->dev,
			    "FIFO disabled (rx trigger of 1 not supported)\n");
		return true;
	}

	netdev_warn(sl->dev,
		    "could not program FIFO for sub-frame latency (trigger=1 and disable both unsupported, err %d); slave-publish role disabled\n",
		    err);
	return false;
}


/* TTY-transport adapters for the lin-dev UART byte-protocol layer.
 * These small shims translate from struct lin_uart's transport-
 * agnostic vtable to the TTY ldisc API stored in u->io_priv (set by
 * sllin_ldisc_open() to the underlying tty_struct *).
 */

static int sllin_io_write(struct lin_uart *u, const u8 *buf, int len)
{
	struct tty_struct *tty = u->io_priv;

	return tty->ops->write(tty, buf, len);
}

static int sllin_io_break_ctl(struct lin_uart *u, int state)
{
	struct tty_struct *tty = u->io_priv;

	return tty->ops->break_ctl(tty, state);
}

static int sllin_io_set_baud(struct lin_uart *u, u32 baud)
{
	struct tty_struct *tty = u->io_priv;

	return sltty_change_speed(tty, baud);
}

static void sllin_io_flush_buffer(struct lin_uart *u)
{
	struct tty_struct *tty = u->io_priv;

	if (tty->ops->flush_buffer)
		tty->ops->flush_buffer(tty);
}

static void sllin_io_tx_wakeup_arm(struct lin_uart *u)
{
	struct tty_struct *tty = u->io_priv;

	set_bit(TTY_DO_WRITE_WAKEUP, &tty->flags);
}

static void sllin_io_tx_wakeup_disarm(struct lin_uart *u)
{
	struct tty_struct *tty = u->io_priv;

	clear_bit(TTY_DO_WRITE_WAKEUP, &tty->flags);
}

/* Drain the TTY flip buffer in the LIN kthread's SCHED_FIFO context.
 * The budget caps the per-call work so the kthread cannot be made to
 * spend unbounded time draining a backlog; it also bounds the
 * worst-case cross-context wait on the buffer mutex against a
 * concurrent workqueue drainer. A LIN frame fits in well under 64
 * bytes, so 64 gives generous headroom for transient back-pressure
 * without overshooting the slot deadline.
 */
static void sllin_io_drain_rx(struct lin_uart *u)
{
	struct tty_struct *tty = u->io_priv;

	tty_port_drain_flip_buffer(tty->port, 64);
}

static tty_rx_token_t sllin_io_rx_token(struct lin_uart *u)
{
	struct tty_struct *tty = u->io_priv;

	return tty_port_rx_token(tty->port);
}

static const struct lin_uart_io_ops sllin_io_ops = {
	.write		= sllin_io_write,
	.break_ctl	= sllin_io_break_ctl,
	.set_baud	= sllin_io_set_baud,
	.flush_buffer	= sllin_io_flush_buffer,
	.tx_wakeup_arm	= sllin_io_tx_wakeup_arm,
	.tx_wakeup_disarm = sllin_io_tx_wakeup_disarm,
	.drain_rx	= sllin_io_drain_rx,
	.rx_token	= sllin_io_rx_token,
};

/* ------------------------------------------------------------------ */
/* TTY ldisc rx/wakeup wrappers - thin trampolines to lin-dev helpers  */
/* ------------------------------------------------------------------ */

static void sllin_write_wakeup(struct tty_struct *tty)
{
	struct sllin *sl = tty->disc_data;

	if (!sl || !netif_running(sl->dev))
		return;
	lin_uart_write_wakeup(&sl->u);
}

static void sllin_receive_buf(struct tty_struct *tty,
			      const u8 *cp, const u8 *fp, size_t count)
{
	struct sllin *sl = tty->disc_data;

	if (!sl || !netif_running(sl->dev))
		return;
	lin_uart_receive_buf(&sl->u, cp, fp, count);
}


/* ------------------------------------------------------------------ */
/* lin_dev_ops                                                         */
/* ------------------------------------------------------------------ */

static int sllin_op_master_start(struct lin_dev *ld)
{
	struct sllin *sl = netdev_priv(ld->dev);

	set_bit(LIN_SCHED_F_MASTER_RUNNING, &sl->s.flags);
	return 0;
}

static int sllin_op_master_stop(struct lin_dev *ld)
{
	struct sllin *sl = netdev_priv(ld->dev);
	unsigned long flags;

	spin_lock_irqsave(&sl->lock, flags);
	clear_bit(LIN_SCHED_F_MASTER_RUNNING, &sl->s.flags);
	lin_sched_stop_locked(&sl->s);

	/* If an in-flight header_send() is still parked waiting for the
	 * kthread, cancel it: the master claim is being released so the
	 * core will not accept any further results either way.
	 */
	if (test_and_clear_bit(LIN_SCHED_F_HDR_REQ, &sl->s.flags) ||
	    sl->s.hdr_status == -EINPROGRESS) {
		sl->s.hdr_status = -ECANCELED;
		set_bit(LIN_SCHED_F_HDR_DONE, &sl->s.flags);
	}
	spin_unlock_irqrestore(&sl->lock, flags);

	wake_up(&sl->kwt_wq);
	return 0;
}

static int sllin_op_set_response(struct lin_dev *ld, u8 lin_id,
				 const u8 *data, u8 len, bool enhanced)
{
	struct sllin *sl = netdev_priv(ld->dev);
	unsigned long flags;

	spin_lock_irqsave(&sl->lock, flags);
	lin_resp_set_locked(&sl->resp, lin_id, data, len, enhanced);
	spin_unlock_irqrestore(&sl->lock, flags);
	return 0;
}

static int sllin_op_clear_response(struct lin_dev *ld, u8 lin_id)
{
	struct sllin *sl = netdev_priv(ld->dev);
	unsigned long flags;

	spin_lock_irqsave(&sl->lock, flags);
	lin_resp_clear_locked(&sl->resp, lin_id);
	spin_unlock_irqrestore(&sl->lock, flags);
	return 0;
}

static int sllin_op_schedule_load(struct lin_dev *ld,
				  const struct lin_schedule *sched)
{
	struct sllin *sl = netdev_priv(ld->dev);
	struct lin_schedule *copy, *old;
	unsigned long flags;

	copy = kmemdup(sched, struct_size(sched, entry, sched->entry_count),
		       GFP_KERNEL);
	if (!copy)
		return -ENOMEM;

	spin_lock_irqsave(&sl->lock, flags);
	old = lin_sched_load_locked(&sl->s, copy);
	spin_unlock_irqrestore(&sl->lock, flags);

	kfree(old);
	return 0;
}

static int sllin_op_schedule_delete(struct lin_dev *ld, u8 handle)
{
	struct sllin *sl = netdev_priv(ld->dev);
	struct lin_schedule *old;
	unsigned long flags;

	/* Refuse to delete the schedule the engine is currently running.
	 * Dropping it would leave LIN_SCHED_F_RUNNING set with
	 * sched[active] = NULL — the kthread defensively NULL-checks and
	 * silently stops emitting, but the engine reports as running.
	 * Force the caller to LIN_RAW_SCHEDULE_STOP first.
	 */
	spin_lock_irqsave(&sl->lock, flags);
	if (test_bit(LIN_SCHED_F_RUNNING, &sl->s.flags) &&
	    sl->s.active == handle) {
		spin_unlock_irqrestore(&sl->lock, flags);
		return -EBUSY;
	}
	old = lin_sched_take_locked(&sl->s, handle);
	spin_unlock_irqrestore(&sl->lock, flags);

	kfree(old);
	return 0;
}

static int sllin_op_schedule_activate(struct lin_dev *ld, u8 handle)
{
	struct sllin *sl = netdev_priv(ld->dev);
	unsigned long flags;
	unsigned long timeout_j;

	spin_lock_irqsave(&sl->lock, flags);
	if (!test_bit(LIN_SCHED_F_RUNNING, &sl->s.flags)) {
		sl->s.active = handle;
		sl->s.slot = 0;
		sl->s.next_slot = ktime_get();
		set_bit(LIN_SCHED_F_RUNNING, &sl->s.flags);
		spin_unlock_irqrestore(&sl->lock, flags);
		wake_up(&sl->kwt_wq);
		return 0;
	}

	reinit_completion(&sl->s.activate_done);
	sl->s.activate_to = handle;
	sl->s.activate_req = true;
	timeout_j = usecs_to_jiffies(LIN_RAW_SCHEDULE_SLOT_MAX_US) +
		    msecs_to_jiffies(100);
	spin_unlock_irqrestore(&sl->lock, flags);

	wake_up(&sl->kwt_wq);

	if (!wait_for_completion_timeout(&sl->s.activate_done, timeout_j)) {
		int ret = -ETIMEDOUT;

		spin_lock_irqsave(&sl->lock, flags);
		if (!sl->s.activate_req && sl->s.active == handle)
			ret = 0;
		else
			sl->s.activate_req = false;
		spin_unlock_irqrestore(&sl->lock, flags);
		return ret;
	}
	return 0;
}

static int sllin_op_schedule_stop(struct lin_dev *ld)
{
	struct sllin *sl = netdev_priv(ld->dev);
	unsigned long flags;

	spin_lock_irqsave(&sl->lock, flags);
	lin_sched_stop_locked(&sl->s);
	spin_unlock_irqrestore(&sl->lock, flags);

	wake_up(&sl->kwt_wq);
	return 0;
}

static int sllin_op_header_send(struct lin_dev *ld, u8 lin_id,
				const u8 *data, u8 len, bool enhanced)
{
	struct sllin *sl = netdev_priv(ld->dev);
	unsigned long flags;
	unsigned long timeout_j;
	int ret;

	spin_lock_irqsave(&sl->lock, flags);
	if (sl->s.hdr_status == -EINPROGRESS) {
		spin_unlock_irqrestore(&sl->lock, flags);
		return -EBUSY;
	}
	sl->s.hdr_id = lin_id & LIN_ID_MASK;
	if (len) {
		memcpy(sl->s.hdr_data, data, len);
		sl->s.hdr_len = len;
	} else {
		sl->s.hdr_len = 0;
	}
	sl->s.hdr_enhanced = enhanced;
	sl->s.hdr_status = -EINPROGRESS;
	clear_bit(LIN_SCHED_F_HDR_DONE, &sl->s.flags);
	set_bit(LIN_SCHED_F_HDR_REQ, &sl->s.flags);
	spin_unlock_irqrestore(&sl->lock, flags);

	wake_up(&sl->kwt_wq);

	timeout_j = usecs_to_jiffies(LIN_RAW_SCHEDULE_SLOT_MAX_US) +
		    msecs_to_jiffies(100);
	if (!wait_event_timeout(sl->kwt_wq,
				test_bit(LIN_SCHED_F_HDR_DONE, &sl->s.flags),
				timeout_j)) {
		spin_lock_irqsave(&sl->lock, flags);
		sl->s.hdr_status = -ETIMEDOUT;
		spin_unlock_irqrestore(&sl->lock, flags);
	}

	spin_lock_irqsave(&sl->lock, flags);
	ret = sl->s.hdr_status;
	if (ret == -EINPROGRESS)
		ret = -EIO;
	sl->s.hdr_status = 0;
	clear_bit(LIN_SCHED_F_HDR_DONE, &sl->s.flags);
	spin_unlock_irqrestore(&sl->lock, flags);
	return ret < 0 ? ret : 0;
}

/* Wakeup pulse: drive the line dominant for ~5 ms. Per LIN 2.1+ the
 * pulse must be 250..5000 us; we sit at the upper end so even slow
 * receivers latch it. break_ctl is a required ldisc_open precondition
 * (see sllin_ldisc_open) so it is always available here.
 */
static int sllin_op_wakeup_send(struct lin_dev *ld)
{
	struct sllin *sl = netdev_priv(ld->dev);
	struct tty_struct *tty = sl->tty;
	int ret;

	ret = tty->ops->break_ctl(tty, -1);
	if (ret) {
		netdev_warn(sl->dev, "wakeup break_ctl assert failed: %d\n",
			    ret);
		return ret;
	}
	usleep_range(2000, 5000);
	ret = tty->ops->break_ctl(tty, 0);
	if (ret) {
		netdev_warn(sl->dev,
			    "wakeup break_ctl release failed (%d); bus may be wedged low\n",
			    ret);
		return ret;
	}
	usleep_range(100, 200);

	/* Bus-observed event (no owner tags), delivered to wakeup
	 * subscribers via the standard rx path.
	 */
	lin_uart_emit_bus_event(&sl->u, LIN_ID_NONE, LIN_F_WAKEUP, 0);
	return 0;
}

static int sllin_op_set_bitrate(struct lin_dev *ld, u32 bitrate)
{
	struct sllin *sl = netdev_priv(ld->dev);

	if (bitrate < 1000 || bitrate > 20000)
		return -EINVAL;

	return lin_uart_set_baud(&sl->u, bitrate);
}

static const struct lin_dev_ops sllin_lin_ops = {
	.master_start		= sllin_op_master_start,
	.master_stop		= sllin_op_master_stop,
	.set_response		= sllin_op_set_response,
	.clear_response		= sllin_op_clear_response,
	.schedule_load		= sllin_op_schedule_load,
	.schedule_delete	= sllin_op_schedule_delete,
	.schedule_activate	= sllin_op_schedule_activate,
	.schedule_stop		= sllin_op_schedule_stop,
	.header_send		= sllin_op_header_send,
	.wakeup_send		= sllin_op_wakeup_send,
	.set_bitrate		= sllin_op_set_bitrate,
};

/* ------------------------------------------------------------------ */
/* netdev ops                                                          */
/* ------------------------------------------------------------------ */

static int sllin_netdev_open(struct net_device *dev)
{
	netif_carrier_on(dev);
	netif_start_queue(dev);
	return 0;
}

static int sllin_netdev_stop(struct net_device *dev)
{
	netif_stop_queue(dev);
	netif_carrier_off(dev);
	return 0;
}

static netdev_tx_t sllin_netdev_xmit(struct sk_buff *skb,
				     struct net_device *dev)
{
	/* LIN frames originate in the schedule engine and publisher
	 * responses, not from dev_queue_xmit. Drop anything that
	 * reaches here (e.g. AF_PACKET injection).
	 */
	kfree_skb(skb);
	dev->stats.tx_dropped++;
	return NETDEV_TX_OK;
}

static const struct net_device_ops sllin_netdev_ops = {
	.ndo_open	= sllin_netdev_open,
	.ndo_stop	= sllin_netdev_stop,
	.ndo_start_xmit	= sllin_netdev_xmit,
};

/* ------------------------------------------------------------------ */
/* TTY ldisc open / close                                              */
/* ------------------------------------------------------------------ */

/* Pull the underlying TTY's current baud from termios so the link's
 * initial bitrate matches whatever the operator configured (`stty
 * 19200 < /dev/ttyS0` before ldattach is the conventional pattern).
 * Returns LIN_DEFAULT_BITRATE when the TTY did not advertise a baud.
 */
static u32 sllin_tty_baud(struct tty_struct *tty)
{
	speed_t s;

	down_read(&tty->termios_rwsem);
	s = tty_termios_baud_rate(&tty->termios);
	up_read(&tty->termios_rwsem);

	if (s < 1000 || s > 20000)
		return LIN_DEFAULT_BITRATE;
	return s;
}

/* Allocate and register the sllin netdev that backs @tty. The naming
 * is left to the kernel ("sllin%d"); the user can rename via `ip link
 * set sllin0 name foo` afterwards.
 *
 * No rtnl_lock here: the TTY layer serialises ldisc open/close per-tty,
 * there is no global driver state to protect (each link is its own
 * struct sllin), and lin_register_netdev() takes rtnl internally — a
 * recursive grab here would deadlock.
 */
static int sllin_ldisc_open(struct tty_struct *tty)
{
	struct net_device *dev;
	struct lin_dev *ld;
	struct sllin *sl;
	int err;

	if (!capable(CAP_NET_ADMIN))
		return -EPERM;
	if (!tty->ops->write)
		return -EOPNOTSUPP;
	/* break_ctl is the only break-pulse path now; fail attach
	 * cleanly on UARTs that do not expose it, rather than silently
	 * falling back to a baud-switch workaround that the original
	 * out-of-tree sllin used.
	 */
	if (!tty->ops->break_ctl)
		return -EOPNOTSUPP;

	/* tty_set_ldisc closes the prior ldisc (which nulls disc_data)
	 * before invoking our open, so no "already attached" check is
	 * needed here.
	 */

	dev = alloc_lindev(sizeof(struct sllin), &sllin_lin_ops);
	if (!dev)
		return -ENOMEM;

	dev->netdev_ops = &sllin_netdev_ops;
	strscpy(dev->name, "sllin%d", IFNAMSIZ);

	ld = lin_get_ml_priv(dev);
	ld->caps = LIN_CAP_SPORADIC | LIN_CAP_EVENT | LIN_CAP_DIAG |
		   LIN_CAP_CHK_ENH | LIN_CAP_WAKEUP;
	/* LIN_CAP_PUB_SLAVE is added below after the FIFO probe succeeds —
	 * the cap reflects the transport's actual ability to meet the
	 * spec's header-RX → response-TX window, not just a static driver
	 * feature.
	 */
	ld->bitrate = sllin_tty_baud(tty);

	sl = netdev_priv(dev);
	sl->dev = dev;
	sl->tty = tty;
	spin_lock_init(&sl->lock);
	lin_resp_table_init(&sl->resp);
	init_waitqueue_head(&sl->kwt_wq);
	lin_sched_init(&sl->s);

	/* Initialise the embedded UART byte-protocol state with the
	 * TTY-transport io_ops vtable; lin_uart_init() also wires up
	 * the per-frame RX timeout hrtimer internally.
	 */
	lin_uart_init(&sl->u, dev, &sl->resp, &sl->s, &sl->lock, &sl->kwt_wq,
		      &sllin_io_ops, tty, ld->bitrate);

	if (sllin_setup_fifo(sl))
		ld->caps |= LIN_CAP_PUB_SLAVE;

	/* Program the UART to the LIN baud before any LIN op can land —
	 * otherwise a sockopt racing with the kthread's initial
	 * change_speed() would operate against the operator's pre-attach
	 * baud (likely 115200 from a debug console).
	 */
	sltty_change_speed(tty, sl->u.baud);

	tty->disc_data = sl;
	tty->receive_room = LIN_UART_BUFF_LEN * 40;	/* no flow control */

	err = lin_register_netdev(dev);
	if (err)
		goto err_free;

	/* Opt in to direct-RX wakes so the kthread can drain in its own
	 * SCHED_FIFO context. Must precede kthread_run so the kthread
	 * sees a stable rx_token cursor on its very first iteration.
	 */
	tty_port_enable_direct_rx(tty->port);

	sl->kwthread = kthread_run(lin_sched_kthread_fn, &sl->u, "sllin/%s",
				   dev->name);
	if (IS_ERR(sl->kwthread)) {
		err = PTR_ERR(sl->kwthread);
		sl->kwthread = NULL;
		tty_port_disable_direct_rx(tty->port);
		lin_unregister_netdev(dev);
		goto err_free;
	}

	return 0;

err_free:
	tty->disc_data = NULL;
	free_lindev(dev);
	return err;
}

static void sllin_ldisc_close(struct tty_struct *tty)
{
	struct sllin *sl = tty->disc_data;

	if (!sl)
		return;

	/* Teardown order is deliberately NOT the inverse of ldisc_open.
	 *
	 * Clear tty->disc_data first so further receive_buf /
	 * write_wakeup callbacks bail at sllin_receive_buf() /
	 * sllin_write_wakeup().
	 *
	 * Then call lin_unregister_netdev() — this fires
	 * NETDEV_GOING_DOWN, which is what the LIN core's policy_lock +
	 * going_down gate uses to fence in-flight sockopts off the
	 * driver ops, and which then force-releases all bound LIN
	 * sockets. Only after that point are we guaranteed no further
	 * lin_dev_ops calls can land.
	 *
	 * The kthread must remain alive across lin_unregister_netdev():
	 * the per-socket force-release path calls into ops like
	 * sllin_op_master_stop(), which wake sl->kwt_wq to unblock a
	 * parked header_send waiter. Stopping the kthread first would
	 * leave such waiters parked for their full wait_event_timeout
	 * instead of waking with -ECANCELED.
	 *
	 * sl->tty must also stay valid across this window: ops like
	 * @wakeup_send and @set_bitrate dereference it to drive
	 * break_ctl / change_speed, and the underlying tty_struct
	 * outlives ldisc_close (the TTY layer owns it). Clearing
	 * sl->tty here opened a NULL-deref race against those ops;
	 * leave it set and let free_lindev() reap the storage.
	 *
	 * Once the going_down fence is in place we kthread_stop(),
	 * tear down the schedule storage the kthread was reading, and
	 * free the netdev.
	 */
	tty->disc_data = NULL;

	lin_unregister_netdev(sl->dev);

	/* Stop the direct-RX wake stream before kthread_stop. The
	 * kthread's wait predicate stays valid (kthread_should_stop
	 * trips first), but disabling here is the symmetric inverse of
	 * the open-path enable and means no late wakes touch state the
	 * kthread is in the process of releasing.
	 */
	tty_port_disable_direct_rx(tty->port);

	if (sl->kwthread) {
		kthread_stop(sl->kwthread);
		sl->kwthread = NULL;
	}

	lin_sched_destroy(&sl->s);
	free_lindev(sl->dev);
}

static int sllin_ldisc_ioctl(struct tty_struct *tty, unsigned int cmd,
			     unsigned long arg)
{
	struct sllin *sl = tty->disc_data;
	unsigned int n;

	if (!sl)
		return -EINVAL;

	switch (cmd) {
	case SIOCGIFNAME:
		n = strlen(sl->dev->name) + 1;
		if (copy_to_user((void __user *)arg, sl->dev->name, n))
			return -EFAULT;
		return 0;
	default:
		return tty_mode_ioctl(tty, cmd, arg);
	}
}

static struct tty_ldisc_ops sllin_ldisc = {
	.owner		= THIS_MODULE,
	.num		= N_LIN,
	.name		= "sllin",
	.open		= sllin_ldisc_open,
	.close		= sllin_ldisc_close,
	.ioctl		= sllin_ldisc_ioctl,
	.receive_buf	= sllin_receive_buf,
	.write_wakeup	= sllin_write_wakeup,
};

/* ------------------------------------------------------------------ */
/* Module init / exit                                                  */
/* ------------------------------------------------------------------ */

static int __init sllin_init(void)
{
	int err;

	BUILD_BUG_ON(N_LIN >= NR_LDISCS);
	err = tty_register_ldisc(&sllin_ldisc);
	if (err)
		pr_err("can't register line discipline (err %d)\n", err);
	return err;
}

static void __exit sllin_exit(void)
{
	tty_unregister_ldisc(&sllin_ldisc);
}

module_init(sllin_init);
module_exit(sllin_exit);

MODULE_DESCRIPTION("Serial line LIN interface (TTY line discipline)");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Kyle Bader <kyle.bader94@gmail.com>");
MODULE_ALIAS_LDISC(N_LIN);
