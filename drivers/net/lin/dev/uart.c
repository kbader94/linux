// SPDX-License-Identifier: GPL-2.0
/*
 * drivers/net/lin/dev/uart.c - LIN-over-UART byte-protocol layer.
 *
 * Implements the LIN-over-UART wire format common to sllin (TTY ldisc
 * frontend) and sdlin (serdev frontend): break_ctl timing, byte-level
 * state machine, checksum + PID parity helpers, master/slave RX
 * parsers, TX buffering with re-entry barrier, FIFO Control framework
 * probe and LIN_CAP_PUB_SLAVE eligibility check.
 *
 * Built only when CONFIG_LIN_DEV_HELPERS_UART=y (auto-selected by
 * sllin and sdlin).
 *
 * Author: Kyle Bader <kyle.bader94@gmail.com>
 * Copyright (c) 2026 Kyle Bader
 */

#include <linux/delay.h>
#include <linux/export.h>
#include <linux/hrtimer.h>
#include <linux/netdevice.h>
#include <linux/skbuff.h>
#include <linux/spinlock.h>
#include <linux/string.h>

#include <linux/lin.h>
#include <linux/lin/dev.h>
#include <linux/lin/drv.h>
#include <linux/lin/uart.h>

#include <uapi/linux/lin/error.h>

/* Number of UART bit times per data character (8N1 = 1 start + 8 data
 * + 1 stop). Used for the per-frame RX timeout sizing below.
 */
#define LIN_UART_SAMPLES_PER_CHAR	10

/* Per-frame RX timeout: 24 character times at the current baud.
 *
 * Rationale: the longest LIN frame the master schedules is
 * break + sync + PID + 8 data + checksum = 12 character times, plus
 * inter-byte response space (per spec, the slave has up to 1.4 x
 * nominal frame time to answer). Twenty-four character times is the
 * round trip a generous slave plus a quiet bus needs; below that we
 * report LIN_ERR_NO_RESPONSE.
 */
#define LIN_UART_CHARS_TO_TIMEOUT	24

/* ----------------------------------------------------------------
 * PID parity table
 * ----------------------------------------------------------------
 *
 * Maps a 6-bit ID to the 2-bit protection field (bits 6..7); the PID
 * on the wire is the OR of the two.
 *
 * Bit generation per LIN spec:
 *   P0 = ID0 ^ ID1 ^ ID2 ^ ID4         -> table bit 6
 *   P1 = ~(ID1 ^ ID3 ^ ID4 ^ ID5)      -> table bit 7
 *
 * Precomputed for the full 64 entries because table lookup is cheaper
 * than recomputing on every header emission, and both the TX header
 * build path and the RX validation path consult it.
 */
static const u8 lin_uart_id_parity_table[64] = {
	0x80, 0xc0, 0x40, 0x00, 0xc0, 0x80, 0x00, 0x40,
	0x00, 0x40, 0xc0, 0x80, 0x40, 0x00, 0x80, 0xc0,
	0x40, 0x00, 0x80, 0xc0, 0x00, 0x40, 0xc0, 0x80,
	0xc0, 0x80, 0x00, 0x40, 0x80, 0xc0, 0x40, 0x00,
	0x00, 0x40, 0xc0, 0x80, 0x40, 0x00, 0x80, 0xc0,
	0x80, 0xc0, 0x40, 0x00, 0xc0, 0x80, 0x00, 0x40,
	0xc0, 0x80, 0x00, 0x40, 0x80, 0xc0, 0x40, 0x00,
	0x40, 0x00, 0x80, 0xc0, 0x00, 0x40, 0xc0, 0x80,
};

u8 lin_uart_pid(u8 lin_id)
{
	lin_id &= LIN_ID_MASK;
	return lin_id | lin_uart_id_parity_table[lin_id];
}
EXPORT_SYMBOL_GPL(lin_uart_pid);

u8 lin_uart_checksum(const u8 *buf, int end, bool enhanced)
{
	unsigned int csum = 0;
	int i;

	i = enhanced ? LIN_UART_BUFF_ID : LIN_UART_BUFF_DATA;
	for (; i < end; i++) {
		csum += buf[i];
		if (csum > 0xff)
			csum -= 0xff;
	}
	return ~csum & 0xff;
}
EXPORT_SYMBOL_GPL(lin_uart_checksum);

/* ----------------------------------------------------------------
 * State initialisation and reset
 * ----------------------------------------------------------------
 */

/* Forward decl of the per-frame RX timeout handler so lin_uart_init()
 * can wire the embedded hrtimer at init time without exposing the
 * handler to frontends.
 */
static enum hrtimer_restart lin_uart_rx_timeout_handler(struct hrtimer *t);

/* Compute the per-frame RX bound at @baud (24 character times). Shared
 * between init and the runtime set_baud path so the constants only
 * live in one place.
 */
static ktime_t lin_uart_rx_timeout_for_baud(u32 baud)
{
	return ns_to_ktime((NSEC_PER_SEC / baud) *
			   LIN_UART_SAMPLES_PER_CHAR *
			   LIN_UART_CHARS_TO_TIMEOUT);
}

void lin_uart_init(struct lin_uart *u, struct net_device *dev,
		   struct lin_resp_table *resp, struct lin_sched *sched,
		   spinlock_t *lock, wait_queue_head_t *wq,
		   const struct lin_uart_io_ops *io, void *io_priv,
		   u32 baud)
{
	u->dev = dev;
	u->resp = resp;
	u->sched = sched;
	u->lock = lock;
	u->wq = wq;
	u->io = io;
	u->io_priv = io_priv;
	u->baud = baud;
	u->state = LIN_UART_IDLE;
	u->cur_id = LIN_ID_NONE;
	u->rx_timer_timeout = lin_uart_rx_timeout_for_baud(baud);
	hrtimer_setup(&u->rx_timer, lin_uart_rx_timeout_handler,
		      CLOCK_MONOTONIC, HRTIMER_MODE_REL);
}
EXPORT_SYMBOL_GPL(lin_uart_init);

int lin_uart_set_baud(struct lin_uart *u, u32 baud)
{
	unsigned long flags;

	if (!baud)
		return -EINVAL;

	/* @baud and @rx_timer_timeout are read by the kthread; the
	 * ktime_t in particular is not safely torn-read on 32-bit, so
	 * publish both atomically against the kthread under the lock.
	 */
	spin_lock_irqsave(u->lock, flags);
	u->baud = baud;
	u->rx_timer_timeout = lin_uart_rx_timeout_for_baud(baud);
	spin_unlock_irqrestore(u->lock, flags);

	/* Reprogram the transport. May sleep (e.g. tty termios update),
	 * so run it outside the lock. Drivers without a runtime-changeable
	 * baud (fixed-rate hardware) leave @set_baud NULL.
	 */
	if (u->io->set_baud)
		return u->io->set_baud(u, baud);
	return 0;
}
EXPORT_SYMBOL_GPL(lin_uart_set_baud);

void lin_uart_reset_buffs(struct lin_uart *u)
{
	u->rx_cnt = 0;
	u->rx_expect = 0;
	u->rx_lim = LIN_UART_BUFF_LEN;
	u->tx_cnt = 0;
	u->tx_lim = 0;
	u->id_to_send = false;
	u->data_to_send = false;
	u->resp_len_known = false;
	u->header_received = false;
	u->rx_len_unknown = false;
	u->cur_id = LIN_ID_NONE;
	u->cur_we_publish = false;
	u->cur_master_emit = false;
	u->cur_enhanced = false;
}
EXPORT_SYMBOL_GPL(lin_uart_reset_buffs);

/* ----------------------------------------------------------------
 * Frame assembly and validation
 * ----------------------------------------------------------------
 */

int lin_uart_setup_msg(struct lin_uart *u, bool response_only, bool enhanced,
		       u8 id, const u8 *data, u8 len)
{
	if (id > LIN_ID_MASK || len > LIN_MAX_DLEN)
		return -EINVAL;

	if (!response_only) {
		u->rx_cnt = 0;
		u->tx_cnt = 0;
		u->rx_expect = 0;
		u->rx_lim = LIN_UART_BUFF_LEN;
	}

	u->tx_buff[LIN_UART_BUFF_BREAK] = 0;
	u->tx_buff[LIN_UART_BUFF_SYNC]  = 0x55;
	u->tx_buff[LIN_UART_BUFF_ID]    = lin_uart_pid(id);
	u->tx_lim = LIN_UART_BUFF_DATA;

	if (data && len) {
		u->tx_lim += len;
		memcpy(u->tx_buff + LIN_UART_BUFF_DATA, data, len);
		u->tx_buff[u->tx_lim] = lin_uart_checksum(u->tx_buff,
							  u->tx_lim, enhanced);
		u->tx_lim++;
	}
	if (len)
		u->rx_lim = LIN_UART_BUFF_DATA + len + 1;

	u->cur_enhanced = enhanced;
	return 0;
}
EXPORT_SYMBOL_GPL(lin_uart_setup_msg);

int lin_uart_rx_validate(struct lin_uart *u)
{
	u8 wire = u->rx_buff[u->rx_cnt - 1];
	u8 cs;

	cs = lin_uart_checksum(u->rx_buff, u->rx_cnt - 1, u->cur_enhanced);
	if (cs == wire)
		return 0;
	cs = lin_uart_checksum(u->rx_buff, u->rx_cnt - 1, !u->cur_enhanced);
	if (cs == wire) {
		u->cur_enhanced = !u->cur_enhanced;
		return 0;
	}
	return -EBADMSG;
}
EXPORT_SYMBOL_GPL(lin_uart_rx_validate);

/* ----------------------------------------------------------------
 * Frame upcalls (rx-side delivery)
 * ----------------------------------------------------------------
 */

void lin_uart_emit_bus_event(struct lin_uart *u, u8 lin_id, u32 flags,
			     u32 err_mask)
{
	struct lin_frame f = {
		.lin_id   = lin_id,
		.flags    = flags,
		.err_mask = err_mask,
	};
	struct sk_buff *skb;

	skb = alloc_lin_skb(u->dev, &f);
	if (skb)
		netif_rx(skb);
}
EXPORT_SYMBOL_GPL(lin_uart_emit_bus_event);

void lin_uart_deliver_rx(struct lin_uart *u)
{
	unsigned int emit = 0;
	struct lin_frame f;
	int len = u->rx_cnt - LIN_UART_BUFF_DATA - 1;

	memset(&f, 0, sizeof(f));
	f.lin_id = u->rx_buff[LIN_UART_BUFF_ID] & LIN_ID_MASK;
	f.len    = len;
	if (u->cur_enhanced)
		f.flags |= LIN_F_CHK_ENH;
	memcpy(f.data, u->rx_buff + LIN_UART_BUFF_DATA, len);

	if (u->cur_master_emit)
		emit |= LIN_EMIT_MASTER;
	if (u->cur_we_publish)
		emit |= LIN_EMIT_PUBLISHER;

	u->dev->stats.rx_packets++;
	u->dev->stats.rx_bytes += f.len;

	if (emit) {
		lin_loopback_rx(u->dev, &f, emit, u->cur_id);
	} else {
		struct sk_buff *skb = alloc_lin_skb(u->dev, &f);

		if (skb)
			netif_rx(skb);
	}
}
EXPORT_SYMBOL_GPL(lin_uart_deliver_rx);

/* ----------------------------------------------------------------
 * TX path
 * ----------------------------------------------------------------
 *
 * Two entry points feed the same loop:
 *
 *   lin_uart_send_tx_buff()   - called by the schedule engine /
 *                               state machine to push a freshly
 *                               assembled frame onto the bus.
 *   lin_uart_write_wakeup()   - called by the frontend's
 *                               transport-level wakeup hook (TTY
 *                               write_wakeup / serdev equivalent)
 *                               to drain whatever did not fit in
 *                               the initial push.
 *
 * The LIN_UART_F_TXBUFF_RQ / LIN_UART_F_TXBUFF_INPR atomic bit pair
 * guards the buffer cursor (@tx_cnt / @tx_lim) against reentrance:
 * if the wakeup fires while send_tx_buff is mid-push, the wakeup
 * sets RQ and bails, send_tx_buff observes RQ on its next loop and
 * retries with the updated cursor. The pattern is the same as the
 * out-of-tree sllin's original SLF_TXBUFF_RQ/INPR design.
 */

int lin_uart_send_tx_buff(struct lin_uart *u)
{
	int remains;
	int res;

	set_bit(LIN_UART_F_TXBUFF_RQ, &u->flags);
	do {
		if (unlikely(test_and_set_bit(LIN_UART_F_TXBUFF_INPR,
					      &u->flags)))
			return 0;	/* re-entry; the outer loop picks
					 * up where we left off.
					 */

		clear_bit(LIN_UART_F_TXBUFF_RQ, &u->flags);
		smp_mb__after_atomic();

		remains = u->tx_lim - u->tx_cnt;

		res = u->io->write(u, u->tx_buff + u->tx_cnt, remains);
		if (res < 0)
			goto err;

		remains -= res;
		u->tx_cnt += res;

		if (remains > 0) {
			if (u->io->tx_wakeup_arm)
				u->io->tx_wakeup_arm(u);
			res = u->io->write(u, u->tx_buff + u->tx_cnt, remains);
			if (res < 0) {
				if (u->io->tx_wakeup_disarm)
					u->io->tx_wakeup_disarm(u);
				goto err;
			}
			remains -= res;
			u->tx_cnt += res;
		}

		clear_bit(LIN_UART_F_TXBUFF_INPR, &u->flags);
		smp_mb__after_atomic();
	} while (unlikely(test_bit(LIN_UART_F_TXBUFF_RQ, &u->flags)));

	return 0;

err:
	clear_bit(LIN_UART_F_TXBUFF_INPR, &u->flags);
	return -EIO;
}
EXPORT_SYMBOL_GPL(lin_uart_send_tx_buff);

int lin_uart_send_break(struct lin_uart *u)
{
	unsigned long min_us, max_us;
	int ret;

	u->rx_cnt = LIN_UART_BUFF_BREAK;
	u->rx_expect = LIN_UART_BUFF_BREAK + 1;
	u->state = LIN_UART_BREAK_SENT;

	ret = u->io->break_ctl(u, -1);
	if (ret) {
		netdev_warn(u->dev,
			    "break_ctl assert failed (%d); LIN header not driven\n",
			    ret);
		return ret;
	}

	min_us = (1000000UL * LIN_UART_SAMPLES_PER_CHAR) / u->baud;
	max_us = min_us + 50;
	usleep_range(min_us, max_us);

	ret = u->io->break_ctl(u, 0);
	if (ret) {
		netdev_warn(u->dev,
			    "break_ctl release failed (%d); bus may be wedged low\n",
			    ret);
		return ret;
	}

	min_us = 1000000UL / u->baud;
	max_us = min_us + 30;
	usleep_range(min_us, max_us);

	if (u->io->flush_buffer)
		u->io->flush_buffer(u);

	u->tx_cnt = LIN_UART_BUFF_SYNC;

	set_bit(LIN_UART_F_RXEVENT, &u->flags);
	wake_up(u->wq);
	return 0;
}
EXPORT_SYMBOL_GPL(lin_uart_send_break);

void lin_uart_write_wakeup(struct lin_uart *u)
{
	int actual = 0;
	int remains;

	set_bit(LIN_UART_F_TXBUFF_RQ, &u->flags);
	do {
		if (unlikely(test_and_set_bit(LIN_UART_F_TXBUFF_INPR,
					      &u->flags)))
			return;

		clear_bit(LIN_UART_F_TXBUFF_RQ, &u->flags);
		smp_mb__after_atomic();

		remains = u->tx_lim - u->tx_cnt;

		if (remains > 0) {
			actual = u->io->write(u, u->tx_buff + u->tx_cnt,
					      remains);
			u->tx_cnt += actual;
			remains -= actual;
		}

		clear_bit(LIN_UART_F_TXBUFF_INPR, &u->flags);
		smp_mb__after_atomic();
	} while (unlikely(test_bit(LIN_UART_F_TXBUFF_RQ, &u->flags)));

	if (remains > 0 && actual >= 0)
		return;	/* called again when the transport frees room */

	if (u->io->tx_wakeup_disarm)
		u->io->tx_wakeup_disarm(u);
	set_bit(LIN_UART_F_TXEVENT, &u->flags);
	wake_up(u->wq);
}
EXPORT_SYMBOL_GPL(lin_uart_write_wakeup);

/* ----------------------------------------------------------------
 * RX path
 * ----------------------------------------------------------------
 */

/* Validate the assembled frame and deliver it (or report a
 * checksum-class error), then reset the rx cursors for the next
 * header. Caller holds @u->lock.
 *
 * Idempotent: gated on @header_received, which the helper clears on
 * delivery. Three call sites — the slave parser's break-cancel
 * branch, the parser's frame-complete branch, and the frontend's
 * timeout handler — can all race to "finish" the same in-flight
 * frame; the gate guarantees a single delivery and a single reset.
 * The previous design called this from softirq (the rx-timer handler
 * called it directly without the lock), which raced the parser's
 * unlocked byte appends in lin_uart_slave_receive_buf().
 */
void lin_uart_slave_finish_rx(struct lin_uart *u)
{
	if (!u->header_received)
		return;

	if (lin_uart_rx_validate(u) < 0) {
		u8 id = u->rx_buff[LIN_UART_BUFF_ID] & LIN_ID_MASK;

		u->dev->stats.rx_crc_errors++;
		u->dev->stats.rx_errors++;
		lin_uart_emit_bus_event(u, id, LIN_F_ERR, LIN_ERR_CHECKSUM);
	} else {
		lin_uart_deliver_rx(u);
	}

	u->rx_cnt = 0;
	u->rx_expect = LIN_UART_BUFF_ID + 1;
	u->rx_len_unknown = false;
	u->header_received = false;
}
EXPORT_SYMBOL_GPL(lin_uart_slave_finish_rx);

/* Master-side rx parser: the master has driven the header, then
 * expects the response (its own break/sync/PID/data/checksum bytes
 * echoed by the UART loop-back, then the slave's response bytes).
 *
 * The break is driven by break_ctl, not a UART tx, so it does not
 * echo back as a UART character. Synthesise a leading 0x00 in
 * rx_buff when the first byte off the wire is the sync (0x55),
 * keeping the buffer layout uniform with the response-bearing slots
 * that follow.
 */
static void lin_uart_master_receive_buf(struct lin_uart *u, const u8 *cp,
					const u8 *fp, size_t count)
{
	while (count--) {
		if (fp && *fp++) {
			/* Anything but the leading break is a real
			 * framing error; flag and bail.
			 */
			if (u->rx_cnt > LIN_UART_BUFF_BREAK) {
				set_bit(LIN_UART_F_ERROR, &u->flags);
				wake_up(u->wq);
				return;
			}
		}

		if (u->rx_cnt == LIN_UART_BUFF_BREAK && *cp == 0x55)
			u->rx_buff[u->rx_cnt++] = 0x00;

		if (u->rx_cnt < LIN_UART_BUFF_LEN)
			u->rx_buff[u->rx_cnt++] = *cp++;
	}

	if (u->rx_cnt >= u->rx_expect) {
		set_bit(LIN_UART_F_RXEVENT, &u->flags);
		wake_up(u->wq);
	}
}

/* Slave-side rx parser: receives a header (break/sync/PID) from an
 * external master, looks up the registered response, and (if
 * present) pushes the response bytes onto the wire. Headers for IDs
 * we do not publish drift past us as bus-observed traffic.
 *
 * Length disambiguation: LIN 1.x carries no DLC on the header, so a
 * slave that has no entry for the incoming PID does not know how
 * many bytes follow. We grant up to LIN_MAX_DLEN bytes plus checksum
 * and arm the rx timer; when either bound trips we deliver whatever
 * we collected.
 */
static void lin_uart_slave_receive_buf(struct lin_uart *u, const u8 *cp,
				       const u8 *fp, size_t count)
{
	struct lin_resp_entry snap;
	u8 lin_id;

	while (count--) {
		if (fp && *fp++) {
			/* If we were mid-frame with an unknown length,
			 * the leading break of the next header tells us
			 * the previous frame is done; deliver it. Cancel
			 * the rx-timer outside @u->lock — the cancel can
			 * wait for an in-flight handler — then take the
			 * lock so the deliver races safely against the
			 * frontend's parallel timeout handler.
			 */
			if (u->rx_len_unknown && u->rx_cnt >= LIN_UART_BUFF_ID) {
				unsigned long flags;

				hrtimer_cancel(&u->rx_timer);
				spin_lock_irqsave(u->lock, flags);
				lin_uart_slave_finish_rx(u);
				spin_unlock_irqrestore(u->lock, flags);
				set_bit(LIN_UART_F_RXEVENT, &u->flags);
				wake_up(u->wq);
			}

			u->rx_cnt = 0;
			u->rx_expect = LIN_UART_BUFF_ID + 1;
			u->rx_len_unknown = false;
			u->header_received = false;
		}

		if (u->rx_cnt < LIN_UART_BUFF_LEN) {
			if (u->rx_cnt == LIN_UART_BUFF_BREAK && *cp == 0x55)
				u->rx_buff[u->rx_cnt++] = 0x00;

			if (u->rx_cnt == LIN_UART_BUFF_SYNC) {
				if (*cp == 0x00) {
					/* Stray break char; ignore. */
					cp++;
					continue;
				}
				if (*cp != 0x55)
					break;	/* malformed sync; bail */
			}

			u->rx_buff[u->rx_cnt++] = *cp++;
		}

		if (!u->header_received && u->rx_cnt >= LIN_UART_BUFF_ID + 1) {
			unsigned long flags;

			lin_id = u->rx_buff[LIN_UART_BUFF_ID] & LIN_ID_MASK;

			spin_lock_irqsave(u->lock, flags);
			u->state = LIN_UART_ID_RECEIVED;
			if (lin_resp_get_locked(u->resp, lin_id, &snap)) {
				u->rx_expect += snap.len + 1;
				u->rx_len_unknown = false;
				u->cur_id = lin_id;
				u->cur_we_publish = true;
				u->cur_master_emit = false;
				u->cur_enhanced = snap.enhanced;
				wake_up(u->wq);
			} else {
				u->rx_expect += LIN_MAX_DLEN + 1;
				u->rx_len_unknown = true;
				u->cur_id = lin_id;
				u->cur_we_publish = false;
				u->cur_master_emit = false;
				u->cur_enhanced = false;
			}
			spin_unlock_irqrestore(u->lock, flags);

			u->header_received = true;
			hrtimer_start(&u->rx_timer,
				      ktime_add(ktime_get(),
						u->rx_timer_timeout),
				      HRTIMER_MODE_ABS);
			continue;
		}

		if (u->header_received && u->rx_cnt >= u->rx_expect) {
			unsigned long flags;

			hrtimer_cancel(&u->rx_timer);
			spin_lock_irqsave(u->lock, flags);
			lin_uart_slave_finish_rx(u);
			spin_unlock_irqrestore(u->lock, flags);
			set_bit(LIN_UART_F_RXEVENT, &u->flags);
			wake_up(u->wq);
		}
	}
}

void lin_uart_receive_buf(struct lin_uart *u, const u8 *cp, const u8 *fp,
			  size_t count)
{
	/* Use the master-side parser when we are driving a header (we
	 * sourced the break and PID, so we expect to see them echoed
	 * back) and the slave-side parser otherwise.
	 */
	if (u->cur_master_emit)
		lin_uart_master_receive_buf(u, cp, fp, count);
	else
		lin_uart_slave_receive_buf(u, cp, fp, count);
}
EXPORT_SYMBOL_GPL(lin_uart_receive_buf);

/* Per-frame RX timeout: arm-and-forget; the helper fires this when
 * the configured byte budget has elapsed without the rx parser
 * reaching its rx_expect. Wired by lin_uart_init().
 *
 * Softirq-safe minimum: set the flag and wake the kthread. The
 * frontend's TMOUTEVENT handler holds @u->lock, disambiguates the
 * master "no response" case from a slave-side unknown-length frame
 * (where the budget is the legitimate end-of-frame), and calls
 * lin_uart_slave_finish_rx() under the lock for the latter.
 * Mutating the rx state directly from here would race the parser's
 * unlocked byte appends in lin_uart_slave_receive_buf().
 */
static enum hrtimer_restart lin_uart_rx_timeout_handler(struct hrtimer *t)
{
	struct lin_uart *u = container_of(t, struct lin_uart, rx_timer);

	set_bit(LIN_UART_F_TMOUTEVENT, &u->flags);
	wake_up(u->wq);
	return HRTIMER_NORESTART;
}
