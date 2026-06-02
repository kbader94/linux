// SPDX-License-Identifier: GPL-2.0
/*
 * drivers/net/lin/dev/kthread.c - LIN schedule kthread driver.
 *
 * Hosts the transport-agnostic kthread that drives a LIN bus from a
 * host-side driver: slot evaluation, wire-format state machine, error
 * and timeout handling, header_send coordination, and slave-side
 * publish-on-header dispatch. Each frontend (sllin, sdlin, future
 * hardware drivers) embeds a struct lin_uart, initialises a
 * struct lin_sched and struct lin_resp_table, attaches them via
 * lin_uart_init(), and spawns lin_sched_kthread_fn via kthread_run().
 *
 * Locking: the per-link spinlock (@u->lock) protects all schedule
 * and response-cache state plus the wire-state cursor and per-slot
 * tags. The kthread takes it for state-mutation sections and drops
 * it before sleeping. The io_ops->write hook may sleep on future
 * transports (sdlin's serdev write, USB bridges) — see uart.h for
 * the rx/tx cursor consistency contract that lets send_tx_buff
 * safely run outside the lock.
 *
 * Author: Kyle Bader <kyle.bader94@gmail.com>
 * Copyright (c) 2026 Kyle Bader
 */

#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/export.h>
#include <linux/hrtimer.h>
#include <linux/kthread.h>
#include <linux/ktime.h>
#include <linux/netdevice.h>
#include <linux/sched.h>
#include <linux/sched/types.h>
#include <linux/spinlock.h>
#include <linux/string.h>
#include <linux/wait.h>

#include <linux/lin.h>
#include <linux/lin/drv.h>
#include <linux/lin/uart.h>

#include <uapi/linux/lin/error.h>

/* ------------------------------------------------------------------ */
/* Slot fire helpers                                                   */
/* ------------------------------------------------------------------ */

/* Stage a header (and optional publisher response) into the tx buffer
 * and per-slot tags so the next state-machine tick drives the wire.
 * Caller holds @u->lock.
 */
static int fire_header_locked(struct lin_uart *u, u8 id,
			      const u8 *data, u8 len, bool enhanced,
			      bool master_emit, bool we_publish)
{
	lin_uart_reset_buffs(u);

	if (lin_uart_setup_msg(u, false, enhanced, id, data, len) < 0)
		return -EINVAL;

	u->id_to_send = true;
	u->data_to_send = (data != NULL);
	u->resp_len_known = (len > 0);
	u->cur_id = id;
	u->cur_master_emit = master_emit;
	u->cur_we_publish = we_publish;
	u->cur_enhanced = enhanced;
	u->dev->stats.tx_packets++;
	u->dev->stats.tx_bytes += len;
	return 0;
}

/* Pull one slot worth of fire data: for TYPE_UNCOND / TYPE_DIAG the
 * single member is the ID; for TYPE_SPORADIC we walk the priority list
 * and pick the first dirty publisher; for TYPE_EVENT we count dirty
 * group members and either fire the unique answerer or notify and
 * divert. Returns true if the slot generated a header, false if it
 * stayed silent (and the engine should sleep through it).
 *
 * Caller holds @u->lock.
 */
static bool run_slot_locked(struct lin_uart *u,
			    const struct lin_schedule_entry *e)
{
	struct lin_resp_entry snap;
	u8 id;

	switch (e->type) {
	case LIN_SCHED_TYPE_UNCOND:
	case LIN_SCHED_TYPE_DIAG:
		id = e->members[0] & LIN_ID_MASK;
		if (lin_resp_snapshot_for_emit_locked(u->resp, id, &snap)) {
			return fire_header_locked(u, id, snap.data,
						  snap.len, snap.enhanced,
						  true, true) == 0;
		}
		/* No publisher: fire the header and let the slave (or
		 * NO_RESPONSE timeout) speak.
		 */
		return fire_header_locked(u, id, NULL, 0, false,
					  true, false) == 0;

	case LIN_SCHED_TYPE_SPORADIC:
		id = lin_resp_sporadic_pick_locked(u->resp, e->members,
						   e->member_count, &snap);
		if (id == LIN_ID_NONE)
			return false;
		return fire_header_locked(u, id, snap.data, snap.len,
					  snap.enhanced, true, true) == 0;

	case LIN_SCHED_TYPE_EVENT: {
		const struct lin_schedule *cr = u->sched->sched[e->cr_handle];
		u8 trigger = e->members[0] & LIN_ID_MASK;
		u8 member_ids[LIN_RAW_SCHEDULE_ENTRIES_MAX];
		unsigned int i, n, dirty;
		u8 first = 0;

		if (!cr)
			return false;

		/* Flatten the CR schedule's TYPE_UNCOND entries into a
		 * member-id list for the helper. The core guarantees a
		 * CR schedule is uncond-only (lin_schedule_validate), so
		 * the type filter here is belt-and-suspenders.
		 */
		for (i = 0, n = 0; i < cr->entry_count; i++) {
			const struct lin_schedule_entry *ce = &cr->entry[i];

			if (ce->type != LIN_SCHED_TYPE_UNCOND)
				continue;
			member_ids[n++] = ce->members[0];
		}

		dirty = lin_resp_event_count_dirty_locked(u->resp,
							  member_ids, n,
							  &first);
		if (dirty == 0)
			return false;
		if (dirty == 1) {
			lin_resp_snapshot_for_emit_locked(u->resp, first,
							  &snap);
			/* The trigger ID is the header, but the response
			 * is owned by the answerer (first). Pass first as
			 * the publisher resp_id via u->cur_id.
			 */
			if (fire_header_locked(u, trigger, snap.data,
					       snap.len, snap.enhanced,
					       true, true) == 0) {
				u->cur_id = first;
				return true;
			}
			return false;
		}
		/* Collision: notify, then divert to the CR schedule. */
		lin_uart_emit_bus_event(u, trigger, LIN_F_EVENT_COLLISION, 0);
		if (!u->sched->diverted) {
			u->sched->saved_handle = u->sched->active;
			u->sched->saved_slot = u->sched->slot + 1;
			u->sched->active = e->cr_handle;
			u->sched->slot = 0;
			u->sched->diverted = true;
			u->sched->divert_pending = true;
		}
		return false;
	}

	default:
		return false;
	}
}

/* ------------------------------------------------------------------ */
/* Mainline kthread loop helpers                                       */
/* ------------------------------------------------------------------ */

/* Wait for either: a wakeable bit to assert, the next slot boundary
 * to pass, or kthread_stop. Returns true if the boundary deadline
 * elapsed and the engine should advance a slot, false if it was woken
 * for an rx/tx/error event (or shutdown) and should pump those first.
 */
static bool kthread_wait(struct lin_uart *u)
{
	ktime_t now = ktime_get();
	ktime_t rem;

	if (test_bit(LIN_SCHED_F_RUNNING, &u->sched->flags) &&
	    !ktime_after(u->sched->next_slot, now))
		return true;	/* deadline already passed */

	rem = test_bit(LIN_SCHED_F_RUNNING, &u->sched->flags) ?
		ktime_sub(u->sched->next_slot, now) : ms_to_ktime(1000);

	wait_event_hrtimeout(*u->wq,
		kthread_should_stop() ||
		test_bit(LIN_UART_F_RXEVENT, &u->flags) ||
		test_bit(LIN_UART_F_TXEVENT, &u->flags) ||
		test_bit(LIN_UART_F_TMOUTEVENT, &u->flags) ||
		test_bit(LIN_UART_F_ERROR, &u->flags) ||
		test_bit(LIN_SCHED_F_HDR_REQ, &u->sched->flags) ||
		u->state == LIN_UART_ID_RECEIVED,
		rem);

	if (test_bit(LIN_SCHED_F_RUNNING, &u->sched->flags) &&
	    !ktime_after(u->sched->next_slot, ktime_get()))
		return true;
	return false;
}

/* Try to fire the next header into the bus when idle. Two sources:
 *
 *   - an ad-hoc lin_dev_ops.header_send() request parked on
 *     LIN_SCHED_F_HDR_REQ; the kthread snapshots the request fields under the
 *     lock, refuses stale requests left by a racing master_stop
 *     (-ECANCELED), and falls through to send_break;
 *   - the current slot of the active master schedule, if the
 *     slot-boundary deadline has elapsed. The slot may stay silent
 *     (TYPE_SPORADIC/TYPE_EVENT with no dirty members), in which case
 *     the bookkeeping is still advanced so the next call evaluates the
 *     following slot.
 *
 * On a silent or stale path, returns without firing; the mainline's
 * kthread_wait() then sleeps to the next boundary and the loop
 * retries.
 */
static void kthread_try_fire(struct lin_uart *u)
{
	u8 hdr_id = 0, hdr_data[LIN_MAX_DLEN], hdr_len = 0;
	bool fire_now = false, hdr_enh = false, hdr_pub = false;
	unsigned long flags;

	spin_lock_irqsave(u->lock, flags);

	lin_sched_apply_activate_locked(u->sched);

	if (test_bit(LIN_SCHED_F_HDR_REQ, &u->sched->flags)) {
		/* Drop the request if master_stop ran between
		 * header_send's setsockopt and the kthread waking up —
		 * LIN_SCHED_F_MASTER_RUNNING being clear means the master claim
		 * has been released, so the request is stale.
		 */
		if (!test_bit(LIN_SCHED_F_MASTER_RUNNING, &u->sched->flags)) {
			u->sched->hdr_status = -ECANCELED;
			clear_bit(LIN_SCHED_F_HDR_REQ, &u->sched->flags);
			set_bit(LIN_SCHED_F_HDR_DONE, &u->sched->flags);
			spin_unlock_irqrestore(u->lock, flags);
			wake_up(u->wq);
			return;
		}
		/* Snapshot the request fields. If the caller supplied no
		 * data but we own a registered response for the same ID,
		 * snapshot that as the publisher payload.
		 */
		hdr_id = u->sched->hdr_id;
		hdr_enh = u->sched->hdr_enhanced;
		if (u->sched->hdr_len) {
			memcpy(hdr_data, u->sched->hdr_data, u->sched->hdr_len);
			hdr_len = u->sched->hdr_len;
			hdr_pub = false;
		} else {
			struct lin_resp_entry snap;

			if (lin_resp_snapshot_for_emit_locked(u->resp,
							      hdr_id,
							      &snap)) {
				memcpy(hdr_data, snap.data, snap.len);
				hdr_len = snap.len;
				hdr_enh = snap.enhanced;
				hdr_pub = true;
			}
		}
		clear_bit(LIN_SCHED_F_HDR_REQ, &u->sched->flags);
		fire_now = true;
	} else if (test_bit(LIN_SCHED_F_RUNNING, &u->sched->flags) &&
		   u->sched->active >= 0 && u->sched->sched[u->sched->active]) {
		const struct lin_schedule *s = u->sched->sched[u->sched->active];
		const struct lin_schedule_entry *e;
		ktime_t dur;
		bool fired;

		if (u->sched->slot >= s->entry_count)
			u->sched->slot = 0;
		e = &s->entry[u->sched->slot];

		if (ktime_after(u->sched->next_slot, ktime_get())) {
			spin_unlock_irqrestore(u->lock, flags);
			return;	/* slot boundary not yet reached */
		}

		dur = lin_sched_slot_duration(s, e);
		fired = run_slot_locked(u, e);
		u->sched->next_slot = ktime_add(ktime_get(), dur);
		lin_sched_advance_slot_locked(u->sched, s);

		if (!fired) {
			spin_unlock_irqrestore(u->lock, flags);
			return;	/* silent slot; sleep through it */
		}
	}
	spin_unlock_irqrestore(u->lock, flags);

	if (fire_now) {
		spin_lock_irqsave(u->lock, flags);
		if (fire_header_locked(u, hdr_id,
				       hdr_len ? hdr_data : NULL,
				       hdr_len, hdr_enh,
				       true, hdr_pub) < 0) {
			u->sched->hdr_status = -EINVAL;
			set_bit(LIN_SCHED_F_HDR_DONE, &u->sched->flags);
			spin_unlock_irqrestore(u->lock, flags);
			wake_up(u->wq);
			return;
		}
		spin_unlock_irqrestore(u->lock, flags);
	}

	if ((test_bit(LIN_SCHED_F_RUNNING, &u->sched->flags) || fire_now) &&
	    u->id_to_send) {
		/* On failure the wire never sees a valid break, so the
		 * frame this slot was supposed to drive is lost. The
		 * helper logged the underlying transport error; report
		 * LIN_ERR_SYNC on the bus-event channel so userspace
		 * sees a slot failure (not the misleading NO_RESPONSE
		 * the rx-timer would otherwise raise), reset wire state
		 * to IDLE, and unblock a parked header_send with -EIO.
		 */
		if (lin_uart_send_break(u) < 0) {
			u->dev->stats.tx_errors++;
			lin_uart_emit_bus_event(u, u->cur_id,
						LIN_F_ERR, LIN_ERR_SYNC);
			lin_uart_reset_buffs(u);
			u->state = LIN_UART_IDLE;

			spin_lock_irqsave(u->lock, flags);
			if (u->sched->hdr_status == -EINPROGRESS) {
				u->sched->hdr_status = -EIO;
				set_bit(LIN_SCHED_F_HDR_DONE,
					&u->sched->flags);
				wake_up(u->wq);
			}
			spin_unlock_irqrestore(u->lock, flags);
		}
	}
}

/* Handle LIN_UART_F_ERROR (framing error on the wire, or send_tx_buff
 * failure). Report it as a bus event when a frame was in flight, sleep
 * a recovery interval (~ten character times) so the bus settles, reset
 * wire state to IDLE, and unblock a parked header_send with -EIO.
 */
static void kthread_handle_error(struct lin_uart *u)
{
	unsigned long flags, us;

	hrtimer_cancel(&u->rx_timer);
	if (u->state != LIN_UART_IDLE) {
		u->dev->stats.rx_frame_errors++;
		u->dev->stats.rx_errors++;
		lin_uart_emit_bus_event(u, u->cur_id, LIN_F_ERR,
					LIN_ERR_FRAMING);
	}

	us = (1000000UL * 10 * 10) / u->baud;
	usleep_range(us, us + 50);
	lin_uart_reset_buffs(u);
	u->state = LIN_UART_IDLE;

	spin_lock_irqsave(u->lock, flags);
	if (test_bit(LIN_SCHED_F_HDR_REQ, &u->sched->flags) ||
	    u->sched->hdr_status == -EINPROGRESS) {
		u->sched->hdr_status = -EIO;
		set_bit(LIN_SCHED_F_HDR_DONE, &u->sched->flags);
		wake_up(u->wq);
	}
	spin_unlock_irqrestore(u->lock, flags);
}

/* Handle LIN_UART_F_TMOUTEVENT (rx_timer fired). Two flavours:
 *
 *   - Master in flight (@cur_master_emit): the slave never responded
 *     to a header we sent. Report NO_RESPONSE, reset wire state to
 *     IDLE, and unblock a parked header_send with success —
 *     header_send does not promise a response, so a no-response
 *     slot is its normal "header emitted, slave silent" outcome.
 *
 *   - Slave observing an unknown-length frame: we received a header
 *     for an ID we don't publish, granted the worst-case 8-byte
 *     budget plus checksum, and the byte budget elapsed. Deliver
 *     whatever the parser collected via lin_uart_slave_finish_rx()
 *     under @u->lock — the bus event is the legitimate end of the
 *     observed frame, not an error. The lock serialises this
 *     delivery against the parser's own break-cancel and
 *     frame-complete finish paths; the helper is idempotent via the
 *     @header_received gate.
 *
 *   - Stale timer (anything else): nothing to do.
 */
static void kthread_handle_timeout(struct lin_uart *u)
{
	unsigned long flags;

	hrtimer_cancel(&u->rx_timer);

	if (u->cur_master_emit) {
		u->dev->stats.rx_errors++;
		lin_uart_emit_bus_event(u, u->cur_id, LIN_F_ERR,
					LIN_ERR_NO_RESPONSE);
		lin_uart_reset_buffs(u);
		u->state = LIN_UART_IDLE;
	} else if (u->rx_len_unknown && u->rx_cnt > LIN_UART_BUFF_DATA) {
		spin_lock_irqsave(u->lock, flags);
		lin_uart_slave_finish_rx(u);
		spin_unlock_irqrestore(u->lock, flags);
	}

	spin_lock_irqsave(u->lock, flags);
	if (u->sched->hdr_status == -EINPROGRESS) {
		u->sched->hdr_status = 0;
		set_bit(LIN_SCHED_F_HDR_DONE, &u->sched->flags);
		wake_up(u->wq);
	}
	spin_unlock_irqrestore(u->lock, flags);
}

/* Drive the wire-format state machine forward by one tick.
 * RESPONSE_WAIT and RESPONSE_SENT may not have collected enough bytes
 * yet — they return without state change, and the next mainline
 * iteration sleeps for more bytes (or for the rx_timer to escalate to
 * TMOUTEVENT).
 */
static void kthread_step_state(struct lin_uart *u)
{
	switch (u->state) {
	case LIN_UART_IDLE:
		break;

	case LIN_UART_BREAK_SENT:
		u->state = LIN_UART_ID_SENT;
		lin_uart_send_tx_buff(u);
		break;

	case LIN_UART_ID_SENT:
		hrtimer_cancel(&u->rx_timer);
		u->id_to_send = false;
		if (u->data_to_send) {
			lin_uart_send_tx_buff(u);
			u->rx_expect = u->tx_lim;
			u->state = LIN_UART_RESPONSE_SENT;
		} else {
			if (u->resp_len_known)
				u->rx_expect = u->rx_lim;
			else
				u->rx_expect = LIN_UART_BUFF_DATA + 2;
			u->state = LIN_UART_RESPONSE_WAIT;
		}
		hrtimer_start(&u->rx_timer,
			      ktime_add(ktime_get(), u->rx_timer_timeout),
			      HRTIMER_MODE_ABS);
		break;

	case LIN_UART_RESPONSE_WAIT: {
		unsigned long flags;

		if (u->rx_cnt < u->rx_expect)
			return;
		hrtimer_cancel(&u->rx_timer);
		if (lin_uart_rx_validate(u) < 0) {
			u->dev->stats.rx_crc_errors++;
			u->dev->stats.rx_errors++;
			lin_uart_emit_bus_event(u, u->cur_id, LIN_F_ERR,
					     LIN_ERR_CHECKSUM);
		} else {
			lin_uart_deliver_rx(u);
		}
		/* Master frame is done; clear the per-slot tags so a
		 * subsequent external header lands in the slave parser
		 * via the receive_buf dispatcher.
		 */
		u->cur_master_emit = false;
		u->cur_we_publish = false;
		u->cur_id = LIN_ID_NONE;
		u->state = LIN_UART_IDLE;

		spin_lock_irqsave(u->lock, flags);
		if (u->sched->hdr_status == -EINPROGRESS) {
			u->sched->hdr_status = 0;
			set_bit(LIN_SCHED_F_HDR_DONE, &u->sched->flags);
			wake_up(u->wq);
		}
		spin_unlock_irqrestore(u->lock, flags);
		break;
	}

	case LIN_UART_ID_RECEIVED: {
		u8 id = u->rx_buff[LIN_UART_BUFF_ID] & LIN_ID_MASK;
		struct lin_resp_entry snap;
		bool publish = false;
		unsigned long flags;

		spin_lock_irqsave(u->lock, flags);
		if (lin_resp_snapshot_for_emit_locked(u->resp, id, &snap) &&
		    snap.len) {
			u->cur_id = id;
			u->cur_we_publish = true;
			u->cur_master_emit = false;
			u->cur_enhanced = snap.enhanced;
			if (lin_uart_setup_msg(u, true, snap.enhanced, id,
					    snap.data, snap.len) == 0) {
				u->rx_expect = u->tx_lim;
				u->data_to_send = true;
				u->resp_len_known = true;
				u->tx_cnt = LIN_UART_BUFF_DATA;
				u->dev->stats.tx_packets++;
				u->dev->stats.tx_bytes += snap.len;
				publish = true;
			}
		}
		spin_unlock_irqrestore(u->lock, flags);

		/* Drive the wire and arm the rx-timer outside the lock:
		 * lin_uart_send_tx_buff's RQ/INPR re-entry barrier is the
		 * intended serialisation primitive (see uart.h), and the
		 * transport's write hook may sleep on future frontends
		 * (sdlin's serdev write, a USB bridge, etc.). Both tx_buff
		 * and rx_expect were published under @lock above, so the
		 * parser sees consistent cursors when echo bytes arrive.
		 */
		if (publish) {
			lin_uart_send_tx_buff(u);
			hrtimer_start(&u->rx_timer,
				      ktime_add(ktime_get(),
						u->rx_timer_timeout),
				      HRTIMER_MODE_ABS);
		}
		u->state = LIN_UART_IDLE;
		break;
	}

	case LIN_UART_RESPONSE_SENT: {
		unsigned long flags;

		if (u->rx_cnt < u->tx_lim)
			return;
		hrtimer_cancel(&u->rx_timer);
		lin_uart_deliver_rx(u);
		/* Same reason as RESPONSE_WAIT. */
		u->cur_master_emit = false;
		u->cur_we_publish = false;
		u->cur_id = LIN_ID_NONE;
		u->state = LIN_UART_IDLE;

		spin_lock_irqsave(u->lock, flags);
		if (u->sched->hdr_status == -EINPROGRESS) {
			u->sched->hdr_status = 0;
			set_bit(LIN_SCHED_F_HDR_DONE, &u->sched->flags);
			wake_up(u->wq);
		}
		spin_unlock_irqrestore(u->lock, flags);
		break;
	}
	}
}

/* ------------------------------------------------------------------ */
/* Public entry                                                        */
/* ------------------------------------------------------------------ */

int lin_sched_kthread_fn(void *data)
{
	struct lin_uart *u = data;

	sched_set_fifo(current);

	/* The UART was already programmed to the LIN baud by the
	 * frontend before the netdev was registered; nothing to do at
	 * kthread entry beyond arming the slot deadline.
	 */
	u->sched->next_slot = ktime_get();

	while (!kthread_should_stop()) {
		if (u->state == LIN_UART_IDLE)
			kthread_try_fire(u);

		kthread_wait(u);

		if (test_and_clear_bit(LIN_UART_F_ERROR, &u->flags))
			kthread_handle_error(u);
		if (test_and_clear_bit(LIN_UART_F_TMOUTEVENT, &u->flags))
			kthread_handle_timeout(u);
		test_and_clear_bit(LIN_UART_F_RXEVENT, &u->flags);
		test_and_clear_bit(LIN_UART_F_TXEVENT, &u->flags);

		kthread_step_state(u);
	}

	hrtimer_cancel(&u->rx_timer);
	return 0;
}
EXPORT_SYMBOL_GPL(lin_sched_kthread_fn);
