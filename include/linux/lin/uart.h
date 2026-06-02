/* SPDX-License-Identifier: GPL-2.0 */
/*
 * include/linux/lin/uart.h - LIN-over-UART byte-protocol helpers.
 *
 * Public surface for the UART-specific half of lin-dev. UART-based
 * frontends (sllin's TTY ldisc, sdlin's serdev driver) embed a
 * struct lin_uart in their per-link state and call into these
 * helpers for:
 *
 *   - Byte-level state machine (BREAK_SENT -> ID_SENT ->
 *     RESPONSE_WAIT/SENT).
 *   - Break pulse generation (driver-provided break_ctl + timed
 *     sleep), via the transport vtable.
 *   - Wire format: PID parity, classic and enhanced checksum,
 *     frame assembly.
 *   - Master / slave RX byte parsers, with synthesised leading 0x00
 *     for the break (which is not a UART character).
 *   - TX buffering with a re-entry barrier.
 *   - FIFO Control framework probe and the LIN_CAP_PUB_SLAVE
 *     eligibility decision.
 *
 * The frontend embeds a struct lin_uart in its per-link state and
 * supplies a struct lin_uart_io_ops vtable carrying its transport-
 * specific TX, break_ctl, set_baud, and FIFO-port primitives; the
 * helpers drive the wire protocol on top.
 *
 * Symbols defined here are exported by lin-dev.ko, gated on
 * CONFIG_LIN_DEV_HELPERS_UART.
 *
 * Author: Kyle Bader <kyle.bader94@gmail.com>
 * Copyright (c) 2026 Kyle Bader
 */

#ifndef _LIN_UART_H
#define _LIN_UART_H

#include <linux/hrtimer.h>
#include <linux/spinlock_types.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/lin.h>		/* LIN_ID_MASK, LIN_MAX_DLEN, LIN_ID_NONE */

/* ----------------------------------------------------------------
 * Frame buffer layout
 * ----------------------------------------------------------------
 */
#define LIN_UART_BUFF_BREAK	0
#define LIN_UART_BUFF_SYNC	1
#define LIN_UART_BUFF_ID	2
#define LIN_UART_BUFF_DATA	3
#define LIN_UART_BUFF_LEN	(1 + 1 + 1 + LIN_MAX_DLEN + 1)

/* ----------------------------------------------------------------
 * Wire-format helpers
 * ----------------------------------------------------------------
 *
 * The on-wire layout each frame produces is:
 *
 *   BREAK   ~13 bit times dominant (driven by break_ctl, not a
 *           UART character; the RX parsers synthesise a leading 0x00
 *           so internal buffers line up with the response-bearing
 *           slots).
 *   SYNC    0x55 (10 bits 8N1).
 *   PID     6-bit lin_id ORed with its 2-bit parity. lin_uart_pid()
 *           computes this from a lookup table.
 *   data    0..LIN_MAX_DLEN bytes.
 *   CHK     classic (data only) or enhanced (PID + data) checksum.
 *           lin_uart_checksum() computes either; the @enhanced
 *           argument selects.
 */

/**
 * lin_uart_pid - compute the on-wire protected ID for @lin_id.
 * @lin_id: 6-bit frame ID (low bits; high bits ignored).
 *
 * Returns the byte to transmit in the PID slot: the 6-bit ID in the
 * low bits, ORed with the two LIN parity bits in bits 6..7. Both
 * master headers and slave RX validators use this.
 */
u8 lin_uart_pid(u8 lin_id);

/**
 * lin_uart_checksum - compute a LIN frame checksum.
 * @buf:      assembled frame buffer (starting at the BREAK slot).
 * @end:      one past the last data byte (i.e. the checksum slot
 *            index itself).
 * @enhanced: true for LIN 2.x enhanced checksum (folds the PID byte
 *            into the sum), false for classic (data bytes only).
 *
 * Returns the one's-complement, carry-folded sum used as the LIN
 * checksum byte. The PID is expected at @buf[LIN_UART_BUFF_ID] and
 * data starts at @buf[LIN_UART_BUFF_DATA].
 */
u8 lin_uart_checksum(const u8 *buf, int end, bool enhanced);

/* ----------------------------------------------------------------
 * struct lin_uart - per-link byte-protocol state
 * ----------------------------------------------------------------
 *
 * Frontends (sllin, sdlin) embed this in their per-link state and
 * fill out @dev, @resp, @io, and @baud at attach time via
 * lin_uart_init().
 *
 * Locking: the helpers operate under a per-link spinlock that the
 * frontend owns. Functions named *_locked() expect the lock held on
 * entry. The struct itself contains no lock; the frontend supplies
 * one.
 */

enum lin_uart_state {
	LIN_UART_IDLE = 0,
	LIN_UART_BREAK_SENT,
	LIN_UART_ID_SENT,
	LIN_UART_RESPONSE_WAIT,		/* master waits for slave bytes  */
	LIN_UART_ID_RECEIVED,		/* slave saw a header, may answer*/
	LIN_UART_RESPONSE_SENT,		/* response bytes pushed         */
};

struct lin_uart;
struct lin_resp_table;
struct lin_sched;
struct net_device;

/**
 * struct lin_uart_io_ops - transport vtable for the byte-protocol layer.
 * @write:        push @len bytes from @buf out on the bus. Returns
 *                bytes written or negative errno.
 * @break_ctl:    drive the LIN break low (state -1) or release it
 *                (state 0). Required.
 * @set_baud:     reprogram the bus bit rate. Called from the bitrate
 *                set path and at attach.
 * @flush_buffer: optional - flush the underlying TX buffer.
 * @tx_wakeup_arm:    arm the transport so the frontend's write_wakeup
 *                    hook will be invoked when TX space frees.
 *                    Optional; needed for split TX pushes.
 * @tx_wakeup_disarm: disarm the wakeup hook.
 *
 * All callbacks may sleep unless documented otherwise; the byte
 * state machine runs from a kthread context.
 */
struct lin_uart_io_ops {
	int  (*write)(struct lin_uart *u, const u8 *buf, int len);
	int  (*break_ctl)(struct lin_uart *u, int state);
	int  (*set_baud)(struct lin_uart *u, u32 baud);
	void (*flush_buffer)(struct lin_uart *u);
	void (*tx_wakeup_arm)(struct lin_uart *u);
	void (*tx_wakeup_disarm)(struct lin_uart *u);
};

/**
 * struct lin_uart - per-link UART byte-protocol state.
 * @dev:    backpointer to the LIN netdev (for stats, alloc_lin_skb,
 *          lin_loopback_rx).
 * @resp:   per-link publisher response table (frontend-owned; the
 *          helper module accesses it through this pointer when
 *          assembling slave responses and slot fires).
 * @io:     transport vtable; immutable after lin_uart_init().
 * @io_priv: opaque cookie the frontend can stash for its io_ops to
 *           recover its own state (typically a tty_struct * for
 *           sllin or a serdev_device * for sdlin). Helpers treat
 *           this as a black box.
 * @state:  wire state machine cursor.
 * @rx_buff/@tx_buff: assembly buffers for a single LIN frame.
 * @rx_cnt/@rx_expect/@rx_lim: receive-side counters; @rx_cnt is the
 *          number of bytes accumulated, @rx_expect is how many we
 *          expect before delivering the frame, @rx_lim is the upper
 *          bound of accepted data bytes.
 * @tx_cnt/@tx_lim: transmit counters; @tx_cnt is the byte offset
 *          into tx_buff already pushed, @tx_lim is the index just
 *          past the last byte we want to push (checksum slot
 *          inclusive).
 * @id_to_send/@data_to_send/@resp_len_known: master-side flags set
 *          by lin_uart_setup_msg() to drive the state machine.
 * @header_received/@rx_len_unknown: slave-side flags set by the RX
 *          parser when a header has been decoded but the response
 *          length is not pre-known from the cache.
 * @baud:   current bus bit rate in bits per second.
 * @rx_timer/@rx_timer_timeout: per-frame RX bound (24 character
 *          times). Frontend initialises the timer; the helpers arm
 *          and cancel it.
 * @cur_id: ID of the frame currently being assembled (LIN_ID_NONE
 *          when idle). Stamped on loopback emissions.
 * @cur_we_publish:  this frame's response is owned by a local
 *                   publisher; tag loopback as LIN_EMIT_PUBLISHER.
 * @cur_master_emit: the local master fired this frame's header; tag
 *                   loopback as LIN_EMIT_MASTER.
 * @cur_enhanced:    checksum class for this frame.
 * @flags:  atomic bits for cross-context coordination; see the
 *          LIN_UART_F_* defines below. Set/clear/test with the
 *          standard {set,clear,test}_bit() primitives; no external
 *          lock required.
 */
struct lin_uart {
	struct net_device		*dev;
	struct lin_resp_table		*resp;
	struct lin_sched		*sched;
	const struct lin_uart_io_ops	*io;
	void				*io_priv;

	/* Frontend-owned coordination primitives. The byte-machine
	 * helpers serialise mutation of the wire-state with @lock and
	 * wake the engine's kthread (or workqueue) via @wq when an
	 * event bit is set on @flags. Both are stored as pointers so
	 * the frontend keeps ownership and can pick the lock primitive
	 * appropriate to its transport (spinlock for sllin/sdlin,
	 * possibly a mutex for a USB driver).
	 */
	spinlock_t			*lock;
	wait_queue_head_t		*wq;

	enum lin_uart_state		state;
	u8				rx_buff[LIN_UART_BUFF_LEN];
	u8				tx_buff[LIN_UART_BUFF_LEN];
	int				rx_cnt;
	int				rx_expect;
	int				rx_lim;
	int				tx_cnt;
	int				tx_lim;
	bool				id_to_send;
	bool				data_to_send;
	bool				resp_len_known;
	bool				header_received;
	bool				rx_len_unknown;

	u32				baud;

	struct hrtimer			rx_timer;
	ktime_t				rx_timer_timeout;

	u8				cur_id;
	bool				cur_we_publish;
	bool				cur_master_emit;
	bool				cur_enhanced;

	unsigned long			flags;
};

/* @flags bits. All cross-context wakeup signals; no external lock. */
#define LIN_UART_F_RXEVENT	0
#define LIN_UART_F_TXEVENT	1
#define LIN_UART_F_TMOUTEVENT	2
#define LIN_UART_F_ERROR	3
#define LIN_UART_F_TXBUFF_RQ	4
#define LIN_UART_F_TXBUFF_INPR	5

/**
 * lin_uart_init - initialise a per-link UART byte-protocol state.
 * @u:    state to initialise. Caller must zero the struct before
 *        invocation; the typical pattern is embedding it in a kzalloc'd
 *        parent.
 * @dev:  the LIN netdev this state serves.
 * @resp: pointer to the frontend's per-link response cache.
 * @sched: pointer to the frontend's per-link schedule engine state.
 *        The shared kthread driver (lin_sched_kthread_fn) reads this
 *        via @u to evaluate slots, so the frontend must initialise
 *        the lin_sched (lin_sched_init()) before attaching.
 * @io:   transport vtable; stored as-is and treated as immutable.
 * @io_priv: opaque cookie returned to the io_ops on each call.
 * @baud: initial bus bit rate.
 *
 * Sets @state to LIN_UART_IDLE, @cur_id to LIN_ID_NONE, and arms the
 * per-frame RX bound for the configured baud. Does NOT call into the
 * io_ops; the caller is expected to program the underlying UART to
 * @baud separately (so the helper can be invoked before the netdev
 * is registered).
 */
void lin_uart_init(struct lin_uart *u, struct net_device *dev,
		   struct lin_resp_table *resp, struct lin_sched *sched,
		   spinlock_t *lock, wait_queue_head_t *wq,
		   const struct lin_uart_io_ops *io, void *io_priv,
		   u32 baud);

/**
 * lin_uart_set_baud - reprogram the bus bit rate.
 * @u:    state.
 * @baud: new bit rate, bits per second.
 *
 * Updates @u->baud and the per-frame RX timeout under @u->lock so the
 * kthread sees both fields advance atomically, then reprograms the
 * transport via @io->set_baud (called outside the lock since the
 * transport hook may sleep — e.g. TTY termios update). Drivers with a
 * fixed-rate transport leave @io->set_baud NULL; the helper still
 * updates the in-memory baud and returns 0 in that case.
 *
 * Returns 0 on success, -EINVAL on @baud == 0, or the errno
 * propagated from @io->set_baud.
 */
int lin_uart_set_baud(struct lin_uart *u, u32 baud);

/**
 * lin_uart_reset_buffs - reset wire-state cursors and per-slot tags.
 * @u: state.
 *
 * Clears the rx/tx counters, header/length flags, and the per-slot
 * loopback tags. Does NOT touch @state, @baud, or @rx_timer. Called
 * by the schedule engine after a frame completes and by the error
 * recovery path.
 */
void lin_uart_reset_buffs(struct lin_uart *u);

/**
 * lin_uart_setup_msg - assemble the on-wire tx_buff for a frame.
 * @u:             state.
 * @response_only: when true, leave the break/sync/PID alone and only
 *                 append response bytes + checksum (slave-side
 *                 publish path after an external header is decoded).
 * @enhanced:      checksum class for this frame.
 * @id:            6-bit frame ID.
 * @data:          response payload (NULL when this header carries no
 *                 local response).
 * @len:           payload length (0 when @data is NULL).
 *
 * Returns 0 on success, -EINVAL when @id or @len is out of range. The
 * frontend caller holds the per-link lock.
 */
int lin_uart_setup_msg(struct lin_uart *u, bool response_only,
		       bool enhanced, u8 id, const u8 *data, u8 len);

/**
 * lin_uart_rx_validate - check the assembled rx_buff's checksum.
 * @u: state.
 *
 * Tries the @cur_enhanced class first and falls back to the opposite
 * class so a publisher disagreeing with the master's expected
 * checksum class still parses. On a fallback success the helper
 * updates @cur_enhanced to reflect what actually validated.
 *
 * Returns 0 on validation success, -EBADMSG on mismatch.
 */
int lin_uart_rx_validate(struct lin_uart *u);

/**
 * lin_uart_emit_bus_event - synthesise an rx-side bus-event frame.
 * @u:        state.
 * @lin_id:   ID for the event (or LIN_ID_NONE).
 * @flags:    LIN_F_* (LIN_F_ERR, LIN_F_WAKEUP, LIN_F_EVENT_COLLISION).
 * @err_mask: LIN_ERR_* mask when @flags includes LIN_F_ERR; else 0.
 *
 * Constructs the lin_frame, allocates an rx skb via alloc_lin_skb(),
 * and delivers it via netif_rx(). Used by the schedule engine for
 * NO_RESPONSE / CHECKSUM / collision notifications and by wakeup
 * detection. The frame carries no owner tags (bus-observed).
 */
void lin_uart_emit_bus_event(struct lin_uart *u, u8 lin_id, u32 flags,
			     u32 err_mask);

/**
 * lin_uart_deliver_rx - upcall the assembled rx_buff as a LIN frame.
 * @u: state.
 *
 * Builds the lin_frame from @rx_buff and the per-slot @cur_* tags;
 * delivers via lin_loopback_rx() when the frame was sourced locally
 * (master fire and/or local publisher) so RECV_OWN_MSGS gating
 * works, or via alloc_lin_skb()/netif_rx() when bus-observed.
 * Updates the netdev stats.
 */
void lin_uart_deliver_rx(struct lin_uart *u);

/**
 * lin_uart_slave_finish_rx - validate and deliver the in-flight slave frame.
 * @u: state.
 *
 * Validates the checksum and delivers via lin_uart_deliver_rx() (or
 * emits a checksum bus event on validation failure), then resets the
 * rx cursors for the next header. Caller holds @u->lock.
 *
 * Idempotent via the @header_received gate: returns silently when
 * called against an already-delivered or not-yet-completed frame.
 * Three call sites race to finish the same frame — the slave
 * parser's break-cancel branch, the parser's frame-complete branch,
 * and the frontend's rx-timer kthread handler — and the gate is
 * what guarantees a single delivery.
 */
void lin_uart_slave_finish_rx(struct lin_uart *u);

/* ----------------------------------------------------------------
 * TX path
 * ----------------------------------------------------------------
 */

/**
 * lin_uart_send_tx_buff - push tx_buff[@tx_cnt..@tx_lim] to the bus.
 * @u: state.
 *
 * Drives the transport's @io->write hook until either the buffer is
 * drained or the transport has no more room. When the transport
 * stalls partway, the helper arms the wakeup hook (@io->tx_wakeup_arm)
 * so the frontend's write_wakeup will be invoked once space frees.
 *
 * Returns 0 on success, -EIO if a @io->write callback returned a
 * negative error.
 *
 * Caller does NOT hold the per-link lock; the helper coordinates
 * re-entry via the LIN_UART_F_TXBUFF_RQ / LIN_UART_F_TXBUFF_INPR
 * atomic bit pair on @flags.
 */
int lin_uart_send_tx_buff(struct lin_uart *u);

/**
 * lin_uart_send_break - drive the LIN break / sync sequence.
 * @u: state.
 *
 * Asserts break via @io->break_ctl(-1), sleeps one character time at
 * the configured baud (>= 13 bit times per spec), releases break,
 * sleeps one bit time as the in-frame gap, then flushes the
 * transport's TX queue and seeds @tx_cnt so the next push starts at
 * the sync byte. Sets @state to LIN_UART_BREAK_SENT.
 *
 * Returns 0 on success or the errno propagated from break_ctl. On
 * break_ctl(0) failure the helper emits a netdev_warn and returns
 * the error so the caller can decide whether to retry.
 */
int lin_uart_send_break(struct lin_uart *u);

/**
 * lin_uart_write_wakeup - frontend's write_wakeup -> uart helper.
 * @u: state.
 *
 * Continues a previously stalled lin_uart_send_tx_buff() push. Call
 * from the frontend's TTY/serdev write_wakeup callback. The helper
 * disarms the transport's tx wakeup once the buffer drains.
 */
void lin_uart_write_wakeup(struct lin_uart *u);

/* ----------------------------------------------------------------
 * RX path
 * ----------------------------------------------------------------
 */

/**
 * lin_uart_receive_buf - frontend's receive_buf -> uart helper.
 * @u:     state.
 * @cp:    byte stream from the transport.
 * @fp:    parallel flag stream (NULL when transport has no
 *         per-byte error markers).
 * @count: number of bytes in @cp.
 *
 * Drives the byte state machine forward by @count bytes. Internally
 * dispatches to the master-side parser when @cur_master_emit is true
 * (we drove the header and expect to see it echoed back), or the
 * slave-side parser otherwise. Sets LIN_UART_F_RXEVENT,
 * LIN_UART_F_ERROR, or LIN_UART_F_TMOUTEVENT on @flags as
 * appropriate and wakes @wq.
 *
 * The slave parser consults @resp (the frontend-owned response
 * cache) under @lock when an external header is decoded; the
 * caller (e.g. TTY ldisc receive_buf, serdev receive_buf) does NOT
 * need to hold the lock.
 */
void lin_uart_receive_buf(struct lin_uart *u, const u8 *cp, const u8 *fp,
			  size_t count);

#endif /* _LIN_UART_H */
