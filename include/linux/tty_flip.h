/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _LINUX_TTY_FLIP_H
#define _LINUX_TTY_FLIP_H

#include <linux/tty_buffer.h>
#include <linux/tty_port.h>

struct tty_ldisc;

int tty_buffer_set_limit(struct tty_port *port, int limit);
unsigned int tty_buffer_space_avail(struct tty_port *port);
int tty_buffer_request_room(struct tty_port *port, size_t size);
size_t __tty_insert_flip_string_flags(struct tty_port *port, const u8 *chars,
				      const u8 *flags, bool mutable_flags,
				      size_t size);
size_t tty_prepare_flip_string(struct tty_port *port, u8 **chars, size_t size);
void tty_flip_buffer_push(struct tty_port *port);

/**
 * tty_insert_flip_string_fixed_flag - add characters to the tty buffer
 * @port: tty port
 * @chars: characters
 * @flag: flag value for each character
 * @size: size
 *
 * Queue a series of bytes to the tty buffering. All the characters passed are
 * marked with the supplied flag.
 *
 * Returns: the number added.
 */
static inline size_t tty_insert_flip_string_fixed_flag(struct tty_port *port,
						       const u8 *chars, u8 flag,
						       size_t size)
{
	return __tty_insert_flip_string_flags(port, chars, &flag, false, size);
}

/**
 * tty_insert_flip_string_flags - add characters to the tty buffer
 * @port: tty port
 * @chars: characters
 * @flags: flag bytes
 * @size: size
 *
 * Queue a series of bytes to the tty buffering. For each character the flags
 * array indicates the status of the character.
 *
 * Returns: the number added.
 */
static inline size_t tty_insert_flip_string_flags(struct tty_port *port,
						  const u8 *chars,
						  const u8 *flags, size_t size)
{
	return __tty_insert_flip_string_flags(port, chars, flags, true, size);
}

/**
 * tty_insert_flip_char - add one character to the tty buffer
 * @port: tty port
 * @ch: character
 * @flag: flag byte
 *
 * Queue a single byte @ch to the tty buffering, with an optional flag.
 */
static inline size_t tty_insert_flip_char(struct tty_port *port, u8 ch, u8 flag)
{
	struct tty_buffer *tb = port->buf.tail;
	int change;

	change = !tb->flags && (flag != TTY_NORMAL);
	if (!change && tb->used < tb->size) {
		if (tb->flags)
			*flag_buf_ptr(tb, tb->used) = flag;
		*char_buf_ptr(tb, tb->used++) = ch;
		return 1;
	}
	return __tty_insert_flip_string_flags(port, &ch, &flag, false, 1);
}

static inline size_t tty_insert_flip_string(struct tty_port *port,
					    const u8 *chars, size_t size)
{
	return tty_insert_flip_string_fixed_flag(port, chars, TTY_NORMAL, size);
}

size_t tty_ldisc_receive_buf(struct tty_ldisc *ld, const u8 *p, const u8 *f,
			     size_t count);

void tty_buffer_lock_exclusive(struct tty_port *port);
void tty_buffer_unlock_exclusive(struct tty_port *port);

/*
 * Direct-RX flip-buffer drain for opt-in non-terminal consumers.
 *
 * The normal flip-buffer flow is asynchronous: drivers commit bytes
 * with tty_flip_buffer_push() and a workqueue later runs
 * flush_to_ldisc() which calls @port->client_ops->receive_buf(). For
 * latency-tolerant terminal-style consumers this is fine. For
 * non-terminal consumers — serdev-attached packetised protocols, LIN
 * and other industrial buses, HCI-style packetised UART transports,
 * GNSS/PPS-adjacent paths — the workqueue's scheduling latency can
 * dominate end-to-end RX latency, particularly on single-CPU systems
 * under load.
 *
 * Direct-RX lets such a consumer arrange to be notified at IRQ-time
 * commit and drain the flip buffer from its own scheduling context.
 * The wake/drain path runs alongside the existing workqueue path; the
 * workqueue remains the unconditional fallback for slow, absent, or
 * unregistered consumers. Net effect when active: tty_flip_buffer_push
 * adds one wake_up() (a no-op when the wait queue is empty); the
 * registered consumer can typically drain bytes in tens of µs rather
 * than waiting on the system_dfl_wq pick. Net effect when inactive:
 * one branch on a boolean flag, no other behaviour change.
 *
 * Lifetime: @reader_wait and @reader_seq are embedded in tty_bufhead,
 * so a stale IRQ-context wake_up after a process-context disable is
 * harmless — there is no external pointer to dereference.
 */

/* Opaque cursor a consumer captures before waiting and replays in the
 * wait predicate to spot new commits. Treated as an opaque scalar; do
 * not order or arithmetic-compare. Compare only with !=.
 */
typedef unsigned long tty_rx_token_t;

/*
 * tty_port_drain_flip_buffer - drain committed flip-buffer bytes to the
 *                              registered client in caller's context.
 * @port:   tty port.
 * @budget: maximum bytes to forward this call. Pass SIZE_MAX to drain
 *          until the buffer is empty (the normal workqueue path uses
 *          this). Smaller budgets bound the per-call work for RT
 *          consumers so a backlog cannot monopolise a SCHED_FIFO
 *          kthread; combined with the bounded cross-context wait on
 *          @port->buf.lock, the worst-case per-call time is bounded
 *          regardless of who else is draining.
 *
 * Process context only; may sleep. Serialises against the workqueue
 * via @port->buf.lock. Returns the number of bytes forwarded to
 * @port->client_ops->receive_buf().
 */
int tty_port_drain_flip_buffer(struct tty_port *port, size_t budget);

/* Enable / disable direct-RX wakes for @port. Must be balanced.
 * Disable before freeing any state the consumer's wait predicate
 * touches; subsequent IRQ-context tty_flip_buffer_push() calls become
 * pure no-ops with respect to the direct-RX path (still scheduling
 * the workqueue).
 */
void tty_port_enable_direct_rx(struct tty_port *port);
void tty_port_disable_direct_rx(struct tty_port *port);

/* Capture the producer cursor for the wait predicate. */
tty_rx_token_t tty_port_rx_token(struct tty_port *port);

/* Predicate helper: true iff the cursor has advanced since @since. */
bool tty_port_rx_pending(struct tty_port *port, tty_rx_token_t since);

/* The per-port wait queue producers wake when direct-RX is enabled.
 * Consumers wait on this directly; the queue is embedded in
 * tty_bufhead, so a returned pointer is valid for the lifetime of the
 * tty_port.
 */
wait_queue_head_t *tty_port_rx_waitqueue(struct tty_port *port);

#endif /* _LINUX_TTY_FLIP_H */
