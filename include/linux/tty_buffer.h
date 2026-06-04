/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _LINUX_TTY_BUFFER_H
#define _LINUX_TTY_BUFFER_H

#include <linux/atomic.h>
#include <linux/llist.h>
#include <linux/mutex.h>
#include <linux/wait.h>
#include <linux/workqueue.h>

struct tty_buffer {
	union {
		struct tty_buffer *next;
		struct llist_node free;
	};
	unsigned int used;
	unsigned int size;
	unsigned int commit;
	unsigned int lookahead;		/* Lazy update on recv, can become less than "read" */
	unsigned int read;
	bool flags;
	/* Data points here */
	u8 data[] __aligned(sizeof(unsigned long));
};

static inline u8 *char_buf_ptr(struct tty_buffer *b, unsigned int ofs)
{
	return b->data + ofs;
}

static inline u8 *flag_buf_ptr(struct tty_buffer *b, unsigned int ofs)
{
	return char_buf_ptr(b, ofs) + b->size;
}

struct tty_bufhead {
	struct tty_buffer *head;	/* Queue head */
	struct work_struct work;
	struct mutex	   lock;
	atomic_t	   priority;
	struct tty_buffer sentinel;
	struct llist_head free;		/* Free queue head */
	atomic_t	   mem_used;    /* In-use buffers excluding free list */
	int		   mem_limit;
	struct tty_buffer *tail;	/* Active buffer */

	/* Opt-in early-RX notification for non-terminal consumers.
	 *
	 * Enabled by tty_port_enable_direct_rx(); when set, every
	 * tty_flip_buffer_push() increments @reader_seq and wakes
	 * @reader_wait in addition to scheduling the normal flip
	 * workqueue. A registered consumer (a kthread, an RT userspace
	 * reader, ...) waits on @reader_wait and drains the flip buffer
	 * via tty_port_drain_flip_buffer() in its own scheduling context,
	 * skipping the workqueue's scheduling latency on the fast path.
	 *
	 * @reader_seq is the producer-side cursor: the consumer saves a
	 * snapshot before waiting (tty_port_rx_token()), and the wait
	 * predicate compares the snapshot against the current value
	 * (tty_port_rx_pending()) — this makes the wake a state change
	 * the predicate can observe, not just a pulse.
	 *
	 * @reader_wait, @reader_seq, and @reader_enabled have the same
	 * lifetime as the tty_port (they are embedded here, not stored as
	 * external pointers), so an IRQ-context wake_up cannot UAF
	 * against a process-context teardown.
	 *
	 * The workqueue path remains unconditional — direct-RX is an
	 * optimisation, not a replacement: a slow or absent consumer
	 * still gets bytes delivered via the regular flush_to_ldisc.
	 */
	wait_queue_head_t  reader_wait;
	atomic_long_t	   reader_seq;
	bool		   reader_enabled;
};

/*
 * When a break, frame error, or parity error happens, these codes are
 * stuffed into the flags buffer.
 */
#define TTY_NORMAL	0
#define TTY_BREAK	1
#define TTY_FRAME	2
#define TTY_PARITY	3
#define TTY_OVERRUN	4

#endif
