/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
/*
 * Shared header between n_drx_test.ko (test line discipline) and
 * drx_rtt (userspace measurement program).
 */
#ifndef _N_DRX_TEST_UAPI_H
#define _N_DRX_TEST_UAPI_H

#include <linux/types.h>
#include <linux/ioctl.h>

/*
 * Event delivered by /dev/n_drx_test on each blocking read. One event
 * per receive_buf() invocation in the test ldisc.
 *
 * @rx_ktime_ns: kernel CLOCK_MONOTONIC nanosecond timestamp captured
 *               at the entry of receive_buf(). Directly comparable to
 *               a userspace clock_gettime(CLOCK_MONOTONIC) sample.
 * @rx_count:    cumulative bytes seen by receive_buf() since the
 *               last DRX_TEST_IOC_RESET.
 * @seq:         monotonic event counter; the read() side uses this to
 *               coalesce multiple receive_buf() invocations between
 *               reads.
 */
struct drx_test_event {
	__u64	rx_ktime_ns;
	__u32	rx_count;
	__u32	_pad;
	__u64	seq;
};

/*
 * Mode controls whether the test ldisc participates in the TTY's
 * direct-RX path. Both modes funnel through the same receive_buf()
 * timestamping; only the wake/drain plumbing that gets a byte to
 * receive_buf() differs.
 */
#define DRX_TEST_MODE_OFF	0	/* workqueue path (flush_to_ldisc) */
#define DRX_TEST_MODE_ON	1	/* direct-RX path (kthread drain) */

#define DRX_TEST_IOC_MAGIC	'D'
#define DRX_TEST_IOC_SET_MODE	_IOW(DRX_TEST_IOC_MAGIC, 1, unsigned int)
#define DRX_TEST_IOC_GET_MODE	_IOR(DRX_TEST_IOC_MAGIC, 2, unsigned int)
#define DRX_TEST_IOC_RESET	_IO(DRX_TEST_IOC_MAGIC,  3)

#endif /* _N_DRX_TEST_UAPI_H */
