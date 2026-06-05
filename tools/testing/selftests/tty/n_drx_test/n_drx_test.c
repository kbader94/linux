// SPDX-License-Identifier: GPL-2.0
/*
 * n_drx_test - direct-RX RTT measurement line discipline (test only).
 *
 * Companion to the userspace drx_rtt program. Drives end-to-end
 * latency measurement from a byte being committed to a TTY port's
 * flip buffer to that byte arriving at receive_buf(), under two
 * regimes selectable at runtime:
 *
 *   - DRX_TEST_MODE_OFF: receive_buf() fires from the normal flush
 *     workqueue (flush_to_ldisc() scheduled on system_dfl_wq).
 *
 *   - DRX_TEST_MODE_ON: the ldisc opts in to the TTY direct-RX path
 *     via tty_port_enable_direct_rx() and spawns a SCHED_FIFO kthread
 *     that drains the flip buffer via tty_port_drain_flip_buffer() in
 *     its own context. The workqueue still runs in parallel as the
 *     fallback; it typically loses the race and finds the buffer
 *     already drained.
 *
 * Both modes drive bytes through the same receive_buf() hook, which
 * timestamps each invocation with ktime_get_ns() and exposes the
 * stamp to userspace via a misc character device (/dev/n_drx_test).
 * A paired measurement of mode-off vs mode-on with everything else
 * held constant isolates the direct-RX latency benefit cleanly.
 *
 * Usage: attach to a UART (with external TX-to-RX loopback) via
 * ioctl(tty_fd, TIOCSETD, &N_DEVELOPMENT); see drx_rtt.c.
 *
 * The N_DEVELOPMENT (29) line discipline slot is documented in
 * include/uapi/linux/tty.h as "Manual out-of-tree testing", which is
 * exactly this module's purpose.
 */

#define pr_fmt(fmt) "n_drx_test: " fmt

#include <linux/atomic.h>
#include <linux/fs.h>
#include <linux/kthread.h>
#include <linux/ktime.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/poll.h>
#include <linux/sched/types.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/tty_ldisc.h>
#include <linux/uaccess.h>
#include <linux/wait.h>

#include "n_drx_test_uapi.h"

static int ldisc_num = N_DEVELOPMENT;
module_param(ldisc_num, int, 0444);
MODULE_PARM_DESC(ldisc_num,
		 "TTY line discipline number to register (default N_DEVELOPMENT = 29)");

/*
 * Per-attach state. Only one TTY may be attached at a time; the test
 * program is the only consumer and serialises by usage discipline.
 */
struct drx_test_port {
	struct tty_struct	*tty;
	unsigned int		mode;

	/* Event delivery to userspace via /dev/n_drx_test. */
	spinlock_t		event_lock;
	wait_queue_head_t	event_wq;
	atomic_long_t		event_seq;
	ktime_t			last_rx;
	atomic_t		rx_count;

	/* Direct-RX kthread, only spawned in MODE_ON. */
	struct task_struct	*kthread;

	struct miscdevice	miscdev;
};

/*
 * Singleton: one attached port at a time. Module load registers
 * neither the ldisc nor the misc cdev for any port; both come up when
 * a TTY does TIOCSETD(N_DEVELOPMENT), and both go down when it
 * switches back to N_TTY.
 */
static struct drx_test_port *current_port;
static DEFINE_MUTEX(current_port_lock);

/*
 * Direct-RX consumer kthread. Waits on the TTY's reader_wait queue
 * (woken by tty_flip_buffer_push() when reader_enabled is set),
 * drains the flip buffer in its own SCHED_FIFO context, and refreshes
 * its token so the next wait is correctly armed.
 *
 * Each iteration's drain forwards bytes through receive_buf() in this
 * kthread's context, which is exactly the latency win we want to
 * measure: receive_buf() runs as soon as we wake, not after a
 * workqueue's scheduler pick.
 */
static int drx_test_kthread_fn(void *data)
{
	struct drx_test_port *port = data;
	struct tty_port *tp = port->tty->port;
	tty_rx_token_t tok;

	sched_set_fifo(current);
	tok = tty_port_rx_token(tp);

	while (!kthread_should_stop()) {
		wait_event(*tty_port_rx_waitqueue(tp),
			   kthread_should_stop() ||
			   tty_port_rx_pending(tp, tok));

		if (kthread_should_stop())
			break;

		tty_port_drain_flip_buffer(tp, SIZE_MAX);
		tok = tty_port_rx_token(tp);
	}

	return 0;
}

/* ------------------------------------------------------------------ */
/* Misc cdev: /dev/n_drx_test                                          */
/* ------------------------------------------------------------------ */

struct drx_test_reader {
	struct drx_test_port	*port;
	long			reader_seq;
};

static int drx_test_miscdev_open(struct inode *inode, struct file *file)
{
	struct miscdevice *misc = file->private_data;
	struct drx_test_port *port = container_of(misc, struct drx_test_port,
						  miscdev);
	struct drx_test_reader *r;

	r = kzalloc(sizeof(*r), GFP_KERNEL);
	if (!r)
		return -ENOMEM;
	r->port = port;
	r->reader_seq = 0;
	file->private_data = r;
	return 0;
}

static int drx_test_miscdev_release(struct inode *inode, struct file *file)
{
	kfree(file->private_data);
	return 0;
}

static ssize_t drx_test_miscdev_read(struct file *file, char __user *buf,
				     size_t count, loff_t *ppos)
{
	struct drx_test_reader *r = file->private_data;
	struct drx_test_port *port = r->port;
	struct drx_test_event event;
	long saved_seq;
	int ret;

	if (count < sizeof(event))
		return -EINVAL;

	saved_seq = r->reader_seq;
	ret = wait_event_interruptible(port->event_wq,
				       atomic_long_read(&port->event_seq) !=
				       saved_seq);
	if (ret)
		return ret;

	spin_lock(&port->event_lock);
	event.rx_ktime_ns = ktime_to_ns(port->last_rx);
	event._pad = 0;
	event.rx_count = atomic_read(&port->rx_count);
	event.seq = atomic_long_read(&port->event_seq);
	spin_unlock(&port->event_lock);

	r->reader_seq = event.seq;

	if (copy_to_user(buf, &event, sizeof(event)))
		return -EFAULT;
	return sizeof(event);
}

static __poll_t drx_test_miscdev_poll(struct file *file, poll_table *wait)
{
	struct drx_test_reader *r = file->private_data;
	struct drx_test_port *port = r->port;

	poll_wait(file, &port->event_wq, wait);
	if (atomic_long_read(&port->event_seq) != r->reader_seq)
		return EPOLLIN | EPOLLRDNORM;
	return 0;
}

static long drx_test_set_mode(struct drx_test_port *port, unsigned int mode)
{
	struct tty_port *tp = port->tty->port;
	struct task_struct *th;

	if (mode != DRX_TEST_MODE_OFF && mode != DRX_TEST_MODE_ON)
		return -EINVAL;
	if (mode == port->mode)
		return 0;

	if (mode == DRX_TEST_MODE_ON) {
		tty_port_enable_direct_rx(tp);
		th = kthread_run(drx_test_kthread_fn, port, "drx_test");
		if (IS_ERR(th)) {
			tty_port_disable_direct_rx(tp);
			return PTR_ERR(th);
		}
		port->kthread = th;
	} else {
		if (port->kthread) {
			kthread_stop(port->kthread);
			port->kthread = NULL;
		}
		tty_port_disable_direct_rx(tp);
	}
	port->mode = mode;
	return 0;
}

static long drx_test_miscdev_ioctl(struct file *file, unsigned int cmd,
				   unsigned long arg)
{
	struct drx_test_reader *r = file->private_data;
	struct drx_test_port *port = r->port;
	unsigned int mode;

	switch (cmd) {
	case DRX_TEST_IOC_SET_MODE:
		if (copy_from_user(&mode, (void __user *)arg, sizeof(mode)))
			return -EFAULT;
		return drx_test_set_mode(port, mode);

	case DRX_TEST_IOC_GET_MODE:
		mode = port->mode;
		if (copy_to_user((void __user *)arg, &mode, sizeof(mode)))
			return -EFAULT;
		return 0;

	case DRX_TEST_IOC_RESET:
		spin_lock(&port->event_lock);
		port->last_rx = 0;
		atomic_set(&port->rx_count, 0);
		atomic_long_set(&port->event_seq, 0);
		r->reader_seq = 0;
		spin_unlock(&port->event_lock);
		return 0;

	default:
		return -ENOTTY;
	}
}

static const struct file_operations drx_test_miscdev_fops = {
	.owner		= THIS_MODULE,
	.open		= drx_test_miscdev_open,
	.release	= drx_test_miscdev_release,
	.read		= drx_test_miscdev_read,
	.poll		= drx_test_miscdev_poll,
	.unlocked_ioctl	= drx_test_miscdev_ioctl,
};

/* ------------------------------------------------------------------ */
/* TTY ldisc                                                           */
/* ------------------------------------------------------------------ */

static int drx_test_ldisc_open(struct tty_struct *tty)
{
	struct drx_test_port *port;
	int err;

	mutex_lock(&current_port_lock);
	if (current_port) {
		mutex_unlock(&current_port_lock);
		return -EBUSY;
	}

	port = kzalloc(sizeof(*port), GFP_KERNEL);
	if (!port) {
		mutex_unlock(&current_port_lock);
		return -ENOMEM;
	}

	port->tty = tty;
	port->mode = DRX_TEST_MODE_OFF;
	spin_lock_init(&port->event_lock);
	init_waitqueue_head(&port->event_wq);
	atomic_long_set(&port->event_seq, 0);
	atomic_set(&port->rx_count, 0);

	port->miscdev.minor	= MISC_DYNAMIC_MINOR;
	port->miscdev.name	= "n_drx_test";
	port->miscdev.fops	= &drx_test_miscdev_fops;

	err = misc_register(&port->miscdev);
	if (err) {
		kfree(port);
		mutex_unlock(&current_port_lock);
		return err;
	}

	tty->disc_data = port;
	current_port = port;
	mutex_unlock(&current_port_lock);
	return 0;
}

static void drx_test_ldisc_close(struct tty_struct *tty)
{
	struct drx_test_port *port = tty->disc_data;
	struct tty_port *tp = tty->port;

	if (!port)
		return;

	if (port->mode == DRX_TEST_MODE_ON) {
		if (port->kthread) {
			kthread_stop(port->kthread);
			port->kthread = NULL;
		}
		tty_port_disable_direct_rx(tp);
		port->mode = DRX_TEST_MODE_OFF;
	}

	misc_deregister(&port->miscdev);

	mutex_lock(&current_port_lock);
	current_port = NULL;
	mutex_unlock(&current_port_lock);

	tty->disc_data = NULL;
	kfree(port);
}

/*
 * The measurement point. Called from either the workqueue
 * (flush_to_ldisc) or the direct-RX kthread (via
 * tty_port_drain_flip_buffer); both go through the same receive_buf
 * pathway in tty_buffer.c, so the timestamping is identical and any
 * delta between modes is the direct-RX wake/drain benefit.
 */
static void drx_test_ldisc_receive_buf(struct tty_struct *tty, const u8 *cp,
				       const u8 *fp, size_t count)
{
	struct drx_test_port *port = tty->disc_data;
	ktime_t now = ktime_get();
	unsigned long flags;

	if (!port)
		return;

	spin_lock_irqsave(&port->event_lock, flags);
	port->last_rx = now;
	atomic_add(count, &port->rx_count);
	atomic_long_inc(&port->event_seq);
	spin_unlock_irqrestore(&port->event_lock, flags);

	wake_up(&port->event_wq);
}

static struct tty_ldisc_ops drx_test_ldisc = {
	.owner		= THIS_MODULE,
	.num		= N_DEVELOPMENT,
	.name		= "n_drx_test",
	.open		= drx_test_ldisc_open,
	.close		= drx_test_ldisc_close,
	.receive_buf	= drx_test_ldisc_receive_buf,
};

static int __init n_drx_test_init(void)
{
	int err;

	drx_test_ldisc.num = ldisc_num;
	err = tty_register_ldisc(&drx_test_ldisc);
	if (err)
		pr_err("tty_register_ldisc(%d) failed: %d\n", ldisc_num, err);
	else
		pr_info("registered on ldisc slot %d\n", ldisc_num);
	return err;
}
module_init(n_drx_test_init);

static void __exit n_drx_test_exit(void)
{
	tty_unregister_ldisc(&drx_test_ldisc);
}
module_exit(n_drx_test_exit);

MODULE_DESCRIPTION("TTY direct-RX RTT measurement line discipline");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Kyle Bader <kyle.bader94@gmail.com>");
