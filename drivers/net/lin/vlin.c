// SPDX-License-Identifier: (GPL-2.0 OR BSD-3-Clause)
/*
 * vlin.c - Virtual LIN interface
 *
 * Software-only LIN interface, the LIN analogue of vcan. The interface
 * is its own bus: a master socket's schedule drives headers, registered
 * publishers supply responses, and every resulting frame is looped back
 * to the sockets bound to the interface via the LIN core. There is no
 * hardware and no wire timing — frames are delivered cooked — but the
 * schedule's per-slot duration (slot_us) is honoured so an activated
 * schedule produces paced, observable traffic.
 *
 * Author: Kyle Bader <kyle.bader94@gmail.com>
 * Copyright (c) 2026 Kyle Bader
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/completion.h>
#include <linux/if_arp.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/overflow.h>
#include <linux/skbuff.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>
#include <linux/lin.h>
#include <linux/lin/dev.h>
#include <linux/lin/error.h>
#include <net/rtnetlink.h>

#define DRV_NAME "vlin"

/* Per-ID response state: the bytes a publisher registered for an ID,
 * which the engine emits when a slot for that ID fires. @dirty tracks
 * "updated since last emit" for sporadic/event member selection — set
 * here on every set_response, consumed by later commits.
 */
struct vlin_response {
	u8	data[LIN_MAX_DLEN];
	u8	len;
	bool	enhanced;
	bool	present;
	bool	dirty;
};

struct vlin_priv {
	struct net_device	*dev;

	/* Guards the response table, the loaded-schedule pointers, and the
	 * engine run state against the schedule engine (a delayed_work that
	 * runs asynchronously to the lin_dev_ops, which the LIN core
	 * serialises under its own policy_lock).
	 */
	spinlock_t		lock;

	struct vlin_response	resp[LIN_ID_MASK + 1];

	/* Deep copies of loaded schedules, indexed by handle. */
	struct lin_schedule	*sched[LIN_RAW_SCHEDULES_MAX];

	int			active;		/* active handle, or -1 */
	unsigned int		slot;		/* next slot in active sched */
	bool			running;	/* engine should keep firing */

	/* Deferred schedule activation. The core's activate contract requires
	 * the switch to wait for the in-flight slot to finish; rather than
	 * cancel the engine mid-slot, schedule_activate() records the target
	 * in @activate_to / @activate_req and blocks on @activate_done. The
	 * engine applies the switch at its next slot boundary (after the
	 * current slot's slot_us has elapsed) and completes the waiter.
	 */
	bool			activate_req;
	u8			activate_to;
	struct completion	activate_done;

	struct delayed_work	engine;
};

/* --- frame emission --- */

/* Account one frame on the virtual bus. vlin is the whole bus rather than a
 * single node, so every frame it generates is both put on the bus (tx) and
 * taken off it (rx) — the same convention vcan uses for a virtual loopback
 * interface. Counted at emission, so a frame the loopback gate later drops
 * (no interested socket) still shows as bus traffic, matching reality on a
 * shared wire. Per-node master/slave tx-vs-rx attribution is a property of a
 * single node's interface, which vlin is not; a per-node driver (hardware,
 * or a future connected-pair vxlin) follows that contract instead — see
 * Documentation/networking/lin.rst.
 */
static void vlin_count_frame(struct net_device *dev, u8 len)
{
	struct net_device_stats *stats = &dev->stats;

	stats->tx_packets++;
	stats->tx_bytes += len;
	stats->rx_packets++;
	stats->rx_bytes += len;
}

/* Header fired for @lin_id, nobody answered: deliver a NO_RESPONSE error
 * frame. It is a bus-observed condition, not data a local socket put on
 * the bus, so it goes through the plain rx path with NULL owner tags
 * (always delivered to error subscribers, never gated by RECV_OWN_MSGS).
 */
static void vlin_emit_no_response(struct net_device *dev, u8 lin_id)
{
	struct lin_frame f = {
		.lin_id   = lin_id,
		.flags    = LIN_F_ERR,
		.len      = 0,
		.err_mask = LIN_ERR_NO_RESPONSE,
	};
	struct sk_buff *skb = alloc_lin_skb(dev, &f);

	vlin_count_frame(dev, 0);
	if (skb)
		netif_rx(skb);
}

/* Successful data frame: header @header_id carrying @r's response bytes,
 * tagged MASTER|PUBLISHER so the core attributes it to the master that
 * scheduled the header and the publisher that owns the response, and
 * honours LIN_RAW_RECV_OWN_MSGS for both. @resp_id selects which
 * publisher to tag (the ID the response data belongs to); it equals
 * @header_id for the frame types handled here.
 */
static void vlin_emit_response(struct net_device *dev, u8 header_id, u8 resp_id,
			       const struct vlin_response *r)
{
	struct lin_frame f = {
		.lin_id = header_id,
		.flags  = r->enhanced ? LIN_F_CHK_ENH : 0,
		.len    = r->len,
	};

	memcpy(f.data, r->data, r->len);
	vlin_count_frame(dev, r->len);
	lin_loopback_rx(dev, &f, LIN_EMIT_MASTER | LIN_EMIT_PUBLISHER, resp_id);
}

/* --- schedule engine --- */

/* Duration of @e in jiffies (at least one tick). */
static unsigned long vlin_slot_delay(const struct lin_schedule *s,
				     const struct lin_schedule_entry *e)
{
	u32 us = e->slot_us ? e->slot_us : s->default_slot_us;
	unsigned long j = usecs_to_jiffies(us);

	return j ? j : 1;
}

/* Run one unconditional / diagnostic slot: fire the header for its
 * single member and emit the registered response, or a NO_RESPONSE
 * error if no publisher owns the ID. Called with vp->lock held; the
 * emit helpers only queue an skb (GFP_ATOMIC) to netif_rx and never
 * sleep, so holding the lock across them is safe.
 */
static void vlin_run_uncond(struct vlin_priv *vp,
			    const struct lin_schedule_entry *e)
{
	u8 id = e->members[0] & LIN_ID_MASK;
	struct vlin_response *r = &vp->resp[id];

	lockdep_assert_held(&vp->lock);

	if (r->present) {
		r->dirty = false;
		vlin_emit_response(vp->dev, id, id, r);
	} else {
		vlin_emit_no_response(vp->dev, id);
	}
}

static void vlin_engine_work(struct work_struct *w)
{
	struct vlin_priv *vp = container_of(to_delayed_work(w),
					    struct vlin_priv, engine);
	const struct lin_schedule *s;
	const struct lin_schedule_entry *e;
	unsigned long delay;

	spin_lock(&vp->lock);

	/* Apply a deferred activation at this slot boundary: the previous
	 * slot's slot_us has elapsed (we were re-armed after it), so switching
	 * here honours the activate-at-boundary contract. Wake the blocked
	 * schedule_activate() caller once the switch is in place.
	 */
	if (vp->running && vp->activate_req) {
		vp->active = vp->activate_to;
		vp->slot = 0;
		vp->activate_req = false;
		complete(&vp->activate_done);
	}

	if (!vp->running || vp->active < 0 || !vp->sched[vp->active]) {
		spin_unlock(&vp->lock);
		return;
	}

	s = vp->sched[vp->active];
	if (vp->slot >= s->entry_count)
		vp->slot = 0;
	e = &s->entry[vp->slot];

	switch (e->type) {
	case LIN_SCHED_TYPE_UNCOND:
	case LIN_SCHED_TYPE_DIAG:
		vlin_run_uncond(vp, e);
		break;
	default:
		/* TYPE_SPORADIC / TYPE_EVENT are not advertised in caps yet,
		 * so the core rejects them at load; later commits add their
		 * handling here.
		 */
		break;
	}

	/* Re-arm for the next slot after this slot's duration. */
	delay = vlin_slot_delay(s, e);
	if (++vp->slot >= s->entry_count)
		vp->slot = 0;
	if (vp->running)
		schedule_delayed_work(&vp->engine, delay);

	spin_unlock(&vp->lock);
}

/* Stop the engine and forget the active schedule. Safe to call with no
 * schedule active. Runs from the lin_dev_ops (under the core's
 * policy_lock, a mutex), so the sleeping cancel is fine; the work
 * callback never takes policy_lock, so there is no deadlock.
 */
static void vlin_engine_halt(struct vlin_priv *vp)
{
	spin_lock(&vp->lock);
	vp->running = false;
	vp->active = -1;
	vp->activate_req = false;
	spin_unlock(&vp->lock);

	cancel_delayed_work_sync(&vp->engine);
}

/* --- lin_dev_ops --- */

static int vlin_master_start(struct lin_dev *ld)
{
	/* Nothing to enable up front; the engine starts on schedule
	 * activation.
	 */
	return 0;
}

static int vlin_master_stop(struct lin_dev *ld)
{
	vlin_engine_halt(netdev_priv(ld->dev));
	return 0;
}

static int vlin_set_response(struct lin_dev *ld, u8 lin_id,
			     const u8 *data, u8 len, bool enhanced_checksum)
{
	struct vlin_priv *vp = netdev_priv(ld->dev);
	u8 id = lin_id & LIN_ID_MASK;

	spin_lock(&vp->lock);
	memcpy(vp->resp[id].data, data, len);
	vp->resp[id].len = len;
	vp->resp[id].enhanced = enhanced_checksum;
	vp->resp[id].present = true;
	vp->resp[id].dirty = true;
	spin_unlock(&vp->lock);
	return 0;
}

static int vlin_clear_response(struct lin_dev *ld, u8 lin_id)
{
	struct vlin_priv *vp = netdev_priv(ld->dev);
	u8 id = lin_id & LIN_ID_MASK;

	spin_lock(&vp->lock);
	vp->resp[id].present = false;
	vp->resp[id].dirty = false;
	spin_unlock(&vp->lock);
	return 0;
}

static int vlin_schedule_load(struct lin_dev *ld,
			      const struct lin_schedule *sched)
{
	struct vlin_priv *vp = netdev_priv(ld->dev);
	struct lin_schedule *copy, *old;

	copy = kmemdup(sched, struct_size(sched, entry, sched->entry_count),
		       GFP_KERNEL);
	if (!copy)
		return -ENOMEM;

	/* The core rejects (re)loading the active handle, so @copy never
	 * replaces a schedule the engine is currently running.
	 */
	spin_lock(&vp->lock);
	old = vp->sched[sched->handle];
	vp->sched[sched->handle] = copy;
	spin_unlock(&vp->lock);

	kfree(old);
	return 0;
}

static int vlin_schedule_delete(struct lin_dev *ld, u8 handle)
{
	struct vlin_priv *vp = netdev_priv(ld->dev);
	struct lin_schedule *old;

	/* The core rejects deleting the active handle and stops the engine
	 * before the forced release-all delete, so the engine is not
	 * running this handle here.
	 */
	spin_lock(&vp->lock);
	old = vp->sched[handle];
	vp->sched[handle] = NULL;
	spin_unlock(&vp->lock);

	kfree(old);
	return 0;
}

static int vlin_schedule_activate(struct lin_dev *ld, u8 handle)
{
	struct vlin_priv *vp = netdev_priv(ld->dev);
	unsigned long timeout;

	spin_lock(&vp->lock);

	/* No schedule in flight: start the new one from slot 0 immediately,
	 * matching a master beginning a schedule. Nothing to wait for.
	 */
	if (!vp->running) {
		vp->active = handle;
		vp->slot = 0;
		vp->running = true;
		schedule_delayed_work(&vp->engine, 0);
		spin_unlock(&vp->lock);
		return 0;
	}

	/* A schedule is running: the core contract says activation waits for
	 * the in-flight slot to finish rather than cutting it short. Record
	 * the request; the engine switches at its next slot boundary and
	 * completes us.
	 *
	 * Bound the wait by the maximum possible slot duration plus a margin,
	 * not by the active schedule's slots: during an event-collision
	 * diversion vp->active names a different schedule than the one whose
	 * slot delay is currently armed, so a per-schedule bound could expire
	 * before a healthy engine reaches the boundary. The absolute ceiling
	 * is always >= the armed delay, so -ETIMEDOUT means a wedged workqueue.
	 */
	reinit_completion(&vp->activate_done);
	vp->activate_to = handle;
	vp->activate_req = true;
	timeout = usecs_to_jiffies(LIN_RAW_SCHEDULE_SLOT_MAX_US) +
		  msecs_to_jiffies(100);
	spin_unlock(&vp->lock);

	if (!wait_for_completion_timeout(&vp->activate_done, timeout)) {
		int ret = -ETIMEDOUT;

		/* The wait gave up, but the engine sets vp->active and clears
		 * @activate_req before it completes us, so it may have applied
		 * the switch in the same instant the timeout fired. Re-check
		 * under the lock: if the request was consumed and we are now on
		 * @handle, the activation really happened — report success so
		 * the core's active_schedule stays in sync. Otherwise drop the
		 * stale request and report the timeout.
		 */
		spin_lock(&vp->lock);
		if (!vp->activate_req && vp->active == handle)
			ret = 0;
		else
			vp->activate_req = false;
		spin_unlock(&vp->lock);
		return ret;
	}
	return 0;
}

static int vlin_schedule_stop(struct lin_dev *ld)
{
	/* Immediate halt is correct here only because vlin frames are atomic:
	 * each slot is a single netif_rx() / lin_loopback_rx(), so there is no
	 * partially-transmitted slot to corrupt and a following sleep / wakeup
	 * / header_send cannot interleave with one. A driver with byte-level TX
	 * (e.g. a UART or SPI LIN bridge) must instead quiesce at the next slot
	 * boundary per the schedule_stop contract, letting the in-flight slot's
	 * bytes drain before the bus is reused.
	 */
	vlin_engine_halt(netdev_priv(ld->dev));
	return 0;
}

static int vlin_header_send(struct lin_dev *ld, u8 lin_id,
			    const u8 *data, u8 len, bool enhanced_checksum)
{
	struct vlin_priv *vp = netdev_priv(ld->dev);
	u8 id = lin_id & LIN_ID_MASK;
	struct vlin_response snap;
	bool present;

	/* The core only reaches header_send with no schedule active, so the
	 * engine is idle and cannot race this emission.
	 */
	if (len > 0) {
		/* Write transaction: the master supplies the data (e.g. the
		 * LIN_RAW_SLEEP go-to-sleep frame on 0x3C).
		 */
		struct lin_frame f = {
			.lin_id = lin_id,
			.flags  = enhanced_checksum ? LIN_F_CHK_ENH : 0,
			.len    = len,
		};

		memcpy(f.data, data, len);
		vlin_count_frame(vp->dev, len);
		lin_loopback_rx(vp->dev, &f, LIN_EMIT_MASTER, lin_id);
		return 0;
	}

	/* Read transaction: emit the header and deliver the registered
	 * response, or NO_RESPONSE if no publisher owns the ID.
	 */
	spin_lock(&vp->lock);
	present = vp->resp[id].present;
	if (present) {
		snap = vp->resp[id];
		vp->resp[id].dirty = false;
	}
	spin_unlock(&vp->lock);

	if (present)
		vlin_emit_response(vp->dev, id, id, &snap);
	else
		vlin_emit_no_response(vp->dev, id);
	return 0;
}

static int vlin_wakeup_send(struct lin_dev *ld)
{
	struct lin_frame f = {
		.lin_id = LIN_ID_NONE,
		.flags  = LIN_F_WAKEUP,
		.len    = 0,
	};
	struct sk_buff *skb = alloc_lin_skb(ld->dev, &f);

	/* A wakeup is a bus event every node observes: deliver it via the
	 * plain rx path (owners NULL) so it reaches every LIN_RAW_WAKEUP_FILTER
	 * subscriber.
	 */
	vlin_count_frame(ld->dev, 0);
	if (skb)
		netif_rx(skb);
	return 0;
}

static const struct lin_dev_ops vlin_lin_ops = {
	.master_start		= vlin_master_start,
	.master_stop		= vlin_master_stop,
	.set_response		= vlin_set_response,
	.clear_response		= vlin_clear_response,
	.schedule_load		= vlin_schedule_load,
	.schedule_delete	= vlin_schedule_delete,
	.schedule_activate	= vlin_schedule_activate,
	.schedule_stop		= vlin_schedule_stop,
	.header_send		= vlin_header_send,
	.wakeup_send		= vlin_wakeup_send,
};

/* --- netdev ops --- */

static int vlin_open(struct net_device *dev)
{
	netif_carrier_on(dev);
	return 0;
}

static int vlin_stop(struct net_device *dev)
{
	netif_carrier_off(dev);
	/* The core's GOING_DOWN / master-release path already quiesced the
	 * engine; halt again defensively so nothing is left armed.
	 */
	vlin_engine_halt(netdev_priv(dev));
	return 0;
}

static netdev_tx_t vlin_xmit(struct sk_buff *skb, struct net_device *dev)
{
	/* LIN has no socket-to-wire transmit path: frames are produced by
	 * the schedule engine and publisher responses, not by
	 * dev_queue_xmit. Drop anything that reaches here (e.g. an
	 * AF_PACKET injection) rather than dereferencing it.
	 */
	kfree_skb(skb);
	dev->stats.tx_dropped++;
	return NETDEV_TX_OK;
}

static const struct net_device_ops vlin_netdev_ops = {
	.ndo_open	= vlin_open,
	.ndo_stop	= vlin_stop,
	.ndo_start_xmit	= vlin_xmit,
};

/* --- rtnl_link / module --- */

static void vlin_setup(struct net_device *dev)
{
	struct vlin_priv *vp;
	struct lin_dev *ld;

	lin_setup(dev);
	dev->netdev_ops = &vlin_netdev_ops;
	dev->needs_free_netdev = true;

	/* Initialise the embedded struct lin_dev (the rtnl core allocated
	 * the netdev, so alloc_lindev() is not on this path) and advertise
	 * the capabilities this driver implements.
	 */
	lin_dev_init(dev, &vlin_lin_ops, sizeof(struct vlin_priv));
	ld = lin_get_ml_priv(dev);
	ld->caps = LIN_CAP_DIAG | LIN_CAP_CHK_ENH | LIN_CAP_WAKEUP;

	vp = netdev_priv(dev);
	vp->dev = dev;
	spin_lock_init(&vp->lock);
	vp->active = -1;
	init_completion(&vp->activate_done);
	INIT_DELAYED_WORK(&vp->engine, vlin_engine_work);
}

static struct rtnl_link_ops vlin_link_ops __read_mostly = {
	.kind		= DRV_NAME,
	.priv_size	= ALIGN(sizeof(struct vlin_priv), NETDEV_ALIGN) +
			  sizeof(struct lin_dev),
	.setup		= vlin_setup,
};

static __init int vlin_init(void)
{
	return rtnl_link_register(&vlin_link_ops);
}

static __exit void vlin_exit(void)
{
	rtnl_link_unregister(&vlin_link_ops);
}

module_init(vlin_init);
module_exit(vlin_exit);

MODULE_DESCRIPTION("Virtual LIN interface");
MODULE_LICENSE("Dual BSD/GPL");
MODULE_AUTHOR("Kyle Bader <kyle.bader94@gmail.com>");
MODULE_ALIAS_RTNL_LINK(DRV_NAME);
