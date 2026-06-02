// SPDX-License-Identifier: GPL-2.0
/*
 * drivers/net/lin/dev/skb.c - LIN skb construction and loopback synthesis.
 *
 * Hosts the device-side skb helpers every host-side LIN driver uses to
 * push frames at the LIN core: the bus-observed rx constructor
 * (alloc_lin_skb) and the owner-tagged loopback synthesiser
 * (lin_loopback_rx). Mirrors the role of drivers/net/can/dev/skb.c in
 * the SocketCAN stack.
 *
 * Also hosts lin-dev.ko's module metadata, since this is the only TU
 * in the module that carries unconditionally-built, exported helpers
 * every consumer touches.
 *
 * Author: Kyle Bader <kyle.bader94@gmail.com>
 * Copyright (c) 2026 Kyle Bader
 */

#include <linux/export.h>
#include <linux/if_arp.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/rcupdate.h>
#include <linux/skbuff.h>
#include <net/sock.h>

#include <linux/lin.h>
#include <linux/lin/core.h>
#include <linux/lin/dev.h>
#include <linux/lin/skb.h>

MODULE_DESCRIPTION("LIN bus driver helpers");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Kyle Bader <kyle.bader94@gmail.com>");

/* Drop the sock references taken by lin_loopback_rx() when the
 * synthesised dispatch skb is freed. The references were held for
 * the lifetime of the dispatch walk so that lin_raw_rcv() could
 * compare prv->master_owner / prv->publisher_owner against its own
 * sk safely under rcu_read_lock().
 */
static void lin_loopback_skb_destructor(struct sk_buff *skb)
{
	struct lin_skb_priv *prv = lin_skb_prv(skb);

	if (prv->master_owner)
		sock_put(prv->master_owner);
	if (prv->publisher_owner)
		sock_put(prv->publisher_owner);
}

/**
 * alloc_lin_skb - build an rx-shaped skb carrying a LIN frame
 * @dev:   the LIN netdev the frame is associated with
 * @frame: the LIN frame contents to copy into the skb
 *
 * Allocates and populates an skb in the shape the LIN rx path expects
 * (LIN_MTU, ETH_P_LIN, lin_skb_priv stamped with the ifindex), with the
 * ownership tags left NULL. This is the constructor for bus-sourced
 * frames a driver observed on the wire — received data and error
 * frames — which the driver hands to netif_rx() directly.
 * lin_loopback_rx() builds on this for the self-emission case and
 * additionally stamps the master/publisher owners. Mirrors
 * alloc_can_skb().
 *
 * Callable from process or softirq context; allocates with GFP_ATOMIC
 * via netdev_alloc_skb().
 *
 * Return: the new skb, or NULL on allocation failure.
 */
struct sk_buff *alloc_lin_skb(struct net_device *dev,
			      const struct lin_frame *frame)
{
	struct sk_buff *skb;
	struct lin_skb_priv *prv;

	skb = netdev_alloc_skb(dev, LIN_MTU);
	if (!skb)
		return NULL;

	skb_put_data(skb, frame, sizeof(*frame));
	skb->protocol  = htons(ETH_P_LIN);
	skb->pkt_type  = PACKET_HOST;
	skb->ip_summed = CHECKSUM_UNNECESSARY;

	prv = lin_skb_prv(skb);
	prv->ifindex	     = dev->ifindex;
	prv->skbcnt	     = 0;
	prv->master_owner    = NULL;
	prv->publisher_owner = NULL;

	return skb;
}
EXPORT_SYMBOL_GPL(alloc_lin_skb);

void lin_loopback_rx(struct net_device *dev, const struct lin_frame *frame,
		     unsigned int emit_flags, u8 resp_id)
{
	struct lin_dev *ld;
	struct sk_buff *skb;
	struct lin_skb_priv *prv;
	struct sock *master_owner = NULL;
	struct sock *publisher_owner = NULL;
	int yes_votes = 0;

	if (!dev || dev->type != ARPHRD_LIN)
		return;

	WARN_ON_ONCE(emit_flags & ~(LIN_EMIT_MASTER | LIN_EMIT_PUBLISHER));
	emit_flags &= LIN_EMIT_MASTER | LIN_EMIT_PUBLISHER;

	ld = lin_get_ml_priv(dev);
	if (!ld)
		return;

	rcu_read_lock();

	/* OR-rule loopback gate. Each tagged stakeholder casts a vote
	 * via the common lin_sk_wants_loopback() accessor (reads the
	 * LIN_RAW_LOOPBACK flag from the struct lin_sock base every
	 * PF_LIN sock embeds). Synth happens iff at least one local
	 * stakeholder voted yes. Tags are populated regardless of the
	 * vote so LIN_RAW_RECV_OWN_MSGS gating in the rx callback works
	 * off the tags independently of the synth gate.
	 */
	if (emit_flags & LIN_EMIT_MASTER) {
		struct sock *sk = rcu_dereference(ld->master_sk);

		if (sk && refcount_inc_not_zero(&sk->sk_refcnt)) {
			if (lin_sk_wants_loopback(sk))
				yes_votes++;
			master_owner = sk;
		}
	}
	if (emit_flags & LIN_EMIT_PUBLISHER) {
		/* @resp_id, not frame->lin_id: the publisher that sourced the
		 * response owns the ID the data belongs to, which differs
		 * from the header ID for event-triggered frames (header
		 * carries the trigger; the response comes from the answering
		 * frame's ID). For unconditional/sporadic frames callers
		 * pass frame->lin_id, so the lookup is equivalent.
		 */
		u8 id = resp_id & LIN_ID_MASK;
		struct sock *sk = rcu_dereference(ld->publishers[id]);

		if (sk && refcount_inc_not_zero(&sk->sk_refcnt)) {
			if (lin_sk_wants_loopback(sk))
				yes_votes++;
			publisher_owner = sk;
		}
	}

	rcu_read_unlock();

	if (yes_votes == 0)
		goto err_drop_refs;

	skb = alloc_lin_skb(dev, frame);
	if (!skb)
		goto err_drop_refs;

	prv = lin_skb_prv(skb);
	prv->master_owner    = master_owner;
	prv->publisher_owner = publisher_owner;

	skb->destructor = lin_loopback_skb_destructor;

	netif_rx(skb);
	return;

err_drop_refs:
	if (master_owner)
		sock_put(master_owner);
	if (publisher_owner)
		sock_put(publisher_owner);
}
EXPORT_SYMBOL_GPL(lin_loopback_rx);
