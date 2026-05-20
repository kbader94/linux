/* SPDX-License-Identifier: (GPL-2.0 OR BSD-3-Clause) */
/*
 * linux/lin/dev.h
 *
 * Definitions for LIN network device registration and driver contract.
 *
 * Author: Kyle Bader <kyle.bader94@gmail.com>
 * Copyright (c) 2026 Kyle Bader
 */

#ifndef _LIN_DEV_H
#define _LIN_DEV_H

#include <linux/lin.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/netdevice.h>
#include <linux/types.h>

struct lin_frame;
struct sk_buff;

/*
 * Loopback emission tags. Identify which role on the originating
 * interface sourced an emission, so the core can stamp the
 * synthesised loopback skb with the right ownership pointers and
 * subscribers can apply LIN_RAW_RECV_OWN_MSGS gating per-socket.
 *
 * LIN_EMIT_MASTER:    the master schedule (or an ad-hoc header send)
 *                     fired the header that produced this frame.
 * LIN_EMIT_PUBLISHER: a registered publisher's response payload formed
 *                     the data portion of this frame.
 *
 * The two flags are independent and may be combined: a frame whose
 * header was master-emitted AND whose response came from a publisher
 * carries both bits, and the resulting skb tags both owners so each
 * socket's LIN_RAW_RECV_OWN_MSGS state is respected on dispatch.
 */
enum lin_emit_flags {
	LIN_EMIT_MASTER		= BIT(0),
	LIN_EMIT_PUBLISHER	= BIT(1),
};

/**
 * struct lin_dev_rcv_lists - per-interface rx subscriber lists
 * @by_id:     one bucket per 6-bit LIN ID for single-ID filters
 *             (id_mask == LIN_ID_MASK, no flag constraint). LIN's
 *             ID space is small enough that a direct array index is
 *             both simpler and faster than a hash bucket; this is the
 *             primary deviation from SocketCAN's receiver-list shape.
 * @match_all: subscribers with id_mask == 0 and flags_mask == 0
 *             (and LIN_FILT_INV clear) — every non-error frame is
 *             delivered to these.
 * @filter:    subscribers with non-trivial id_mask or flags_mask
 *             (non-inverted). Walked against every non-error frame.
 * @inv:       inverted filters (LIN_FILT_INV set in lin_filter.flags).
 *             Walked against every non-error frame, match inverted.
 * @err:       subscribers to error frames, registered via
 *             LIN_RAW_ERR_FILTER. Match is against frame.err_mask.
 * @entries:   total subscriber count across all buckets; used for the
 *             fast "no listeners" rx early-exit.
 *
 * Present once per LIN netdev (embedded in struct lin_dev) and once
 * per network namespace (for ifindex-0 "any" subscribers). Readers
 * walk the lists under rcu_read_lock(); writers update under the
 * net-ns-wide rcvlists_lock.
 */
struct lin_dev_rcv_lists {
	struct hlist_head	by_id[LIN_ID_MASK + 1];
	struct hlist_head	match_all;
	struct hlist_head	filter;
	struct hlist_head	inv;
	struct hlist_head	err;
	int			entries;
};

/**
 * struct lin_dev - per-netdev LIN state owned by the core
 * @dev:         backpointer to the owning net_device
 * @rcv_lists:   rx subscriber lists for this interface; see
 *               struct lin_dev_rcv_lists
 * @policy_lock: serialises mutation of cross-socket policy state on
 *               this interface (master claim, publisher registry,
 *               schedules — added by later commits). Held across
 *               driver op calls so the single-master-per-interface
 *               and single-publisher-per-ID invariants hold even when
 *               driver ops sleep, without serialising on rtnl_lock.
 *               Lock ordering: rtnl_lock (when held by the caller)
 *               -> sock_lock(sk) -> ld->policy_lock.
 *
 * Installed on net_device.ml_priv with type tag ML_PRIV_LIN by
 * alloc_lindev(). Subsequent commits extend this structure with the
 * cross-socket policy state the LIN core tracks: master-role claim
 * (commit #5), publisher-ownership registry (commit #5), and loaded
 * schedule tracking (commit #6). The actual schedule execution and
 * hardware response table live in the driver; the core forwards
 * validated state to it via struct lin_dev_ops.
 */
struct lin_dev {
	struct net_device		*dev;
	struct lin_dev_rcv_lists	 rcv_lists;
	struct mutex			 policy_lock;
};

/* Initialize a struct lin_dev_rcv_lists in place. */
void lin_dev_rcv_lists_init(struct lin_dev_rcv_lists *rl);

static inline struct lin_dev *lin_get_ml_priv(struct net_device *dev)
{
	return netdev_get_ml_priv(dev, ML_PRIV_LIN);
}

static inline void lin_set_ml_priv(struct net_device *dev, struct lin_dev *ld)
{
	netdev_set_ml_priv(dev, ld, ML_PRIV_LIN);
}

/**
 * lin_setup - configure net_device fields for a LIN interface
 * @dev: the netdev being prepared
 *
 * Used as the setup callback to alloc_netdev(). alloc_lindev() passes
 * this automatically; drivers that allocate via alloc_netdev_mq()
 * directly should invoke it from their own setup function.
 */
void lin_setup(struct net_device *dev);

/**
 * lin_dev_init - initialise the LIN core's embedded struct lin_dev
 * @dev:         a net_device whose private area was sized to hold the
 *               driver private region followed by struct lin_dev
 * @sizeof_priv: size of the driver private region preceding the lin_dev
 *
 * Initialises the core-owned struct lin_dev embedded in netdev_priv()
 * and tags dev->ml_priv. alloc_lindev() calls this for the usual
 * alloc/register path; drivers that create their interface through
 * rtnl_link_ops invoke it directly from their setup callback (the rtnl
 * core allocates the netdev, so alloc_lindev() is not on that path).
 */
void lin_dev_init(struct net_device *dev, int sizeof_priv);

/**
 * alloc_lindev - allocate a LIN network device
 * @sizeof_priv: size of the driver's private data area, in bytes
 *
 * Allocates a net_device with enough room for the driver's private
 * area and the LIN core's struct lin_dev. The driver's private area
 * is reachable via netdev_priv(); the core's lin_dev is reachable
 * via lin_get_ml_priv(). On success, the returned netdev has
 * dev->type == ARPHRD_LIN and dev->ml_priv tagged ML_PRIV_LIN.
 *
 * Return: the new net_device on success, NULL on allocation failure.
 */
struct net_device *alloc_lindev(int sizeof_priv);

/**
 * free_lindev - release a LIN network device allocated with alloc_lindev()
 * @dev: the netdev to free
 *
 * Paired with alloc_lindev() at end of life, mirroring the SocketCAN
 * alloc/register/unregister/free contract. Call after
 * lin_unregister_netdev(), or directly if driver setup fails before
 * registration. Wraps free_netdev().
 */
void free_lindev(struct net_device *dev);

/**
 * lin_register_netdev - register a LIN network device with the network stack
 * @dev: the netdev to register
 *
 * Wraps register_netdev() and performs LIN-specific sanity checks.
 * Leaves the interface in the carrier-off state; drivers transition to
 * carrier-on from their ndo_open() once the controller is ready to
 * exchange frames.
 *
 * Return: 0 on success, a negative errno on failure.
 */
int  lin_register_netdev(struct net_device *dev);

/**
 * lin_unregister_netdev - unregister a LIN network device
 * @dev: the netdev to unregister
 *
 * Wraps unregister_netdev(). After this returns the interface is no
 * longer visible to the network stack, but the net_device remains
 * allocated; call free_lindev() to release it.
 */
void lin_unregister_netdev(struct net_device *dev);

/**
 * lin_loopback_rx - synthesise a tagged rx skb for an emitted frame
 * @dev:        originating LIN netdev
 * @frame:      LIN frame contents that were placed on the wire
 *              (master header + publisher response, as actually
 *              emitted)
 * @emit_flags: union of LIN_EMIT_* describing which role(s) on
 *              this interface sourced the emission. The core uses
 *              these bits to tag the synthesised skb with the
 *              originating master and/or publisher socket so
 *              subscribers can honour LIN_RAW_RECV_OWN_MSGS.
 *
 * LIN transceivers typically don't surface their own transmissions
 * on rx (sllin-class UART bridges, single-wire half-duplex SPI
 * bridges, and so on read back the line internally to validate the
 * byte stream but do not raise a host-visible rx event for the
 * frame). To preserve the SocketCAN-style "every emission is
 * observable from every socket on the bus" abstraction, the core
 * synthesises an rx-shaped skb here and feeds it through the
 * normal subscriber-dispatch path. Drivers whose hardware natively
 * echoes its own transmissions advertise LIN_CAP_LOOPBACK; on
 * those interfaces the core suppresses the synthetic skb to avoid
 * double delivery.
 *
 * Callable from process or softirq context. Allocates with
 * GFP_ATOMIC.
 */
void lin_loopback_rx(struct net_device *dev, const struct lin_frame *frame,
		     unsigned int emit_flags);

#endif /* _LIN_DEV_H */
