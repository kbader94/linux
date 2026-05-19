// SPDX-License-Identifier: (GPL-2.0 OR BSD-3-Clause)
/*
 * dev.c - LIN network device registration helpers
 *
 * Author: Kyle Bader <kyle.bader94@gmail.com>
 * Copyright (c) 2026 Kyle Bader
 *
 * Modelled on drivers/net/can/dev/dev.c: drivers allocate a netdev via
 * alloc_lindev() which reserves space for both their private area and
 * the LIN core's struct lin_dev, then call lin_register_netdev() to
 * expose the interface to the network stack.
 */

#include <linux/export.h>
#include <linux/if_arp.h>
#include <linux/mutex.h>
#include <linux/netdevice.h>
#include <linux/rcupdate.h>
#include <linux/lin.h>
#include <linux/lin/core.h>
#include <linux/lin/dev.h>
#include <net/sock.h>

void lin_setup(struct net_device *dev)
{
	dev->type		= ARPHRD_LIN;
	dev->mtu		= LIN_MTU;
	dev->min_mtu		= LIN_MTU;
	dev->max_mtu		= LIN_MTU;
	dev->hard_header_len	= 0;
	dev->addr_len		= 0;
	dev->tx_queue_len	= 0;

	/* LIN does not carry ARP or multicast; frame emission timing is
	 * driven by the driver's schedule engine (which the LIN core
	 * configures via lin_dev_ops), not by qdisc queueing, so the
	 * netdev has no TX queue of its own.
	 */
	dev->flags		= IFF_NOARP;
	dev->priv_flags		|= IFF_NO_QUEUE;
}
EXPORT_SYMBOL(lin_setup);

/**
 * lin_dev_init - initialise the LIN core's embedded struct lin_dev
 * @dev:         a net_device whose private area was sized to hold the
 *               driver private region followed by struct lin_dev
 * @ops:         driver-provided LIN ops vtable, stored on the lin_dev
 * @sizeof_priv: size of the driver private region preceding the lin_dev,
 *               in bytes (the same value passed to alloc_lindev())
 *
 * Locates the embedded struct lin_dev within netdev_priv(), initialises
 * the core-owned state, and tags dev->ml_priv with ML_PRIV_LIN. Factored
 * out of alloc_lindev() so drivers that allocate their netdev through
 * rtnl_link_ops (and therefore cannot call alloc_lindev()) can run the
 * same initialisation from their setup callback.
 */
void lin_dev_init(struct net_device *dev, const struct lin_dev_ops *ops,
		  int sizeof_priv)
{
	struct lin_dev *ld = (struct lin_dev *)((char *)netdev_priv(dev) +
						ALIGN(sizeof_priv, NETDEV_ALIGN));

	ld->dev = dev;
	ld->ops = ops;
	mutex_init(&ld->policy_lock);
	lin_dev_rcv_lists_init(&ld->rcv_lists);
	RCU_INIT_POINTER(ld->master_sk, NULL);
	ld->going_down = false;

	lin_set_ml_priv(dev, ld);
}
EXPORT_SYMBOL(lin_dev_init);

struct net_device *alloc_lindev(int sizeof_priv,
				const struct lin_dev_ops *ops)
{
	struct net_device *dev;
	int size;

	if (!ops)
		return NULL;

	/* Memory layout within netdev_priv():
	 *
	 *   +-----------------------------+
	 *   | driver's private data       |  <- netdev_priv(dev)
	 *   +-----------------------------+
	 *   | struct lin_dev              |  <- lin_get_ml_priv(dev)
	 *   +-----------------------------+
	 */
	size = ALIGN(sizeof_priv, NETDEV_ALIGN) + sizeof(struct lin_dev);

	dev = alloc_netdev(size, "lin%d", NET_NAME_UNKNOWN, lin_setup);
	if (!dev)
		return NULL;

	lin_dev_init(dev, ops, sizeof_priv);

	return dev;
}
EXPORT_SYMBOL(alloc_lindev);

void lin_dev_rcv_lists_init(struct lin_dev_rcv_lists *rl)
{
	int i;

	for (i = 0; i <= LIN_ID_MASK; i++)
		INIT_HLIST_HEAD(&rl->by_id[i]);

	INIT_HLIST_HEAD(&rl->match_all);
	INIT_HLIST_HEAD(&rl->filter);
	INIT_HLIST_HEAD(&rl->inv);
	INIT_HLIST_HEAD(&rl->err);
	rl->entries = 0;
}
EXPORT_SYMBOL(lin_dev_rcv_lists_init);

void free_lindev(struct net_device *dev)
{
	free_netdev(dev);
}
EXPORT_SYMBOL(free_lindev);

/* Cross-socket policy helpers: master claim.
 * All mutating calls are serialised by ld->policy_lock; rx readers use
 * rcu_read_lock() around dereferences of ld->master_sk and gate every
 * subsequent sock field access on refcount_inc_not_zero(&sk->sk_refcnt)
 * so a sock observed mid-teardown is skipped rather than held. That
 * removes any need to defer the writer's sock_put across an RCU grace
 * period: clear the slot under policy_lock, drop the held reference
 * inline, and concurrent rx readers either grabbed their own reference
 * first (sock stays alive) or see a dead refcount and skip (sock_free
 * completes via sk_rcu independently).
 *
 * The mutex (rather than rtnl_lock) keeps a sleeping driver op from
 * blocking unrelated subsystems: the worst case is contention with
 * other policy operations on the same interface, plus blocking
 * NETDEV_UNREGISTER for that interface (the notifier needs sock_lock).
 */

int lin_master_claim(struct net_device *dev, struct sock *sk)
{
	struct lin_dev *ld = lin_get_ml_priv(dev);
	struct sock *current_master;
	int err;

	might_sleep();

	lockdep_assert_held(&ld->policy_lock);

	if (!ld->ops->master_start)
		return -EOPNOTSUPP;

	current_master = rcu_dereference_protected(ld->master_sk,
						   lockdep_is_held(&ld->policy_lock));
	if (current_master == sk)
		return 0;
	if (current_master)
		return -EBUSY;

	/* Driver op may sleep; call before publishing the pointer so we
	 * never expose a claim the hardware has not acknowledged.
	 */
	err = ld->ops->master_start(ld);
	if (err)
		return err;

	sock_hold(sk);
	rcu_assign_pointer(ld->master_sk, sk);
	return 0;
}
EXPORT_SYMBOL(lin_master_claim);

int lin_master_release(struct net_device *dev, struct sock *sk)
{
	struct lin_dev *ld = lin_get_ml_priv(dev);
	struct sock *current_master;
	int err;

	might_sleep();

	lockdep_assert_held(&ld->policy_lock);

	current_master = rcu_dereference_protected(ld->master_sk,
						   lockdep_is_held(&ld->policy_lock));
	if (current_master != sk)
		return 0;

	/* Best-effort teardown: clear core state regardless of driver
	 * errors. A wedged driver surfaces via dmesg, not via a
	 * propagated errno that userspace has no realistic way to
	 * handle — the only meaningful userspace response to "release
	 * failed" is "close the socket," which goes through this same
	 * path anyway.
	 */
	rcu_assign_pointer(ld->master_sk, NULL);

	err = ld->ops->master_stop(ld);
	if (err)
		netdev_err(dev, "LIN master_stop returned %d on release; driver may be in an inconsistent state\n",
			   err);

	sock_put(current_master);
	return 0;
}
EXPORT_SYMBOL(lin_master_release);

int lin_register_netdev(struct net_device *dev)
{
	struct lin_dev *ld = lin_get_ml_priv(dev);
	int master_ops;

	if (dev->type != ARPHRD_LIN || !ld)
		return -EINVAL;

	/* Enforce paired lifecycle ops. A driver that implements one half
	 * of a pair but not the other would strand hardware state on
	 * release (publisher unset without clear_response leaves the
	 * response table entry live; master claim release without
	 * master_stop leaves the schedule engine running). Catch it at
	 * registration time so the driver author fixes their vtable
	 * rather than debugging a stale-state bug later.
	 */
	if (!!ld->ops->set_response != !!ld->ops->clear_response)
		return -EINVAL;

	/* Master ops: master_{start,stop} and the four
	 * schedule_* ops must be either all set or all NULL. A driver
	 * supporting only part of the master role can't usefully run
	 * a schedule.
	 */
	master_ops = !!ld->ops->master_start + !!ld->ops->master_stop +
		     !!ld->ops->schedule_load + !!ld->ops->schedule_delete +
		     !!ld->ops->schedule_activate + !!ld->ops->schedule_stop;
	if (master_ops != 0 && master_ops != 6)
		return -EINVAL;

	/* Caps that imply master capability cannot be set on a
	 * slave-only driver. LIN_CAP_CHK_ENH is allowed on either
	 * because a slave node may also publish enhanced-checksum
	 * frames. LIN_CAP_DIAG is also role-agnostic — it indicates
	 * the driver handles the diagnostic ID range correctly, which
	 * matters for both master-side schedule routing and slave-side
	 * transport responses.
	 */
	if (master_ops == 0 &&
	    (ld->caps & (LIN_CAP_SPORADIC | LIN_CAP_EVENT)))
		return -EINVAL;

	/* @header_send is the kernel's one-shot emission primitive,
	 * reachable only from the master role (lin_header_send() requires
	 * the caller hold the master claim). A slave-only driver that
	 * supplies it has an unreachable op, so reject the configuration
	 * at registration time.
	 */
	if (master_ops == 0 && ld->ops->header_send)
		return -EINVAL;

	/* Mark the interface as carrier-off until the driver opens it and
	 * the bus is usable, matching the convention established by
	 * register_candev(). Drivers call netif_carrier_on() from their
	 * ndo_open once the controller is ready to exchange frames.
	 */
	netif_carrier_off(dev);

	return register_netdev(dev);
}
EXPORT_SYMBOL(lin_register_netdev);

void lin_unregister_netdev(struct net_device *dev)
{
	unregister_netdev(dev);
}
EXPORT_SYMBOL(lin_unregister_netdev);
