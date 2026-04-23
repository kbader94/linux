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
#include <linux/lin.h>
#include <linux/lin/dev.h>

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
 * @sizeof_priv: size of the driver private region preceding the lin_dev,
 *               in bytes (the same value passed to alloc_lindev())
 *
 * Locates the embedded struct lin_dev within netdev_priv(), initialises
 * the core-owned state, and tags dev->ml_priv with ML_PRIV_LIN. Factored
 * out of alloc_lindev() so drivers that allocate their netdev through
 * rtnl_link_ops (and therefore cannot call alloc_lindev()) can run the
 * same initialisation from their setup callback.
 */
void lin_dev_init(struct net_device *dev, int sizeof_priv)
{
	struct lin_dev *ld = (struct lin_dev *)((char *)netdev_priv(dev) +
						ALIGN(sizeof_priv, NETDEV_ALIGN));

	ld->dev = dev;
	mutex_init(&ld->policy_lock);

	lin_set_ml_priv(dev, ld);
}
EXPORT_SYMBOL(lin_dev_init);

struct net_device *alloc_lindev(int sizeof_priv)
{
	struct net_device *dev;
	int size;

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

	lin_dev_init(dev, sizeof_priv);

	return dev;
}
EXPORT_SYMBOL(alloc_lindev);

void free_lindev(struct net_device *dev)
{
	free_netdev(dev);
}
EXPORT_SYMBOL(free_lindev);

int lin_register_netdev(struct net_device *dev)
{
	if (dev->type != ARPHRD_LIN || !lin_get_ml_priv(dev))
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
