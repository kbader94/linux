// SPDX-License-Identifier: GPL-2.0
/*
 * net/lin/netlink.c
 *
 * rtnetlink integration for LIN network devices: helpers that drivers
 * plug into their rtnl_link_ops to advertise and accept per-LIN-link
 * attributes (LIN_CAP_* bitmask, bus bit rate, slave-publisher cap-gate
 * override; controller state and LIN xstats are still deferred to land
 * alongside their first real consumer).
 *
 * Author: Kyle Bader <kyle.bader94@gmail.com>
 * Copyright (c) 2026 Kyle Bader
 */

#include <linux/lin/core.h>
#include <linux/lin/dev.h>
#include <linux/netdevice.h>
#include <linux/skbuff.h>
#include <net/netlink.h>
#include <net/rtnetlink.h>

#include <uapi/linux/lin/netlink.h>

/**
 * lin_link_get_size - rtnl_link_ops .get_size for LIN devices
 * @dev: the LIN netdev
 *
 * Returns the size of the LIN-specific attribute block carried inside
 * IFLA_INFO_DATA on RTM_*LINK messages. Drivers point their
 * rtnl_link_ops .get_size at this helper (or call it from their own).
 */
size_t lin_link_get_size(const struct net_device *dev)
{
	return nla_total_size(sizeof(u32)) +	/* IFLA_LIN_CAPS             */
	       nla_total_size(sizeof(u32)) +	/* IFLA_LIN_BITRATE          */
	       nla_total_size(sizeof(u8));	/* IFLA_LIN_FORCE_PUB_SLAVE  */
}
EXPORT_SYMBOL(lin_link_get_size);

/**
 * lin_link_fill_info - rtnl_link_ops .fill_info for LIN devices
 * @skb: netlink skb being built
 * @dev: the LIN netdev
 *
 * Writes the LIN-specific attribute block into @skb. Drivers point their
 * rtnl_link_ops .fill_info at this helper (or call it from their own
 * after emitting any driver-specific attributes).
 */
int lin_link_fill_info(struct sk_buff *skb, const struct net_device *dev)
{
	/* lin_get_ml_priv() takes a non-const net_device; this fill_info is
	 * read-only, so the cast is safe.
	 */
	struct lin_dev *ld = lin_get_ml_priv((struct net_device *)dev);

	if (!ld)
		return -EINVAL;
	if (nla_put_u32(skb, IFLA_LIN_CAPS, ld->caps))
		return -EMSGSIZE;
	if (nla_put_u32(skb, IFLA_LIN_BITRATE, ld->bitrate))
		return -EMSGSIZE;
	if (nla_put_u8(skb, IFLA_LIN_FORCE_PUB_SLAVE,
		       ld->force_pub_slave ? 1 : 0))
		return -EMSGSIZE;
	return 0;
}
EXPORT_SYMBOL(lin_link_fill_info);

/*
 * IFLA_LIN_* policy. Only writable attributes are listed: IFLA_LIN_CAPS
 * is dump-only (driver-immutable), so it is intentionally omitted —
 * userspace attempting to set it gets a policy rejection from the rtnl
 * core, not a silent no-op.
 */
const struct nla_policy lin_link_policy[IFLA_LIN_MAX + 1] = {
	[IFLA_LIN_BITRATE]		= { .type = NLA_U32 },
	[IFLA_LIN_FORCE_PUB_SLAVE]	= { .type = NLA_U8 },
};
EXPORT_SYMBOL(lin_link_policy);

/**
 * lin_link_changelink - rtnl_link_ops .changelink for LIN devices
 * @dev:    the LIN netdev being modified
 * @tb:     parsed IFLA_* link-level attributes
 * @data:   parsed IFLA_LIN_* attribute block from IFLA_INFO_DATA
 * @extack: netlink ack/error context
 *
 * Applies LIN-specific link-level mutations carried in IFLA_INFO_DATA.
 * Two writable attributes today:
 *   - IFLA_LIN_BITRATE: calls the driver's @set_bitrate op and updates
 *     the cached @lin_dev.bitrate on success.
 *   - IFLA_LIN_FORCE_PUB_SLAVE: operator override of the slave-
 *     publisher cap gate. Toggles @lin_dev.force_pub_slave; the
 *     publisher admission check in lin_publisher_set() honours it.
 *     Transitions are logged at INFO level so the operator's choice
 *     is auditable.
 *
 * Drivers point their rtnl_link_ops .changelink at this helper directly
 * (or call it from their own after handling driver-specific attrs); the
 * shared lin_link_ops already does so for the alloc_lindev() path.
 *
 * Return: 0 on success; -EOPNOTSUPP if a write was attempted against a
 * driver that does not implement the corresponding op; or another
 * -errno propagated from the driver.
 */
int lin_link_changelink(struct net_device *dev, struct nlattr *tb[],
			struct nlattr *data[], struct netlink_ext_ack *extack)
{
	struct lin_dev *ld = lin_get_ml_priv(dev);
	int ret;

	if (!ld)
		return -ENODEV;
	if (!data)
		return 0;

	if (data[IFLA_LIN_BITRATE]) {
		u32 bitrate = nla_get_u32(data[IFLA_LIN_BITRATE]);

		if (!ld->ops->set_bitrate) {
			NL_SET_ERR_MSG_ATTR(extack, data[IFLA_LIN_BITRATE],
					    "driver does not support runtime bitrate changes");
			return -EOPNOTSUPP;
		}
		ret = ld->ops->set_bitrate(ld, bitrate);
		if (ret) {
			NL_SET_ERR_MSG_ATTR(extack, data[IFLA_LIN_BITRATE],
					    "driver rejected bitrate");
			return ret;
		}
		ld->bitrate = bitrate;
	}

	if (data[IFLA_LIN_FORCE_PUB_SLAVE]) {
		bool force = !!nla_get_u8(data[IFLA_LIN_FORCE_PUB_SLAVE]);

		if (force != ld->force_pub_slave) {
			ld->force_pub_slave = force;
			if (force)
				netdev_info(dev,
					    "LIN_CAP_PUB_SLAVE cap-gate overridden by operator; slave-publisher timing is not enforced\n");
			else
				netdev_info(dev,
					    "LIN_CAP_PUB_SLAVE override cleared\n");
		}
	}

	return 0;
}
EXPORT_SYMBOL(lin_link_changelink);

/*
 * Shared rtnl_link_ops for LIN devices that have no driver-specific
 * userspace-creatable type — i.e. hardware drivers going through the
 * alloc_lindev() / lin_register_netdev() path. Mirrors SocketCAN's
 * can_link_ops: registered once at LIN core module init, and assigned to
 * each lin_dev by alloc_lindev() unless the driver supplies its own
 * rtnl_link_ops (vlin's case, where the link is created via
 * `ip link add type vlin`).
 *
 * Either path exposes IFLA_LIN_CAPS, IFLA_LIN_BITRATE, and
 * IFLA_LIN_FORCE_PUB_SLAVE through the same fill_info helper, and
 * accepts IFLA_LIN_BITRATE / IFLA_LIN_FORCE_PUB_SLAVE writes through
 * the same changelink helper.
 */
struct rtnl_link_ops lin_link_ops __read_mostly = {
	.kind		= "lin",
	.maxtype	= IFLA_LIN_MAX,
	.policy		= lin_link_policy,
	.get_size	= lin_link_get_size,
	.fill_info	= lin_link_fill_info,
	.changelink	= lin_link_changelink,
};
EXPORT_SYMBOL(lin_link_ops);

int __init lin_link_ops_register(void)
{
	return rtnl_link_register(&lin_link_ops);
}

void lin_link_ops_unregister(void)
{
	rtnl_link_unregister(&lin_link_ops);
}
