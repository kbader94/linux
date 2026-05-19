// SPDX-License-Identifier: (GPL-2.0 OR BSD-3-Clause)
/*
 * raw.c - Raw sockets for protocol family LIN
 *
 * Author: Kyle Bader <kyle.bader94@gmail.com>
 * Copyright (c) 2026 Kyle Bader
 *
 * Modelled on net/can/raw.c, adapted for LIN's single-master protocol
 * model: frame emission timing is owned by the driver's schedule
 * engine (configured by the LIN core via lin_dev_ops), and
 * bind/publish/master sockopts scope per ifindex. Implements the rx
 * side (LIN_RAW_FILTER, LIN_RAW_ERR_FILTER, LIN_RAW_JOIN_FILTERS,
 * LIN_RAW_RECV_OWN_MSGS) plus bind(), getname(), the notifier that
 * tears down bound sockets when their netdev disappears, and the
 * LIN_RAW_MASTER role-claim sockopt. The publisher / schedule /
 * send-header sockopts land in later commits.
 */

#include <linux/if_arp.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/net.h>
#include <linux/netdevice.h>
#include <linux/rtnetlink.h>
#include <linux/slab.h>
#include <linux/socket.h>
#include <linux/spinlock.h>
#include <linux/string.h>
#include <linux/uaccess.h>
#include <linux/lin.h>
#include <linux/lin/core.h>
#include <linux/lin/dev.h>
#include <linux/lin/error.h>
#include <linux/lin/raw.h>
#include <linux/lin/skb.h>
#include <net/sock.h>
#include <net/net_namespace.h>

MODULE_DESCRIPTION("PF_LIN raw protocol");
MODULE_LICENSE("Dual BSD/GPL");
MODULE_AUTHOR("Kyle Bader <kyle.bader94@gmail.com>");
MODULE_ALIAS("lin-proto-1");

#define LIN_RAW_MIN_NAMELEN \
	(offsetof(struct sockaddr_lin, lin_ifindex) + sizeof(int))

/*
 * Per-cpu state used to suppress duplicate delivery when a single
 * frame matches multiple of this socket's filters, and to implement
 * LIN_RAW_JOIN_FILTERS (AND-combine) semantics.
 */
struct lin_uniqframe {
	const struct sk_buff	*skb;
	__u32			skbcnt;
	unsigned int		join_rx_count;
};

/*
 * Per-socket state for LIN_RAW. The filter array is stored inline in
 * @dfilter while @count == 1 (the default case) and allocated on the
 * heap with @filter pointing at it otherwise.
 */
struct lin_raw_sock {
	struct lin_sock		lin;	/* must be first; embeds struct sock and
					 * the common loopback / recv_own_msgs
					 * flags - see <linux/lin/core.h>
					 */
	struct net_device	*dev;
	netdevice_tracker	dev_tracker;
	struct list_head	notifier;
	int			ifindex;
	unsigned int		bound:1;
	unsigned int		is_master:1;	/* LIN_RAW_MASTER held */
	unsigned int		join_filters:1;
	__u32			err_mask;
	int			count;
	struct lin_uniqframe __percpu	*uniq;
	struct lin_filter	dfilter;	/* default single filter */
	struct lin_filter	*filter;	/* pointer to filter(s) */
};

static LIST_HEAD(lin_raw_notifier_list);
static DEFINE_SPINLOCK(lin_raw_notifier_lock);
static struct lin_raw_sock *lin_raw_busy_notifier;

static inline struct lin_raw_sock *lin_raw_sk(const struct sock *sk)
{
	return (struct lin_raw_sock *)sk;
}

/* rx callback — invoked by the core for each subscription match. */

static void lin_raw_rcv(struct sk_buff *oskb, void *data)
{
	struct sock *sk = data;
	struct lin_raw_sock *ro = lin_raw_sk(sk);
	struct sk_buff *skb;
	struct sockaddr_lin *addr;

	/* LIN_RAW_RECV_OWN_MSGS gating against the loopback skb tags.
	 * lin_loopback_rx() stamps master_owner and/or publisher_owner
	 * at emission time so we can identify the originating socket(s)
	 * here without racing against ld->master_sk / ld->publishers
	 * mutations. Bus-sourced rx leaves both pointers NULL and
	 * always passes the gate. Error frames are delivered
	 * unconditionally — they carry diagnostic information the
	 * originator should still see.
	 */
	if (!ro->lin.recv_own_msgs) {
		const struct lin_frame *lf =
			(const struct lin_frame *)oskb->data;

		if (!(lf->flags & LIN_F_ERR)) {
			struct lin_skb_priv *prv = lin_skb_prv(oskb);

			if (prv->master_owner == sk ||
			    prv->publisher_owner == sk)
				return;
		}
	}

	/* Deduplicate matches for the same underlying skb.
	 *
	 * The LIN core may invoke this callback multiple times when a
	 * frame matches overlapping filters on this socket (e.g. a
	 * specific-ID filter and a match-all filter). lin_skb_prv
	 * stamps each frame with a unique skbcnt in lin_receive();
	 * we track the last-seen skb per cpu and short-circuit on a
	 * repeat.
	 *
	 * When LIN_RAW_JOIN_FILTERS is enabled the opposite rule
	 * applies: we must see matches from every enabled filter
	 * before delivering, so we count matches per frame and gate
	 * delivery on join_rx_count >= ro->count.
	 *
	 * Error frames bypass both checks. They route through the
	 * disjoint LIN_RAW_ERR_FILTER list, so a socket sees at most
	 * one rx callback per error skb — dedup is unnecessary, and
	 * applying the JOIN_FILTERS gate against ro->count (the data-
	 * filter count) would silently suppress every error delivery
	 * when ro->count > 1.
	 */
	if (!(((const struct lin_frame *)oskb->data)->flags & LIN_F_ERR)) {
		if (this_cpu_ptr(ro->uniq)->skb == oskb &&
		    this_cpu_ptr(ro->uniq)->skbcnt == lin_skb_prv(oskb)->skbcnt) {
			if (!ro->join_filters)
				return;

			this_cpu_inc(ro->uniq->join_rx_count);
			if (this_cpu_ptr(ro->uniq)->join_rx_count < ro->count)
				return;
		} else {
			this_cpu_ptr(ro->uniq)->skb = oskb;
			this_cpu_ptr(ro->uniq)->skbcnt = lin_skb_prv(oskb)->skbcnt;
			this_cpu_ptr(ro->uniq)->join_rx_count = 1;
			/* With AND-semantics, the first match is not enough. */
			if (ro->join_filters && ro->count > 1)
				return;
		}
	}

	/* Clone so each delivered socket gets its own skb instance. */
	skb = skb_clone(oskb, GFP_ATOMIC);
	if (!skb)
		return;

	/* Stuff the originating ifindex into the cloned skb->cb so
	 * recvmsg can return the per-frame source. The cloned skb is
	 * independent of the dispatch skb, so we're free to reuse cb
	 * here; the dispatch skb still holds lin_skb_priv for any
	 * remaining filter walkers.
	 */
	sock_skb_cb_check_size(sizeof(struct sockaddr_lin));
	addr = (struct sockaddr_lin *)skb->cb;
	memset(addr, 0, sizeof(*addr));
	addr->lin_family = AF_LIN;
	addr->lin_ifindex = skb->dev->ifindex;

	if (sock_queue_rcv_skb(sk, skb) < 0)
		kfree_skb(skb);
}

/* filter enable / disable helpers */

static int lin_raw_enable_filters(struct net *net, struct net_device *dev,
				  struct sock *sk,
				  const struct lin_filter *filter, int count)
{
	int err = 0;
	int i;

	for (i = 0; i < count; i++) {
		err = lin_rx_register(net, dev,
				      filter[i].lin_id, filter[i].id_mask,
				      filter[i].flags, filter[i].flags_mask,
				      0,
				      lin_raw_rcv, sk, "raw", sk);
		if (err) {
			/* unwind successful installs */
			while (--i >= 0)
				lin_rx_unregister(net, dev,
						  filter[i].lin_id,
						  filter[i].id_mask,
						  filter[i].flags,
						  filter[i].flags_mask,
						  0, lin_raw_rcv, sk);
			break;
		}
	}

	return err;
}

static void lin_raw_disable_filters(struct net *net, struct net_device *dev,
				    struct sock *sk,
				    const struct lin_filter *filter, int count)
{
	int i;

	for (i = 0; i < count; i++)
		lin_rx_unregister(net, dev,
				  filter[i].lin_id, filter[i].id_mask,
				  filter[i].flags, filter[i].flags_mask,
				  0, lin_raw_rcv, sk);
}

static int lin_raw_enable_errfilter(struct net *net, struct net_device *dev,
				    struct sock *sk, __u32 err_mask)
{
	if (!err_mask)
		return 0;

	return lin_rx_register(net, dev, 0, 0, 0, 0, err_mask,
			       lin_raw_rcv, sk, "raw", sk);
}

static void lin_raw_disable_errfilter(struct net *net, struct net_device *dev,
				      struct sock *sk, __u32 err_mask)
{
	if (err_mask)
		lin_rx_unregister(net, dev, 0, 0, 0, 0, err_mask,
				  lin_raw_rcv, sk);
}

static int lin_raw_enable_allfilters(struct net *net, struct net_device *dev,
				     struct sock *sk)
{
	struct lin_raw_sock *ro = lin_raw_sk(sk);
	int err;

	err = lin_raw_enable_filters(net, dev, sk, ro->filter, ro->count);
	if (err)
		return err;

	err = lin_raw_enable_errfilter(net, dev, sk, ro->err_mask);
	if (err)
		lin_raw_disable_filters(net, dev, sk, ro->filter, ro->count);

	return err;
}

static void lin_raw_disable_allfilters(struct net *net,
				       struct net_device *dev,
				       struct sock *sk)
{
	struct lin_raw_sock *ro = lin_raw_sk(sk);

	lin_raw_disable_filters(net, dev, sk, ro->filter, ro->count);
	lin_raw_disable_errfilter(net, dev, sk, ro->err_mask);
}

/* Release this socket's per-bus policy state on @dev: the master claim
 * (publisher entries are added with the publisher registry). Shared by
 * close(), rebind, and the GOING_DOWN / UNREGISTER notifiers so every
 * teardown path drops the same driver-side ownership — otherwise a path
 * that forgets it (as the rebind path originally did) leaves the core's
 * master_sk / publishers[] pointing at a gone socket with held refs and
 * the driver's schedule engine / response table still live.
 *
 * Caller must hold rtnl_lock so these driver ops cannot race the netdev
 * close path (ndo_stop); the policy_lock taken here serialises against
 * other policy operations on the interface.
 */
static void lin_raw_drop_dev_policy(struct lin_raw_sock *ro,
				    struct net_device *dev)
{
	struct lin_dev *ld = lin_get_ml_priv(dev);
	struct sock *sk = &ro->lin.sk;

	ASSERT_RTNL();

	if (!ld)
		return;

	mutex_lock(&ld->policy_lock);
	if (ro->is_master) {
		lin_master_release(dev, sk);
		ro->is_master = 0;
	}
	mutex_unlock(&ld->policy_lock);
}

/* netdev event notifier — per-socket teardown of LIN_RAW sockets bound
 * to a closing / vanishing dev.
 *
 * Phase split with the core notifier:
 *
 *   The af_lin core registers its own netdev notifier at higher
 *   priority. The core's GOING_DOWN handler sets ld->going_down under
 *   ld->policy_lock so any sockopt waiting on the lock returns
 *   -ENETDOWN on wakeup before touching the driver; UP clears it.
 *   That makes the lifecycle of @going_down independent of which
 *   protocol modules have sockets bound.
 *
 *   This per-protocol notifier handles only the per-socket state:
 *   force-release of the master claim at GOING_DOWN, sk_err signaling
 *   at DOWN, and the full unbind at UNREGISTER. We do not touch
 *   @going_down here — that's the core's job.
 *
 * Lock ordering across these paths: rtnl_lock (held by the dev_close
 * / unregister_netdevice caller) -> lock_sock(sk) -> ld->policy_lock.
 * Taking rtnl_lock again here would deadlock; we don't, and don't need
 * to because the per-device policy_lock already serialises us against
 * concurrent sockopts.
 */

static void lin_raw_notify(struct lin_raw_sock *ro, unsigned long msg,
			   struct net_device *dev)
{
	struct sock *sk = &ro->lin.sk;

	if (!net_eq(dev_net(dev), sock_net(sk)))
		return;
	if (ro->dev != dev)
		return;

	switch (msg) {
	case NETDEV_UNREGISTER:
		lock_sock(sk);
		if (ro->bound) {
			lin_raw_drop_dev_policy(ro, dev);
			lin_raw_disable_allfilters(dev_net(dev), dev, sk);
			netdev_put(ro->dev, &ro->dev_tracker);
		}
		ro->ifindex = 0;
		ro->bound = 0;
		ro->dev = NULL;
		release_sock(sk);

		sk->sk_err = ENODEV;
		if (!sock_flag(sk, SOCK_DEAD))
			sk_error_report(sk);
		break;

	case NETDEV_GOING_DOWN:
		/* The core's higher-priority netdev notifier has already
		 * set ld->going_down under policy_lock, so any sockopt
		 * blocked on the lock will see -ENETDOWN on wakeup before
		 * touching the driver. Our job here is the per-socket
		 * teardown: drop the master claim while the driver is
		 * still alive. Filters, binding, and the netdev reference
		 * survive so the socket remains usable as a passive
		 * observer when the interface returns.
		 */
		lock_sock(sk);
		if (ro->bound)
			lin_raw_drop_dev_policy(ro, dev);
		release_sock(sk);
		break;

	case NETDEV_DOWN:
		sk->sk_err = ENETDOWN;
		if (!sock_flag(sk, SOCK_DEAD))
			sk_error_report(sk);
		break;
	}
}

static int lin_raw_notifier(struct notifier_block *nb, unsigned long msg,
			    void *ptr)
{
	struct net_device *dev = netdev_notifier_info_to_dev(ptr);
	struct lin_raw_sock *ro;

	if (dev->type != ARPHRD_LIN)
		return NOTIFY_DONE;
	if (msg != NETDEV_UNREGISTER && msg != NETDEV_DOWN &&
	    msg != NETDEV_GOING_DOWN)
		return NOTIFY_DONE;
	if (unlikely(lin_raw_busy_notifier))	/* guard against reentrancy */
		return NOTIFY_DONE;

	spin_lock(&lin_raw_notifier_lock);
	list_for_each_entry(ro, &lin_raw_notifier_list, notifier) {
		lin_raw_busy_notifier = ro;
		spin_unlock(&lin_raw_notifier_lock);
		lin_raw_notify(ro, msg, dev);
		spin_lock(&lin_raw_notifier_lock);
		lin_raw_busy_notifier = NULL;
	}
	spin_unlock(&lin_raw_notifier_lock);

	return NOTIFY_DONE;
}

static struct notifier_block lin_raw_notifier_block __read_mostly = {
	.notifier_call = lin_raw_notifier,
};

/* socket lifecycle */

static int lin_raw_init(struct sock *sk)
{
	struct lin_raw_sock *ro = lin_raw_sk(sk);

	ro->bound		= 0;
	ro->is_master		= 0;
	ro->ifindex		= 0;
	ro->dev			= NULL;
	ro->lin.loopback		= 1;
	ro->lin.recv_own_msgs	= 0;
	ro->join_filters	= 0;
	ro->err_mask		= 0;

	/* default filter: match every frame ID, no flag constraint */
	ro->dfilter.lin_id	= 0;
	ro->dfilter.id_mask	= 0;
	ro->dfilter.flags	= 0;
	ro->dfilter.flags_mask	= 0;
	ro->filter		= &ro->dfilter;
	ro->count		= 1;

	ro->uniq = alloc_percpu(struct lin_uniqframe);
	if (unlikely(!ro->uniq))
		return -ENOMEM;

	/* register socket with the raw notifier list */
	spin_lock(&lin_raw_notifier_lock);
	list_add_tail(&ro->notifier, &lin_raw_notifier_list);
	spin_unlock(&lin_raw_notifier_lock);

	return 0;
}

static int lin_raw_release(struct socket *sock)
{
	struct sock *sk = sock->sk;
	struct lin_raw_sock *ro;
	struct net *net;
	unsigned long warn_deadline;

	if (!sk)
		return 0;

	ro = lin_raw_sk(sk);
	net = sock_net(sk);

	/* Bound the diagnostic visibility: if the notifier is stuck on
	 * this socket for more than five seconds it's almost always
	 * because a driver op held under policy_lock has exceeded its
	 * required timeout (see the lin_dev_ops kdoc — drivers MUST
	 * bound schedule_activate / header_send / wakeup_send). We
	 * still wait — there's no safe way to free @ro while another
	 * CPU may be reading it — but we log once so the buggy driver
	 * surfaces in dmesg instead of an unexplained close() hang.
	 */
	warn_deadline = jiffies + 5 * HZ;

	/* Take rtnl_lock first. Unlike CAN_RAW — whose release only drops
	 * core-side rx filters — LIN's teardown invokes driver policy ops
	 * (master_stop via lin_master_release; clear_response via the
	 * publisher teardown) that must not race the netdev close path's
	 * ndo_stop(). The GOING_DOWN / UNREGISTER notifiers that perform the
	 * same teardown already run under rtnl; serialise this path with
	 * them. With rtnl held no netdev notifier can run, so the
	 * busy-notifier wait below is satisfied immediately (any in-flight
	 * notifier completed before we acquired rtnl) and is kept only as
	 * defence. Acquisition order matches bind(): rtnl_lock -> lock_sock.
	 */
	rtnl_lock();

	spin_lock(&lin_raw_notifier_lock);
	while (lin_raw_busy_notifier == ro) {
		spin_unlock(&lin_raw_notifier_lock);
		schedule_timeout_uninterruptible(1);
		WARN_ONCE(time_after(jiffies, warn_deadline),
			  "lin_raw: notifier stuck on sk %p for > 5s; driver missing the timeout required by lin_dev_ops?\n",
			  ro);
		spin_lock(&lin_raw_notifier_lock);
	}
	list_del(&ro->notifier);
	spin_unlock(&lin_raw_notifier_lock);

	/* lock_sock(sk) serialises against the notifier's own lock_sock
	 * section, which mutates ro->dev / ro->bound.
	 */
	lock_sock(sk);

	if (ro->bound) {
		/* Tear down in the order that keeps the bus quiet for the
		 * longest possible window: stop header emission (master
		 * release) first, then unhook rx filters, then drop the
		 * netdev reference. Use force variants — the socket is
		 * going away and we must free core-side ownership
		 * regardless of whether the driver can cleanly quiesce.
		 */
		if (ro->dev) {
			lin_raw_drop_dev_policy(ro, ro->dev);
			lin_raw_disable_allfilters(dev_net(ro->dev),
						   ro->dev, sk);
			netdev_put(ro->dev, &ro->dev_tracker);
		} else {
			lin_raw_disable_allfilters(net, NULL, sk);
		}
	}

	/* Driver-policy teardown is done; drop rtnl before the grace-period
	 * wait below so it never blocks unrelated rtnetlink users. The
	 * remaining teardown needs only lock_sock.
	 */
	rtnl_unlock();

	if (ro->count > 1)
		kfree(ro->filter);

	ro->ifindex	= 0;
	ro->bound	= 0;
	ro->dev		= NULL;
	ro->count	= 0;

	/* lin_rx_unregister() unlinks the receiver via hlist_del_rcu()
	 * and schedules its free with call_rcu, but returns immediately.
	 * An rx walker on another CPU that already observed one of our
	 * receivers under rcu_read_lock can still enter lin_raw_rcv() and
	 * dereference per-socket state — notably the per-cpu ro->uniq
	 * dedup tracker. Wait one grace period before free_percpu() so it
	 * cannot race with such an in-flight walker.
	 *
	 * Done unconditionally rather than under "if (ro->bound)", because
	 * the NETDEV_UNREGISTER notifier in lin_raw_notify() tears down
	 * filters AND zeros ro->bound on our behalf when the bound dev is
	 * unregistered before close. In that ordering, by the time we
	 * reach this point ro->bound is already 0, but the call_rcu()
	 * callbacks the notifier scheduled may not have run yet and
	 * walkers can still be in flight on the freed-after-this per-cpu
	 * tracker. The sock itself is pinned across the grace period by
	 * the sock_hold each lin_rx_unregister() takes before call_rcu;
	 * ro->uniq is the separate allocation that free_percpu() releases
	 * synchronously and would otherwise be the UAF target.
	 */
	synchronize_rcu();
	free_percpu(ro->uniq);

	sock_orphan(sk);
	sock->sk = NULL;

	release_sock(sk);

	sock_prot_inuse_add(net, sk->sk_prot, -1);
	sock_put(sk);

	return 0;
}

static int lin_raw_bind(struct socket *sock, struct sockaddr_unsized *uaddr,
			int len)
{
	struct sockaddr_lin *addr = (struct sockaddr_lin *)uaddr;
	struct sock *sk = sock->sk;
	struct lin_raw_sock *ro = lin_raw_sk(sk);
	struct net_device *dev = NULL;
	int ifindex;
	int err = 0;
	int notify_enetdown = 0;

	if (len < LIN_RAW_MIN_NAMELEN)
		return -EINVAL;
	if (addr->lin_family != AF_LIN)
		return -EINVAL;

	rtnl_lock();
	lock_sock(sk);

	if (ro->bound && addr->lin_ifindex == ro->ifindex)
		goto out;

	if (addr->lin_ifindex) {
		dev = dev_get_by_index(sock_net(sk), addr->lin_ifindex);
		if (!dev) {
			err = -ENODEV;
			goto out;
		}
		if (dev->type != ARPHRD_LIN) {
			err = -ENODEV;
			goto out_put_dev;
		}
		if (!(dev->flags & IFF_UP))
			notify_enetdown = 1;

		ifindex = dev->ifindex;

		err = lin_raw_enable_allfilters(sock_net(sk), dev, sk);
		if (err)
			goto out_put_dev;

	} else {
		ifindex = 0;

		err = lin_raw_enable_allfilters(sock_net(sk), NULL, sk);
	}

	if (!err) {
		if (ro->bound) {
			/* Drop the old bus binding: release this socket's
			 * policy state (master claim / publisher entries) on
			 * the old dev, then its rx filters and netdev ref.
			 * rtnl_lock is already held above, satisfying
			 * lin_raw_drop_dev_policy()'s contract.
			 */
			if (ro->dev) {
				lin_raw_drop_dev_policy(ro, ro->dev);
				lin_raw_disable_allfilters(dev_net(ro->dev),
							   ro->dev, sk);
				netdev_put(ro->dev, &ro->dev_tracker);
			} else {
				lin_raw_disable_allfilters(sock_net(sk),
							   NULL, sk);
			}
		}
		ro->ifindex = ifindex;
		ro->bound = 1;
		ro->dev = dev;
		if (ro->dev)
			netdev_hold(ro->dev, &ro->dev_tracker, GFP_KERNEL);
	}

out_put_dev:
	/* remove potential reference from dev_get_by_index() */
	dev_put(dev);
out:
	release_sock(sk);
	rtnl_unlock();

	if (notify_enetdown) {
		sk->sk_err = ENETDOWN;
		if (!sock_flag(sk, SOCK_DEAD))
			sk_error_report(sk);
	}

	return err;
}

static int lin_raw_getname(struct socket *sock, struct sockaddr *uaddr,
			   int peer)
{
	struct sockaddr_lin *addr = (struct sockaddr_lin *)uaddr;
	struct sock *sk = sock->sk;
	struct lin_raw_sock *ro = lin_raw_sk(sk);

	if (peer)
		return -EOPNOTSUPP;

	memset(addr, 0, LIN_RAW_MIN_NAMELEN);
	addr->lin_family = AF_LIN;
	addr->lin_ifindex = ro->ifindex;

	return LIN_RAW_MIN_NAMELEN;
}

static int lin_raw_sock_no_ioctlcmd(struct socket *sock, unsigned int cmd,
				    unsigned long arg)
{
	/* LIN_RAW defines no protocol-level ioctls of its own. Returning
	 * -ENOIOCTLCMD defers to the generic socket layer for netdev-level
	 * handling (SIOCGIFINDEX, SIOCGIFNAME, ...), matching the CAN_RAW
	 * convention.
	 */
	return -ENOIOCTLCMD;
}

/* sockopt: LIN_RAW_FILTER / LIN_RAW_ERR_FILTER / flags */

static int lin_raw_set_filter(struct sock *sk, sockptr_t optval,
			      unsigned int optlen)
{
	struct lin_raw_sock *ro = lin_raw_sk(sk);
	struct lin_filter *filter = NULL;
	struct lin_filter sfilter;
	struct net_device *dev = NULL;
	int count = 0;
	int err = 0;

	/* optlen == 0 clears all data filters: the socket receives no data
	 * frames thereafter (LIN_RAW_ERR_FILTER / LIN_RAW_WAKEUP_FILTER
	 * subscriptions are unaffected). This matches CAN_RAW_FILTER and is
	 * the only way for a bound master/publisher socket to opt out of the
	 * default match-all data subscription. The teardown below handles
	 * count == 0 with a NULL filter set.
	 */
	if (optlen % sizeof(struct lin_filter) != 0)
		return -EINVAL;
	if (optlen > LIN_RAW_FILTER_MAX * sizeof(struct lin_filter))
		return -EINVAL;

	count = optlen / sizeof(struct lin_filter);

	if (count > 1) {
		filter = memdup_sockptr(optval, optlen);
		if (IS_ERR(filter))
			return PTR_ERR(filter);
	} else if (count == 1) {
		if (copy_from_sockptr(&sfilter, optval, sizeof(sfilter)))
			return -EFAULT;
	}

	/* Reject filters that would route into the wrong subscriber
	 * list or never match.
	 *
	 *   - @lin_id / @id_mask: the LIN frame ID is 6-bit; the upper
	 *     two bits of the byte are reserved (on the wire they
	 *     carry the parity that forms the Protected ID, which the
	 *     kernel computes itself). A caller that mistakenly
	 *     passes a PID instead of the raw ID would configure a
	 *     filter that can never match.
	 *
	 *   - @flags: struct lin_filter targets data frames only.
	 *     LIN_F_ERR and LIN_F_WAKEUP frames route to disjoint
	 *     subscriber lists (LIN_RAW_ERR_FILTER and
	 *     LIN_RAW_WAKEUP_FILTER respectively) and never reach the
	 *     data-filter walkers, so setting those bits here
	 *     silently produces a filter that fires zero times. Only
	 *     LIN_F_CHK_ENH (data-frame match bit) and LIN_FILT_INV
	 *     (the routing/invert bit) are valid.
	 *
	 *   - @flags_mask: covers only the LIN_F_* match bits;
	 *     LIN_FILT_INV is a routing bit, not affected by the
	 *     mask. Same restriction as @flags except LIN_FILT_INV
	 *     also has no business here.
	 */
	{
		const struct lin_filter *f = (count > 1) ? filter : &sfilter;
		int i;

		for (i = 0; i < count; i++) {
			if ((f[i].lin_id | f[i].id_mask) & ~LIN_ID_MASK) {
				if (count > 1)
					kfree(filter);
				return -EINVAL;
			}
			if (f[i].flags & ~(LIN_F_CHK_ENH | LIN_FILT_INV)) {
				if (count > 1)
					kfree(filter);
				return -EINVAL;
			}
			if (f[i].flags_mask & ~LIN_F_CHK_ENH) {
				if (count > 1)
					kfree(filter);
				return -EINVAL;
			}
			if (memchr_inv(f[i].__res, 0, sizeof(f[i].__res))) {
				if (count > 1)
					kfree(filter);
				return -EINVAL;
			}
			/* An inverted filter with both masks zero can never
			 * match: the dispatch predicate reduces to
			 * (x & 0) != (y & 0), which is unconditionally
			 * false, so the entry silently occupies a slot and
			 * delivers nothing. This is almost always a caller
			 * bug (zero-initialised struct with the INV bit
			 * toggled, expecting "match everything else"); a
			 * user who really wants no data subscription should
			 * not install a filter. Reject the shape up front.
			 */
			if ((f[i].flags & LIN_FILT_INV) &&
			    f[i].id_mask == 0 && f[i].flags_mask == 0) {
				if (count > 1)
					kfree(filter);
				return -EINVAL;
			}
		}
	}

	lock_sock(sk);

	dev = ro->dev;
	if (ro->bound && dev) {
		if (dev->reg_state != NETREG_REGISTERED) {
			if (count > 1)
				kfree(filter);
			err = -ENODEV;
			goto out_unlock;
		}
	}

	if (ro->bound) {
		/* install new filters before removing the old ones so the
		 * socket never drops below its subscription set.
		 */
		if (count > 1)
			err = lin_raw_enable_filters(sock_net(sk), dev, sk,
						     filter, count);
		else if (count == 1)
			err = lin_raw_enable_filters(sock_net(sk), dev, sk,
						     &sfilter, 1);

		if (err) {
			if (count > 1)
				kfree(filter);
			goto out_unlock;
		}

		lin_raw_disable_filters(sock_net(sk), dev, sk,
					ro->filter, ro->count);
	}

	/* commit new filter set onto the socket */
	if (ro->count > 1)
		kfree(ro->filter);

	if (count == 1) {
		ro->dfilter = sfilter;
		ro->filter = &ro->dfilter;
	} else {
		ro->filter = filter;
	}
	ro->count = count;

out_unlock:
	release_sock(sk);
	return err;
}

static int lin_raw_set_err_filter(struct sock *sk, sockptr_t optval,
				  unsigned int optlen)
{
	struct lin_raw_sock *ro = lin_raw_sk(sk);
	__u32 err_mask;
	struct net_device *dev;
	int err = 0;

	if (optlen != sizeof(err_mask))
		return -EINVAL;
	if (copy_from_sockptr(&err_mask, optval, sizeof(err_mask)))
		return -EFAULT;

	lock_sock(sk);

	dev = ro->dev;

	if (ro->bound) {
		if (dev && dev->reg_state != NETREG_REGISTERED) {
			err = -ENODEV;
			goto out;
		}

		err = lin_raw_enable_errfilter(sock_net(sk), dev, sk, err_mask);
		if (err)
			goto out;

		lin_raw_disable_errfilter(sock_net(sk), dev, sk, ro->err_mask);
	}

	ro->err_mask = err_mask;

out:
	release_sock(sk);
	return err;
}

static int lin_raw_set_flag(struct sock *sk, int optname, sockptr_t optval,
			    unsigned int optlen)
{
	struct lin_raw_sock *ro = lin_raw_sk(sk);
	int flag;

	if (optlen != sizeof(flag))
		return -EINVAL;
	if (copy_from_sockptr(&flag, optval, sizeof(flag)))
		return -EFAULT;

	lock_sock(sk);

	switch (optname) {
	case LIN_RAW_LOOPBACK:
		ro->lin.loopback = !!flag;
		break;
	case LIN_RAW_RECV_OWN_MSGS:
		ro->lin.recv_own_msgs = !!flag;
		break;
	case LIN_RAW_JOIN_FILTERS:
		ro->join_filters = !!flag;
		break;
	default:
		/* The dispatcher gates which optnames reach here; this
		 * arm exists so future flag-style sockopts that forget
		 * to update the switch produce a no-op rather than
		 * silently writing to whichever bitfield happens to be
		 * selected.
		 */
		break;
	}

	release_sock(sk);
	return 0;
}

/* LIN_RAW_MASTER: claim or release master role on the bound interface. */

static int lin_raw_set_master(struct sock *sk, sockptr_t optval,
			      unsigned int optlen)
{
	struct lin_raw_sock *ro = lin_raw_sk(sk);
	struct lin_dev *ld;
	int flag, err = 0;

	if (optlen != sizeof(flag))
		return -EINVAL;
	if (copy_from_sockptr(&flag, optval, sizeof(flag)))
		return -EFAULT;

	lock_sock(sk);

	if (!ro->bound || !ro->dev) {
		err = -EOPNOTSUPP;
		goto out;
	}
	if (ro->dev->reg_state != NETREG_REGISTERED) {
		err = -ENODEV;
		goto out;
	}

	ld = lin_get_ml_priv(ro->dev);
	if (!ld) {
		err = -ENODEV;
		goto out;
	}

	mutex_lock(&ld->policy_lock);
	/* Re-test IFF_UP under policy_lock and combine with going_down
	 * (see struct lin_dev kdoc): together they close the dev_close
	 * race window between NETDEV_GOING_DOWN and IFF_UP clearing.
	 */
	if (!(ro->dev->flags & IFF_UP) || ld->going_down) {
		mutex_unlock(&ld->policy_lock);
		err = -ENETDOWN;
		goto out;
	}
	if (flag) {
		if (!ro->is_master) {
			err = lin_master_claim(ro->dev, sk);
			if (!err)
				ro->is_master = 1;
		}
	} else if (ro->is_master) {
		/* Release is best-effort and always succeeds at the core
		 * level; a misbehaving driver surfaces via dmesg, not via
		 * a propagated errno (see lin_master_release()'s kdoc).
		 */
		lin_master_release(ro->dev, sk);
		ro->is_master = 0;
	}
	mutex_unlock(&ld->policy_lock);

out:
	release_sock(sk);
	return err;
}

static int lin_raw_setsockopt(struct socket *sock, int level, int optname,
			      sockptr_t optval, unsigned int optlen)
{
	struct sock *sk = sock->sk;

	if (level != SOL_LIN_RAW)
		return -EINVAL;

	switch (optname) {
	case LIN_RAW_FILTER:
		return lin_raw_set_filter(sk, optval, optlen);
	case LIN_RAW_ERR_FILTER:
		return lin_raw_set_err_filter(sk, optval, optlen);
	case LIN_RAW_LOOPBACK:
	case LIN_RAW_RECV_OWN_MSGS:
	case LIN_RAW_JOIN_FILTERS:
		return lin_raw_set_flag(sk, optname, optval, optlen);
	case LIN_RAW_MASTER:
		return lin_raw_set_master(sk, optval, optlen);
	default:
		return -ENOPROTOOPT;
	}
}

static int lin_raw_getsockopt(struct socket *sock, int level, int optname,
			      char __user *optval, int __user *optlen)
{
	struct sock *sk = sock->sk;
	struct lin_raw_sock *ro = lin_raw_sk(sk);
	int len, val, err = 0;
	void *data = NULL;

	if (level != SOL_LIN_RAW)
		return -EINVAL;

	if (get_user(len, optlen))
		return -EFAULT;
	if (len < 0)
		return -EINVAL;

	switch (optname) {
	case LIN_RAW_FILTER:
		lock_sock(sk);
		if (ro->count > 0) {
			int fsize = ro->count * sizeof(struct lin_filter);

			/* User buffer too small? Surface the required size
			 * and -ERANGE rather than silently returning a
			 * truncated filter list, matching the SocketCAN
			 * convention for CAN_RAW_FILTER getsockopt. The
			 * caller can re-allocate and retry.
			 */
			if (len < fsize) {
				err = -ERANGE;
				release_sock(sk);
				if (put_user(fsize, optlen))
					err = -EFAULT;
				return err;
			}
			len = fsize;
			if (copy_to_user(optval, ro->filter, len))
				err = -EFAULT;
		} else {
			len = 0;
		}
		release_sock(sk);
		if (!err)
			err = put_user(len, optlen);
		return err;

	case LIN_RAW_ERR_FILTER:
		if (len < sizeof(__u32))
			return -EINVAL;
		len = sizeof(__u32);
		data = &ro->err_mask;
		break;

	case LIN_RAW_LOOPBACK:
		if (len < sizeof(int))
			return -EINVAL;
		len = sizeof(int);
		val = ro->lin.loopback;
		data = &val;
		break;

	case LIN_RAW_RECV_OWN_MSGS:
		if (len < sizeof(int))
			return -EINVAL;
		len = sizeof(int);
		val = ro->lin.recv_own_msgs;
		data = &val;
		break;

	case LIN_RAW_JOIN_FILTERS:
		if (len < sizeof(int))
			return -EINVAL;
		len = sizeof(int);
		val = ro->join_filters;
		data = &val;
		break;

	default:
		return -ENOPROTOOPT;
	}

	if (put_user(len, optlen))
		return -EFAULT;
	if (copy_to_user(optval, data, len))
		return -EFAULT;
	return 0;
}

static int lin_raw_recvmsg(struct socket *sock, struct msghdr *msg,
			   size_t size, int flags)
{
	struct sock *sk = sock->sk;
	struct sk_buff *skb;
	int err = 0;

	skb = skb_recv_datagram(sk, flags, &err);
	if (!skb)
		return err;

	if (size < skb->len)
		msg->msg_flags |= MSG_TRUNC;
	else
		size = skb->len;

	err = memcpy_to_msg(msg, skb->data, size);
	if (err < 0) {
		skb_free_datagram(sk, skb);
		return err;
	}

	sock_recv_cmsgs(msg, sk, skb);

	if (msg->msg_name) {
		__sockaddr_check_size(sizeof(struct sockaddr_lin));
		msg->msg_namelen = sizeof(struct sockaddr_lin);
		memcpy(msg->msg_name, skb->cb, msg->msg_namelen);
	}

	skb_free_datagram(sk, skb);

	return size;
}

static const struct proto_ops lin_raw_ops = {
	.family		= PF_LIN,
	.release	= lin_raw_release,
	.bind		= lin_raw_bind,
	.connect	= sock_no_connect,
	.socketpair	= sock_no_socketpair,
	.accept		= sock_no_accept,
	.getname	= lin_raw_getname,
	.poll		= datagram_poll,
	.ioctl		= lin_raw_sock_no_ioctlcmd,
	.gettstamp	= sock_gettstamp,
	.listen		= sock_no_listen,
	.shutdown	= sock_no_shutdown,
	.setsockopt	= lin_raw_setsockopt,
	.getsockopt	= lin_raw_getsockopt,
	.sendmsg	= sock_no_sendmsg,
	.recvmsg	= lin_raw_recvmsg,
	.mmap		= sock_no_mmap,
};

static struct proto lin_raw_proto __read_mostly = {
	.name		= "LIN_RAW",
	.owner		= THIS_MODULE,
	.obj_size	= sizeof(struct lin_raw_sock),
	.init		= lin_raw_init,
};

static const struct lin_proto lin_raw_protocol = {
	.type			= SOCK_RAW,
	.protocol		= LIN_RAW,
	.ops			= &lin_raw_ops,
	.prot			= &lin_raw_proto,
};

static __init int lin_raw_module_init(void)
{
	int err;

	pr_debug("lin: raw protocol\n");

	err = register_netdevice_notifier(&lin_raw_notifier_block);
	if (err)
		return err;

	err = lin_proto_register(&lin_raw_protocol);
	if (err) {
		pr_err("lin: registration of raw protocol failed\n");
		unregister_netdevice_notifier(&lin_raw_notifier_block);
	}

	return err;
}

static __exit void lin_raw_module_exit(void)
{
	lin_proto_unregister(&lin_raw_protocol);
	unregister_netdevice_notifier(&lin_raw_notifier_block);
}

module_init(lin_raw_module_init);
module_exit(lin_raw_module_exit);
