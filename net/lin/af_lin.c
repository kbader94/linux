// SPDX-License-Identifier: (GPL-2.0 OR BSD-3-Clause)
/*
 * af_lin.c - Protocol family LIN core module
 *            (used by different LIN protocol modules)
 *
 * Author: Kyle Bader <kyle.bader94@gmail.com>
 * Copyright (c) 2026 Kyle Bader
 *
 * Modelled on net/can/af_can.c, with a LIN-native rx filter table:
 * LIN's 6-bit frame ID space is small enough to bucket subscribers
 * directly into a 64-slot array rather than CAN's id/mask hashlists.
 */

#include <linux/if_arp.h>
#include <linux/if_ether.h>
#include <linux/init.h>
#include <linux/kmod.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/net.h>
#include <linux/netdevice.h>
#include <linux/rcupdate.h>
#include <linux/skbuff.h>
#include <linux/slab.h>
#include <linux/socket.h>
#include <linux/stddef.h>
#include <linux/lin.h>
#include <linux/lin/core.h>
#include <linux/lin/dev.h>
#include <linux/lin/skb.h>
#include <net/net_namespace.h>
#include <net/netns/generic.h>
#include <net/sock.h>

MODULE_DESCRIPTION("Local Interconnect Network PF_LIN core");
MODULE_LICENSE("Dual BSD/GPL");
MODULE_AUTHOR("Kyle Bader <kyle.bader94@gmail.com>");

MODULE_ALIAS_NETPROTO(PF_LIN);

/* Table of registered LIN protocols, indexed by protocol id. */
static const struct lin_proto __rcu *proto_tab[LIN_NPROTO] __read_mostly;
static DEFINE_MUTEX(proto_tab_lock);

/* Monotonic counter used to stamp lin_skb_priv.skbcnt on rx so
 * protocol modules can deduplicate a single frame across overlapping
 * filter matches (see struct lin_uniqframe in raw.c).
 */
static atomic_t lin_skbcounter = ATOMIC_INIT(0);

/* Per-socket subscription entry. Lives in a hlist on either a
 * lin_dev->rcv_lists bucket (for ifindex-bound subscriptions) or on
 * the per-netns all-devices rcv_lists (for ifindex-0 bound sockets).
 * Shared by all LIN protocol modules; lin_rx_register() allocates it
 * and lin_rx_unregister() releases it via call_rcu.
 */
struct lin_receiver {
	struct hlist_node	list;
	struct rcu_head		rcu;
	struct sock		*sk;

	u8			lin_id;
	u8			id_mask;
	u8			flags;		/* LIN_F_* match bits */
	u8			flags_mask;
	u32			err_mask;	/* 0 => data-frame filter */

	void			(*func)(struct sk_buff *skb, void *data);
	void			*data;
	const char		*ident;
	unsigned long		matches;
};

/* Per-network-namespace state. Holds the "ifindex=0" subscriber lists
 * and a single lock guarding rcv_lists mutations across every LIN
 * netdev in this namespace.
 */
struct lin_pernet {
	struct lin_dev_rcv_lists	rx_alldev_list;
	/* serialises rcv-list mutations across every LIN netdev in this
	 * namespace; rx walkers iterate the lists under rcu_read_lock().
	 */
	spinlock_t			rcvlists_lock;
};

static unsigned int lin_pernet_id __read_mostly;

static inline struct lin_pernet *lin_pernet(struct net *net)
{
	return net_generic(net, lin_pernet_id);
}

static struct kmem_cache *lin_rcv_cache __read_mostly;

/* af_lin socket helpers */

void lin_sock_destruct(struct sock *sk)
{
	skb_queue_purge(&sk->sk_receive_queue);
	skb_queue_purge(&sk->sk_error_queue);
}
EXPORT_SYMBOL(lin_sock_destruct);

static const struct lin_proto *lin_get_proto(int protocol)
{
	const struct lin_proto *lp;

	rcu_read_lock();
	lp = rcu_dereference(proto_tab[protocol]);
	if (lp && !try_module_get(lp->prot->owner))
		lp = NULL;
	rcu_read_unlock();

	return lp;
}

static inline void lin_put_proto(const struct lin_proto *lp)
{
	module_put(lp->prot->owner);
}

static int lin_create(struct net *net, struct socket *sock, int protocol,
		      int kern)
{
	const struct lin_proto *lp;
	struct sock *sk;
	int err = 0;

	sock->state = SS_UNCONNECTED;

	if (protocol < 0 || protocol >= LIN_NPROTO)
		return -EINVAL;

	lp = lin_get_proto(protocol);

#ifdef CONFIG_MODULES
	if (!lp) {
		/* try to load the matching protocol module */
		err = request_module("lin-proto-%d", protocol);
		if (err)
			pr_err_ratelimited("lin: request_module (lin-proto-%d) failed.\n",
					   protocol);

		lp = lin_get_proto(protocol);
	}
#endif

	if (!lp)
		return -EPROTONOSUPPORT;

	if (lp->type != sock->type) {
		err = -EPROTOTYPE;
		goto errout;
	}

	sock->ops = lp->ops;

	sk = sk_alloc(net, PF_LIN, GFP_KERNEL, lp->prot, kern);
	if (!sk) {
		err = -ENOMEM;
		goto errout;
	}

	/* sk_alloc() leaves sk_protocol at zero; record the LIN protocol
	 * id so lin_loopback_rx() can find the protocol's lin_proto via
	 * proto_tab[sk->sk_protocol] when querying the per-socket
	 * loopback flag.
	 */
	sk->sk_protocol = protocol;

	sock_init_data(sock, sk);
	sk->sk_destruct = lin_sock_destruct;

	if (sk->sk_prot->init)
		err = sk->sk_prot->init(sk);

	if (err) {
		sock_orphan(sk);
		sock_put(sk);
		sock->sk = NULL;
	} else {
		sock_prot_inuse_add(net, sk->sk_prot, 1);
	}

errout:
	lin_put_proto(lp);
	return err;
}

/* af_lin protocol registration */

/**
 * lin_proto_register - register a LIN transport protocol
 * @lp: pointer to LIN protocol registration structure
 *
 * Return:
 *  0 on success
 *  -EINVAL invalid (out of range) protocol number
 *  -EBUSY  protocol already in use
 *  any error from proto_register()
 */
int lin_proto_register(const struct lin_proto *lp)
{
	int proto = lp->protocol;
	int err;

	if (proto < 0 || proto >= LIN_NPROTO) {
		pr_err("lin: protocol number %d out of range\n", proto);
		return -EINVAL;
	}

	err = proto_register(lp->prot, 0);
	if (err)
		return err;

	mutex_lock(&proto_tab_lock);

	if (rcu_access_pointer(proto_tab[proto])) {
		pr_err("lin: protocol %d already registered\n", proto);
		err = -EBUSY;
	} else {
		RCU_INIT_POINTER(proto_tab[proto], lp);
	}

	mutex_unlock(&proto_tab_lock);

	if (err)
		proto_unregister(lp->prot);

	return err;
}
EXPORT_SYMBOL(lin_proto_register);

/**
 * lin_proto_unregister - unregister a previously registered LIN protocol
 * @lp: pointer to LIN protocol registration structure
 */
void lin_proto_unregister(const struct lin_proto *lp)
{
	int proto = lp->protocol;

	if (WARN_ON_ONCE(proto < 0 || proto >= LIN_NPROTO)) {
		pr_err("lin: protocol number %d out of range\n", proto);
		return;
	}

	mutex_lock(&proto_tab_lock);
	if (WARN_ON_ONCE(rcu_access_pointer(proto_tab[proto]) != lp)) {
		/* Mismatched register/unregister or double-unregister.
		 * Leave the slot alone — it may belong to a different
		 * caller — and skip proto_unregister() too: we can't
		 * trust @lp to describe state we own. The warning carries
		 * a stack trace for the buggy caller to act on.
		 *
		 * Consequence the buggy caller should know about: the
		 * proto_register() allocation done at register time is
		 * intentionally NOT undone here. Tearing it down would
		 * require trusting that @lp->prot belongs to this caller,
		 * which the slot mismatch already disproves. Better to
		 * leak the proto registration (recoverable, visible in
		 * /proc/net/protocols) than to free state owned by an
		 * unrelated subsystem. Fix the caller.
		 */
		mutex_unlock(&proto_tab_lock);
		return;
	}
	RCU_INIT_POINTER(proto_tab[proto], NULL);
	mutex_unlock(&proto_tab_lock);

	synchronize_rcu();

	proto_unregister(lp->prot);
}
EXPORT_SYMBOL(lin_proto_unregister);

/* rx subscription list management */

static struct lin_dev_rcv_lists *
lin_find_rcv_lists(struct net *net, struct net_device *dev)
{
	if (dev)
		return &lin_get_ml_priv(dev)->rcv_lists;

	return &lin_pernet(net)->rx_alldev_list;
}

/* Pick the bucket for a new receiver based on its filter shape. */
static struct hlist_head *
lin_rcv_bucket(struct lin_dev_rcv_lists *rl, __u8 *lin_id, __u8 *id_mask,
	       __u8 *flags, __u8 flags_mask, __u32 err_mask)
{
	bool inv = *flags & LIN_FILT_INV;

	/* strip the routing bit so match comparisons use only LIN_F_* */
	*flags &= ~LIN_FILT_INV;

	if (err_mask)
		return &rl->err;

	if (inv)
		return &rl->inv;

	/* normalize the ID to the masked bits so by_id lookups are stable */
	*lin_id &= *id_mask;

	/* Single-ID filters land in by_id[]; the dispatch walker there
	 * also checks flags_mask so flag-constrained single-ID filters
	 * behave correctly.
	 */
	if (*id_mask == LIN_ID_MASK)
		return &rl->by_id[*lin_id & LIN_ID_MASK];

	/* Only filters that impose no constraint at all go in match_all,
	 * because the match_all walker performs unconditional delivery.
	 * Anything with a flags_mask constraint must land in the generic
	 * filter bucket so the walker checks flags.
	 */
	if (*id_mask == 0 && flags_mask == 0)
		return &rl->match_all;

	return &rl->filter;
}

int lin_rx_register(struct net *net, struct net_device *dev,
		    __u8 lin_id, __u8 id_mask, __u8 flags, __u8 flags_mask,
		    __u32 err_mask,
		    void (*func)(struct sk_buff *, void *),
		    void *data, const char *ident, struct sock *sk)
{
	struct lin_pernet *lp = lin_pernet(net);
	struct lin_dev_rcv_lists *rl;
	struct hlist_head *bucket;
	struct lin_receiver *r;

	if (dev && (dev->type != ARPHRD_LIN || !lin_get_ml_priv(dev)))
		return -ENODEV;
	if (dev && !net_eq(net, dev_net(dev)))
		return -ENODEV;

	r = kmem_cache_alloc(lin_rcv_cache, GFP_KERNEL);
	if (!r)
		return -ENOMEM;

	spin_lock_bh(&lp->rcvlists_lock);

	rl = lin_find_rcv_lists(net, dev);
	bucket = lin_rcv_bucket(rl, &lin_id, &id_mask, &flags, flags_mask,
				err_mask);

	r->lin_id	= lin_id;
	r->id_mask	= id_mask;
	r->flags	= flags;
	r->flags_mask	= flags_mask;
	r->err_mask	= err_mask;
	r->func		= func;
	r->data		= data;
	r->ident	= ident;
	r->matches	= 0;
	r->sk		= sk;

	hlist_add_head_rcu(&r->list, bucket);
	rl->entries++;

	spin_unlock_bh(&lp->rcvlists_lock);
	return 0;
}
EXPORT_SYMBOL(lin_rx_register);

static void lin_rx_delete_receiver(struct rcu_head *rp)
{
	struct lin_receiver *r = container_of(rp, struct lin_receiver, rcu);
	struct sock *sk = r->sk;

	kmem_cache_free(lin_rcv_cache, r);
	if (sk)
		sock_put(sk);
}

void lin_rx_unregister(struct net *net, struct net_device *dev,
		       __u8 lin_id, __u8 id_mask, __u8 flags, __u8 flags_mask,
		       __u32 err_mask,
		       void (*func)(struct sk_buff *, void *),
		       void *data)
{
	struct lin_pernet *lp = lin_pernet(net);
	struct lin_dev_rcv_lists *rl;
	struct hlist_head *bucket;
	struct lin_receiver *r = NULL;
	__u8 norm_flags = flags;
	__u8 norm_id_mask = id_mask;
	__u8 norm_lin_id = lin_id;

	if (dev && (dev->type != ARPHRD_LIN || !lin_get_ml_priv(dev)))
		return;
	if (dev && !net_eq(net, dev_net(dev)))
		return;

	spin_lock_bh(&lp->rcvlists_lock);

	rl = lin_find_rcv_lists(net, dev);
	bucket = lin_rcv_bucket(rl, &norm_lin_id, &norm_id_mask, &norm_flags,
				flags_mask, err_mask);

	hlist_for_each_entry_rcu(r, bucket, list,
				 lockdep_is_held(&lp->rcvlists_lock)) {
		if (r->lin_id == norm_lin_id &&
		    r->id_mask == norm_id_mask &&
		    r->flags == norm_flags &&
		    r->flags_mask == flags_mask &&
		    r->err_mask == err_mask &&
		    r->func == func &&
		    r->data == data)
			break;
	}

	if (WARN_ONCE(!r,
		      "lin: receive list entry not found for dev %s, id 0x%02x, mask 0x%02x\n",
		      LIN_DNAME(dev), lin_id, id_mask))
		goto out;

	hlist_del_rcu(&r->list);
	if (rl->entries > 0)
		rl->entries--;

out:
	spin_unlock_bh(&lp->rcvlists_lock);

	if (r) {
		if (r->sk)
			sock_hold(r->sk);
		call_rcu(&r->rcu, lin_rx_delete_receiver);
	}
}
EXPORT_SYMBOL(lin_rx_unregister);

/* rx dispatch */

static inline void lin_deliver(struct sk_buff *skb, struct lin_receiver *r)
{
	r->func(skb, r->data);
	r->matches++;
}

static int lin_rcv_filter(struct lin_dev_rcv_lists *rl, struct sk_buff *skb)
{
	const struct lin_frame *lf = (const struct lin_frame *)skb->data;
	struct lin_receiver *r;
	int matches = 0;

	if (rl->entries == 0)
		return 0;

	if (lf->flags & LIN_F_ERR) {
		hlist_for_each_entry_rcu(r, &rl->err, list) {
			if (lf->err_mask & r->err_mask) {
				lin_deliver(skb, r);
				matches++;
			}
		}
		return matches;
	}

	/* Single-ID bucket: every entry matches by ID; check flag constraint */
	hlist_for_each_entry_rcu(r, &rl->by_id[lf->lin_id & LIN_ID_MASK],
				 list) {
		if ((lf->flags & r->flags_mask) ==
		    (r->flags & r->flags_mask)) {
			lin_deliver(skb, r);
			matches++;
		}
	}

	/* Match-all bucket: unconditional delivery */
	hlist_for_each_entry_rcu(r, &rl->match_all, list) {
		lin_deliver(skb, r);
		matches++;
	}

	/* Generic mask filter */
	hlist_for_each_entry_rcu(r, &rl->filter, list) {
		if ((lf->lin_id & r->id_mask) ==
		    (r->lin_id & r->id_mask) &&
		    (lf->flags & r->flags_mask) ==
		    (r->flags & r->flags_mask)) {
			lin_deliver(skb, r);
			matches++;
		}
	}

	/* Inverted filter */
	hlist_for_each_entry_rcu(r, &rl->inv, list) {
		if ((lf->lin_id & r->id_mask) !=
		    (r->lin_id & r->id_mask) ||
		    (lf->flags & r->flags_mask) !=
		    (r->flags & r->flags_mask)) {
			lin_deliver(skb, r);
			matches++;
		}
	}

	return matches;
}

static void lin_receive(struct sk_buff *skb, struct net_device *dev)
{
	struct lin_dev *ld = lin_get_ml_priv(dev);
	struct net *net = dev_net(dev);

	/* Stamp a unique skb identifier so protocol rx paths can dedup
	 * matches across overlapping filters. Drivers may pre-populate
	 * this field (for locally-synthesised loopback); only stamp
	 * when still zero.
	 */
	while (!lin_skb_prv(skb)->skbcnt)
		lin_skb_prv(skb)->skbcnt = atomic_inc_return(&lin_skbcounter);

	rcu_read_lock();
	lin_rcv_filter(&lin_pernet(net)->rx_alldev_list, skb);
	lin_rcv_filter(&ld->rcv_lists, skb);
	rcu_read_unlock();

	consume_skb(skb);
}

/* af_lin rx packet type handler */

static int lin_rcv(struct sk_buff *skb, struct net_device *dev,
		   struct packet_type *pt, struct net_device *orig_dev)
{
	if (unlikely(dev->type != ARPHRD_LIN ||
		     !lin_get_ml_priv(dev) ||
		     !lin_is_lin_skb(skb))) {
		pr_warn_once("PF_LIN: dropped non-conform LIN skbuff: dev type %d, len %d\n",
			     dev->type, skb->len);
		kfree_skb(skb);
		return NET_RX_DROP;
	}

	lin_receive(skb, dev);
	return NET_RX_SUCCESS;
}

/* Pernet init / exit */

static int __net_init lin_pernet_init(struct net *net)
{
	struct lin_pernet *lp = lin_pernet(net);

	spin_lock_init(&lp->rcvlists_lock);
	lin_dev_rcv_lists_init(&lp->rx_alldev_list);
	return 0;
}

static void __net_exit lin_pernet_exit(struct net *net)
{
	struct lin_pernet *lp = lin_pernet(net);
	struct lin_dev_rcv_lists *rl = &lp->rx_alldev_list;
	unsigned int i;

	/* By the time a network namespace is torn down, every socket
	 * inside it should already be dead, and every lin_rx_register()
	 * subscription on the all-devices list should have been matched
	 * by a lin_rx_unregister(). A non-empty bucket here is a
	 * subscription leak — flag it loudly so the responsible protocol
	 * module gets fixed rather than silently leaving call_rcu-pinned
	 * sock references stranded.
	 */
	for (i = 0; i <= LIN_ID_MASK; i++)
		WARN_ONCE(!hlist_empty(&rl->by_id[i]),
			  "PF_LIN: pernet exit with non-empty rx_alldev_list by_id[%u]\n",
			  i);
	WARN_ONCE(!hlist_empty(&rl->match_all),
		  "PF_LIN: pernet exit with non-empty rx_alldev_list match_all\n");
	WARN_ONCE(!hlist_empty(&rl->filter),
		  "PF_LIN: pernet exit with non-empty rx_alldev_list filter\n");
	WARN_ONCE(!hlist_empty(&rl->inv),
		  "PF_LIN: pernet exit with non-empty rx_alldev_list inv\n");
	WARN_ONCE(!hlist_empty(&rl->err),
		  "PF_LIN: pernet exit with non-empty rx_alldev_list err\n");
	WARN_ONCE(rl->entries != 0,
		  "PF_LIN: pernet exit with rx_alldev_list entries=%d\n",
		  rl->entries);
}

static struct pernet_operations lin_pernet_ops __read_mostly = {
	.init	= lin_pernet_init,
	.exit	= lin_pernet_exit,
	.id	= &lin_pernet_id,
	.size	= sizeof(struct lin_pernet),
};

/* af_lin module init / exit */

static const struct net_proto_family lin_family_ops = {
	.family = PF_LIN,
	.create = lin_create,
	.owner  = THIS_MODULE,
};

static struct packet_type lin_packet __read_mostly = {
	.type = cpu_to_be16(ETH_P_LIN),
	.func = lin_rcv,
};

/* Core netdev notifier — manages ld->going_down across the dev_close
 * window so policy ops cannot race ndo_stop().
 *
 * Background. dev_close() runs under rtnl_lock and unfolds as:
 *
 *     1. call_netdevice_notifiers(NETDEV_GOING_DOWN)   IFF_UP still set
 *     2. ops->ndo_stop(dev)
 *     3. dev->flags &= ~IFF_UP
 *     4. call_netdevice_notifiers(NETDEV_DOWN)         IFF_UP cleared
 *
 * A LIN policy sockopt runs without rtnl_lock. If it sampled IFF_UP
 * anywhere between steps 1 and 3 the test would pass, the sockopt
 * would acquire ld->policy_lock and call into a driver op while
 * ndo_stop() was still running on the rtnl-holding CPU.
 *
 * socketCAN does not have this race: every CAN emission routes
 * through dev_queue_xmit() and inherits the standard tx pipeline's
 * dev_deactivate() drain. socketCAN also carries no cross-socket
 * policy state. LIN tracks per-interface master / publisher /
 * schedule ownership and calls driver ops outside the tx pipeline,
 * so the dev_deactivate guarantee does not apply to us; the LIN core
 * must build the equivalent.
 *
 * That equivalent is @policy_lock + @going_down. This notifier owns
 * the flag:
 *
 *   - NETDEV_GOING_DOWN (step 1, IFF_UP still set, driver still alive):
 *       take policy_lock, set going_down = true, drop the lock.
 *       Any sockopt currently blocked on policy_lock observes
 *       going_down on wakeup and bails with -ENETDOWN without
 *       calling any driver op. Per-protocol notifiers (priority 0;
 *       this one runs at priority 1) then perform their per-socket
 *       force-release while the driver is still safe to call.
 *
 *   - NETDEV_UP: clear going_down so policy ops are accepted again.
 *       Userspace re-establishes master / publishers / schedules
 *       explicitly — the kernel does not replay them across a
 *       down/up cycle. Doing this in the core notifier (rather than
 *       in a per-socket walk) keeps the flag lifecycle correct even
 *       when no LIN socket is bound across the bounce.
 *
 * NETDEV_DOWN is handled per-protocol (sk_err = ENETDOWN) and does
 * not require any policy-state work — by then IFF_UP is cleared and
 * the IFF_UP gate in sockopts catches new entrants. NETDEV_UNREGISTER
 * is also per-protocol (full socket unbind).
 *
 * Lock ordering: rtnl_lock (held by dev_close caller) ->
 * ld->policy_lock. We never take rtnl ourselves here.
 */
static int lin_netdev_event(struct notifier_block *nb, unsigned long msg,
			    void *ptr)
{
	struct net_device *dev = netdev_notifier_info_to_dev(ptr);
	struct lin_dev *ld;

	if (dev->type != ARPHRD_LIN)
		return NOTIFY_DONE;
	if (msg != NETDEV_GOING_DOWN && msg != NETDEV_UP)
		return NOTIFY_DONE;

	ld = lin_get_ml_priv(dev);
	if (!ld)
		return NOTIFY_DONE;

	mutex_lock(&ld->policy_lock);
	switch (msg) {
	case NETDEV_GOING_DOWN:
		ld->going_down = true;
		break;
	case NETDEV_UP:
		ld->going_down = false;
		break;
	}
	mutex_unlock(&ld->policy_lock);

	return NOTIFY_DONE;
}

/* Priority 1 so this core notifier runs before any protocol notifier
 * (the LIN_RAW per-socket notifier registers at the default priority
 * 0). The protocol notifier depends on @going_down already being set
 * when its NETDEV_GOING_DOWN handler runs, so any sockopt that
 * observes the per-socket force-release also observes going_down ==
 * true under policy_lock and refuses to re-claim.
 */
static struct notifier_block lin_netdev_notifier __read_mostly = {
	.notifier_call	= lin_netdev_event,
	.priority	= 1,
};

static __init int lin_init(void)
{
	int err;

	pr_debug("lin: local interconnect network core\n");

	lin_rcv_cache = kmem_cache_create("lin_receiver",
					  sizeof(struct lin_receiver),
					  0, 0, NULL);
	if (!lin_rcv_cache)
		return -ENOMEM;

	err = register_pernet_subsys(&lin_pernet_ops);
	if (err)
		goto out_cache;

	err = register_netdevice_notifier(&lin_netdev_notifier);
	if (err)
		goto out_pernet;

	err = sock_register(&lin_family_ops);
	if (err)
		goto out_notifier;

	dev_add_pack(&lin_packet);

	return 0;

out_notifier:
	unregister_netdevice_notifier(&lin_netdev_notifier);
out_pernet:
	unregister_pernet_subsys(&lin_pernet_ops);
out_cache:
	kmem_cache_destroy(lin_rcv_cache);
	return err;
}

static __exit void lin_exit(void)
{
	dev_remove_pack(&lin_packet);
	sock_unregister(PF_LIN);
	unregister_netdevice_notifier(&lin_netdev_notifier);
	unregister_pernet_subsys(&lin_pernet_ops);

	rcu_barrier();

	kmem_cache_destroy(lin_rcv_cache);
}

module_init(lin_init);
module_exit(lin_exit);
