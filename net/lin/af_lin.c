// SPDX-License-Identifier: (GPL-2.0 OR BSD-3-Clause)
/*
 * af_lin.c - Protocol family LIN core module
 *            (used by different LIN protocol modules)
 *
 * Author: Kyle Bader <kyle.bader94@gmail.com>
 * Copyright (c) 2026 Kyle Bader
 *
 * Modelled on net/can/af_can.c.
 */

#include <linux/init.h>
#include <linux/kmod.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/net.h>
#include <linux/rcupdate.h>
#include <linux/skbuff.h>
#include <linux/socket.h>
#include <linux/stddef.h>
#include <linux/lin.h>
#include <linux/lin/core.h>
#include <net/net_namespace.h>
#include <net/sock.h>

MODULE_DESCRIPTION("Local Interconnect Network PF_LIN core");
MODULE_LICENSE("Dual BSD/GPL");
MODULE_AUTHOR("Kyle Bader <kyle.bader94@gmail.com>");

MODULE_ALIAS_NETPROTO(PF_LIN);

/* Table of registered LIN protocols, indexed by protocol id. */
static const struct lin_proto __rcu *proto_tab[LIN_NPROTO] __read_mostly;
static DEFINE_MUTEX(proto_tab_lock);

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

/* af_lin module init / exit */

static const struct net_proto_family lin_family_ops = {
	.family = PF_LIN,
	.create = lin_create,
	.owner  = THIS_MODULE,
};

static __init int lin_init(void)
{
	int err;

	pr_debug("lin: local interconnect network core\n");

	err = sock_register(&lin_family_ops);
	if (err)
		return err;

	return 0;
}

static __exit void lin_exit(void)
{
	sock_unregister(PF_LIN);
	rcu_barrier();
}

module_init(lin_init);
module_exit(lin_exit);
