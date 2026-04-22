// SPDX-License-Identifier: (GPL-2.0 OR BSD-3-Clause)
/*
 * raw.c - Raw sockets for protocol family LIN
 *
 * Author: Kyle Bader <kyle.bader94@gmail.com>
 * Copyright (c) 2026 Kyle Bader
 *
 * Modelled on net/can/raw.c. This initial revision is the scaffolding
 * required for socket() / close(): every protocol operation other than
 * those two returns -EOPNOTSUPP, to be filled in by subsequent commits
 * (bind, filter, publish, master claim, schedule).
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/net.h>
#include <linux/socket.h>
#include <linux/lin.h>
#include <linux/lin/core.h>
#include <linux/lin/raw.h>
#include <net/sock.h>

MODULE_DESCRIPTION("PF_LIN raw protocol");
MODULE_LICENSE("Dual BSD/GPL");
MODULE_AUTHOR("Kyle Bader <kyle.bader94@gmail.com>");
MODULE_ALIAS("lin-proto-1");

/*
 * Per-socket state for LIN_RAW. Most fields are added in the commits
 * that implement bind / filter / publish / master / schedule; this
 * scaffolding commit defines only the outer container.
 */
struct lin_raw_sock {
	struct lin_sock lin;	/* must be first; embeds struct sock and the
				 * common loopback / recv_own_msgs flags - see
				 * <linux/lin/core.h>
				 */
};

static int lin_raw_init(struct sock *sk)
{
	return 0;
}

static int lin_raw_release(struct socket *sock)
{
	struct sock *sk = sock->sk;
	struct net *net;

	if (!sk)
		return 0;

	net = sock_net(sk);

	lock_sock(sk);

	sock_orphan(sk);
	sock->sk = NULL;

	release_sock(sk);

	sock_prot_inuse_add(net, sk->sk_prot, -1);
	sock_put(sk);

	return 0;
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

static int lin_raw_setsockopt(struct socket *sock, int level, int optname,
			      sockptr_t optval, unsigned int optlen)
{
	if (level != SOL_LIN_RAW)
		return -EINVAL;

	return -EOPNOTSUPP;
}

static int lin_raw_getsockopt(struct socket *sock, int level, int optname,
			      char __user *optval, int __user *optlen)
{
	if (level != SOL_LIN_RAW)
		return -EINVAL;

	return -EOPNOTSUPP;
}

static const struct proto_ops lin_raw_ops = {
	.family		= PF_LIN,
	.release	= lin_raw_release,
	.bind		= sock_no_bind,
	.connect	= sock_no_connect,
	.socketpair	= sock_no_socketpair,
	.accept		= sock_no_accept,
	.getname	= sock_no_getname,
	.poll		= datagram_poll,
	.ioctl		= lin_raw_sock_no_ioctlcmd,
	.gettstamp	= sock_gettstamp,
	.listen		= sock_no_listen,
	.shutdown	= sock_no_shutdown,
	.setsockopt	= lin_raw_setsockopt,
	.getsockopt	= lin_raw_getsockopt,
	.sendmsg	= sock_no_sendmsg,
	.recvmsg	= sock_no_recvmsg,
	.mmap		= sock_no_mmap,
};

static struct proto lin_raw_proto __read_mostly = {
	.name		= "LIN_RAW",
	.owner		= THIS_MODULE,
	.obj_size	= sizeof(struct lin_raw_sock),
	.init		= lin_raw_init,
};

static const struct lin_proto lin_raw_protocol = {
	.type		= SOCK_RAW,
	.protocol	= LIN_RAW,
	.ops		= &lin_raw_ops,
	.prot		= &lin_raw_proto,
};

static __init int lin_raw_module_init(void)
{
	int err;

	pr_debug("lin: raw protocol\n");

	err = lin_proto_register(&lin_raw_protocol);
	if (err)
		pr_err("lin: registration of raw protocol failed\n");

	return err;
}

static __exit void lin_raw_module_exit(void)
{
	lin_proto_unregister(&lin_raw_protocol);
}

module_init(lin_raw_module_init);
module_exit(lin_raw_module_exit);
