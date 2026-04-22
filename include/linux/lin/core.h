/* SPDX-License-Identifier: (GPL-2.0 OR BSD-3-Clause) */
/*
 * linux/lin/core.h
 *
 * Prototypes and definitions for LIN protocol modules using the PF_LIN core
 *
 * Author: Kyle Bader <kyle.bader94@gmail.com>
 * Copyright (c) 2026 Kyle Bader
 */

#ifndef _LIN_CORE_H
#define _LIN_CORE_H

#include <linux/lin.h>
#include <linux/netdevice.h>
#include <linux/skbuff.h>
#include <net/sock.h>

#define LIN_DNAME(dev) ((dev) ? (dev)->name : "any")

/**
 * struct lin_proto - LIN protocol registration structure
 * @type:              socket type (e.g. SOCK_RAW for LIN_RAW)
 * @protocol:          LIN protocol identifier (LIN_RAW, ...)
 * @ops:      socket-level proto_ops for sock->ops
 * @prot:     struct proto template passed to sk_alloc()
 */
struct lin_proto {
	int			type;
	int			protocol;
	const struct proto_ops	*ops;
	struct proto		*prot;
};

/**
 * struct lin_sock - common base for every PF_LIN per-socket state.
 * @sk:            kernel sock; must be first so container_of from
 *                 struct sock * works.
 * @loopback:      LIN_RAW_LOOPBACK (default true). The emitter side
 *                 of the loopback policy: lin_loopback_rx() consults
 *                 this on each tagged stakeholder to decide whether
 *                 a self-emitted frame should be synthesised at all
 *                 (mirrors the @loop argument to SocketCAN's
 *                 can_send()).
 * @recv_own_msgs: LIN_RAW_RECV_OWN_MSGS (default false). The receiver
 *                 side: even if synthesis happens, the rx callback
 *                 only delivers a self-emission back to its owner
 *                 when this flag is set (mirrors CAN_RAW_RECV_OWN_MSGS).
 *
 * Every per-protocol sock struct (struct lin_raw_sock, future
 * lin_isotp_sock, ...) embeds this at offset 0. Because @sk is first
 * here and @lin is first in the embedding struct, the conventional
 * lin_<proto>_sk() cast from struct sock * still works at zero
 * offset.
 */
struct lin_sock {
	struct sock	sk;
	bool		loopback;
	bool		recv_own_msgs;
};

/**
 * lin_sk_wants_loopback - read the per-socket LIN_RAW_LOOPBACK flag.
 * @sk: PF_LIN sock (any protocol).
 *
 * Returns true when the sock has opted in to loopback synthesis for
 * its emissions. Called from lin_loopback_rx() while building the
 * OR-vote across an emission's tagged stakeholders.
 */
static inline bool lin_sk_wants_loopback(const struct sock *sk)
{
	return container_of(sk, struct lin_sock, sk)->loopback;
}

/**
 * lin_sk_recv_own_msgs - read the per-socket LIN_RAW_RECV_OWN_MSGS flag.
 * @sk: PF_LIN sock (any protocol).
 *
 * Returns true when the sock wants its own emissions delivered back
 * through the rx path. Consulted by protocol rx callbacks
 * (lin_raw_rcv, future lin_isotp_rcv, ...) when filtering self-
 * emission echoes against the skb's owner tags.
 */
static inline bool lin_sk_recv_own_msgs(const struct sock *sk)
{
	return container_of(sk, struct lin_sock, sk)->recv_own_msgs;
}

/* PF_LIN core API used by protocol modules (af_lin.c). */

int  lin_proto_register(const struct lin_proto *lp);
void lin_proto_unregister(const struct lin_proto *lp);

void lin_sock_destruct(struct sock *sk);

#endif /* !_LIN_CORE_H */
