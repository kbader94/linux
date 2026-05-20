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

/**
 * lin_rx_register - subscribe a socket to matching LIN frames
 * @net:        target network namespace
 * @dev:        target netdev, or NULL to subscribe on every LIN
 *              interface in @net (ifindex-0 / "any" binding)
 * @lin_id:     filter id value (lin_id & id_mask is matched)
 * @id_mask:    filter id mask; LIN_ID_MASK for a single-ID filter,
 *              0 with flags_mask 0 for a "match-all" filter
 * @flags:      LIN_F_* match bits, plus LIN_FILT_INV to invert the match
 * @flags_mask: which LIN_F_* bits participate in matching
 * @err_mask:   LIN_ERR_* mask for error-frame subscription; 0 for a
 *              data-frame filter (the two subscription kinds share one
 *              registration API, with err_mask != 0 routing to the
 *              error list)
 * @func:       callback invoked for each matching skb; must not free
 *              the skb (use skb_clone() if the callback wants to keep
 *              it beyond the call)
 * @data:       opaque pointer passed to @func; typically the owning sock
 * @ident:      debug identifier (e.g. "raw")
 * @sk:         owning sock, or NULL. The caller is responsible for
 *              keeping @sk alive for the duration of the subscription
 *              (typically by calling lin_rx_unregister() before the
 *              owning socket is freed). On unregister, the core takes
 *              a reference on @sk that is dropped only after the RCU
 *              grace period, so concurrent rx walkers that already
 *              observed the receiver can safely deref @sk.
 *
 * Return: 0 on success, -ENODEV if @dev is set but not a LIN netdev,
 *         -ENOMEM if the receiver entry could not be allocated.
 */
int lin_rx_register(struct net *net, struct net_device *dev,
		    __u8 lin_id, __u8 id_mask, __u8 flags, __u8 flags_mask,
		    __u32 err_mask,
		    void (*func)(struct sk_buff *skb, void *data),
		    void *data, const char *ident, struct sock *sk);

/**
 * lin_rx_unregister - remove a previously registered subscription
 *
 * Arguments must match the values passed to lin_rx_register(). At most
 * one matching entry is removed.
 */
void lin_rx_unregister(struct net *net, struct net_device *dev,
		       __u8 lin_id, __u8 id_mask, __u8 flags, __u8 flags_mask,
		       __u32 err_mask,
		       void (*func)(struct sk_buff *skb, void *data),
		       void *data);

#endif /* !_LIN_CORE_H */
