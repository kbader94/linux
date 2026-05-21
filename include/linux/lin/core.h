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

/* Defined in <uapi/linux/lin/raw.h>; only referenced here by pointer
 * in the lin_schedule_load() prototype.
 */
struct lin_schedule;

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

/**
 * lin_rx_register_wakeup - subscribe a socket to bus wakeup signals
 * @net:    target network namespace
 * @dev:    target netdev, or NULL for ifindex-0 binding
 * @func:   callback invoked for each wakeup frame
 * @data:   opaque pointer passed to @func
 * @ident:  debug identifier
 * @sk:     owning sock (for accounting + ref bookkeeping)
 *
 * Wakeup subscriptions have no filter shape: every wakeup-flagged
 * frame on the bound interface (or every LIN interface in @net for
 * ifindex-0) is delivered. Off by default — sockets must call this
 * helper explicitly to receive wakeup events. Used by LIN_RAW's
 * LIN_RAW_WAKEUP_FILTER sockopt.
 */
int  lin_rx_register_wakeup(struct net *net, struct net_device *dev,
			    void (*func)(struct sk_buff *skb, void *data),
			    void *data, const char *ident, struct sock *sk);

/**
 * lin_rx_unregister_wakeup - remove a wakeup subscription
 *
 * Arguments must match the values passed to lin_rx_register_wakeup().
 * At most one matching entry is removed.
 */
void lin_rx_unregister_wakeup(struct net *net, struct net_device *dev,
			      void (*func)(struct sk_buff *skb, void *data),
			      void *data);

/**
 * lin_master_claim - claim the master role on a LIN interface
 * @dev: target LIN netdev (must not be NULL)
 * @sk:  socket acquiring the role
 *
 * Caller must hold ld->policy_lock. Forwards to the driver's
 * master_start op after verifying no other socket holds the role.
 * Takes a reference on @sk which is released by lin_master_release().
 *
 * Return: 0 on success, -EBUSY if another socket holds the claim,
 *         -EOPNOTSUPP if the driver does not support the master role.
 */
int  lin_master_claim(struct net_device *dev, struct sock *sk);

/**
 * lin_master_release - release the master role on a LIN interface
 * @dev: target LIN netdev
 * @sk:  socket releasing the role (must be the current master, else no-op)
 *
 * Caller must hold ld->policy_lock. Best-effort teardown: the master_sk
 * slot is cleared and the driver's master_stop op is invoked. Driver
 * errors are logged via netdev_err but do not block the core slot from
 * freeing — once a caller has asked for release, holding the slot
 * because the driver glitched is strictly worse than letting go.
 *
 * Always returns 0 when @sk is the current master (or no-op 0 when it
 * isn't). Userspace gets a single clear answer ("release succeeded");
 * a misbehaving driver surfaces via dmesg, which is the right channel
 * for the operator / driver author to act on.
 *
 * Acquire is still strict (lin_master_claim propagates driver errors)
 * because a failed claim is a real "can't proceed." A failed release
 * has no useful userspace recovery action — the only meaningful
 * follow-up is close, which converges here anyway.
 *
 * Return: 0 (always).
 */
int  lin_master_release(struct net_device *dev, struct sock *sk);

/**
 * lin_publisher_set - register or update a publisher for a frame ID
 * @dev:    target LIN netdev
 * @sk:     socket asserting publisher ownership
 * @lin_id: 6-bit LIN frame ID
 * @data:   response payload bytes (@len bytes)
 * @len:    length of @data (1..LIN_MAX_DLEN)
 * @enh:    true for enhanced checksum, false for classic
 *
 * Caller must hold ld->policy_lock. On first registration, takes a
 * reference on @sk; subsequent calls by the same socket update the
 * stored data in place. Registrations are sticky — the driver
 * responds to every matching header until the socket explicitly
 * clears the registration or the socket is closed.
 *
 * Diagnostic IDs (0x3C / 0x3D) require the driver to advertise
 * LIN_CAP_DIAG, and must use classic checksum per LIN spec —
 * @enh == true on 0x3C / 0x3D returns -EINVAL.
 *
 * Returns -EBUSY if another socket already owns @lin_id, -EOPNOTSUPP
 * if the driver does not implement set_response (or if @lin_id is
 * a diagnostic ID and the driver lacks LIN_CAP_DIAG), -EINVAL if
 * @lin_id is reserved (0x3E / 0x3F), @len is out of range, or @enh
 * is true on a diagnostic ID.
 */
int  lin_publisher_set(struct net_device *dev, struct sock *sk,
		       u8 lin_id, const u8 *data, u8 len, bool enh);

/**
 * lin_publisher_clear - release a publisher registration
 * @dev:    target LIN netdev
 * @sk:     socket whose registration is being released
 * @lin_id: 6-bit LIN frame ID
 *
 * Caller must hold ld->policy_lock. Best-effort teardown: the core
 * publisher slot is cleared first, then the driver's clear_response op
 * is invoked. Driver errors are logged via netdev_err but do not
 * propagate — the slot is freed for reassignment regardless, and a
 * subsequent set_response from a new owner overwrites any stale
 * driver-side entry.
 *
 * Returns -ENOENT when @sk does not currently own @lin_id. That's
 * distinct from a driver-side glitch: it's a userspace API contract
 * signal ("you asked to release a publisher you don't own") and the
 * caller can surface it to userspace as the truth — nothing was
 * released because there was nothing to release.
 *
 * Return: 0 on success, -EINVAL if @lin_id is out of range,
 *         -ENOENT if @sk does not own @lin_id.
 */
int  lin_publisher_clear(struct net_device *dev, struct sock *sk,
			 u8 lin_id);

/**
 * lin_publisher_release_all - force-release every publisher registration
 *                             owned by @sk on @dev
 * @dev: target LIN netdev
 * @sk:  socket whose registrations are being released
 *
 * Caller must hold ld->policy_lock. Iterates the per-interface
 * publisher table and calls lin_publisher_clear() for every entry
 * pointing at @sk. Used by teardown paths (socket close,
 * NETDEV_UNREGISTER, bind-away) so the protocol module can free all
 * of @sk's publisher state in one call without keeping a per-socket
 * shadow of which IDs it owns. The iteration is bounded by
 * LIN_ID_MASK + 1 (64 entries) and runs entirely under policy_lock,
 * so there is no race with concurrent publisher mutations.
 */
void lin_publisher_release_all(struct net_device *dev, struct sock *sk);

/**
 * lin_schedule_load - install or replace a master schedule
 * @dev:       target LIN netdev
 * @sk:        owning socket; must currently hold the master role
 * @sched:     pre-validated schedule struct (handle, entries, timings)
 * @buf_size:  byte length of the @sched buffer as passed from userspace;
 *             used by the core to bound entry-count validation
 *
 * Validates the schedule structure (field ranges, reserved bits, per-
 * entry frame types, reserved IDs, and that every TYPE_SPORADIC member
 * has a publisher already registered on @dev), then forwards to the
 * driver's schedule_load op. On success, marks the handle loaded in
 * ld->schedules_loaded.
 *
 * Caller must hold ld->policy_lock.
 *
 * Return: 0 on success, -EPERM if @sk is not the current master,
 *         -EOPNOTSUPP if the driver does not implement schedule ops,
 *         -EINVAL on validation failure, -EBUSY if @sched->handle is
 *         the currently-active schedule (caller must stop first),
 *         -errno from the driver.
 */
int  lin_schedule_load(struct net_device *dev, struct sock *sk,
		       const struct lin_schedule *sched, size_t buf_size);

/**
 * lin_schedule_delete - remove a loaded schedule
 * @dev:    target LIN netdev
 * @sk:     owning socket; must currently hold the master role
 * @handle: schedule handle to delete
 *
 * Caller must hold ld->policy_lock. Rejects DELETE of the
 * currently-active handle with -EBUSY (the caller must stop first)
 * and DELETE of an unloaded handle with -ENOENT. Calls the driver's
 * schedule_delete op first; only on success does the core clear
 * ld->schedules_loaded[handle].
 */
int  lin_schedule_delete(struct net_device *dev, struct sock *sk,
			 u8 handle);

/**
 * lin_schedule_activate - begin running a loaded schedule
 * @dev:    target LIN netdev
 * @sk:     owning socket; must currently hold the master role
 * @handle: schedule handle to activate
 *
 * Caller must hold ld->policy_lock. Verifies @handle is loaded
 * (returns -ENOENT otherwise) and then synchronously dispatches the
 * swap via the driver's schedule_activate op, which blocks until the
 * previously-active schedule (if any) has completed its in-flight
 * slot and the new schedule is running. Sporadic-publisher presence
 * is validated at lin_schedule_load() time, not here.
 *
 * Idempotent: activating the currently-active handle returns 0 with
 * no driver op invoked, no wait, no state change.
 *
 * On successful return the previous schedule is fully inactive on
 * the wire, so a subsequent LIN_RAW_SCHEDULE_LOAD on the previously-
 * active handle is race-free. Bounded by one slot duration of the
 * prior schedule (typically 10-50 ms).
 */
int  lin_schedule_activate(struct net_device *dev, struct sock *sk,
			   u8 handle);

/**
 * lin_schedule_stop - stop the currently-active schedule
 * @dev: target LIN netdev
 * @sk:  owning socket; must currently hold the master role
 *
 * Caller must hold ld->policy_lock. Calls the driver's schedule_stop
 * op and, on success, clears ld->active_schedule. Returns 0 when no
 * schedule is active.
 */
int  lin_schedule_stop(struct net_device *dev, struct sock *sk);

/**
 * lin_schedule_release_all - force-release all schedule state for @sk
 * @dev: target LIN netdev
 * @sk:  socket whose schedule state is being forcibly released
 *
 * Caller must hold ld->policy_lock. Intended for teardown paths
 * (socket close, NETDEV_UNREGISTER, bind-away, master release).
 * Issues schedule_stop (if an active schedule exists) and
 * schedule_delete for every loaded handle on a best-effort basis,
 * logging driver failures. Always clears core-side tracking state
 * when done.
 */
void lin_schedule_release_all(struct net_device *dev, struct sock *sk);

/**
 * lin_header_send - fire a single LIN header out-of-schedule
 * @dev:    target LIN netdev
 * @sk:     owning socket; must currently hold the master role
 * @lin_id: 6-bit LIN frame ID
 * @data:   master-published payload bytes (@len bytes), or NULL/0 for a
 *          read transaction where the slave is expected to respond
 * @len:    0 for a read transaction, 1..LIN_MAX_DLEN for a write
 * @enh:    true for enhanced checksum, false for classic
 *
 * The kernel's one-shot header emission primitive. Not exposed to
 * userspace directly; the core currently invokes it only for the
 * LIN_RAW_SLEEP command frame, and a future LIN transport-protocol
 * module would build on it too.
 *
 * Caller must hold ld->policy_lock. Validates role, op presence, and
 * ID range, then synchronously dispatches the emission via the
 * driver's header_send op, which blocks until the full frame slot
 * has completed on the wire (header + master data, or header +
 * slave response window for reads). On successful return the bus
 * is idle and any response has already been delivered via the
 * normal rx path; a subsequent LIN_RAW_SCHEDULE_ACTIVATE is
 * race-free. Bounded by one frame slot duration of the configured
 * LIN bitrate, typically 5-20 ms.
 *
 * Diagnostic IDs (0x3C / 0x3D) require the driver to advertise
 * LIN_CAP_DIAG, and must use classic checksum per LIN spec —
 * @enh == true on 0x3C / 0x3D returns -EINVAL.
 *
 * Returns -EOPNOTSUPP if the driver does not implement the header_send
 * op (or if @lin_id is a diagnostic ID and the driver lacks
 * LIN_CAP_DIAG, or @enh is true and the driver lacks LIN_CAP_CHK_ENH),
 * -EPERM if @sk is not the current master, -EINVAL on validation
 * failure (out-of-range @lin_id, reserved ID 0x3E / 0x3F,
 * @len > LIN_MAX_DLEN, or @enh on a diagnostic ID), -EBUSY if a
 * schedule is currently active on this interface or if @len > 0 and
 * the ID has an existing publisher, -errno from the driver (e.g.
 * -ETIMEDOUT if hardware fails to complete the slot within the
 * bounded wait).
 */
int  lin_header_send(struct net_device *dev, struct sock *sk,
		     u8 lin_id, const u8 *data, u8 len, bool enh);

/**
 * lin_wakeup_send - drive a bus wakeup pulse on a LIN interface
 * @dev: target LIN netdev
 *
 * Caller must hold ld->policy_lock. Per LIN 2.1+, any node (master
 * or slave) may wake the bus by holding it dominant for 250-5000us;
 * this helper does not check the master claim. It does require the
 * driver to advertise LIN_CAP_WAKEUP and rejects with -EBUSY if a
 * schedule is currently active on the interface — wakeup signaling
 * and active traffic don't mix; the bus should be idle (typically
 * asleep) when a wakeup is issued.
 *
 * Synchronous: blocks until the wakeup pulse has completed on the
 * wire (~5 ms maximum per spec). On successful return the bus is
 * recessive and a subsequent LIN_RAW_SCHEDULE_ACTIVATE is race-free
 * against the pulse.
 *
 * Returns -EOPNOTSUPP if the driver does not advertise
 * LIN_CAP_WAKEUP, -EBUSY if a schedule is active, -errno from the
 * driver (e.g. -ETIMEDOUT if hardware fails to complete the pulse).
 */
int  lin_wakeup_send(struct net_device *dev);

#endif /* !_LIN_CORE_H */
