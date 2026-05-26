/* SPDX-License-Identifier: (GPL-2.0 OR BSD-3-Clause) */
/*
 * linux/lin/dev.h
 *
 * Definitions for LIN network device registration and driver contract.
 *
 * Author: Kyle Bader <kyle.bader94@gmail.com>
 * Copyright (c) 2026 Kyle Bader
 */

#ifndef _LIN_DEV_H
#define _LIN_DEV_H

#include <linux/bitmap.h>
#include <linux/lin.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/netdevice.h>
#include <linux/types.h>
#include <net/netlink.h>
#include <uapi/linux/lin/netlink.h>
#include <uapi/linux/lin/raw.h>

struct lin_dev;
struct lin_frame;
struct lin_schedule;
struct sk_buff;
struct sock;

/*
 * Driver capability flags (LIN_CAP_*) are defined in
 * <uapi/linux/lin/netlink.h> so the kernel and userspace share one set
 * of values. Drivers populate struct lin_dev.caps with the union of
 * features they implement before lin_register_netdev(); the core
 * checks each user-facing feature against these flags and returns
 * -EOPNOTSUPP for unsupported requests, and the bitmask is exported to
 * userspace under IFLA_LIN_CAPS via lin_link_fill_info().
 */

/*
 * LIN frame checksum.
 *
 * The checksum byte is the bitwise inverse of the eight-bit one's-complement
 * sum (a modulo-255 sum with end-around carry) over the response data bytes.
 * The LIN 2.x "enhanced" form additionally folds in the protected identifier
 * (PID); the "classic" form covers the data bytes only. Diagnostic frames
 * (0x3C / 0x3D) always use the classic form per spec, regardless of
 * LIN_CAP_CHK_ENH.
 *
 * Drivers compute the byte on transmit and validate it on receive (a
 * mismatch is reported up as LIN_F_ERR | LIN_ERR_CHECKSUM); these helpers are
 * the single canonical implementation. The LIN core never places a checksum
 * byte on the wire itself, so it does not call these directly.
 */
static inline u8 lin_classic_checksum(const u8 *data, u8 len)
{
	unsigned int sum = 0;
	u8 i;

	for (i = 0; i < len; i++) {
		sum += data[i];
		if (sum > 0xff)
			sum -= 0xff;	/* end-around carry */
	}
	return (u8)~sum;
}

static inline u8 lin_enhanced_checksum(u8 pid, const u8 *data, u8 len)
{
	unsigned int sum = pid;
	u8 i;

	for (i = 0; i < len; i++) {
		sum += data[i];
		if (sum > 0xff)
			sum -= 0xff;	/* end-around carry */
	}
	return (u8)~sum;
}

/*
 * Loopback emission tags. Identify which role on the originating
 * interface sourced an emission, so the core can stamp the
 * synthesised loopback skb with the right ownership pointers and
 * subscribers can apply LIN_RAW_RECV_OWN_MSGS gating per-socket.
 *
 * LIN_EMIT_MASTER:    the master schedule (or an ad-hoc header send)
 *                     fired the header that produced this frame.
 * LIN_EMIT_PUBLISHER: a registered publisher's response payload formed
 *                     the data portion of this frame.
 *
 * The two flags are independent and may be combined: a frame whose
 * header was master-emitted AND whose response came from a publisher
 * carries both bits, and the resulting skb tags both owners so each
 * socket's LIN_RAW_RECV_OWN_MSGS state is respected on dispatch.
 */
enum lin_emit_flags {
	LIN_EMIT_MASTER		= BIT(0),
	LIN_EMIT_PUBLISHER	= BIT(1),
};

/**
 * struct lin_dev_ops - LIN driver contract
 * @master_start:   called when a socket claims the LIN_RAW_MASTER role.
 *                  The driver should enable its schedule engine /
 *                  header-generation path. Leave NULL if the hardware
 *                  is slave-only; the core then rejects LIN_RAW_MASTER
 *                  with -EOPNOTSUPP. Must be paired with @master_stop.
 * @master_stop:    called when the master role is released. Should
 *                  quiesce header generation. Must be paired with
 *                  @master_start — lin_register_netdev() rejects a
 *                  driver that supplies one without the other.
 * @set_response:   install or update the response bytes for @lin_id in
 *                  the driver's response table. The driver is expected
 *                  to transmit these bytes whenever it observes an
 *                  incoming header matching @lin_id. Leave NULL if the
 *                  hardware cannot host a response table; the core
 *                  then rejects publisher registration with -EOPNOTSUPP.
 *                  Must be paired with @clear_response. Responses are
 *                  sticky — the driver responds to every matching
 *                  header until @clear_response is invoked. For an ID
 *                  that participates in a TYPE_SPORADIC slot, a
 *                  @set_response call also marks that member dirty (see
 *                  LIN_CAP_SPORADIC); the driver clears the dirty flag
 *                  when it emits the member.
 * @clear_response: remove the response table entry for @lin_id. Must
 *                  be paired with @set_response — lin_register_netdev()
 *                  rejects a driver that supplies one without the
 *                  other, so the core can call @clear_response
 *                  unconditionally on publisher teardown and rely on
 *                  the hardware entry actually going away.
 * @schedule_load:  install or replace the schedule identified by
 *                  @sched->handle in the driver's own (often
 *                  hardware-backed) schedule table. The core has
 *                  already validated the struct (handle/entry ranges,
 *                  reserved fields, frame-type field, per-entry
 *                  lin_id). The driver copies what it needs; the
 *                  buffer is released by the core after this call.
 *
 *                  The core uses this op's return value to decide
 *                  whether to mark @sched->handle as loaded in the
 *                  per-interface schedules_loaded bitmap; returning
 *                  success after partial mutation would desynchronise
 *                  that tracking from actual driver state, and
 *                  subsequent schedule_activate / schedule_delete
 *                  calls would reference a handle whose contents
 *                  differ from what the core thinks is loaded.
 * @schedule_delete: remove the schedule identified by @handle from
 *                   the driver. The core rejects DELETE of the
 *                   currently-active handle, so the driver can
 *                   assume it is inactive.
 * @schedule_activate: begin executing the schedule identified by
 *                     @handle and BLOCK until the swap has taken
 *                     effect (the previous schedule, if any, has
 *                     completed its in-flight slot). The core has
 *                     verified @handle is loaded and that it is not
 *                     already the active handle (the core treats
 *                     same-handle activation as idempotent and skips
 *                     this op).
 *
 *                     Drivers must implement the wait with a bounded
 *                     timeout — typically wait_event_timeout against
 *                     a slot-boundary completion signal, with the
 *                     timeout set to the maximum slot duration of
 *                     the previously-active schedule plus a small
 *                     margin. On timeout the driver should return
 *                     -ETIMEDOUT (or another suitable -errno),
 *                     indicating a hardware fault. The wait is
 *                     uninterruptible by signal — given the slot-
 *                     bounded ceiling (typically 10-50 ms), the
 *                     latency is comparable to other LIN driver ops
 *                     (master_start, schedule_load) and a Ctrl-C
 *                     during the wait will be delivered immediately
 *                     after this op returns.
 * @schedule_stop:  quiesce schedule execution (next slot boundary).
 *                  After this op returns, no further header
 *                  emissions for the previously-active schedule
 *                  should occur.
 * @header_send:    the kernel's one-shot header emission primitive:
 *                  fire a single LIN header out-of-schedule and BLOCK
 *                  until the full frame slot has completed.
 *                  @data / @len carry the master's response payload
 *                  for write transactions (@len > 0); for read
 *                  transactions (@len == 0) the driver emits the
 *                  header alone and delivers the slave's response
 *                  (or NO_RESPONSE) via the normal rx path. For
 *                  read transactions the wait covers the response
 *                  window too — by the time this op returns the
 *                  bus is idle and any response has already been
 *                  pushed up via lin_loopback_rx() / netif_rx().
 *                  Optional, and reachable only from the master
 *                  role — lin_header_send() rejects with -EPERM
 *                  unless the caller holds the master claim, and the
 *                  master role can only be acquired when the master
 *                  ops are present. lin_register_netdev() therefore
 *                  rejects this op on a slave-only driver with
 *                  -EINVAL. Not exposed to userspace directly; the
 *                  core currently invokes it only for the LIN_RAW_SLEEP
 *                  command frame, and a future LIN transport-protocol
 *                  module would build on it too.
 *
 *                  Drivers must implement the wait with a bounded
 *                  timeout — typically wait_event_timeout against
 *                  a slot-completion signal, sized to the
 *                  configured LIN bitrate plus a small margin
 *                  (one frame at 20 kbps is roughly 6 ms; the
 *                  bound should be ~2x the worst-case slot
 *                  duration). On timeout the driver should return
 *                  -ETIMEDOUT, indicating a hardware fault. Same
 *                  uninterruptible-by-signal contract as
 *                  @schedule_activate.
 * @wakeup_send:    drive a bus wakeup pulse (dominant for 250-5000us
 *                  per LIN 2.1+) and BLOCK until the pulse has fully
 *                  completed on the wire. Optional; pair with
 *                  LIN_CAP_WAKEUP. Role-agnostic — slave-only drivers
 *                  may supply this op without the master ops.
 *
 *                  Drivers must implement the wait with a bounded
 *                  timeout (the pulse is at most ~5 ms per spec; a
 *                  ~10 ms bound leaves headroom). On timeout return
 *                  -ETIMEDOUT. Same uninterruptible-by-signal
 *                  contract as @schedule_activate / @header_send.
 *                  On successful return the bus is back to recessive
 *                  and a subsequent LIN_RAW_SCHEDULE_ACTIVATE cannot
 *                  collide with the pulse.
 *
 * Op pairing enforced by lin_register_netdev():
 *   - @set_response / @clear_response: both or neither
 *   - The master ops (@master_start, @master_stop,
 *     @schedule_load, @schedule_delete, @schedule_activate,
 *     @schedule_stop): either all six or none. A driver that
 *     doesn't support the master role leaves them all NULL, and
 *     the core rejects LIN_RAW_MASTER / LIN_RAW_SCHEDULE_* with
 *     -EOPNOTSUPP.
 *   - @header_send is optional, but only meaningful with the master
 *     ops — lin_register_netdev() rejects a slave-only driver that
 *     supplies it, since lin_header_send() requires the master role
 *     and no socket can claim master without the master ops.
 *   - @wakeup_send is optional and gated by LIN_CAP_WAKEUP. The cap
 *     and the op must agree; no master-ops dependency since LIN
 *     allows any node to wake the bus.
 *
 * The pairing rules above yield four valid driver configurations.
 * The master ops and the response ops (@set_response /
 * @clear_response) are independent; either may be absent without
 * implying the other:
 *
 *   - Full master: master ops + response ops. Drives schedules and
 *     publishes responses on owned IDs. The general case.
 *   - Logger master: master ops, no response ops. Drives schedules
 *     for stimulus and observes slave-published responses without
 *     ever publishing itself. Useful for bus analyzers and test
 *     harnesses. TYPE_UNCOND schedule entries work; TYPE_SPORADIC
 *     schedules fail at LIN_RAW_SCHEDULE_LOAD with -EINVAL because
 *     no publishers can be registered; LIN_RAW_PUBLISH itself
 *     returns -EOPNOTSUPP.
 *   - Publishing slave: response ops, no master ops. Responds to
 *     headers driven by an external master. The typical slave-side
 *     LIN node role.
 *   - Observer slave: neither. Passive rx only; receives the
 *     frames an external master and other slaves put on the wire.
 *
 * All ops run with the owning lin_dev's policy_lock held. Ops may
 * sleep, but while one is in flight every other policy operation on
 * the same interface is serialised behind it, a concurrent socket
 * close() on the same interface stalls in lin_raw_release()'s
 * notifier-quiesce wait, and a concurrent NETDEV_UNREGISTER on the
 * interface blocks rtnl_lock until the op returns. Drivers MUST
 * therefore enforce their own timeouts on every remote-I/O path (USB
 * transfer timeouts, MMIO completion timeouts, slot-boundary
 * completion waits) — the LIN core has no safe way to cancel an
 * in-progress driver call.
 *
 * Total per-op driver runtime MUST stay within
 * LIN_RAW_SCHEDULE_SLOT_MAX_US (one second) plus a small
 * implementation margin. The cap on slot duration is what bounds
 * lin_raw_release()'s wait against the per-protocol notifier: if a
 * driver exceeds it, close() on a LIN socket can stall arbitrarily.
 * lin_raw_release() emits a WARN_ONCE in dmesg if the wait crosses
 * five seconds, naming the stuck sock, so a violating driver
 * surfaces during testing rather than as an opaque hang in
 * production.
 *
 * The one-second cap is the ceiling for bus-event ops
 * (@schedule_activate, @header_send, @wakeup_send), which wait on
 * wire-level completion and are sized by the slot duration of the
 * configured LIN bitrate (typically 5-50 ms). State-update ops
 * (@master_start, @master_stop, @set_response, @clear_response,
 * @schedule_load, @schedule_delete, @schedule_stop) do not wait on
 * the bus and SHOULD complete in ten milliseconds or less — they
 * touch driver-internal tables, USB control transfers, or MMIO,
 * none of which justify a slot-scale budget. This tighter bound
 * matters on teardown: NETDEV_UNREGISTER drains a master socket by
 * chaining @schedule_stop, up to LIN_RAW_SCHEDULES_MAX
 * @schedule_delete calls, @master_stop, and up to (LIN_ID_MASK + 1)
 * @clear_response calls under @policy_lock. Holding state-update
 * ops to ten milliseconds keeps the aggregate teardown wait around
 * one second worst-case; the five-second WARN_ONCE in
 * lin_raw_release() is then a meaningful "your driver is stuck"
 * signal rather than a maybe-just-slow ceiling.
 *
 * Lifecycle scope: ops may be invoked at any point between
 * lin_register_netdev() return and lin_unregister_netdev() return,
 * inclusive of the NETDEV_UNREGISTER notifier walk that runs inside
 * lin_unregister_netdev() itself. Drivers MUST keep the state these
 * callbacks reference alive across that boundary and release it only
 * after lin_unregister_netdev() returns. Hot-unpluggable hardware may
 * already be inaccessible by the time the unregister walk fires; ops
 * should return -ENODEV (or another -errno) gracefully in that case.
 * The LIN core treats driver errors during teardown as best-effort and
 * logs them via netdev_err without propagating to userspace.
 */
struct lin_dev_ops {
	int (*master_start)(struct lin_dev *ld);
	int (*master_stop)(struct lin_dev *ld);

	int (*set_response)(struct lin_dev *ld, u8 lin_id,
			    const u8 *data, u8 len, bool enhanced_checksum);
	int (*clear_response)(struct lin_dev *ld, u8 lin_id);

	int (*schedule_load)(struct lin_dev *ld,
			     const struct lin_schedule *sched);
	int (*schedule_delete)(struct lin_dev *ld, u8 handle);
	int (*schedule_activate)(struct lin_dev *ld, u8 handle);
	int (*schedule_stop)(struct lin_dev *ld);

	int (*header_send)(struct lin_dev *ld, u8 lin_id,
			   const u8 *data, u8 len, bool enhanced_checksum);

	int (*wakeup_send)(struct lin_dev *ld);

	/*
	 * @set_bitrate: program the bus bit rate in bits per second.
	 *
	 * Invoked from the rtnetlink changelink path (IFLA_LIN_BITRATE)
	 * under @policy_lock. The core updates the cached
	 * @lin_dev.bitrate after a successful return; drivers should not
	 * mutate that field directly outside of the pre-register
	 * initialisation window.
	 *
	 * Return 0 on success, -EINVAL for a rate the hardware cannot
	 * program (out of divider range, etc.), or another -errno the
	 * driver propagates. Drivers that cannot change bus speed at
	 * runtime leave this NULL; the netlink path returns -EOPNOTSUPP
	 * in that case. Per-op timing budget: ten milliseconds (state-
	 * update class; touches driver-internal config or a brief MMIO
	 * sequence, not a wire-level wait).
	 */
	int (*set_bitrate)(struct lin_dev *ld, u32 bitrate);
};

/**
 * struct lin_dev_rcv_lists - per-interface rx subscriber lists
 * @by_id:     one bucket per 6-bit LIN ID for single-ID filters
 *             (id_mask == LIN_ID_MASK, no flag constraint). LIN's
 *             ID space is small enough that a direct array index is
 *             both simpler and faster than a hash bucket; this is the
 *             primary deviation from SocketCAN's receiver-list shape.
 * @match_all: subscribers with id_mask == 0 and flags_mask == 0
 *             (and LIN_FILT_INV clear) — every non-error frame is
 *             delivered to these.
 * @filter:    subscribers with non-trivial id_mask or flags_mask
 *             (non-inverted). Walked against every non-error frame.
 * @inv:       inverted filters (LIN_FILT_INV set in lin_filter.flags).
 *             Walked against every non-error frame, match inverted.
 * @err:       subscribers to error frames, registered via
 *             LIN_RAW_ERR_FILTER. Match is against frame.err_mask.
 * @wakeup:    subscribers to bus-level wakeup signals, registered via
 *             LIN_RAW_WAKEUP_FILTER. Receives every frame with
 *             LIN_F_WAKEUP set; no per-subscriber masking — wakeup
 *             carries no class data.
 * @entries:   total subscriber count across all buckets; used for the
 *             fast "no listeners" rx early-exit.
 *
 * Present once per LIN netdev (embedded in struct lin_dev) and once
 * per network namespace (for ifindex-0 "any" subscribers). Readers
 * walk the lists under rcu_read_lock(); writers update under the
 * net-ns-wide rcvlists_lock.
 */
struct lin_dev_rcv_lists {
	struct hlist_head	by_id[LIN_ID_MASK + 1];
	struct hlist_head	match_all;
	struct hlist_head	filter;
	struct hlist_head	inv;
	struct hlist_head	err;
	struct hlist_head	wakeup;
	int			entries;
};

/**
 * struct lin_dev - per-netdev LIN state owned by the core
 * @dev:         backpointer to the owning net_device
 * @ops:         driver-provided LIN ops; supplied at alloc_lindev()
 *               time and immutable thereafter
 * @caps:        LIN_CAP_* feature flags the driver supports; set by
 *               the driver before lin_register_netdev(), immutable
 *               after
 * @bitrate:     current configured bus bit rate in bits per second,
 *               or 0 if unknown / not yet configured. Drivers MAY
 *               initialise this before lin_register_netdev() (sllin
 *               seeds from the underlying TTY's termios baud, for
 *               example); after register, the core writes it from
 *               the rtnetlink IFLA_LIN_BITRATE path after a
 *               successful @lin_dev_ops.set_bitrate call. Drivers
 *               should treat it as read-only post-register. The
 *               value is exported to userspace via IFLA_LIN_BITRATE.
 * @rcv_lists:   rx subscriber lists for this interface; see
 *               struct lin_dev_rcv_lists
 * @policy_lock: serialises mutation of cross-socket policy state on
 *               this interface (master_sk, publishers, schedules
 *               — added by later commits). Held across driver op
 *               calls so the single-master-per-interface and
 *               single-publisher-per-ID invariants hold even when
 *               driver ops sleep, without serialising on rtnl_lock.
 *               Lock ordering: rtnl_lock (when held by the caller)
 *               -> sock_lock(sk) -> ld->policy_lock.
 * @master_sk:   socket currently holding the LIN_RAW_MASTER claim on
 *               this interface, or NULL. Mutation under @policy_lock;
 *               rx readers use rcu_dereference under rcu_read_lock()
 *               and gate every subsequent sock dereference on
 *               refcount_inc_not_zero(&sk->sk_refcnt) so a sock
 *               observed mid-teardown is skipped.
 * @publishers:  per-6-bit-ID socket that currently publishes the
 *               response for that ID, or NULL. Same locking rules as
 *               @master_sk.
 * @schedules_loaded: bitmap of schedule handles currently loaded in
 *               the driver. Indexed by handle, size
 *               LIN_RAW_SCHEDULES_MAX. Maintained by the core so it
 *               can reject ACTIVATE/DELETE of unknown handles. All
 *               entries are auto-cleared when master is released.
 * @active_schedule: handle of the currently-running schedule, or -1
 *               if none is active.
 * @sched_cr_refs: per-loaded-schedule bitmask of the collision-resolving
 *               schedule handles it references via TYPE_EVENT slots.
 *               Bit X set in sched_cr_refs[S] means schedule S names
 *               handle X as a TYPE_EVENT collision-resolving table.
 *               Maintained by the core so SCHEDULE_DELETE can refuse
 *               (-EBUSY) deletion of a handle still referenced as a
 *               collision-resolving schedule. Indexed by handle.
 * @going_down:  quiesce flag set by the LIN core's NETDEV_GOING_DOWN
 *               notifier under @policy_lock and cleared on NETDEV_UP.
 *               Sockopt entry points consult it under @policy_lock
 *               and bail with -ENETDOWN before touching driver ops,
 *               so a userspace caller racing with link-down does not
 *               collide with the GOING_DOWN-time force-release of
 *               master state from the per-protocol notifier.
 *
 * Installed on net_device.ml_priv with type tag ML_PRIV_LIN by
 * alloc_lindev(). Subsequent commits extend this structure with
 * publisher-ownership registry and loaded schedule tracking. The
 * actual schedule execution and hardware response table live in the
 * driver; the core forwards validated state to it via struct
 * lin_dev_ops.
 */
struct lin_dev {
	struct net_device		*dev;
	const struct lin_dev_ops	*ops;
	u32				 caps;
	u32				 bitrate;
	struct lin_dev_rcv_lists	 rcv_lists;
	struct mutex			 policy_lock;
	struct sock __rcu		*master_sk;
	struct sock __rcu		*publishers[LIN_ID_MASK + 1];
	DECLARE_BITMAP(schedules_loaded, LIN_RAW_SCHEDULES_MAX);
	/* Per loaded handle: the schedule contains only TYPE_UNCOND entries,
	 * making it eligible as a TYPE_EVENT collision-resolving table. Set at
	 * load, consulted when validating a referencing schedule's cr_handle.
	 */
	DECLARE_BITMAP(sched_uncond_only, LIN_RAW_SCHEDULES_MAX);
	int				 active_schedule;
	unsigned long			 sched_cr_refs[LIN_RAW_SCHEDULES_MAX];
	bool				 going_down;
	/* Operator override of the LIN_CAP_PUB_SLAVE cap-gate, set via
	 * the rtnetlink IFLA_LIN_FORCE_PUB_SLAVE attribute (wired in a
	 * later commit). Default false; toggled under rtnl_lock by the
	 * changelink path. Drivers MUST NOT touch this field — it is
	 * operator policy, not driver capability.
	 */
	bool				 force_pub_slave;
};

/* Initialize a struct lin_dev_rcv_lists in place. */
void lin_dev_rcv_lists_init(struct lin_dev_rcv_lists *rl);

static inline struct lin_dev *lin_get_ml_priv(struct net_device *dev)
{
	return netdev_get_ml_priv(dev, ML_PRIV_LIN);
}

static inline void lin_set_ml_priv(struct net_device *dev, struct lin_dev *ld)
{
	netdev_set_ml_priv(dev, ld, ML_PRIV_LIN);
}

/**
 * lin_setup - configure net_device fields for a LIN interface
 * @dev: the netdev being prepared
 *
 * Used as the setup callback to alloc_netdev(). alloc_lindev() passes
 * this automatically; drivers that allocate via alloc_netdev_mq()
 * directly should invoke it from their own setup function.
 */
void lin_setup(struct net_device *dev);

/**
 * lin_dev_init - initialise the LIN core's embedded struct lin_dev
 * @dev:         a net_device whose private area was sized to hold the
 *               driver private region followed by struct lin_dev
 * @ops:         driver-provided LIN ops vtable, stored on the lin_dev
 * @sizeof_priv: size of the driver private region preceding the lin_dev
 *
 * Initialises the core-owned struct lin_dev embedded in netdev_priv()
 * and tags dev->ml_priv. alloc_lindev() calls this for the usual
 * alloc/register path; drivers that create their interface through
 * rtnl_link_ops invoke it directly from their setup callback (the rtnl
 * core allocates the netdev, so alloc_lindev() is not on that path).
 */
void lin_dev_init(struct net_device *dev, const struct lin_dev_ops *ops,
		  int sizeof_priv);

/**
 * alloc_lindev - allocate a LIN network device
 * @sizeof_priv: size of the driver's private data area, in bytes
 * @ops:         driver-provided LIN ops vtable. Stored on the new
 *               lin_dev and immutable for the lifetime of the
 *               netdev. Must be non-NULL.
 *
 * Allocates a net_device with enough room for the driver's private
 * area and the LIN core's struct lin_dev. The driver's private area
 * is reachable via netdev_priv(); the core's lin_dev is reachable
 * via lin_get_ml_priv(). On success, the returned netdev has
 * dev->type == ARPHRD_LIN and dev->ml_priv tagged ML_PRIV_LIN.
 *
 * The driver must populate any LIN_CAP_* feature flags it supports
 * in lin_get_ml_priv(dev)->caps after this returns and before calling
 * lin_register_netdev(), which validates the cap-to-op pairing.
 *
 * Return: the new net_device on success, NULL on allocation failure
 * or when @ops is NULL.
 */
struct net_device *alloc_lindev(int sizeof_priv,
				const struct lin_dev_ops *ops);

/**
 * free_lindev - release a LIN network device allocated with alloc_lindev()
 * @dev: the netdev to free
 *
 * Paired with alloc_lindev() at end of life, mirroring the SocketCAN
 * alloc/register/unregister/free contract. Call after
 * lin_unregister_netdev(), or directly if driver setup fails before
 * registration. Wraps free_netdev().
 */
void free_lindev(struct net_device *dev);

/**
 * lin_register_netdev - register a LIN network device with the network stack
 * @dev: the netdev to register
 *
 * Wraps register_netdev() and performs LIN-specific sanity checks.
 * Leaves the interface in the carrier-off state; drivers transition to
 * carrier-on from their ndo_open() once the controller is ready to
 * exchange frames.
 *
 * Return: 0 on success, a negative errno on failure.
 */
int  lin_register_netdev(struct net_device *dev);

/**
 * lin_unregister_netdev - unregister a LIN network device
 * @dev: the netdev to unregister
 *
 * Wraps unregister_netdev(). After this returns the interface is no
 * longer visible to the network stack, but the net_device remains
 * allocated; call free_lindev() to release it.
 */
void lin_unregister_netdev(struct net_device *dev);

/**
 * lin_loopback_rx - synthesise a tagged rx skb for an emitted frame
 * @dev:        originating LIN netdev
 * @frame:      LIN frame contents that were placed on the wire
 *              (master header + publisher response, as actually
 *              emitted)
 * @emit_flags: union of LIN_EMIT_* describing which role(s) on
 *              this interface sourced the emission. The core uses
 *              these bits to tag the synthesised skb with the
 *              originating master and/or publisher socket so
 *              subscribers can honour LIN_RAW_RECV_OWN_MSGS.
 * @resp_id:    the 6-bit frame ID whose registered publisher supplied the
 *              response data, used to tag the publisher owner. For
 *              unconditional and sporadic frames this equals
 *              frame->lin_id; for an event-triggered frame it is the
 *              answering frame's ID, since frame->lin_id carries the
 *              trigger ID (which has no publisher). Consulted only when
 *              @emit_flags includes LIN_EMIT_PUBLISHER.
 *
 * LIN frames are generated by the driver's schedule engine, not by a
 * socket sendmsg, so this synthesised rx is the single source of
 * correctly owner-tagged loopback (the tags drive LIN_RAW_RECV_OWN_MSGS
 * and LIN_RAW_LOOPBACK). To preserve the SocketCAN-style "every emission
 * is observable from every socket on the bus" abstraction, the core
 * builds an rx-shaped skb here and feeds it through the normal
 * subscriber-dispatch path.
 *
 * A driver MUST NOT also deliver its own transmissions on rx: a
 * hardware readback/echo arrives untagged (no owning sock) and would
 * both double-deliver and defeat LIN_RAW_RECV_OWN_MSGS. UART/SPI LIN
 * controllers read back the line to validate their own byte stream;
 * the driver must drop that echo and rely on this helper for loopback.
 *
 * Callable from process or softirq context. Allocates with
 * GFP_ATOMIC.
 *
 * Wakeup pulses (lin_dev_ops.wakeup_send) are NOT synthesised
 * through this helper. The originating socket already knows it
 * emitted a wakeup (its setsockopt returned), and bus-sourced
 * wakeup detections flow through the driver's normal rx path with
 * @master_owner / @publisher_owner left NULL. Drivers should not
 * call lin_loopback_rx() for self-emitted wakeup pulses.
 */
void lin_loopback_rx(struct net_device *dev, const struct lin_frame *frame,
		     unsigned int emit_flags, u8 resp_id);

/**
 * alloc_lin_skb - build an rx-shaped skb carrying a LIN frame
 * @dev:   the LIN netdev the frame is associated with
 * @frame: the LIN frame contents to copy into the skb
 *
 * Constructor for bus-sourced frames a driver observed on the wire
 * (received data, error frames, wakeup and event-collision
 * notifications): allocates an skb in the shape the LIN rx path expects,
 * with the ownership tags left NULL, for the driver to hand to
 * netif_rx(). lin_loopback_rx() builds on this for self-emission
 * loopback. Mirrors alloc_can_skb().
 *
 * Return: the new skb, or NULL on allocation failure.
 */
struct sk_buff *alloc_lin_skb(struct net_device *dev,
			      const struct lin_frame *frame);

/*
 * rtnl_link_ops helpers — advertise the LIN-specific attribute block
 * (carried inside IFLA_INFO_DATA, defined in <uapi/linux/lin/netlink.h>)
 * to userspace. Drivers point their rtnl_link_ops .get_size / .fill_info
 * at these directly when they have no driver-specific attributes of
 * their own, or call them from inside their own hooks after emitting
 * driver-specific attributes. Every conforming LIN driver is expected
 * to advertise its capabilities this way; userspace and the selftest
 * suite read the result via RTM_GETLINK rather than feature-probing.
 */
size_t lin_link_get_size(const struct net_device *dev);
int    lin_link_fill_info(struct sk_buff *skb, const struct net_device *dev);

/*
 * IFLA_LIN_* policy and changelink handler. Drivers that register their
 * own rtnl_link_ops point .policy at lin_link_policy, .maxtype at
 * IFLA_LIN_MAX, and .changelink at lin_link_changelink so writable
 * attributes (today IFLA_LIN_BITRATE) flow through the same per-attr
 * dispatch in the LIN core for every driver.
 */
extern const struct nla_policy lin_link_policy[];
int lin_link_changelink(struct net_device *dev, struct nlattr *tb[],
			struct nlattr *data[],
			struct netlink_ext_ack *extack);

/*
 * Shared rtnl_link_ops registered once at LIN core module init.
 * alloc_lindev() assigns it to each new lin_dev so that hardware drivers
 * inherit IFLA_LIN_CAPS advertisement without needing their own
 * rtnl_link_ops — the SocketCAN model (can_link_ops). Drivers that *do*
 * register their own rtnl_link_ops (vlin's case, for
 * `ip link add type vlin`) should plug .get_size = lin_link_get_size and
 * .fill_info = lin_link_fill_info into their own ops.
 */
extern struct rtnl_link_ops lin_link_ops;
int  lin_link_ops_register(void);
void lin_link_ops_unregister(void);

#endif /* _LIN_DEV_H */
