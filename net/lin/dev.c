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
#include <linux/rcupdate.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/lin.h>
#include <linux/lin/core.h>
#include <linux/lin/dev.h>
#include <net/sock.h>

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
 * @ops:         driver-provided LIN ops vtable, stored on the lin_dev
 * @sizeof_priv: size of the driver private region preceding the lin_dev,
 *               in bytes (the same value passed to alloc_lindev())
 *
 * Locates the embedded struct lin_dev within netdev_priv(), initialises
 * the core-owned state, and tags dev->ml_priv with ML_PRIV_LIN. Factored
 * out of alloc_lindev() so drivers that allocate their netdev through
 * rtnl_link_ops (and therefore cannot call alloc_lindev()) can run the
 * same initialisation from their setup callback.
 */
void lin_dev_init(struct net_device *dev, const struct lin_dev_ops *ops,
		  int sizeof_priv)
{
	struct lin_dev *ld = (struct lin_dev *)((char *)netdev_priv(dev) +
						ALIGN(sizeof_priv, NETDEV_ALIGN));

	ld->dev = dev;
	ld->ops = ops;
	mutex_init(&ld->policy_lock);
	lin_dev_rcv_lists_init(&ld->rcv_lists);
	RCU_INIT_POINTER(ld->master_sk, NULL);
	memset(ld->publishers, 0, sizeof(ld->publishers));
	ld->active_schedule = -1;
	bitmap_zero(ld->schedules_loaded, LIN_RAW_SCHEDULES_MAX);
	bitmap_zero(ld->sched_uncond_only, LIN_RAW_SCHEDULES_MAX);
	memset(ld->sched_cr_refs, 0, sizeof(ld->sched_cr_refs));
	ld->going_down = false;

	lin_set_ml_priv(dev, ld);
}
EXPORT_SYMBOL(lin_dev_init);

struct net_device *alloc_lindev(int sizeof_priv,
				const struct lin_dev_ops *ops)
{
	struct net_device *dev;
	int size;

	if (!ops)
		return NULL;

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

	lin_dev_init(dev, ops, sizeof_priv);

	return dev;
}
EXPORT_SYMBOL(alloc_lindev);

void lin_dev_rcv_lists_init(struct lin_dev_rcv_lists *rl)
{
	int i;

	for (i = 0; i <= LIN_ID_MASK; i++)
		INIT_HLIST_HEAD(&rl->by_id[i]);

	INIT_HLIST_HEAD(&rl->match_all);
	INIT_HLIST_HEAD(&rl->filter);
	INIT_HLIST_HEAD(&rl->inv);
	INIT_HLIST_HEAD(&rl->err);
	INIT_HLIST_HEAD(&rl->wakeup);
	rl->entries = 0;
}
EXPORT_SYMBOL(lin_dev_rcv_lists_init);

void free_lindev(struct net_device *dev)
{
	free_netdev(dev);
}
EXPORT_SYMBOL(free_lindev);

/* Cross-socket policy helpers: master claim.
 * All mutating calls are serialised by ld->policy_lock; rx readers use
 * rcu_read_lock() around dereferences of ld->master_sk and gate every
 * subsequent sock field access on refcount_inc_not_zero(&sk->sk_refcnt)
 * so a sock observed mid-teardown is skipped rather than held. That
 * removes any need to defer the writer's sock_put across an RCU grace
 * period: clear the slot under policy_lock, drop the held reference
 * inline, and concurrent rx readers either grabbed their own reference
 * first (sock stays alive) or see a dead refcount and skip (sock_free
 * completes via sk_rcu independently).
 *
 * The mutex (rather than rtnl_lock) keeps a sleeping driver op from
 * blocking unrelated subsystems: the worst case is contention with
 * other policy operations on the same interface, plus blocking
 * NETDEV_UNREGISTER for that interface (the notifier needs sock_lock).
 */

int lin_master_claim(struct net_device *dev, struct sock *sk)
{
	struct lin_dev *ld = lin_get_ml_priv(dev);
	struct sock *current_master;
	int err;

	might_sleep();

	lockdep_assert_held(&ld->policy_lock);

	if (!ld->ops->master_start)
		return -EOPNOTSUPP;

	current_master = rcu_dereference_protected(ld->master_sk,
						   lockdep_is_held(&ld->policy_lock));
	if (current_master == sk)
		return 0;
	if (current_master)
		return -EBUSY;

	/* Driver op may sleep; call before publishing the pointer so we
	 * never expose a claim the hardware has not acknowledged.
	 */
	err = ld->ops->master_start(ld);
	if (err)
		return err;

	sock_hold(sk);
	rcu_assign_pointer(ld->master_sk, sk);
	return 0;
}
EXPORT_SYMBOL(lin_master_claim);

int lin_master_release(struct net_device *dev, struct sock *sk)
{
	struct lin_dev *ld = lin_get_ml_priv(dev);
	struct sock *current_master;
	int err;

	might_sleep();

	lockdep_assert_held(&ld->policy_lock);

	current_master = rcu_dereference_protected(ld->master_sk,
						   lockdep_is_held(&ld->policy_lock));
	if (current_master != sk)
		return 0;

	/* Best-effort teardown: clear core state regardless of driver
	 * errors. A wedged driver surfaces via dmesg, not via a
	 * propagated errno that userspace has no realistic way to
	 * handle — the only meaningful userspace response to "release
	 * failed" is "close the socket," which goes through this same
	 * path anyway.
	 *
	 * Schedule cleanup runs before clearing master_sk so the
	 * driver sees teardown under the same master identity it has
	 * been driving under.
	 */
	lin_schedule_release_all(dev, sk);

	rcu_assign_pointer(ld->master_sk, NULL);

	err = ld->ops->master_stop(ld);
	if (err)
		netdev_err(dev, "LIN master_stop returned %d on release; driver may be in an inconsistent state\n",
			   err);

	sock_put(current_master);
	return 0;
}
EXPORT_SYMBOL(lin_master_release);

int lin_publisher_set(struct net_device *dev, struct sock *sk,
		      u8 lin_id, const u8 *data, u8 len, bool enh)
{
	struct lin_dev *ld = lin_get_ml_priv(dev);
	struct sock *current_owner;
	bool adding;
	int err;

	might_sleep();

	lockdep_assert_held(&ld->policy_lock);

	if (lin_id > LIN_ID_MASK)
		return -EINVAL;
	if (lin_id >= LIN_ID_RESERVED_FIRST)
		return -EINVAL;
	if (!len || len > LIN_MAX_DLEN)
		return -EINVAL;
	if (!ld->ops->set_response)
		return -EOPNOTSUPP;

	/* Diagnostic IDs (0x3C / 0x3D) require the driver to advertise
	 * LIN_CAP_DIAG (the cap means "I route these IDs correctly").
	 * Per LIN spec, diagnostic frames use classic checksum
	 * unconditionally; enhanced checksum on 0x3C / 0x3D is a
	 * protocol violation regardless of driver capability. This block
	 * precedes the LIN_CAP_CHK_ENH check so the caller sees -EINVAL
	 * (the invariant violation) rather than -EOPNOTSUPP (a misleading
	 * driver-capability message) when both apply.
	 */
	if (lin_id == LIN_ID_DIAG_MASTER_REQ ||
	    lin_id == LIN_ID_DIAG_SLAVE_RESP) {
		if (enh)
			return -EINVAL;
		if (!(ld->caps & LIN_CAP_DIAG))
			return -EOPNOTSUPP;
	}

	if (enh && !(ld->caps & LIN_CAP_CHK_ENH))
		return -EOPNOTSUPP;

	/* Slave-only publisher: requires the driver's transport to meet the
	 * LIN spec's header-RX → response-TX timing window. A master that
	 * also publishes its own slot responses sits on the same TX path as
	 * the schedule and is exempt — the timing constraint only bites when
	 * the response is triggered by an external master's incoming header.
	 * See LIN_CAP_PUB_SLAVE in <uapi/linux/lin/netlink.h>.
	 *
	 * @force_pub_slave bypasses the cap check: an operator that knows
	 * their master is permissive enough (or that they are stimulating
	 * the slave-publisher code path for development) can set the
	 * IFLA_LIN_FORCE_PUB_SLAVE rtnetlink attribute to admit publishers
	 * on transports the driver did not advertise the cap for. The
	 * resulting bus timing is the operator's problem.
	 */
	if (rcu_dereference_protected(ld->master_sk,
				      lockdep_is_held(&ld->policy_lock)) != sk &&
	    !(ld->caps & LIN_CAP_PUB_SLAVE) &&
	    !ld->force_pub_slave)
		return -EOPNOTSUPP;

	current_owner = rcu_dereference_protected(ld->publishers[lin_id],
						  lockdep_is_held(&ld->policy_lock));
	if (current_owner && current_owner != sk)
		return -EBUSY;

	adding = !current_owner;

	err = ld->ops->set_response(ld, lin_id, data, len, enh);
	if (err)
		return err;

	if (adding) {
		sock_hold(sk);
		rcu_assign_pointer(ld->publishers[lin_id], sk);
	}
	return 0;
}
EXPORT_SYMBOL(lin_publisher_set);

int lin_publisher_clear(struct net_device *dev, struct sock *sk,
			u8 lin_id)
{
	struct lin_dev *ld = lin_get_ml_priv(dev);
	struct sock *current_owner;
	int err;

	might_sleep();

	lockdep_assert_held(&ld->policy_lock);

	if (lin_id > LIN_ID_MASK)
		return -EINVAL;

	current_owner = rcu_dereference_protected(ld->publishers[lin_id],
						  lockdep_is_held(&ld->policy_lock));
	if (current_owner != sk)
		return -ENOENT;

	/* Best-effort teardown: clear the core slot BEFORE calling the
	 * driver. This guarantees the per-ID slot is freed for
	 * reassignment regardless of the driver's success, at the cost
	 * of allowing the hardware response-table entry to outlive the
	 * core registration if the driver fails. Concurrent re-publishers
	 * on the same ID are not at risk because policy_lock serialises
	 * all publisher mutations, and a subsequent set_response from a
	 * new owner will overwrite any stale driver-side entry.
	 *
	 * Driver errors are logged via netdev_err rather than propagated:
	 * the only meaningful userspace response to "clear failed" would
	 * be retry-or-close, and both converge on the same core-slot-
	 * freed state we already provide. dmesg is the right channel for
	 * the operator / driver author to act on.
	 */
	rcu_assign_pointer(ld->publishers[lin_id], NULL);

	err = ld->ops->clear_response(ld, lin_id);
	if (err)
		netdev_err(dev, "LIN clear_response for ID 0x%02x returned %d on release; driver may still hold stale response\n",
			   lin_id, err);

	sock_put(current_owner);
	return 0;
}
EXPORT_SYMBOL(lin_publisher_clear);

void lin_publisher_release_all(struct net_device *dev, struct sock *sk)
{
	struct lin_dev *ld = lin_get_ml_priv(dev);
	unsigned int id;

	might_sleep();
	lockdep_assert_held(&ld->policy_lock);

	/* Iterate the canonical publisher table looking for entries this
	 * socket owns. Used by teardown paths (socket close,
	 * NETDEV_UNREGISTER, bind-away) so the protocol module doesn't
	 * have to maintain a shadow per-socket bitmap.
	 *
	 * Iteration is bounded and cheap; the per-publisher driver op
	 * dominates the cost.
	 */
	for (id = 0; id <= LIN_ID_MASK; id++) {
		if (rcu_dereference_protected(ld->publishers[id],
					      lockdep_is_held(&ld->policy_lock)) == sk)
			lin_publisher_clear(dev, sk, id);
	}
}
EXPORT_SYMBOL(lin_publisher_release_all);

/* Schedule validation + registry.
 *
 * The driver owns the actual schedule execution and timing. The core
 * keeps just enough metadata to enforce cross-socket policy: a per-dev
 * bitmap of loaded handles and the currently-active handle. Sporadic
 * publisher existence is validated up front at LOAD time (fail-fast)
 * rather than re-checked at ACTIVATE.
 */

static int lin_schedule_validate_entry(const struct lin_dev *ld,
				       const struct lin_schedule_entry *e)
{
	unsigned int i;
	bool seen_diag;

	if (e->flags)
		return -EINVAL;
	/* @cr_handle carries the collision-resolving schedule handle for
	 * TYPE_EVENT only; every other type must leave it zero. The
	 * TYPE_EVENT case below range-checks it, and
	 * lin_schedule_validate() checks it is loaded and not self.
	 */
	if (e->type != LIN_SCHED_TYPE_EVENT && e->cr_handle)
		return -EINVAL;
	if (memchr_inv(e->__res, 0, sizeof(e->__res)))
		return -EINVAL;
	if (e->member_count < 1 || e->member_count > LIN_SLOT_MAX_MEMBERS)
		return -EINVAL;

	/* Trailing members beyond member_count must be zero so a future
	 * extension that uses them as fallback can't be silently mis-fed
	 * by a stale userspace buffer.
	 */
	for (i = e->member_count; i < LIN_SLOT_MAX_MEMBERS; i++) {
		if (e->members[i])
			return -EINVAL;
	}

	/* Member ID validity: in-range, non-reserved, except for the
	 * type-specific exceptions checked below.
	 */
	seen_diag = false;
	for (i = 0; i < e->member_count; i++) {
		u8 id = e->members[i];

		if (id & ~LIN_ID_MASK)
			return -EINVAL;
		if (id == LIN_ID_DIAG_MASTER_REQ ||
		    id == LIN_ID_DIAG_SLAVE_RESP)
			seen_diag = true;
		else if (id >= LIN_ID_RESERVED_FIRST)
			return -EINVAL;
	}

	switch (e->type) {
	case LIN_SCHED_TYPE_UNCOND:
		if (e->member_count != 1)
			return -EINVAL;
		if (seen_diag)
			return -EINVAL;
		break;
	case LIN_SCHED_TYPE_DIAG:
		if (!(ld->caps & LIN_CAP_DIAG))
			return -EOPNOTSUPP;
		if (e->member_count != 1)
			return -EINVAL;
		if (!seen_diag)
			return -EINVAL;
		break;
	case LIN_SCHED_TYPE_SPORADIC:
		if (!(ld->caps & LIN_CAP_SPORADIC))
			return -EOPNOTSUPP;
		if (seen_diag)
			return -EINVAL;
		/* Publisher existence checked separately by caller after
		 * structural validation succeeds for the whole schedule.
		 */
		break;
	case LIN_SCHED_TYPE_EVENT:
		if (!(ld->caps & LIN_CAP_EVENT))
			return -EOPNOTSUPP;
		/* members[0] is the event-trigger ID (already range- and
		 * non-reserved-checked above); it must not be a diagnostic
		 * ID, and the slot carries only the trigger. The associated
		 * unconditional frames live in the collision-resolving
		 * schedule named by @cr_handle, not here.
		 */
		if (seen_diag)
			return -EINVAL;
		if (e->member_count != 1)
			return -EINVAL;
		if (e->cr_handle >= LIN_RAW_SCHEDULES_MAX)
			return -EINVAL;
		/* @cr_handle "already loaded" and "not this schedule's own
		 * handle" are checked in lin_schedule_validate(), which has
		 * sched->handle and the schedules_loaded bitmap.
		 */
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int lin_schedule_validate(const struct lin_dev *ld,
				 const struct lin_schedule *sched,
				 size_t buf_size)
{
	size_t need;
	unsigned int i;
	int err;

	if (!sched)
		return -EINVAL;
	if (sched->flags)
		return -EINVAL;
	if (memchr_inv(sched->__res, 0, sizeof(sched->__res)))
		return -EINVAL;
	if (sched->handle >= LIN_RAW_SCHEDULES_MAX)
		return -EINVAL;
	if (sched->entry_count < 1 ||
	    sched->entry_count > LIN_RAW_SCHEDULE_ENTRIES_MAX)
		return -EINVAL;
	if (sched->default_slot_us > LIN_RAW_SCHEDULE_SLOT_MAX_US)
		return -EINVAL;

	need = sizeof(*sched) +
	       (size_t)sched->entry_count *
	       sizeof(struct lin_schedule_entry);
	if (buf_size != need)
		return -EINVAL;

	for (i = 0; i < sched->entry_count; i++) {
		const struct lin_schedule_entry *e = &sched->entry[i];
		u32 effective_slot_us;

		err = lin_schedule_validate_entry(ld, e);
		if (err)
			return err;

		/* Effective slot duration must be non-zero and within
		 * LIN_RAW_SCHEDULE_SLOT_MAX_US. A per-entry @slot_us of
		 * 0 inherits @sched->default_slot_us; if both are zero
		 * the slot has no defined duration on the wire. The
		 * upper bound prevents userspace from stalling kernel
		 * policy operations (LIN_RAW_SCHEDULE_ACTIVATE blocks
		 * for up to one slot duration of the prior schedule).
		 */
		if (e->slot_us > LIN_RAW_SCHEDULE_SLOT_MAX_US)
			return -EINVAL;
		effective_slot_us = e->slot_us ? e->slot_us :
						 sched->default_slot_us;
		if (!effective_slot_us)
			return -EINVAL;
	}

	/* Publisher existence check for sporadic members. Done after the
	 * structural pass so any -EINVAL/-EOPNOTSUPP from a malformed
	 * entry takes precedence over a missing publisher complaint.
	 */
	for (i = 0; i < sched->entry_count; i++) {
		const struct lin_schedule_entry *e = &sched->entry[i];
		unsigned int j;

		if (e->type != LIN_SCHED_TYPE_SPORADIC)
			continue;

		for (j = 0; j < e->member_count; j++) {
			if (!rcu_dereference_protected(ld->publishers[e->members[j]],
						       lockdep_is_held(&ld->policy_lock)))
				return -EINVAL;
		}
	}

	/* Event collision-resolving references. Each TYPE_EVENT slot names
	 * a collision-resolving schedule via @cr_handle; it must already be
	 * loaded and must not be this schedule itself. Load the
	 * collision-resolving schedule before the schedule that references
	 * it. (Range was checked per-entry above.)
	 */
	for (i = 0; i < sched->entry_count; i++) {
		const struct lin_schedule_entry *e = &sched->entry[i];

		if (e->type != LIN_SCHED_TYPE_EVENT)
			continue;
		if (e->cr_handle == sched->handle)
			return -EINVAL;
		if (!test_bit(e->cr_handle, ld->schedules_loaded))
			return -EINVAL;
		/* A collision diverts the running schedule into the
		 * collision-resolving schedule, which must poll each group
		 * member in its own unconditional slot; reject a cr_handle whose
		 * schedule carries any non-TYPE_UNCOND entry. This also makes a
		 * nested event slot in a collision-resolving schedule
		 * impossible (an event entry is not unconditional).
		 */
		if (!test_bit(e->cr_handle, ld->sched_uncond_only))
			return -EINVAL;
	}

	return 0;
}

/* True if any loaded schedule names @handle as its TYPE_EVENT
 * collision-resolving table. Such a handle is pinned against both delete
 * and reload, so the unconditional-only property checked at the referrer's
 * load stays true for as long as the referrer is loaded.
 */
static bool lin_handle_is_cr_referenced(const struct lin_dev *ld, u8 handle)
{
	unsigned int i;

	for (i = 0; i < LIN_RAW_SCHEDULES_MAX; i++)
		if (test_bit(i, ld->schedules_loaded) &&
		    (ld->sched_cr_refs[i] & BIT(handle)))
			return true;
	return false;
}

int lin_schedule_load(struct net_device *dev, struct sock *sk,
		      const struct lin_schedule *sched, size_t buf_size)
{
	struct lin_dev *ld = lin_get_ml_priv(dev);
	unsigned long cr_refs = 0;
	bool uncond_only = true;
	unsigned int i;
	int err;

	/* sched_cr_refs[] packs the per-schedule referenced-handle set into
	 * one unsigned long per schedule, so the handle space must fit.
	 */
	BUILD_BUG_ON(LIN_RAW_SCHEDULES_MAX > BITS_PER_LONG);

	might_sleep();

	lockdep_assert_held(&ld->policy_lock);

	if (!ld->ops->schedule_load)
		return -EOPNOTSUPP;
	if (rcu_dereference_protected(ld->master_sk, lockdep_is_held(&ld->policy_lock)) != sk)
		return -EPERM;

	err = lin_schedule_validate(ld, sched, buf_size);
	if (err)
		return err;

	/* Replacing the currently-active handle would force the driver
	 * to mutate live schedule execution — every driver would have
	 * to stage atomically to keep the bus traffic well-defined.
	 * Disallow it: callers must LIN_RAW_SCHEDULE_STOP first, load,
	 * then re-activate. Symmetric with schedule_delete's active-
	 * handle rejection.
	 */
	if (ld->active_schedule >= 0 &&
	    sched->handle == (u8)ld->active_schedule)
		return -EBUSY;

	/* Refuse to replace a schedule that any loaded schedule references as
	 * a TYPE_EVENT collision-resolving table. Two hazards: an active
	 * referrer could divert the running engine into the table mid-replace
	 * (the live-execution hazard, as with replacing the active handle
	 * itself); and a replacement could turn the table non-unconditional,
	 * breaking the unconditional-only guarantee that lin_schedule_validate
	 * checked at the referrer's load and that activation never rechecks.
	 * Pinning it here (symmetric with schedule_delete) keeps that
	 * guarantee true for the life of the reference; drop the referrer
	 * first to edit the table.
	 */
	if (lin_handle_is_cr_referenced(ld, sched->handle))
		return -EBUSY;

	err = ld->ops->schedule_load(ld, sched);
	if (err)
		return err;

	/* Record which collision-resolving schedules this one references,
	 * so SCHEDULE_DELETE can refuse to delete a still-referenced
	 * handle. Recomputed on every (re)load, replacing the prior set.
	 * Validation already confirmed each cr_handle is loaded and != self.
	 */
	for (i = 0; i < sched->entry_count; i++) {
		const struct lin_schedule_entry *e = &sched->entry[i];

		if (e->type == LIN_SCHED_TYPE_EVENT)
			cr_refs |= BIT(e->cr_handle);
		if (e->type != LIN_SCHED_TYPE_UNCOND)
			uncond_only = false;
	}
	ld->sched_cr_refs[sched->handle] = cr_refs;

	/* Record eligibility as a collision-resolving table. Recomputed on
	 * every (re)load so a replacing schedule's type mix takes effect.
	 */
	if (uncond_only)
		set_bit(sched->handle, ld->sched_uncond_only);
	else
		clear_bit(sched->handle, ld->sched_uncond_only);

	set_bit(sched->handle, ld->schedules_loaded);
	return 0;
}
EXPORT_SYMBOL(lin_schedule_load);

int lin_schedule_delete(struct net_device *dev, struct sock *sk,
			u8 handle)
{
	struct lin_dev *ld = lin_get_ml_priv(dev);
	int err;

	might_sleep();

	lockdep_assert_held(&ld->policy_lock);

	if (!ld->ops->schedule_delete)
		return -EOPNOTSUPP;
	if (rcu_dereference_protected(ld->master_sk, lockdep_is_held(&ld->policy_lock)) != sk)
		return -EPERM;
	if (handle >= LIN_RAW_SCHEDULES_MAX)
		return -EINVAL;
	if (!test_bit(handle, ld->schedules_loaded))
		return -ENOENT;
	if (ld->active_schedule == handle)
		return -EBUSY;

	/* Refuse deletion while another loaded schedule still names this
	 * handle as its TYPE_EVENT collision-resolving schedule — that
	 * reference would dangle. Delete the referencing schedule first.
	 */
	if (lin_handle_is_cr_referenced(ld, handle))
		return -EBUSY;

	err = ld->ops->schedule_delete(ld, handle);
	if (err)
		return err;

	ld->sched_cr_refs[handle] = 0;
	clear_bit(handle, ld->sched_uncond_only);
	clear_bit(handle, ld->schedules_loaded);
	return 0;
}
EXPORT_SYMBOL(lin_schedule_delete);

int lin_schedule_activate(struct net_device *dev, struct sock *sk,
			  u8 handle)
{
	struct lin_dev *ld = lin_get_ml_priv(dev);
	int err;

	might_sleep();

	lockdep_assert_held(&ld->policy_lock);

	if (!ld->ops->schedule_activate)
		return -EOPNOTSUPP;
	if (rcu_dereference_protected(ld->master_sk, lockdep_is_held(&ld->policy_lock)) != sk)
		return -EPERM;
	if (handle >= LIN_RAW_SCHEDULES_MAX)
		return -EINVAL;
	if (!test_bit(handle, ld->schedules_loaded))
		return -ENOENT;

	/* Idempotent: same-handle activation is a no-op. Skip the driver
	 * op so a bounded wait isn't taken for a swap that wouldn't
	 * happen anyway.
	 */
	if (ld->active_schedule >= 0 &&
	    handle == (u8)ld->active_schedule)
		return 0;

	/* Publisher existence for sporadic members is validated at LOAD
	 * time; the user has been informed of any misconfiguration before
	 * reaching here. If a sporadic member's publisher has since been
	 * unregistered, that slot will fire silently — well-defined
	 * degraded behaviour, not an activation error.
	 */

	err = ld->ops->schedule_activate(ld, handle);
	if (err)
		return err;

	ld->active_schedule = handle;
	return 0;
}
EXPORT_SYMBOL(lin_schedule_activate);

int lin_schedule_stop(struct net_device *dev, struct sock *sk)
{
	struct lin_dev *ld = lin_get_ml_priv(dev);
	int err;

	might_sleep();

	lockdep_assert_held(&ld->policy_lock);

	if (!ld->ops->schedule_stop)
		return -EOPNOTSUPP;
	if (rcu_dereference_protected(ld->master_sk, lockdep_is_held(&ld->policy_lock)) != sk)
		return -EPERM;
	if (ld->active_schedule < 0)
		return 0;	/* idempotent */

	err = ld->ops->schedule_stop(ld);
	if (err)
		return err;

	ld->active_schedule = -1;
	return 0;
}
EXPORT_SYMBOL(lin_schedule_stop);

void lin_schedule_release_all(struct net_device *dev, struct sock *sk)
{
	struct lin_dev *ld = lin_get_ml_priv(dev);
	unsigned int handle;
	int err;

	might_sleep();
	lockdep_assert_held(&ld->policy_lock);

	if (!ld->ops->schedule_stop)
		return;

	if (rcu_dereference_protected(ld->master_sk, lockdep_is_held(&ld->policy_lock)) != sk)
		return;

	if (ld->active_schedule >= 0) {
		err = ld->ops->schedule_stop(ld);
		if (err)
			netdev_err(dev, "LIN schedule_stop op returned %d on forced release; driver may still be running schedule\n",
				   err);
		ld->active_schedule = -1;
	}

	for_each_set_bit(handle, ld->schedules_loaded,
			 LIN_RAW_SCHEDULES_MAX) {
		err = ld->ops->schedule_delete(ld, handle);
		if (err)
			netdev_err(dev, "LIN schedule_delete op for handle %u returned %d on forced release; driver may still hold schedule\n",
				   handle, err);
	}
	bitmap_zero(ld->schedules_loaded, LIN_RAW_SCHEDULES_MAX);
	bitmap_zero(ld->sched_uncond_only, LIN_RAW_SCHEDULES_MAX);
	memset(ld->sched_cr_refs, 0, sizeof(ld->sched_cr_refs));
}
EXPORT_SYMBOL(lin_schedule_release_all);

int lin_header_send(struct net_device *dev, struct sock *sk,
		    u8 lin_id, const u8 *data, u8 len, bool enh)
{
	struct lin_dev *ld = lin_get_ml_priv(dev);

	might_sleep();

	lockdep_assert_held(&ld->policy_lock);

	if (!ld->ops->header_send)
		return -EOPNOTSUPP;
	if (rcu_dereference_protected(ld->master_sk, lockdep_is_held(&ld->policy_lock)) != sk)
		return -EPERM;
	if (lin_id & ~LIN_ID_MASK)
		return -EINVAL;
	if (len > LIN_MAX_DLEN)
		return -EINVAL;
	/* Reserved IDs (0x3E/0x3F) are never legal on the wire. Diagnostic
	 * IDs (0x3C/0x3D) use classic checksum per LIN spec; enhanced
	 * checksum on them is a protocol violation regardless of driver
	 * capability.
	 *
	 * Note we do NOT require LIN_CAP_DIAG to emit a one-shot header on a
	 * diagnostic ID. Putting a single header on 0x3C/0x3D (e.g. the
	 * go-to-sleep command) is a base master operation any master driver
	 * can do; LIN_CAP_DIAG gates diagnostic *transport* — schedule
	 * TYPE_DIAG routing and slave-side responders — not raw emission.
	 * This keeps LIN_RAW_SLEEP available to any master, matching its
	 * documented contract (master + header_send + no active schedule).
	 */
	if (lin_id == LIN_ID_DIAG_MASTER_REQ ||
	    lin_id == LIN_ID_DIAG_SLAVE_RESP) {
		if (enh)
			return -EINVAL;
	} else if (lin_id >= LIN_ID_RESERVED_FIRST) {
		return -EINVAL;
	}

	if (enh && !(ld->caps & LIN_CAP_CHK_ENH))
		return -EOPNOTSUPP;

	/* One-shot emission while a schedule is running would force the
	 * driver to choose between colliding with a scheduled slot,
	 * pre-empting one, or queuing — each with different timing
	 * surprises. Disallow it: the active schedule must be stopped
	 * first. This keeps the emission's effect on the bus
	 * deterministic and removes the driver-defined behavior from the
	 * contract.
	 */
	if (ld->active_schedule >= 0)
		return -EBUSY;

	/* Write transaction (master publishes data) collides with any
	 * existing publisher on the same ID — the wire would carry two
	 * different responses for one header. Read transactions are fine
	 * because the slave's response is the only data on the slot.
	 */
	if (len > 0 && rcu_dereference_protected(ld->publishers[lin_id],
						 lockdep_is_held(&ld->policy_lock)))
		return -EBUSY;

	return ld->ops->header_send(ld, lin_id, data, len, enh);
}
EXPORT_SYMBOL(lin_header_send);

int lin_wakeup_send(struct net_device *dev)
{
	struct lin_dev *ld = lin_get_ml_priv(dev);

	might_sleep();

	lockdep_assert_held(&ld->policy_lock);

	if (!(ld->caps & LIN_CAP_WAKEUP) || !ld->ops->wakeup_send)
		return -EOPNOTSUPP;

	/* Wakeup is meaningful when the bus is quiescent (typically
	 * asleep). An active schedule means the bus is awake and
	 * carrying traffic; a wakeup pulse mid-slot would corrupt
	 * frame timing. Disallow it: callers must stop the schedule
	 * first if one is running.
	 */
	if (ld->active_schedule >= 0)
		return -EBUSY;

	return ld->ops->wakeup_send(ld);
}
EXPORT_SYMBOL(lin_wakeup_send);

int lin_register_netdev(struct net_device *dev)
{
	struct lin_dev *ld = lin_get_ml_priv(dev);
	int master_ops;

	if (dev->type != ARPHRD_LIN || !ld)
		return -EINVAL;

	/* Enforce paired lifecycle ops. A driver that implements one half
	 * of a pair but not the other would strand hardware state on
	 * release (publisher unset without clear_response leaves the
	 * response table entry live; master claim release without
	 * master_stop leaves the schedule engine running). Catch it at
	 * registration time so the driver author fixes their vtable
	 * rather than debugging a stale-state bug later.
	 */
	if (!!ld->ops->set_response != !!ld->ops->clear_response)
		return -EINVAL;

	/* Master ops: master_{start,stop} and the four
	 * schedule_* ops must be either all set or all NULL. A driver
	 * supporting only part of the master role can't usefully run
	 * a schedule.
	 */
	master_ops = !!ld->ops->master_start + !!ld->ops->master_stop +
		     !!ld->ops->schedule_load + !!ld->ops->schedule_delete +
		     !!ld->ops->schedule_activate + !!ld->ops->schedule_stop;
	if (master_ops != 0 && master_ops != 6)
		return -EINVAL;

	/* Caps that imply master capability cannot be set on a
	 * slave-only driver. LIN_CAP_CHK_ENH is allowed on either
	 * because a slave node may also publish enhanced-checksum
	 * frames. LIN_CAP_DIAG is also role-agnostic — it indicates
	 * the driver handles the diagnostic ID range correctly, which
	 * matters for both master-side schedule routing and slave-side
	 * transport responses.
	 */
	if (master_ops == 0 &&
	    (ld->caps & (LIN_CAP_SPORADIC | LIN_CAP_EVENT)))
		return -EINVAL;

	/* @header_send is the kernel's one-shot emission primitive,
	 * reachable only from the master role (lin_header_send() requires
	 * the caller hold the master claim). A slave-only driver that
	 * supplies it has an unreachable op, so reject the configuration
	 * at registration time.
	 */
	if (master_ops == 0 && ld->ops->header_send)
		return -EINVAL;

	/* @wakeup_send is optional and gated by LIN_CAP_WAKEUP. The cap
	 * and the op must agree. No master-ops dependency — any node
	 * may wake the bus.
	 */
	if (!!(ld->caps & LIN_CAP_WAKEUP) != !!ld->ops->wakeup_send)
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
