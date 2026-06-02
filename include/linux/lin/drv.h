/* SPDX-License-Identifier: GPL-2.0 */
/*
 * include/linux/lin/drv.h - shared LIN driver helpers (transport-agnostic).
 *
 * Public surface for the lin-dev module. Every host-side LIN driver
 * may use these helpers regardless of transport:
 *
 *   - Per-ID publisher response cache.
 *   - Master schedule engine state (loaded-schedule storage,
 *     active-cursor + slot-boundary bookkeeping, collision diversion,
 *     ad-hoc header_send request slot).
 *
 * For UART-based drivers (sllin, sdlin), an additional header
 * (<linux/lin/uart.h>) layers the byte-protocol wire format on top of
 * this generic core.
 *
 * Symbols defined here are exported by lin-dev.ko.
 *
 * Author: Kyle Bader <kyle.bader94@gmail.com>
 * Copyright (c) 2026 Kyle Bader
 */

#ifndef _LIN_DRV_H
#define _LIN_DRV_H

#include <linux/completion.h>
#include <linux/ktime.h>
#include <linux/types.h>
#include <linux/lin.h>		/* LIN_MAX_DLEN, LIN_ID_MASK, LIN_ID_NONE */
#include <uapi/linux/lin/raw.h>	/* LIN_RAW_SCHEDULES_MAX, struct lin_schedule */

/* ----------------------------------------------------------------
 * Per-ID publisher response cache
 * ----------------------------------------------------------------
 *
 * Holds the bytes a publisher registered for each 6-bit LIN frame ID,
 * plus a per-entry dirty flag for TYPE_SPORADIC and TYPE_EVENT slot
 * evaluation. Drivers embed a struct lin_resp_table in their per-link
 * state and route their @set_response / @clear_response lin_dev_ops
 * through lin_resp_set_locked() / lin_resp_clear_locked().
 *
 * Locking: the table itself carries no lock. Callers serialise
 * accesses with the driver's existing per-link lock (sllin uses its
 * spinlock; future hardware drivers might use a mutex). All public
 * helpers are *_locked() to make the requirement explicit and let
 * each driver pick its preferred lock primitive without the helper
 * forcing a choice.
 */

struct lin_resp_entry {
	u8	data[LIN_MAX_DLEN];	/* response payload                  */
	u8	len;			/* payload length, 1..LIN_MAX_DLEN   */
	bool	enhanced;		/* enhanced checksum (LIN 2.x)       */
	bool	present;		/* entry has a registered publisher  */
	bool	dirty;			/* updated since last emit; consumed
					 * by TYPE_SPORADIC member-priority
					 * selection and TYPE_EVENT collision
					 * detection.
					 */
};

/**
 * struct lin_resp_table - per-link publisher response cache.
 * @entries: one slot per 6-bit LIN frame ID. The schedule engine
 *           consults these on slot fire (unconditional, sporadic,
 *           event-triggered) and on slave-side header decode.
 *
 * The frontend embeds this in its per-link state. Locking is the
 * frontend's responsibility via the *_locked() helpers.
 */
struct lin_resp_table {
	struct lin_resp_entry entries[LIN_ID_MASK + 1];
};

/**
 * lin_resp_table_init - initialise a response table in-place.
 * @t: table to initialise.
 *
 * Zeroes every entry (no presence, no dirty bits).
 */
void lin_resp_table_init(struct lin_resp_table *t);

/**
 * lin_resp_set_locked - register or update a publisher response.
 * @t:        response table.
 * @lin_id:   6-bit frame ID.
 * @data:     response payload (1..LIN_MAX_DLEN bytes).
 * @len:     payload length.
 * @enhanced: true if the cluster uses enhanced (LIN 2.x) checksum
 *            for this ID, false for classic.
 *
 * Inserts or updates the entry and marks it dirty (matching the
 * sporadic-priority "updated since last emit" semantic). Caller holds
 * the driver's per-link lock.
 */
void lin_resp_set_locked(struct lin_resp_table *t, u8 lin_id,
			 const u8 *data, u8 len, bool enhanced);

/**
 * lin_resp_clear_locked - release a publisher's registered response.
 * @t:      response table.
 * @lin_id: 6-bit frame ID.
 *
 * Marks the entry as absent and clears any dirty bit. Caller holds
 * the driver's per-link lock.
 */
void lin_resp_clear_locked(struct lin_resp_table *t, u8 lin_id);

/**
 * lin_resp_get_locked - read-only inspection of a response entry.
 * @t:      response table.
 * @lin_id: 6-bit frame ID.
 * @out:    receives a copy of the entry on success. Untouched on
 *          absence.
 *
 * Returns true if the entry is present (has a registered publisher),
 * false otherwise. Caller holds the driver's per-link lock. Does not
 * mutate the table; dirty bit untouched.
 */
bool lin_resp_get_locked(const struct lin_resp_table *t, u8 lin_id,
			 struct lin_resp_entry *out);

/**
 * lin_resp_snapshot_for_emit_locked - snapshot an entry for emission.
 * @t:      response table.
 * @lin_id: 6-bit frame ID.
 * @out:    receives a copy of the entry on success.
 *
 * Like lin_resp_get_locked() but additionally clears the dirty flag,
 * matching the "this member just emitted, no longer fresh" transition
 * that follows a slot fire. Returns true on present, false on absent.
 * Caller holds the driver's per-link lock.
 */
bool lin_resp_snapshot_for_emit_locked(struct lin_resp_table *t,
				       u8 lin_id,
				       struct lin_resp_entry *out);

/**
 * lin_resp_sporadic_pick_locked - TYPE_SPORADIC slot evaluation.
 * @t:        response table.
 * @members:  array of candidate IDs in priority order
 *            (members[0] highest).
 * @count:    number of valid IDs in @members
 *            (1..LIN_SLOT_MAX_MEMBERS).
 * @out:      receives a copy of the chosen entry on success.
 *
 * Walks @members in order, selects the first present-and-dirty entry,
 * clears its dirty flag, and returns its 6-bit ID via the return
 * value (with @out filled). Returns LIN_ID_NONE if no member is
 * present-and-dirty, in which case the slot stays silent. Caller
 * holds the driver's per-link lock.
 */
u8 lin_resp_sporadic_pick_locked(struct lin_resp_table *t,
				 const u8 *members, u8 count,
				 struct lin_resp_entry *out);

/**
 * lin_resp_event_count_dirty_locked - TYPE_EVENT slot evaluation.
 * @t:         response table.
 * @members:   array of CR-schedule member IDs to count over.
 * @count:     number of valid IDs in @members.
 * @out_first: receives the lowest-indexed dirty member's ID; only
 *             meaningful when the return value is >= 1.
 *
 * Counts present-and-dirty entries among @members without mutating
 * the table. Used by the event-triggered slot evaluator to decide:
 *
 *   - 0 dirty: the slot stays silent.
 *   - 1 dirty: the unique answerer is *out_first; caller follows up
 *     with lin_resp_snapshot_for_emit_locked() to consume the dirty
 *     flag and read the bytes.
 *   - >1 dirty: collision; caller emits LIN_F_EVENT_COLLISION and
 *     diverts the engine to the slot's collision-resolving schedule.
 *
 * Caller holds the driver's per-link lock.
 */
unsigned int
lin_resp_event_count_dirty_locked(const struct lin_resp_table *t,
				  const u8 *members, unsigned int count,
				  u8 *out_first);

/* ----------------------------------------------------------------
 * Master schedule engine state
 * ----------------------------------------------------------------
 *
 * struct lin_sched holds the per-link state that the master schedule
 * engine reads and writes: loaded-schedule storage (one slot per
 * caller-assigned handle, deep-copied at load), the active-schedule
 * cursor (which handle + which slot is up next + when its deadline
 * is), the TYPE_EVENT collision-diversion bookkeeping, the
 * lin_dev_ops.header_send() ad-hoc request slot, and a flag bitmap
 * the kthread reads via wait predicates.
 *
 * Locking: the helpers operate under the frontend's per-link lock.
 * Functions named *_locked() expect the lock held on entry. The
 * struct itself contains no lock; the frontend supplies one.
 *
 * Flag bits in @flags. Atomic, no external lock required:
 *
 *   LIN_SCHED_F_MASTER_RUNNING - the LIN master role is currently
 *                                claimed by a socket on this link.
 *                                Set by lin_dev_ops.master_start,
 *                                cleared by master_stop.
 *   LIN_SCHED_F_RUNNING        - a schedule is actively cycling.
 *                                Set by lin_dev_ops.schedule_activate,
 *                                cleared by schedule_stop.
 *   LIN_SCHED_F_HDR_REQ        - an ad-hoc header_send() request is
 *                                parked waiting for the kthread.
 *   LIN_SCHED_F_HDR_DONE       - the kthread has completed the
 *                                request and stamped @hdr_status.
 */
#define LIN_SCHED_F_MASTER_RUNNING	0
#define LIN_SCHED_F_RUNNING		1
#define LIN_SCHED_F_HDR_REQ		2
#define LIN_SCHED_F_HDR_DONE		3

/**
 * struct lin_sched - master schedule engine state.
 * @sched: loaded schedules indexed by caller-assigned handle. Each
 *         entry is either NULL (handle not loaded) or a deep-copied
 *         struct lin_schedule owned by this state.
 * @active: handle of the currently-cycling schedule (-1 when none).
 * @slot: index of the next slot to evaluate in @sched[@active].
 * @next_slot: slot-boundary deadline; the kthread waits until then
 *             before re-evaluating.
 * @activate_req: a deferred activate request is pending; the kthread
 *                applies it at the next slot boundary.
 * @activate_to: handle to switch to when @activate_req fires.
 * @activate_done: completion the kthread signals after applying a
 *                 deferred activate; the lin_dev_ops.schedule_activate
 *                 caller waits on this.
 * @diverted: the engine is currently running a TYPE_EVENT collision-
 *            resolving schedule for one cycle (after a collision was
 *            detected on the interrupting schedule's event slot).
 * @divert_pending: the event slot handler just repositioned
 *                  @active/@slot for the divert; the slot-advance
 *                  code skips a step so the CR schedule's first slot
 *                  fires next, not its second.
 * @saved_handle: handle to resume when the divert cycle completes.
 * @saved_slot: slot to resume at when the divert cycle completes.
 * @hdr_id: ad-hoc header_send() target ID.
 * @hdr_data: ad-hoc payload (when @hdr_len > 0; otherwise the
 *            response cache is consulted).
 * @hdr_len: ad-hoc payload length (0 = caller supplied no data).
 * @hdr_enhanced: enhanced checksum class for the ad-hoc send.
 * @hdr_status: -EINPROGRESS while parked, -ECANCELED if master
 *              released, 0 on success, -errno otherwise.
 * @flags: LIN_SCHED_F_* atomic bits.
 */
struct lin_sched {
	struct lin_schedule	*sched[LIN_RAW_SCHEDULES_MAX];
	int			active;
	unsigned int		slot;
	ktime_t			next_slot;

	bool			activate_req;
	u8			activate_to;
	struct completion	activate_done;

	bool			diverted;
	bool			divert_pending;
	int			saved_handle;
	unsigned int		saved_slot;

	u8			hdr_id;
	u8			hdr_data[LIN_MAX_DLEN];
	u8			hdr_len;
	bool			hdr_enhanced;
	int			hdr_status;

	unsigned long		flags;
};

/**
 * lin_sched_init - initialise per-link schedule engine state.
 * @s: state to initialise. Caller must zero the struct before
 *     invocation; the typical pattern is embedding it in a kzalloc'd
 *     parent.
 *
 * Sets @active and @saved_handle to -1, initialises @activate_done.
 * No schedules are loaded; @sched[] is all NULL.
 */
void lin_sched_init(struct lin_sched *s);

/**
 * lin_sched_destroy - free all loaded schedules.
 * @s: state.
 *
 * Frees every non-NULL @sched[handle] and sets them all to NULL. Called
 * at link teardown. Safe to call multiple times.
 */
void lin_sched_destroy(struct lin_sched *s);

/**
 * lin_sched_load_locked - install or replace a loaded schedule.
 * @s:    state.
 * @copy: deep copy of the schedule, allocated by the caller (typically
 *        via kmemdup against the userspace-supplied struct lin_schedule).
 *        @copy->handle picks the slot. On success, @s takes ownership;
 *        the displaced previous schedule (if any) is returned to the
 *        caller for kfree() outside the lock.
 *
 * Returns the displaced previous schedule pointer (may be NULL when
 * the slot was previously empty). The caller is responsible for
 * kfree()ing the returned pointer outside the lock to keep the slab
 * allocator off the locked critical section.
 *
 * Caller holds the per-link lock.
 */
struct lin_schedule *
lin_sched_load_locked(struct lin_sched *s, struct lin_schedule *copy);

/**
 * lin_sched_take_locked - take ownership of a loaded schedule slot.
 * @s:      state.
 * @handle: schedule handle (0..LIN_RAW_SCHEDULES_MAX-1).
 *
 * Returns the displaced pointer (may be NULL). Caller kfree()s
 * outside the lock. Used by lin_dev_ops.schedule_delete and at
 * teardown.
 */
struct lin_schedule *
lin_sched_take_locked(struct lin_sched *s, u8 handle);

/**
 * lin_sched_stop_locked - park the engine and clear cycle state.
 * @s: state.
 *
 * Clears LIN_SCHED_F_RUNNING, sets @active to -1, drops any pending
 * deferred activate request and event-divert state. Used by
 * lin_dev_ops.schedule_stop and on master release.
 */
void lin_sched_stop_locked(struct lin_sched *s);

/**
 * lin_sched_apply_activate_locked - apply a deferred activate request.
 * @s: state.
 *
 * If @activate_req is set, switches @active to @activate_to, resets
 * @slot to 0, clears event-divert state, and signals @activate_done.
 * No-op when @activate_req is clear. Called from the kthread at the
 * top of an idle slot evaluation.
 */
void lin_sched_apply_activate_locked(struct lin_sched *s);

/**
 * lin_sched_advance_slot_locked - bookkeeping after a slot has fired.
 * @s:   state.
 * @cur: pointer to the currently-active schedule (for entry_count
 *       and wrap).
 *
 * Handles end-of-cycle wrap, TYPE_EVENT collision-divert resume
 * (restore @saved_handle/@saved_slot when a CR schedule's single
 * cycle completes), and the @divert_pending one-shot that suppresses
 * advance for the event slot that just diverted.
 */
void lin_sched_advance_slot_locked(struct lin_sched *s,
				   const struct lin_schedule *cur);

/**
 * lin_sched_slot_duration - per-slot timing.
 * @s: schedule being walked.
 * @e: entry being fired.
 *
 * Returns the slot duration (per-entry @slot_us, or
 * @default_slot_us when zero). Falls back to 5 ms when both are zero
 * (defensive; the LIN core's schedule validator rejects this case
 * at load).
 */
ktime_t lin_sched_slot_duration(const struct lin_schedule *s,
				const struct lin_schedule_entry *e);

/**
 * lin_sched_kthread_fn - kthread entry that drives a LIN bus.
 * @data: a struct lin_uart * (the frontend's per-link byte-protocol
 *        state with @sched / @resp / @lock / @wq attached via
 *        lin_uart_init()).
 *
 * Runs the slot-evaluation loop until kthread_should_stop():
 * advances the active master schedule, services ad-hoc
 * lin_dev_ops.header_send() requests, drives the wire-format state
 * machine forward as bytes arrive, handles framing errors and the
 * per-frame rx timeout, and unblocks parked header_send waiters
 * with the appropriate status. Transport-agnostic — every host-side
 * LIN frontend (sllin, sdlin, ...) spawns this entry via kthread_run.
 *
 * Promotes the kthread to SCHED_FIFO (LIN slot timing precision); the
 * caller does not need to do anything beyond kthread_run().
 *
 * Returns 0 on clean shutdown.
 */
int lin_sched_kthread_fn(void *data);

#endif /* _LIN_DRV_H */
