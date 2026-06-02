// SPDX-License-Identifier: GPL-2.0
/*
 * drivers/net/lin/dev/sched.c - LIN master schedule engine state.
 *
 * Holds the per-link schedule-engine state for a LIN driver: the
 * loaded-schedule storage (one slot per caller-assigned handle,
 * deep-copied at load), the active-schedule cursor (which handle and
 * slot is up next, when its deadline is), the TYPE_EVENT collision-
 * diversion bookkeeping, the ad-hoc lin_dev_ops.header_send() request
 * slot, and a flag bitmap the kthread reads via wait predicates.
 *
 * The state struct (struct lin_sched) is embedded by host-side LIN
 * frontends in their per-link container. The transport-specific
 * frontend (sllin, sdlin, a hardware driver, ...) owns the kthread
 * or work item that drives slot evaluation; this module provides the
 * lifecycle and bookkeeping helpers that the kthread consumes.
 *
 * Locking is the caller's responsibility (per the *_locked() naming);
 * see <linux/lin/drv.h> for the contract.
 *
 * Author: Kyle Bader <kyle.bader94@gmail.com>
 * Copyright (c) 2026 Kyle Bader
 */

#include <linux/export.h>
#include <linux/ktime.h>
#include <linux/slab.h>

#include <linux/lin/drv.h>

void lin_sched_init(struct lin_sched *s)
{
	s->active = -1;
	s->saved_handle = -1;
	init_completion(&s->activate_done);
}
EXPORT_SYMBOL_GPL(lin_sched_init);

void lin_sched_destroy(struct lin_sched *s)
{
	unsigned int i;

	for (i = 0; i < LIN_RAW_SCHEDULES_MAX; i++) {
		kfree(s->sched[i]);
		s->sched[i] = NULL;
	}
}
EXPORT_SYMBOL_GPL(lin_sched_destroy);

struct lin_schedule *
lin_sched_load_locked(struct lin_sched *s, struct lin_schedule *copy)
{
	struct lin_schedule *old = s->sched[copy->handle];

	s->sched[copy->handle] = copy;
	return old;
}
EXPORT_SYMBOL_GPL(lin_sched_load_locked);

struct lin_schedule *
lin_sched_take_locked(struct lin_sched *s, u8 handle)
{
	struct lin_schedule *old = s->sched[handle];

	s->sched[handle] = NULL;
	return old;
}
EXPORT_SYMBOL_GPL(lin_sched_take_locked);

void lin_sched_stop_locked(struct lin_sched *s)
{
	clear_bit(LIN_SCHED_F_RUNNING, &s->flags);
	s->active = -1;
	s->activate_req = false;
	s->diverted = false;
	s->divert_pending = false;
}
EXPORT_SYMBOL_GPL(lin_sched_stop_locked);

void lin_sched_apply_activate_locked(struct lin_sched *s)
{
	if (!s->activate_req)
		return;
	s->active = s->activate_to;
	s->slot = 0;
	s->activate_req = false;
	s->diverted = false;
	s->divert_pending = false;
	complete(&s->activate_done);
}
EXPORT_SYMBOL_GPL(lin_sched_apply_activate_locked);

void lin_sched_advance_slot_locked(struct lin_sched *s,
				   const struct lin_schedule *cur)
{
	if (s->divert_pending) {
		s->divert_pending = false;
		return;
	}
	if (++s->slot < cur->entry_count)
		return;

	if (s->diverted) {
		s->active = s->saved_handle;
		s->slot = s->saved_slot;
		if (s->sched[s->active] &&
		    s->slot >= s->sched[s->active]->entry_count)
			s->slot = 0;
		s->diverted = false;
	} else {
		s->slot = 0;
	}
}
EXPORT_SYMBOL_GPL(lin_sched_advance_slot_locked);

ktime_t lin_sched_slot_duration(const struct lin_schedule *s,
				const struct lin_schedule_entry *e)
{
	u32 us = e->slot_us ? e->slot_us : s->default_slot_us;

	if (!us)
		us = 5000;	/* defensive fallback; the LIN core's
				 * schedule validator rejects zero slot
				 * durations at load.
				 */
	return us_to_ktime(us);
}
EXPORT_SYMBOL_GPL(lin_sched_slot_duration);
