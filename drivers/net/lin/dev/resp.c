// SPDX-License-Identifier: GPL-2.0
/*
 * drivers/net/lin/dev/resp.c - LIN per-ID publisher response cache.
 *
 * Holds the bytes a publisher registered for each LIN ID, plus the
 * priority-order dirty-flag bookkeeping that TYPE_SPORADIC and
 * TYPE_EVENT slot evaluation consumes. Drivers route their
 * @set_response / @clear_response lin_dev_ops through the
 * lin_resp_set_locked() / lin_resp_clear_locked() helpers, and the
 * schedule engine (sched.c) consumes the table through the
 * snapshot / sporadic-pick / event-count helpers.
 *
 * Locking is the caller's responsibility (per the *_locked() naming);
 * see <linux/lin/drv.h> for the contract.
 *
 * Author: Kyle Bader <kyle.bader94@gmail.com>
 * Copyright (c) 2026 Kyle Bader
 */

#include <linux/export.h>
#include <linux/string.h>

#include <linux/lin/drv.h>

void lin_resp_table_init(struct lin_resp_table *t)
{
	memset(t, 0, sizeof(*t));
}
EXPORT_SYMBOL_GPL(lin_resp_table_init);

void lin_resp_set_locked(struct lin_resp_table *t, u8 lin_id,
			 const u8 *data, u8 len, bool enhanced)
{
	struct lin_resp_entry *e = &t->entries[lin_id & LIN_ID_MASK];

	memcpy(e->data, data, len);
	e->len = len;
	e->enhanced = enhanced;
	e->present = true;
	e->dirty = true;
}
EXPORT_SYMBOL_GPL(lin_resp_set_locked);

void lin_resp_clear_locked(struct lin_resp_table *t, u8 lin_id)
{
	struct lin_resp_entry *e = &t->entries[lin_id & LIN_ID_MASK];

	e->present = false;
	e->dirty = false;
	e->len = 0;
}
EXPORT_SYMBOL_GPL(lin_resp_clear_locked);

bool lin_resp_get_locked(const struct lin_resp_table *t, u8 lin_id,
			 struct lin_resp_entry *out)
{
	const struct lin_resp_entry *e = &t->entries[lin_id & LIN_ID_MASK];

	if (!e->present)
		return false;
	*out = *e;
	return true;
}
EXPORT_SYMBOL_GPL(lin_resp_get_locked);

bool lin_resp_snapshot_for_emit_locked(struct lin_resp_table *t, u8 lin_id,
				       struct lin_resp_entry *out)
{
	struct lin_resp_entry *e = &t->entries[lin_id & LIN_ID_MASK];

	if (!e->present)
		return false;
	*out = *e;
	e->dirty = false;
	return true;
}
EXPORT_SYMBOL_GPL(lin_resp_snapshot_for_emit_locked);

u8 lin_resp_sporadic_pick_locked(struct lin_resp_table *t,
				 const u8 *members, u8 count,
				 struct lin_resp_entry *out)
{
	u8 i;

	for (i = 0; i < count; i++) {
		u8 id = members[i] & LIN_ID_MASK;
		struct lin_resp_entry *e = &t->entries[id];

		if (e->present && e->dirty) {
			*out = *e;
			e->dirty = false;
			return id;
		}
	}
	return LIN_ID_NONE;
}
EXPORT_SYMBOL_GPL(lin_resp_sporadic_pick_locked);

unsigned int
lin_resp_event_count_dirty_locked(const struct lin_resp_table *t,
				  const u8 *members, unsigned int count,
				  u8 *out_first)
{
	unsigned int dirty = 0;
	unsigned int i;

	for (i = 0; i < count; i++) {
		u8 id = members[i] & LIN_ID_MASK;
		const struct lin_resp_entry *e = &t->entries[id];

		if (e->present && e->dirty) {
			if (!dirty && out_first)
				*out_first = id;
			dirty++;
		}
	}
	return dirty;
}
EXPORT_SYMBOL_GPL(lin_resp_event_count_dirty_locked);
