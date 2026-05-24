// SPDX-License-Identifier: GPL-2.0
/*
 * PF_LIN selftests: sporadic schedule slots. A sporadic slot emits the
 * highest-priority member whose publisher response is fresh ("dirty") and
 * is otherwise silent; the dirty flag is set on publish/write and cleared
 * on emit.
 */
#include "lin_harness.h"

FIXTURE(lin_spor) {
	int ifindex;
	__u32 caps;
	int m;		/* master + publisher */
	int s;		/* subscriber */
};

FIXTURE_SETUP(lin_spor)
{
	self->ifindex = lin_setup_iface(LIN_IF);
	if (self->ifindex <= 0)
		SKIP(return, "need root and the vlin module (ip link add type vlin)");
	ASSERT_EQ(0, lin_query_caps(self->ifindex, &self->caps));
	LIN_SKIP_UNLESS_CAP(self->caps, LIN_CAP_SPORADIC);
	self->m = lin_open_bound(self->ifindex);
	ASSERT_GE(self->m, 0);
	ASSERT_EQ(0, lin_master(self->m, 1));
	self->s = lin_open_bound(self->ifindex);
	ASSERT_GE(self->s, 0);
}

FIXTURE_TEARDOWN(lin_spor)
{
	if (self->m > 0)
		close(self->m);
	if (self->s > 0)
		close(self->s);
}

/* Load + activate a single sporadic slot; returns 0 on success. */
static int load_sporadic(int m, const __u8 *ids, unsigned int n)
{
	union lin_sched_buf buf = {};
	unsigned int i;
	int ret;

	buf.s.entry_count = 1;
	buf.s.default_slot_us = LIN_SLOT_US;
	buf.s.entry[0].type = LIN_SCHED_TYPE_SPORADIC;
	buf.s.entry[0].member_count = n;
	for (i = 0; i < n; i++)
		buf.s.entry[0].members[i] = ids[i];
	ret = lin_sched_load(m, &buf, 1);
	if (ret)
		return ret;
	return lin_sched_activate(m, 0);
}

/* The full dirty-flag lifecycle in one fixture: publish marks dirty, the
 * slot emits once and the flag is consumed (subsequent polls silent), and
 * a re-write via the data plane re-arms the slot.
 */
TEST_F(lin_spor, dirty_lifecycle)
{
	__u8 ids[1] = { 0x10 };
	__u8 d[1] = { 0x77 };
	struct lin_frame f;

	ASSERT_EQ(0, lin_publish(self->m, 0x10, d, 1, 0));	/* marks dirty */
	ASSERT_EQ(0, load_sporadic(self->m, ids, 1));

	/* One emission, then silent once the dirty flag is consumed. */
	ASSERT_EQ(1, lin_recv(self->s, &f, LIN_RECV_MS));
	EXPECT_EQ(0x10, f.lin_id);
	EXPECT_EQ(0x77, f.data[0]);
	EXPECT_TRUE(lin_silent(self->s, LIN_SILENCE_MS));

	/* Re-mark dirty via the data plane => emitted again. */
	ASSERT_EQ(sizeof(struct lin_frame), lin_write(self->m, 0x10, d, 1, 0));
	ASSERT_EQ(1, lin_recv(self->s, &f, LIN_RECV_MS));
	EXPECT_EQ(0x10, f.lin_id);
}

TEST_F(lin_spor, priority_order)
{
	__u8 ids[2] = { 0x10, 0x20 };	/* 0x10 highest priority */
	__u8 a[1] = { 0xa0 }, b[1] = { 0xb0 };
	struct lin_frame f;

	ASSERT_EQ(0, lin_publish(self->m, 0x10, a, 1, 0));
	ASSERT_EQ(0, lin_publish(self->m, 0x20, b, 1, 0));
	ASSERT_EQ(0, load_sporadic(self->m, ids, 2));

	/* Both dirty: the higher-priority member goes first. */
	ASSERT_EQ(1, lin_recv(self->s, &f, LIN_RECV_MS));
	EXPECT_EQ(0x10, f.lin_id);
	ASSERT_EQ(1, lin_recv(self->s, &f, LIN_RECV_MS));
	EXPECT_EQ(0x20, f.lin_id);
	EXPECT_TRUE(lin_silent(self->s, LIN_SILENCE_MS));
}

/* Sporadic publisher existence is checked at LOAD, not ACTIVATE (raw.h /
 * dev.c): if the publisher is unregistered between load and activate, the
 * activation still succeeds and the orphaned slot fires silently — defined
 * degraded behaviour, not an error.
 */
TEST_F(lin_spor, publisher_removed_after_load)
{
	union lin_sched_buf buf = {};
	__u8 d[1] = { 0x77 };

	/* Load while the publisher exists, so the load-time check passes. */
	ASSERT_EQ(0, lin_publish(self->m, 0x10, d, 1, 0));
	buf.s.entry_count = 1;
	buf.s.default_slot_us = LIN_SLOT_US;
	buf.s.entry[0].type = LIN_SCHED_TYPE_SPORADIC;
	buf.s.entry[0].member_count = 1;
	buf.s.entry[0].members[0] = 0x10;
	ASSERT_EQ(0, lin_sched_load(self->m, &buf, 1));

	/* Remove the publisher, then activate: activation succeeds and the now
	 * unowned slot is silent.
	 */
	ASSERT_EQ(0, lin_unpublish(self->m, 0x10));
	ASSERT_EQ(0, lin_sched_activate(self->m, 0));
	EXPECT_TRUE(lin_silent(self->s, LIN_SILENCE_MS));
}

TEST_HARNESS_MAIN
