// SPDX-License-Identifier: GPL-2.0
/*
 * PF_LIN selftests: error frames and LIN_RAW_ERR_FILTER.
 *
 * vlin can produce the NO_RESPONSE error (a scheduled header that no
 * publisher answers). Other error classes — checksum, framing, parity,
 * etc. — require a real bus that can corrupt data on the wire, which a
 * virtual interface never does, so they are not exercised here; a
 * hardware bring-up against a fault-injecting partner (or a future
 * vxlin-style paired driver) would cover them.
 */
#include "lin_harness.h"

#include <linux/lin/error.h>

FIXTURE(lin_err) {
	int ifindex;
	__u32 caps;
	int m;		/* master */
	int s;		/* subscriber */
};

FIXTURE_SETUP(lin_err)
{
	self->ifindex = lin_setup_iface(LIN_IF);
	if (self->ifindex <= 0)
		SKIP(return, "need root and the vlin module (ip link add type vlin)");
	ASSERT_EQ(0, lin_query_caps(self->ifindex, &self->caps));
	self->m = lin_open_bound(self->ifindex);
	ASSERT_GE(self->m, 0);
	ASSERT_EQ(0, lin_master(self->m, 1));
	self->s = lin_open_bound(self->ifindex);
	ASSERT_GE(self->s, 0);
}

FIXTURE_TEARDOWN(lin_err)
{
	if (self->m > 0)
		close(self->m);
	if (self->s > 0)
		close(self->s);
}

/* Schedule a header for an ID with no publisher; the slot yields a
 * NO_RESPONSE error frame.
 */
static int schedule_unanswered(int m, __u8 id)
{
	union lin_sched_buf buf = {};
	int ret;

	buf.s.entry_count = 1;
	buf.s.default_slot_us = LIN_SLOT_US;
	buf.s.entry[0].type = LIN_SCHED_TYPE_UNCOND;
	buf.s.entry[0].member_count = 1;
	buf.s.entry[0].members[0] = id;
	ret = lin_sched_load(m, &buf, 1);
	if (ret)
		return ret;
	return lin_sched_activate(m, 0);
}

TEST_F(lin_err, no_error_without_subscription)
{
	/* Default error mask is 0: a NO_RESPONSE does not reach a socket that
	 * only carries the default data filter.
	 */
	ASSERT_EQ(0, schedule_unanswered(self->m, 0x10));
	EXPECT_TRUE(lin_silent(self->s, LIN_SILENCE_MS));
}

/* Subscribing delivers NO_RESPONSE (one error frame per scheduled slot with
 * no publisher); clearing the subscription stops delivery — after draining
 * frames already queued, no further NO_RESPONSE reaches the socket even
 * though the schedule keeps generating the error every slot.
 */
TEST_F(lin_err, err_filter_toggle)
{
	struct lin_frame f;
	int i;

	ASSERT_EQ(0, lin_err_filter(self->s, LIN_ERR_NO_RESPONSE));
	ASSERT_EQ(0, schedule_unanswered(self->m, 0x10));
	ASSERT_EQ(1, lin_recv(self->s, &f, LIN_RECV_MS));
	EXPECT_NE(0, f.flags & LIN_F_ERR);
	EXPECT_NE(0, f.err_mask & LIN_ERR_NO_RESPONSE);
	EXPECT_EQ(0x10, f.lin_id);
	EXPECT_EQ(0, f.len);

	ASSERT_EQ(0, lin_err_filter(self->s, 0));

	/* Drain frames queued before the disable took effect. The schedule is
	 * still firing NO_RESPONSE every slot, so bound the drain: if delivery
	 * did not actually stop this caps at a few hundred ms and fails here,
	 * rather than receiving an error per slot until the kselftest timeout.
	 */
	for (i = 0; i < 16; i++) {
		if (lin_recv(self->s, &f, LIN_SLOT_US / 1000 + 5) != 1)
			break;
	}
	EXPECT_LT(i, 16);
	EXPECT_TRUE(lin_silent(self->s, LIN_SILENCE_MS));
}

/* Error frames bypass the JOIN_FILTERS (AND) data-filter gate: even with two
 * disjoint data filters that would suppress all data, NO_RESPONSE is
 * delivered.
 */
TEST_F(lin_err, no_response_bypasses_join_filters)
{
	struct lin_filter flt[2] = {
		{ .lin_id = 0x10, .id_mask = LIN_ID_MASK },
		{ .lin_id = 0x20, .id_mask = LIN_ID_MASK },
	};
	struct lin_frame f;
	int i, saw = 0;

	ASSERT_EQ(0, lin_set_filter(self->s, flt, 2));
	ASSERT_EQ(0, lin_setopt_int(self->s, LIN_RAW_JOIN_FILTERS, 1));
	ASSERT_EQ(0, lin_err_filter(self->s, LIN_ERR_NO_RESPONSE));
	ASSERT_EQ(0, schedule_unanswered(self->m, 0x10));

	for (i = 0; i < 16; i++) {
		if (lin_recv(self->s, &f, LIN_RECV_MS) != 1)
			break;
		if ((f.flags & LIN_F_ERR) &&
		    (f.err_mask & LIN_ERR_NO_RESPONSE)) {
			saw = 1;
			break;
		}
	}
	EXPECT_TRUE(saw);
}

TEST_HARNESS_MAIN
