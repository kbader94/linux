// SPDX-License-Identifier: GPL-2.0
/*
 * PF_LIN selftests: receive filters (LIN_RAW_FILTER / JOIN_FILTERS) and
 * getsockopt readback.
 *
 * Filtering is exercised against real bus traffic: a master+publisher
 * socket runs a single-slot unconditional schedule for a chosen ID, and a
 * separate subscriber socket (a non-owner, so it receives by default)
 * checks what its filter lets through.
 */
#include "lin_harness.h"

#include <linux/lin/error.h>

/* Start a looping emitter for @id; returns the master fd (keep it open to
 * keep the schedule running) or -1.
 */
static int start_emitter(int ifindex, __u8 id)
{
	union lin_sched_buf buf = {};
	__u8 data[2] = { id, 0x5a };
	int m = lin_open_bound(ifindex);

	if (m < 0)
		return -1;
	if (lin_master(m, 1) || lin_publish(m, id, data, sizeof(data), 0)) {
		close(m);
		return -1;
	}
	buf.s.entry_count = 1;
	buf.s.default_slot_us = LIN_SLOT_US;
	buf.s.entry[0].type = LIN_SCHED_TYPE_UNCOND;
	buf.s.entry[0].member_count = 1;
	buf.s.entry[0].members[0] = id;
	if (lin_sched_load(m, &buf, 1) || lin_sched_activate(m, 0)) {
		close(m);
		return -1;
	}
	return m;
}

FIXTURE(lin_filter) {
	int ifindex;
	__u32 caps;
};

FIXTURE_SETUP(lin_filter)
{
	self->ifindex = lin_setup_iface(LIN_IF);
	if (self->ifindex <= 0)
		SKIP(return, "need root and the vlin module (ip link add type vlin)");
	ASSERT_EQ(0, lin_query_caps(self->ifindex, &self->caps));
}

FIXTURE_TEARDOWN(lin_filter)
{
}

/* Exact-ID filter (id_mask == LIN_ID_MASK): the matching ID is delivered
 * and a non-matching ID is silent. Default-match (no filter set) is
 * implicitly covered by every other test that subscribes without setting
 * a filter.
 */
TEST_F(lin_filter, exact_id_match)
{
	struct lin_filter flt = { .lin_id = 0x10, .id_mask = LIN_ID_MASK };
	int s = lin_open_bound(self->ifindex);
	struct lin_frame f;
	int m;

	ASSERT_GE(s, 0);
	ASSERT_EQ(0, lin_set_filter(s, &flt, 1));

	/* 0x10 matches -> delivered. */
	m = start_emitter(self->ifindex, 0x10);
	ASSERT_GE(m, 0);
	ASSERT_EQ(1, lin_recv(s, &f, LIN_RECV_MS));
	EXPECT_EQ(0x10, f.lin_id);
	close(m);

	/* 0x20 doesn't match the same filter -> silent. */
	m = start_emitter(self->ifindex, 0x20);
	ASSERT_GE(m, 0);
	EXPECT_TRUE(lin_silent(s, LIN_SILENCE_MS));
	close(s);
	close(m);
}

/* A partial id_mask matches a range of IDs via the generic mask-filter path
 * (distinct from exact-ID and match-all). With lin_id 0x10 / id_mask 0x30,
 * only the masked bits matter: 0x12 matches (0x12 & 0x30 == 0x10) while 0x20
 * does not (0x20 & 0x30 == 0x20).
 */
TEST_F(lin_filter, partial_mask_match)
{
	struct lin_filter flt = { .lin_id = 0x10, .id_mask = 0x30 };
	int s = lin_open_bound(self->ifindex);
	struct lin_frame f;
	int m;

	ASSERT_GE(s, 0);
	ASSERT_EQ(0, lin_set_filter(s, &flt, 1));

	/* 0x20 differs in a masked bit => excluded (and queues nothing). */
	m = start_emitter(self->ifindex, 0x20);
	ASSERT_GE(m, 0);
	EXPECT_TRUE(lin_silent(s, LIN_SILENCE_MS));
	close(m);

	/* 0x12 shares the masked bits => delivered, where an exact-ID filter
	 * on 0x10 would have rejected it.
	 */
	m = start_emitter(self->ifindex, 0x12);
	ASSERT_GE(m, 0);
	ASSERT_EQ(1, lin_recv(s, &f, LIN_RECV_MS));
	EXPECT_EQ(0x12, f.lin_id);
	close(s);
	close(m);
}

TEST_F(lin_filter, inverted_filter)
{
	struct lin_filter flt = {
		.lin_id = 0x10, .id_mask = LIN_ID_MASK, .flags = LIN_FILT_INV,
	};
	int s = lin_open_bound(self->ifindex);
	struct lin_frame f;
	int m;

	ASSERT_GE(s, 0);
	ASSERT_EQ(0, lin_set_filter(s, &flt, 1));

	/* Inverted: 0x10 is excluded ... */
	m = start_emitter(self->ifindex, 0x10);
	ASSERT_GE(m, 0);
	EXPECT_TRUE(lin_silent(s, LIN_SILENCE_MS));
	close(m);

	/* ... but a different ID passes. */
	m = start_emitter(self->ifindex, 0x20);
	ASSERT_GE(m, 0);
	ASSERT_EQ(1, lin_recv(s, &f, LIN_RECV_MS));
	EXPECT_EQ(0x20, f.lin_id);
	close(s);
	close(m);
}

TEST_F(lin_filter, clear_filters_silences_data)
{
	int s = lin_open_bound(self->ifindex);
	int m;

	ASSERT_GE(s, 0);
	/* optlen == 0 clears all data filters. */
	ASSERT_EQ(0, lin_clear_filter(s));
	m = start_emitter(self->ifindex, 0x10);
	ASSERT_GE(m, 0);

	EXPECT_TRUE(lin_silent(s, LIN_SILENCE_MS));
	close(s);
	close(m);
}

TEST_F(lin_filter, join_filters_and)
{
	struct lin_filter flt[2] = {
		{ .lin_id = 0x10, .id_mask = LIN_ID_MASK },
		{ .lin_id = 0x20, .id_mask = LIN_ID_MASK },
	};
	int s = lin_open_bound(self->ifindex);
	struct lin_frame f;
	int m, i;

	ASSERT_GE(s, 0);
	ASSERT_EQ(0, lin_set_filter(s, flt, 2));

	/* OR (default): a 0x10 frame matches the first filter. */
	m = start_emitter(self->ifindex, 0x10);
	ASSERT_GE(m, 0);
	ASSERT_EQ(1, lin_recv(s, &f, LIN_RECV_MS));
	EXPECT_EQ(0x10, f.lin_id);
	close(m);		/* stops emission (master released on close) */

	/* Drain frames the OR phase already queued so they don't masquerade
	 * as AND-phase deliveries below. Bounded: if close() failed to stop
	 * emission, fail fast here instead of looping until the kselftest
	 * timeout.
	 */
	for (i = 0; i < 16; i++) {
		if (lin_recv(s, &f, LIN_SLOT_US / 1000 + 5) != 1)
			break;
	}
	EXPECT_LT(i, 16);

	/* AND: no single frame can match two disjoint ID filters. */
	ASSERT_EQ(0, lin_setopt_int(s, LIN_RAW_JOIN_FILTERS, 1));
	m = start_emitter(self->ifindex, 0x10);
	ASSERT_GE(m, 0);
	EXPECT_TRUE(lin_silent(s, LIN_SILENCE_MS));
	close(s);
	close(m);
}

/* Round-trip readback of all filter-related sockopts. */
TEST_F(lin_filter, getsockopt_roundtrip)
{
	struct lin_filter flt[2] = {
		{ .lin_id = 0x11, .id_mask = LIN_ID_MASK },
		{ .lin_id = 0x22, .id_mask = LIN_ID_MASK },
	};
	struct lin_filter out[2] = {};
	int s = lin_open_bound(self->ifindex);
	socklen_t l;
	__u32 mask;
	int val;

	ASSERT_GE(s, 0);

	/* LIN_RAW_FILTER: set 2-entry array, read it back. */
	ASSERT_EQ(0, lin_set_filter(s, flt, 2));
	l = sizeof(out);
	ASSERT_EQ(0, getsockopt(s, SOL_LIN_RAW, LIN_RAW_FILTER, out, &l));
	EXPECT_EQ(2 * sizeof(struct lin_filter), l);
	EXPECT_EQ(0x11, out[0].lin_id);
	EXPECT_EQ(0x22, out[1].lin_id);

	/* LIN_RAW_ERR_FILTER: set mask, read back. */
	ASSERT_EQ(0, lin_err_filter(s, LIN_ERR_NO_RESPONSE));
	l = sizeof(mask);
	ASSERT_EQ(0, getsockopt(s, SOL_LIN_RAW, LIN_RAW_ERR_FILTER, &mask, &l));
	EXPECT_EQ(LIN_ERR_NO_RESPONSE, mask);

	/* LIN_RAW_JOIN_FILTERS: bool round-trip. */
	ASSERT_EQ(0, lin_setopt_int(s, LIN_RAW_JOIN_FILTERS, 1));
	ASSERT_EQ(0, lin_getopt_int(s, LIN_RAW_JOIN_FILTERS, &val));
	EXPECT_EQ(1, val);

	/* LIN_RAW_WAKEUP_FILTER: bool round-trip. */
	ASSERT_EQ(0, lin_setopt_int(s, LIN_RAW_WAKEUP_FILTER, 1));
	ASSERT_EQ(0, lin_getopt_int(s, LIN_RAW_WAKEUP_FILTER, &val));
	EXPECT_EQ(1, val);
	close(s);
}

TEST_F(lin_filter, getsockopt_short_buffer_erange)
{
	struct lin_filter flt[2] = {
		{ .lin_id = 0x11, .id_mask = LIN_ID_MASK },
		{ .lin_id = 0x22, .id_mask = LIN_ID_MASK },
	};
	struct lin_filter out[1];	/* too small for two filters */
	socklen_t len = sizeof(out);
	int s = lin_open_bound(self->ifindex);
	int ret;

	ASSERT_GE(s, 0);
	ASSERT_EQ(0, lin_set_filter(s, flt, 2));
	ret = getsockopt(s, SOL_LIN_RAW, LIN_RAW_FILTER, out, &len);
	EXPECT_EQ(-1, ret);
	EXPECT_EQ(ERANGE, errno);
	/* The required size is reported back so the caller can retry. */
	EXPECT_EQ(2 * sizeof(struct lin_filter), len);
	close(s);
}

/* Overlapping filters: a specific 0x10 filter plus a match-all. A 0x10 frame
 * matches both, exercising the dedup/join logic that the disjoint-filter test
 * cannot reach.
 */
TEST_F(lin_filter, join_filters_overlap)
{
	struct lin_filter flt[2] = {
		{ .lin_id = 0x10, .id_mask = LIN_ID_MASK },	/* specific */
		{ .id_mask = 0x00 },				/* match-all */
	};
	union lin_sched_buf buf = {};
	__u8 d[1] = { 0x5a };
	int s = lin_open_bound(self->ifindex);
	int mfd = lin_open_bound(self->ifindex);
	struct lin_frame f;

	ASSERT_GE(s, 0);
	ASSERT_GE(mfd, 0);
	ASSERT_EQ(0, lin_set_filter(s, flt, 2));

	/* Long slot so consecutive emissions are far apart: a broken dedup
	 * (one delivery per matching filter) would queue a second copy of the
	 * same frame immediately, well before the next emission.
	 */
	ASSERT_EQ(0, lin_master(mfd, 1));
	ASSERT_EQ(0, lin_publish(mfd, 0x10, d, 1, 0));
	buf.s.entry_count = 1;
	buf.s.default_slot_us = 400000;			/* 400 ms */
	buf.s.entry[0].type = LIN_SCHED_TYPE_UNCOND;
	buf.s.entry[0].member_count = 1;
	buf.s.entry[0].members[0] = 0x10;
	ASSERT_EQ(0, lin_sched_load(mfd, &buf, 1));
	ASSERT_EQ(0, lin_sched_activate(mfd, 0));

	/* OR (default): the frame matches both filters but is delivered once. */
	ASSERT_EQ(1, lin_recv(s, &f, LIN_RECV_MS));
	EXPECT_EQ(0x10, f.lin_id);
	EXPECT_EQ(0, lin_recv(s, &f, 100));	/* no immediate duplicate */

	/* AND: the frame matches both filters, so it is still delivered. */
	ASSERT_EQ(0, lin_setopt_int(s, LIN_RAW_JOIN_FILTERS, 1));
	ASSERT_EQ(1, lin_recv(s, &f, LIN_RECV_MS));
	EXPECT_EQ(0x10, f.lin_id);
	close(s);
	close(mfd);
}

/* Reject paths for malformed filter shapes (all -EINVAL). */
TEST_F(lin_filter, invalid_filter_shapes)
{
	struct lin_filter many[LIN_RAW_FILTER_MAX + 1];
	struct lin_filter flt;
	int s = lin_open_bound(self->ifindex);
	int ret;

	ASSERT_GE(s, 0);

	/* optlen not a multiple of sizeof(struct lin_filter). */
	memset(&flt, 0, sizeof(flt));
	flt.lin_id = 0x10;
	flt.id_mask = LIN_ID_MASK;
	ret = lin_setopt(s, LIN_RAW_FILTER, &flt, sizeof(flt) - 1);
	EXPECT_EQ(-1, ret);
	EXPECT_EQ(EINVAL, errno);

	/* More than LIN_RAW_FILTER_MAX filters: rejected at the admission
	 * boundary (before any allocation), regardless of contents.
	 */
	memset(many, 0, sizeof(many));
	ret = lin_set_filter(s, many, LIN_RAW_FILTER_MAX + 1);
	EXPECT_EQ(-1, ret);
	EXPECT_EQ(EINVAL, errno);

	/* ID range: one representative for the lin_id / id_mask range check. */
	memset(&flt, 0, sizeof(flt));
	flt.lin_id = 0x40;
	flt.id_mask = LIN_ID_MASK;
	ret = lin_set_filter(s, &flt, 1);
	EXPECT_EQ(-1, ret);
	EXPECT_EQ(EINVAL, errno);

	/* Flag bits: one representative for the flags / flags_mask range check. */
	memset(&flt, 0, sizeof(flt));
	flt.flags = LIN_F_WAKEUP;	/* not a valid filter flag */
	ret = lin_set_filter(s, &flt, 1);
	EXPECT_EQ(-1, ret);
	EXPECT_EQ(EINVAL, errno);

	/* Nonzero reserved bytes. */
	memset(&flt, 0, sizeof(flt));
	flt.id_mask = LIN_ID_MASK;
	flt.__res[0] = 1;
	ret = lin_set_filter(s, &flt, 1);
	EXPECT_EQ(-1, ret);
	EXPECT_EQ(EINVAL, errno);

	/* Inverted filter with both masks zero (matches nothing). */
	memset(&flt, 0, sizeof(flt));
	flt.flags = LIN_FILT_INV;
	ret = lin_set_filter(s, &flt, 1);
	EXPECT_EQ(-1, ret);
	EXPECT_EQ(EINVAL, errno);
	close(s);
}

/* Filters set while the socket is unbound are installed by bind(). The
 * error and wakeup subscriptions are off by default, so their post-bind
 * delivery proves they were carried across the bind.
 */
TEST_F(lin_filter, filters_set_before_bind)
{
	LIN_SKIP_UNLESS_CAP(self->caps, LIN_CAP_WAKEUP);
	struct lin_filter dflt = { .lin_id = 0x10, .id_mask = LIN_ID_MASK };
	union lin_sched_buf buf = {};
	__u8 d[1] = { 0x5a };
	struct lin_frame f;
	int s = lin_socket();		/* unbound */
	int w, m, i, saw_data = 0, saw_noresp = 0;

	ASSERT_GE(s, 0);
	ASSERT_EQ(0, lin_set_filter(s, &dflt, 1));
	ASSERT_EQ(0, lin_err_filter(s, LIN_ERR_NO_RESPONSE));
	ASSERT_EQ(0, lin_setopt_int(s, LIN_RAW_WAKEUP_FILTER, 1));
	ASSERT_EQ(0, lin_bind(s, self->ifindex));

	/* Wakeup (bus quiescent): proves the wakeup filter was installed. */
	w = lin_open_bound(self->ifindex);
	ASSERT_GE(w, 0);
	ASSERT_EQ(0, lin_wakeup(w));
	ASSERT_EQ(1, lin_recv(s, &f, LIN_RECV_MS));
	EXPECT_NE(0, f.flags & LIN_F_WAKEUP);
	close(w);

	/* Data + error: schedule a published 0x10 and an unpublished 0x20. */
	m = lin_open_bound(self->ifindex);
	ASSERT_GE(m, 0);
	ASSERT_EQ(0, lin_master(m, 1));
	ASSERT_EQ(0, lin_publish(m, 0x10, d, 1, 0));
	buf.s.entry_count = 2;
	buf.s.default_slot_us = LIN_SLOT_US;
	buf.s.entry[0].type = LIN_SCHED_TYPE_UNCOND;
	buf.s.entry[0].member_count = 1;
	buf.s.entry[0].members[0] = 0x10;
	buf.s.entry[1].type = LIN_SCHED_TYPE_UNCOND;
	buf.s.entry[1].member_count = 1;
	buf.s.entry[1].members[0] = 0x20;
	ASSERT_EQ(0, lin_sched_load(m, &buf, 2));
	ASSERT_EQ(0, lin_sched_activate(m, 0));

	for (i = 0; i < 24 && !(saw_data && saw_noresp); i++) {
		if (lin_recv(s, &f, LIN_RECV_MS) != 1)
			break;
		if (f.lin_id == 0x10 && !(f.flags & LIN_F_ERR))
			saw_data = 1;		/* data filter installed */
		else if (f.lin_id == 0x20 && (f.err_mask & LIN_ERR_NO_RESPONSE))
			saw_noresp = 1;		/* error filter installed */
	}
	EXPECT_TRUE(saw_data);
	EXPECT_TRUE(saw_noresp);
	close(s);
	close(m);
}

/* A filter constrained on LIN_F_CHK_ENH delivers only enhanced-checksum
 * frames; a classic frame for the same ID is dropped.
 */
TEST_F(lin_filter, flags_mask_match)
{
	LIN_SKIP_UNLESS_CAP(self->caps, LIN_CAP_CHK_ENH);
	struct lin_filter flt = {
		.lin_id = 0x10, .id_mask = LIN_ID_MASK,
		.flags = LIN_F_CHK_ENH, .flags_mask = LIN_F_CHK_ENH,
	};
	union lin_sched_buf buf = {};
	__u8 d[1] = { 0x5a };
	int s = lin_open_bound(self->ifindex);
	struct lin_frame f;
	int m;

	ASSERT_GE(s, 0);
	ASSERT_EQ(0, lin_set_filter(s, &flt, 1));

	/* Classic 0x10: no LIN_F_CHK_ENH -> filtered out. */
	m = start_emitter(self->ifindex, 0x10);
	ASSERT_GE(m, 0);
	EXPECT_TRUE(lin_silent(s, LIN_SILENCE_MS));
	close(m);

	/* Enhanced 0x10: carries LIN_F_CHK_ENH -> delivered. */
	m = lin_open_bound(self->ifindex);
	ASSERT_GE(m, 0);
	ASSERT_EQ(0, lin_master(m, 1));
	ASSERT_EQ(0, lin_publish(m, 0x10, d, 1, 1));	/* enhanced */
	buf.s.entry_count = 1;
	buf.s.default_slot_us = LIN_SLOT_US;
	buf.s.entry[0].type = LIN_SCHED_TYPE_UNCOND;
	buf.s.entry[0].member_count = 1;
	buf.s.entry[0].members[0] = 0x10;
	ASSERT_EQ(0, lin_sched_load(m, &buf, 1));
	ASSERT_EQ(0, lin_sched_activate(m, 0));

	ASSERT_EQ(1, lin_recv(s, &f, LIN_RECV_MS));
	EXPECT_EQ(0x10, f.lin_id);
	EXPECT_NE(0, f.flags & LIN_F_CHK_ENH);
	close(s);
	close(m);
}

TEST_HARNESS_MAIN
