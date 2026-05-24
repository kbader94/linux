// SPDX-License-Identifier: GPL-2.0
/*
 * PF_LIN selftests: master schedule validation (synchronous return codes,
 * no bus traffic). Covers structural checks, per-type rules, the event
 * collision-resolving-schedule constraints, and the load/delete/activate
 * ordering rules.
 *
 * The mechanical "tweak one field of a valid schedule, expect EINVAL"
 * branches are bundled into a few umbrella tests so each rejection rule
 * is one sub-case rather than its own TEST_F. Substantive-logic branches
 * (cap interactions, state-dependent EBUSY/ENOENT, CR pinning, sockopt
 * admission) remain individual tests.
 */
#include "lin_harness.h"

static void mk_uncond(union lin_sched_buf *buf, __u8 handle, __u8 id)
{
	memset(buf, 0, sizeof(*buf));
	buf->s.handle = handle;
	buf->s.entry_count = 1;
	buf->s.default_slot_us = LIN_SLOT_US;
	buf->s.entry[0].type = LIN_SCHED_TYPE_UNCOND;
	buf->s.entry[0].member_count = 1;
	buf->s.entry[0].members[0] = id;
}

FIXTURE(lin_val) {
	int ifindex;
	__u32 caps;
	int fd;		/* master socket */
};

FIXTURE_SETUP(lin_val)
{
	self->ifindex = lin_setup_iface(LIN_IF);
	if (self->ifindex <= 0)
		SKIP(return, "need root and the vlin module (ip link add type vlin)");
	ASSERT_EQ(0, lin_query_caps(self->ifindex, &self->caps));
	self->fd = lin_open_bound(self->ifindex);
	ASSERT_GE(self->fd, 0);
	ASSERT_EQ(0, lin_master(self->fd, 1));
}

FIXTURE_TEARDOWN(lin_val)
{
	if (self->fd > 0)
		close(self->fd);
}

/* Positive: well-formed payloads load. */
TEST_F(lin_val, accepted_payloads)
{
	union lin_sched_buf buf;

	/* Basic unconditional schedule. */
	mk_uncond(&buf, 0, 0x10);
	EXPECT_EQ(0, lin_sched_load(self->fd, &buf, 1));

	/* No schedule default slot, but every entry carries its own slot_us:
	 * the effective duration is non-zero, so this is valid.
	 */
	mk_uncond(&buf, 1, 0x10);
	buf.s.default_slot_us = 0;
	buf.s.entry[0].slot_us = LIN_SLOT_US;
	EXPECT_EQ(0, lin_sched_load(self->fd, &buf, 1));
}

/* File-level structural rejections: handle / entry_count / buffer length /
 * reserved fields / trailing-members / slot_us bounds. Each sub-case mutates
 * one field of a valid unconditional schedule and expects EINVAL.
 */
TEST_F(lin_val, malformed_structural_payloads)
{
	union lin_sched_buf buf;

	/* handle out of range */
	mk_uncond(&buf, LIN_RAW_SCHEDULES_MAX, 0x10);
	EXPECT_EQ(-1, lin_sched_load(self->fd, &buf, 1));
	EXPECT_EQ(EINVAL, errno);

	/* entry_count = 0 (and a payload short enough to match) */
	mk_uncond(&buf, 0, 0x10);
	buf.s.entry_count = 0;
	EXPECT_EQ(-1, lin_sched_load(self->fd, &buf, 0));
	EXPECT_EQ(EINVAL, errno);

	/* buffer length disagrees with entry_count */
	mk_uncond(&buf, 0, 0x10);
	buf.s.entry_count = 2;			/* claims 2 ... */
	EXPECT_EQ(-1, lin_sched_load(self->fd, &buf, 1));	/* ... sends 1 */
	EXPECT_EQ(EINVAL, errno);

	/* reserved fields must be zero — schedule-level rep (s.flags) and
	 * entry-level rep (entry.flags). The s.__res / entry.__res variants
	 * hit the same validate_reserved pattern and are not exercised.
	 */
	mk_uncond(&buf, 0, 0x10);
	buf.s.flags = 1;
	EXPECT_EQ(-1, lin_sched_load(self->fd, &buf, 1));
	EXPECT_EQ(EINVAL, errno);

	mk_uncond(&buf, 0, 0x10);
	buf.s.entry[0].flags = 1;
	EXPECT_EQ(-1, lin_sched_load(self->fd, &buf, 1));
	EXPECT_EQ(EINVAL, errno);

	/* members[] beyond member_count must be zero */
	mk_uncond(&buf, 0, 0x10);
	buf.s.entry[0].member_count = 1;
	buf.s.entry[0].members[1] = 0x12;
	EXPECT_EQ(-1, lin_sched_load(self->fd, &buf, 1));
	EXPECT_EQ(EINVAL, errno);

	/* slot_us above the cap — exercised via the schedule-level default; the
	 * per-entry slot_us hits the same clamp and is not exercised separately.
	 */
	mk_uncond(&buf, 0, 0x10);
	buf.s.default_slot_us = LIN_RAW_SCHEDULE_SLOT_MAX_US + 1;
	buf.s.entry[0].slot_us = 0;
	EXPECT_EQ(-1, lin_sched_load(self->fd, &buf, 1));
	EXPECT_EQ(EINVAL, errno);

	/* neither default nor per-entry slot_us set => zero effective duration */
	mk_uncond(&buf, 0, 0x10);
	buf.s.default_slot_us = 0;
	buf.s.entry[0].slot_us = 0;
	EXPECT_EQ(-1, lin_sched_load(self->fd, &buf, 1));
	EXPECT_EQ(EINVAL, errno);
}

/* Per-entry shape and type-specific rejections that don't depend on any
 * driver state. Cap-gated sub-cases are guarded inline so they only run
 * against drivers that advertise the relevant LIN_CAP_*.
 */
TEST_F(lin_val, invalid_entry_payloads)
{
	union lin_sched_buf buf;

	/* unknown entry type */
	mk_uncond(&buf, 0, 0x10);
	buf.s.entry[0].type = 0x7f;
	EXPECT_EQ(-1, lin_sched_load(self->fd, &buf, 1));
	EXPECT_EQ(EINVAL, errno);

	/* member_count over LIN_SLOT_MAX_MEMBERS */
	mk_uncond(&buf, 0, 0x10);
	buf.s.entry[0].member_count = LIN_SLOT_MAX_MEMBERS + 1;
	EXPECT_EQ(-1, lin_sched_load(self->fd, &buf, 1));
	EXPECT_EQ(EINVAL, errno);

	/* reserved-range member ID (0x3E) */
	mk_uncond(&buf, 0, 0x10);
	buf.s.entry[0].members[0] = LIN_ID_RESERVED_FIRST;
	EXPECT_EQ(-1, lin_sched_load(self->fd, &buf, 1));
	EXPECT_EQ(EINVAL, errno);

	/* member ID bits above the 6-bit range (distinct from the reserved check) */
	mk_uncond(&buf, 0, 0x10);
	buf.s.entry[0].members[0] = 0x40;
	EXPECT_EQ(-1, lin_sched_load(self->fd, &buf, 1));
	EXPECT_EQ(EINVAL, errno);

	/* non-event entry must leave cr_handle zero */
	mk_uncond(&buf, 0, 0x10);
	buf.s.entry[0].cr_handle = 1;
	EXPECT_EQ(-1, lin_sched_load(self->fd, &buf, 1));
	EXPECT_EQ(EINVAL, errno);

	/* UNCOND must carry exactly one member: >1 is rejected, even when the
	 * extra ID is itself valid.
	 */
	mk_uncond(&buf, 0, 0x10);
	buf.s.entry[0].member_count = 2;
	buf.s.entry[0].members[1] = 0x11;
	EXPECT_EQ(-1, lin_sched_load(self->fd, &buf, 1));
	EXPECT_EQ(EINVAL, errno);

	/* TYPE_DIAG slot with a non-diagnostic member ID */
	if (self->caps & LIN_CAP_DIAG) {
		mk_uncond(&buf, 0, 0x10);
		buf.s.entry[0].type = LIN_SCHED_TYPE_DIAG;
		EXPECT_EQ(-1, lin_sched_load(self->fd, &buf, 1));
		EXPECT_EQ(EINVAL, errno);
	}

	/* TYPE_EVENT slot must carry exactly one member */
	if (self->caps & LIN_CAP_EVENT) {
		union lin_sched_buf cr, ev;

		mk_uncond(&cr, 1, 0x10);
		ASSERT_EQ(0, lin_sched_load(self->fd, &cr, 1));
		mk_uncond(&ev, 2, 0x20);
		ev.s.entry[0].type = LIN_SCHED_TYPE_EVENT;
		ev.s.entry[0].member_count = 2;
		ev.s.entry[0].cr_handle = 1;
		EXPECT_EQ(-1, lin_sched_load(self->fd, &ev, 1));
		EXPECT_EQ(EINVAL, errno);

		/* TYPE_EVENT cr_handle range-checked per entry, before the
		 * "is it loaded" check.
		 */
		mk_uncond(&ev, 2, 0x30);
		ev.s.entry[0].type = LIN_SCHED_TYPE_EVENT;
		ev.s.entry[0].cr_handle = LIN_RAW_SCHEDULES_MAX;
		EXPECT_EQ(-1, lin_sched_load(self->fd, &ev, 1));
		EXPECT_EQ(EINVAL, errno);
	}
}

TEST_F(lin_val, sporadic_member_needs_publisher)
{
	LIN_SKIP_UNLESS_CAP(self->caps, LIN_CAP_SPORADIC);
	union lin_sched_buf buf;
	__u8 d[1] = { 1 };
	int ret;

	mk_uncond(&buf, 0, 0x10);
	buf.s.entry[0].type = LIN_SCHED_TYPE_SPORADIC;
	buf.s.entry[0].member_count = 1;
	buf.s.entry[0].members[0] = 0x10;	/* no publisher registered */
	ret = lin_sched_load(self->fd, &buf, 1);
	EXPECT_EQ(-1, ret);
	EXPECT_EQ(EINVAL, errno);

	/* With a publisher it loads. */
	ASSERT_EQ(0, lin_publish(self->fd, 0x10, d, 1, 0));
	EXPECT_EQ(0, lin_sched_load(self->fd, &buf, 1));
}

/* Diagnostic IDs (0x3C/0x3D) belong only in a TYPE_DIAG slot; UNCOND,
 * SPORADIC, and EVENT slots reject them via the per-type seen_diag checks.
 */
TEST_F(lin_val, diag_id_wrong_type)
{
	union lin_sched_buf buf, cr, ev;
	__u8 d[1] = { 1 };
	int ret;

	/* UNCOND with a diagnostic member — runs on every driver, since UNCOND
	 * + classic checksum is the baseline contract.
	 */
	mk_uncond(&buf, 0, LIN_ID_DIAG_MASTER_REQ);
	ret = lin_sched_load(self->fd, &buf, 1);
	EXPECT_EQ(-1, ret);
	EXPECT_EQ(EINVAL, errno);

	/* SPORADIC with a diagnostic member. Publish 0x3C first so the
	 * rejection is the type rule, not sporadic missing-publisher — but
	 * that publish needs LIN_CAP_DIAG (publisher_set's diag-id gate),
	 * and the schedule load itself needs LIN_CAP_SPORADIC, so this
	 * sub-case is gated on both being advertised.
	 */
	if ((self->caps & (LIN_CAP_SPORADIC | LIN_CAP_DIAG)) ==
	    (LIN_CAP_SPORADIC | LIN_CAP_DIAG)) {
		ASSERT_EQ(0, lin_publish(self->fd, LIN_ID_DIAG_MASTER_REQ,
					 d, 1, 0));
		mk_uncond(&buf, 0, LIN_ID_DIAG_MASTER_REQ);
		buf.s.entry[0].type = LIN_SCHED_TYPE_SPORADIC;
		ret = lin_sched_load(self->fd, &buf, 1);
		EXPECT_EQ(-1, ret);
		EXPECT_EQ(EINVAL, errno);
	}

	/* EVENT with a diagnostic trigger, against a valid loaded CR schedule
	 * so the diagnostic ID is the only fault — gated on LIN_CAP_EVENT.
	 */
	if (self->caps & LIN_CAP_EVENT) {
		mk_uncond(&cr, 1, 0x10);
		ASSERT_EQ(0, lin_sched_load(self->fd, &cr, 1));
		mk_uncond(&ev, 2, LIN_ID_DIAG_MASTER_REQ);
		ev.s.entry[0].type = LIN_SCHED_TYPE_EVENT;
		ev.s.entry[0].cr_handle = 1;
		ret = lin_sched_load(self->fd, &ev, 1);
		EXPECT_EQ(-1, ret);
		EXPECT_EQ(EINVAL, errno);
	}
}

TEST_F(lin_val, event_cr_not_loaded)
{
	LIN_SKIP_UNLESS_CAP(self->caps, LIN_CAP_EVENT);
	union lin_sched_buf ev;
	int ret;

	mk_uncond(&ev, 2, 0x20);
	ev.s.entry[0].type = LIN_SCHED_TYPE_EVENT;
	ev.s.entry[0].member_count = 1;
	ev.s.entry[0].cr_handle = 5;		/* nothing loaded at 5 */
	ret = lin_sched_load(self->fd, &ev, 1);
	EXPECT_EQ(-1, ret);
	EXPECT_EQ(EINVAL, errno);
}

TEST_F(lin_val, event_cr_must_be_unconditional)
{
	LIN_SKIP_UNLESS_CAP(self->caps, LIN_CAP_EVENT);
	union lin_sched_buf cr, ev;
	__u8 d[1] = { 1 };
	int ret;

	/* CR schedule 1 contains a sporadic entry => not unconditional-only. */
	ASSERT_EQ(0, lin_publish(self->fd, 0x10, d, 1, 0));
	mk_uncond(&cr, 1, 0x10);
	cr.s.entry[0].type = LIN_SCHED_TYPE_SPORADIC;
	ASSERT_EQ(0, lin_sched_load(self->fd, &cr, 1));

	mk_uncond(&ev, 2, 0x20);
	ev.s.entry[0].type = LIN_SCHED_TYPE_EVENT;
	ev.s.entry[0].member_count = 1;
	ev.s.entry[0].cr_handle = 1;
	ret = lin_sched_load(self->fd, &ev, 1);
	EXPECT_EQ(-1, ret);
	EXPECT_EQ(EINVAL, errno);
}

/* While a handle is the active schedule, both re-LOAD and DELETE against it
 * are refused with EBUSY (the only way to clear it is STOP, then operate).
 */
TEST_F(lin_val, ops_busy_when_active)
{
	union lin_sched_buf buf;

	mk_uncond(&buf, 0, 0x10);
	ASSERT_EQ(0, lin_sched_load(self->fd, &buf, 1));
	ASSERT_EQ(0, lin_sched_activate(self->fd, 0));

	EXPECT_EQ(-1, lin_sched_load(self->fd, &buf, 1));
	EXPECT_EQ(EBUSY, errno);
	EXPECT_EQ(-1, lin_sched_delete(self->fd, 0));
	EXPECT_EQ(EBUSY, errno);
}

/* Operating on a handle that isn't loaded returns ENOENT for both
 * SCHEDULE_DELETE and SCHEDULE_ACTIVATE.
 */
TEST_F(lin_val, unloaded_handle_rejections)
{
	EXPECT_EQ(-1, lin_sched_delete(self->fd, 7));
	EXPECT_EQ(ENOENT, errno);

	EXPECT_EQ(-1, lin_sched_activate(self->fd, 7));
	EXPECT_EQ(ENOENT, errno);
}

/* A CR handle referenced by an event schedule is pinned: both DELETE and
 * re-LOAD against it are refused with EBUSY, even when the referrer is
 * inactive (the unconditional-only property must not change behind it).
 * Dropping the referrer first unblocks the delete.
 */
TEST_F(lin_val, referenced_cr_busy_ops)
{
	LIN_SKIP_UNLESS_CAP(self->caps, LIN_CAP_EVENT);
	union lin_sched_buf cr, ev;

	mk_uncond(&cr, 1, 0x10);
	ASSERT_EQ(0, lin_sched_load(self->fd, &cr, 1));
	mk_uncond(&ev, 2, 0x20);
	ev.s.entry[0].type = LIN_SCHED_TYPE_EVENT;
	ev.s.entry[0].member_count = 1;
	ev.s.entry[0].cr_handle = 1;
	ASSERT_EQ(0, lin_sched_load(self->fd, &ev, 1));

	/* DELETE while referenced. */
	EXPECT_EQ(-1, lin_sched_delete(self->fd, 1));
	EXPECT_EQ(EBUSY, errno);

	/* Re-LOAD while referenced. */
	mk_uncond(&cr, 1, 0x11);
	EXPECT_EQ(-1, lin_sched_load(self->fd, &cr, 1));
	EXPECT_EQ(EBUSY, errno);

	/* Drop the referrer first, then the referent deletes. */
	ASSERT_EQ(0, lin_sched_delete(self->fd, 2));
	EXPECT_EQ(0, lin_sched_delete(self->fd, 1));
}

/* setsockopt admission: optlen and handle-range rejections that the typed
 * helpers never exercise.
 */
TEST_F(lin_val, sockopt_admission)
{
	union lin_sched_buf buf;
	int v = 0, ret;

	mk_uncond(&buf, 0, 0x10);

	/* LOAD: optlen shorter than struct lin_schedule. */
	ret = setsockopt(self->fd, SOL_LIN_RAW, LIN_RAW_SCHEDULE_LOAD, &buf, 1);
	EXPECT_EQ(-1, ret);
	EXPECT_EQ(EINVAL, errno);

	/* LOAD: optlen larger than the maximum schedule (rejected before the
	 * buffer is read).
	 */
	ret = setsockopt(self->fd, SOL_LIN_RAW, LIN_RAW_SCHEDULE_LOAD, &buf,
			 sizeof(struct lin_schedule) +
			 (LIN_RAW_SCHEDULE_ENTRIES_MAX + 1) *
			 sizeof(struct lin_schedule_entry));
	EXPECT_EQ(-1, ret);
	EXPECT_EQ(EINVAL, errno);

	/* DELETE / ACTIVATE: wrong optlen. */
	ret = setsockopt(self->fd, SOL_LIN_RAW, LIN_RAW_SCHEDULE_DELETE, &v, 1);
	EXPECT_EQ(-1, ret);
	EXPECT_EQ(EINVAL, errno);
	ret = setsockopt(self->fd, SOL_LIN_RAW, LIN_RAW_SCHEDULE_ACTIVATE, &v, 1);
	EXPECT_EQ(-1, ret);
	EXPECT_EQ(EINVAL, errno);

	/* DELETE / ACTIVATE: handle out of range. */
	ret = lin_sched_delete(self->fd, LIN_RAW_SCHEDULES_MAX);
	EXPECT_EQ(-1, ret);
	EXPECT_EQ(EINVAL, errno);
	ret = lin_sched_activate(self->fd, LIN_RAW_SCHEDULES_MAX);
	EXPECT_EQ(-1, ret);
	EXPECT_EQ(EINVAL, errno);

	/* STOP: no-argument sockopt — non-zero optlen rejected. */
	ret = setsockopt(self->fd, SOL_LIN_RAW, LIN_RAW_SCHEDULE_STOP,
			 &v, sizeof(v));
	EXPECT_EQ(-1, ret);
	EXPECT_EQ(EINVAL, errno);
}

TEST_HARNESS_MAIN
