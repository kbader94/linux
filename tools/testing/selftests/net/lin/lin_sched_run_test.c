// SPDX-License-Identifier: GPL-2.0
/*
 * PF_LIN selftests: unconditional / diagnostic schedule execution, active
 * handle readback, stop, and schedule switching.
 *
 * The master+publisher socket runs the schedule; a separate subscriber
 * socket observes the emitted frames (a non-owner receives by default).
 */
#include "lin_harness.h"

FIXTURE(lin_run) {
	int ifindex;
	__u32 caps;
	int m;		/* master + publisher */
	int s;		/* subscriber */
};

FIXTURE_SETUP(lin_run)
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

FIXTURE_TEARDOWN(lin_run)
{
	if (self->m > 0)
		close(self->m);
	if (self->s > 0)
		close(self->s);
}

/* Receive until a frame with @id is seen (bounded), or fail. */
static int recv_until(int fd, __u8 id, struct lin_frame *out)
{
	int i;

	for (i = 0; i < 32; i++) {
		if (lin_recv(fd, out, LIN_RECV_MS) != 1)
			return 0;
		if (out->lin_id == id)
			return 1;
	}
	return 0;
}

TEST_F(lin_run, uncond_delivers_response)
{
	union lin_sched_buf buf = {};
	__u8 data[3] = { 0x11, 0x22, 0x33 };
	struct lin_frame f;

	ASSERT_EQ(0, lin_publish(self->m, 0x10, data, 3, 0));
	buf.s.entry_count = 1;
	buf.s.default_slot_us = LIN_SLOT_US;
	buf.s.entry[0].type = LIN_SCHED_TYPE_UNCOND;
	buf.s.entry[0].member_count = 1;
	buf.s.entry[0].members[0] = 0x10;
	ASSERT_EQ(0, lin_sched_load(self->m, &buf, 1));
	ASSERT_EQ(0, lin_sched_activate(self->m, 0));

	ASSERT_EQ(1, lin_recv(self->s, &f, LIN_RECV_MS));
	EXPECT_EQ(0x10, f.lin_id);
	EXPECT_EQ(0, f.flags & LIN_F_ERR);
	EXPECT_EQ(3, f.len);
	EXPECT_EQ(0x11, f.data[0]);
	EXPECT_EQ(0x33, f.data[2]);
}

TEST_F(lin_run, multi_slot_cycles)
{
	union lin_sched_buf buf = {};
	__u8 a[1] = { 0xa0 }, b[1] = { 0xb0 };
	struct lin_frame f;

	ASSERT_EQ(0, lin_publish(self->m, 0x10, a, 1, 0));
	ASSERT_EQ(0, lin_publish(self->m, 0x20, b, 1, 0));
	buf.s.entry_count = 2;
	buf.s.default_slot_us = LIN_SLOT_US;
	buf.s.entry[0].type = LIN_SCHED_TYPE_UNCOND;
	buf.s.entry[0].member_count = 1;
	buf.s.entry[0].members[0] = 0x10;
	buf.s.entry[1].type = LIN_SCHED_TYPE_UNCOND;
	buf.s.entry[1].member_count = 1;
	buf.s.entry[1].members[0] = 0x20;
	ASSERT_EQ(0, lin_sched_load(self->m, &buf, 2));
	ASSERT_EQ(0, lin_sched_activate(self->m, 0));

	EXPECT_TRUE(recv_until(self->s, 0x10, &f));
	EXPECT_TRUE(recv_until(self->s, 0x20, &f));
}

/* Active-handle readback across the full activate/idempotent-re-activate/stop
 * cycle: -1 when nothing is active, the handle when active, -1 again after
 * stop. Re-activating the already-active handle is a no-op success that does
 * not interrupt traffic.
 */
TEST_F(lin_run, active_handle_readback)
{
	union lin_sched_buf buf = {};
	__u8 d[1] = { 1 };
	struct lin_frame f;
	int h = -2;

	ASSERT_EQ(0, lin_publish(self->m, 0x10, d, 1, 0));
	buf.s.entry_count = 1;
	buf.s.default_slot_us = LIN_SLOT_US;
	buf.s.entry[0].type = LIN_SCHED_TYPE_UNCOND;
	buf.s.entry[0].member_count = 1;
	buf.s.entry[0].members[0] = 0x10;
	ASSERT_EQ(0, lin_sched_load(self->m, &buf, 1));

	/* Loaded but not yet active. */
	ASSERT_EQ(0, lin_sched_active(self->m, &h));
	EXPECT_EQ(-1, h);

	/* Activate; readback reflects the active handle; traffic flows. */
	ASSERT_EQ(0, lin_sched_activate(self->m, 0));
	ASSERT_EQ(0, lin_sched_active(self->m, &h));
	EXPECT_EQ(0, h);
	ASSERT_EQ(1, lin_recv(self->s, &f, LIN_RECV_MS));

	/* Re-activating the already-active handle is a no-op success and
	 * does not interrupt traffic.
	 */
	EXPECT_EQ(0, lin_sched_activate(self->m, 0));
	ASSERT_EQ(0, lin_sched_active(self->m, &h));
	EXPECT_EQ(0, h);
	EXPECT_EQ(1, lin_recv(self->s, &f, LIN_RECV_MS));

	/* Stop returns the readback to -1. */
	ASSERT_EQ(0, lin_sched_stop(self->m));
	ASSERT_EQ(0, lin_sched_active(self->m, &h));
	EXPECT_EQ(-1, h);
}

TEST_F(lin_run, stop_halts_traffic)
{
	union lin_sched_buf buf = {};
	__u8 d[1] = { 1 };
	struct lin_frame f;
	int i;

	ASSERT_EQ(0, lin_publish(self->m, 0x10, d, 1, 0));
	buf.s.entry_count = 1;
	buf.s.default_slot_us = LIN_SLOT_US;
	buf.s.entry[0].type = LIN_SCHED_TYPE_UNCOND;
	buf.s.entry[0].member_count = 1;
	buf.s.entry[0].members[0] = 0x10;
	ASSERT_EQ(0, lin_sched_load(self->m, &buf, 1));
	ASSERT_EQ(0, lin_sched_activate(self->m, 0));
	ASSERT_EQ(1, lin_recv(self->s, &f, LIN_RECV_MS));

	ASSERT_EQ(0, lin_sched_stop(self->m));
	/* Drain anything already queued, then confirm silence. Bounded: if
	 * stop regresses and slots keep firing, fail fast here instead of
	 * looping until the kselftest timeout.
	 */
	for (i = 0; i < 16; i++) {
		if (lin_recv(self->s, &f, LIN_SLOT_US / 1000 + 5) != 1)
			break;
	}
	EXPECT_LT(i, 16);
	EXPECT_TRUE(lin_silent(self->s, LIN_SILENCE_MS));
}

/* Loading a *different* handle while one is active is allowed (raw.h;
 * dev.c only rejects reloading the active handle). The running schedule is
 * unaffected by the load: the active handle is unchanged and the bus stays on
 * handle 0 until ACTIVATE switches to handle 1.
 */
TEST_F(lin_run, load_while_active)
{
	union lin_sched_buf b0 = {}, b1 = {};
	__u8 a[1] = { 0xa0 }, b[1] = { 0xb0 };
	struct lin_frame f;
	int h = -2, i;

	ASSERT_EQ(0, lin_publish(self->m, 0x10, a, 1, 0));
	ASSERT_EQ(0, lin_publish(self->m, 0x20, b, 1, 0));

	/* Handle 0 uses a long slot (100 ms) so that, after the load below, the
	 * receive queue can be drained to a gap and the next frame is provably
	 * emitted *after* the load. With a short slot, frames queued before the
	 * load could be mistaken for post-load traffic and let a switch-on-load
	 * bug pass unnoticed.
	 */
	b0.s.handle = 0;
	b0.s.entry_count = 1;
	b0.s.default_slot_us = 100000;
	b0.s.entry[0].type = LIN_SCHED_TYPE_UNCOND;
	b0.s.entry[0].member_count = 1;
	b0.s.entry[0].members[0] = 0x10;
	ASSERT_EQ(0, lin_sched_load(self->m, &b0, 1));
	ASSERT_EQ(0, lin_sched_activate(self->m, 0));
	EXPECT_TRUE(recv_until(self->s, 0x10, &f));

	/* Load handle 1 while handle 0 runs: accepted, and must not switch. */
	b1.s.handle = 1;
	b1.s.entry_count = 1;
	b1.s.default_slot_us = LIN_SLOT_US;
	b1.s.entry[0].type = LIN_SCHED_TYPE_UNCOND;
	b1.s.entry[0].member_count = 1;
	b1.s.entry[0].members[0] = 0x20;
	ASSERT_EQ(0, lin_sched_load(self->m, &b1, 1));

	/* The active handle is unchanged by the load (core-level proof). */
	ASSERT_EQ(0, lin_sched_active(self->m, &h));
	EXPECT_EQ(0, h);

	/* Drain frames queued before/during the load to a gap (bounded; the
	 * long slot guarantees the drain terminates), so the next frame is
	 * freshly emitted after the load.
	 */
	for (i = 0; i < 16; i++) {
		if (lin_recv(self->s, &f, LIN_SLOT_US / 1000 + 5) != 1)
			break;
	}
	EXPECT_LT(i, 16);

	/* That post-load frame is still handle 0 (0x10): the load did not move
	 * the running schedule (driver-level proof).
	 */
	ASSERT_EQ(1, lin_recv(self->s, &f, LIN_RECV_MS));
	EXPECT_EQ(0x10, f.lin_id);

	/* Now switch: the active handle and the bus both move to handle 1. */
	ASSERT_EQ(0, lin_sched_activate(self->m, 1));
	ASSERT_EQ(0, lin_sched_active(self->m, &h));
	EXPECT_EQ(1, h);
	EXPECT_TRUE(recv_until(self->s, 0x20, &f));
}

/* Reloading an inactive handle replaces its contents; activation then runs
 * the replacement (vlin frees the old schedule), so the original ID never
 * appears.
 */
TEST_F(lin_run, reload_inactive_replaces)
{
	union lin_sched_buf buf = {};
	__u8 a[1] = { 0xa0 }, b[1] = { 0xb0 };
	struct lin_frame f;

	ASSERT_EQ(0, lin_publish(self->m, 0x10, a, 1, 0));
	ASSERT_EQ(0, lin_publish(self->m, 0x20, b, 1, 0));

	buf.s.handle = 0;
	buf.s.entry_count = 1;
	buf.s.default_slot_us = LIN_SLOT_US;
	buf.s.entry[0].type = LIN_SCHED_TYPE_UNCOND;
	buf.s.entry[0].member_count = 1;
	buf.s.entry[0].members[0] = 0x10;
	ASSERT_EQ(0, lin_sched_load(self->m, &buf, 1));

	/* Reload the same (still inactive) handle with a different member. */
	buf.s.entry[0].members[0] = 0x20;
	ASSERT_EQ(0, lin_sched_load(self->m, &buf, 1));

	/* Activation runs the replacement (0x20); 0x10 would only appear if the
	 * reload had not taken effect.
	 */
	ASSERT_EQ(0, lin_sched_activate(self->m, 0));
	EXPECT_TRUE(recv_until(self->s, 0x20, &f));
}

TEST_F(lin_run, diag_slot)
{
	LIN_SKIP_UNLESS_CAP(self->caps, LIN_CAP_DIAG);
	union lin_sched_buf buf = {};
	__u8 d[2] = { 0x3c, 0x01 };
	struct lin_frame f;

	ASSERT_EQ(0, lin_publish(self->m, LIN_ID_DIAG_MASTER_REQ, d, 2, 0));
	buf.s.entry_count = 1;
	buf.s.default_slot_us = LIN_SLOT_US;
	buf.s.entry[0].type = LIN_SCHED_TYPE_DIAG;
	buf.s.entry[0].member_count = 1;
	buf.s.entry[0].members[0] = LIN_ID_DIAG_MASTER_REQ;
	ASSERT_EQ(0, lin_sched_load(self->m, &buf, 1));
	ASSERT_EQ(0, lin_sched_activate(self->m, 0));

	ASSERT_EQ(1, lin_recv(self->s, &f, LIN_RECV_MS));
	EXPECT_EQ(LIN_ID_DIAG_MASTER_REQ, f.lin_id);
}

TEST_HARNESS_MAIN
