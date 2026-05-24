// SPDX-License-Identifier: GPL-2.0
/*
 * PF_LIN selftests: device lifecycle. Bringing the interface down drains
 * per-bus policy (master / publishers / schedules) while the binding and
 * filters survive; deleting the interface unbinds the socket.
 */
#include "lin_harness.h"

static int run_uncond(int m, __u8 id)
{
	union lin_sched_buf buf = {};
	__u8 d[1] = { 0x5a };
	int ret;

	ret = lin_publish(m, id, d, 1, 0);
	if (ret)
		return ret;
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

FIXTURE(lin_life) {
	int ifindex;
	__u32 caps;
};

FIXTURE_SETUP(lin_life)
{
	self->ifindex = lin_setup_iface(LIN_IF);
	if (self->ifindex <= 0)
		SKIP(return, "need root and the vlin module (ip link add type vlin)");
	ASSERT_EQ(0, lin_query_caps(self->ifindex, &self->caps));
	LIN_REQUIRE_MANAGED_IFACE();
}

FIXTURE_TEARDOWN(lin_life)
{
}

TEST_F(lin_life, down_drains_policy)
{
	__u8 d[1] = { 1 };
	int m = lin_open_bound(self->ifindex);
	int p, h = -2;

	ASSERT_GE(m, 0);
	ASSERT_EQ(0, lin_master(m, 1));
	ASSERT_EQ(0, run_uncond(m, 0x10));	/* publisher 0x10 + schedule 0 */
	ASSERT_EQ(0, lin_sched_active(m, &h));
	ASSERT_EQ(0, h);

	ASSERT_EQ(0, lin_run("ip link set " LIN_IF " down"));
	ASSERT_EQ(0, lin_run("ip link set " LIN_IF " up"));

	/* The schedule was dropped by the drain ... */
	ASSERT_EQ(0, lin_sched_active(m, &h));
	EXPECT_EQ(-1, h);
	/* ... and the master role was released, so it can be re-claimed. */
	EXPECT_EQ(0, lin_master(m, 1));

	/* The loaded schedule was deleted, not merely deactivated: the old
	 * handle is no longer loaded, so re-activating it fails with ENOENT.
	 */
	EXPECT_EQ(-1, lin_sched_activate(m, 0));
	EXPECT_EQ(ENOENT, errno);

	/* The publisher entry m held for 0x10 was released: a different socket
	 * can now register it.
	 */
	p = lin_open_bound(self->ifindex);
	ASSERT_GE(p, 0);
	EXPECT_EQ(0, lin_publish(p, 0x10, d, 1, 0));
	close(p);
	close(m);
}

TEST_F(lin_life, sk_err_enetdown_on_down)
{
	int m = lin_open_bound(self->ifindex);
	int soerr = 0;
	socklen_t l = sizeof(soerr);

	ASSERT_GE(m, 0);
	ASSERT_EQ(0, lin_run("ip link set " LIN_IF " down"));

	ASSERT_EQ(0, getsockopt(m, SOL_SOCKET, SO_ERROR, &soerr, &l));
	EXPECT_EQ(ENETDOWN, soerr);
	close(m);
}

TEST_F(lin_life, policy_op_enetdown_when_down)
{
	int m = lin_open_bound(self->ifindex);
	int ret;

	ASSERT_GE(m, 0);
	ASSERT_EQ(0, lin_run("ip link set " LIN_IF " down"));
	ret = lin_master(m, 1);
	EXPECT_EQ(-1, ret);
	EXPECT_EQ(ENETDOWN, errno);
	close(m);
}

/* The master-gated operations (schedule load/delete/activate/stop and sleep)
 * return -EPERM, not -ENETDOWN, once the interface is down: the going-down
 * drain force-releases the master claim and it cannot be re-acquired while
 * down (LIN_RAW_MASTER itself returns -ENETDOWN), so the is_master check —
 * which precedes the link-state check — fires first. Contrast
 * policy_op_enetdown_when_down, where the role-agnostic LIN_RAW_MASTER claim
 * returns -ENETDOWN.
 */
TEST_F(lin_life, master_ops_eperm_after_down)
{
	union lin_sched_buf buf = {};
	int m = lin_open_bound(self->ifindex);

	ASSERT_GE(m, 0);
	ASSERT_EQ(0, lin_master(m, 1));
	ASSERT_EQ(0, lin_run("ip link set " LIN_IF " down"));

	/* A well-formed schedule payload so LIN_RAW_SCHEDULE_LOAD reaches the
	 * is_master gate rather than failing optlen validation first; the other
	 * ops take a valid in-range handle (0) for the same reason.
	 */
	buf.s.entry_count = 1;
	buf.s.default_slot_us = LIN_SLOT_US;
	buf.s.entry[0].type = LIN_SCHED_TYPE_UNCOND;
	buf.s.entry[0].member_count = 1;
	buf.s.entry[0].members[0] = 0x10;

	EXPECT_EQ(-1, lin_sched_load(m, &buf, 1));
	EXPECT_EQ(EPERM, errno);
	EXPECT_EQ(-1, lin_sched_delete(m, 0));
	EXPECT_EQ(EPERM, errno);
	EXPECT_EQ(-1, lin_sched_activate(m, 0));
	EXPECT_EQ(EPERM, errno);
	EXPECT_EQ(-1, lin_sched_stop(m));
	EXPECT_EQ(EPERM, errno);
	EXPECT_EQ(-1, lin_sleep(m));
	EXPECT_EQ(EPERM, errno);
	close(m);
}

TEST_F(lin_life, filters_survive_down_up)
{
	struct lin_filter flt = { .lin_id = 0x10, .id_mask = LIN_ID_MASK };
	int s = lin_open_bound(self->ifindex);
	struct lin_frame f;
	int soerr = 0, m, i;
	socklen_t l = sizeof(soerr);

	ASSERT_GE(s, 0);
	ASSERT_EQ(0, lin_set_filter(s, &flt, 1));

	ASSERT_EQ(0, lin_run("ip link set " LIN_IF " down"));
	ASSERT_EQ(0, lin_run("ip link set " LIN_IF " up"));

	/* down set sk_err = ENETDOWN on the bound subscriber; consume it so
	 * the next recv() returns a frame rather than the stale error.
	 */
	getsockopt(s, SOL_SOCKET, SO_ERROR, &soerr, &l);

	/* Re-establish a master+schedule after up (its policy was dropped);
	 * the subscriber's filter is still in effect without being re-set.
	 */
	m = lin_open_bound(self->ifindex);
	ASSERT_GE(m, 0);
	ASSERT_EQ(0, lin_master(m, 1));
	ASSERT_EQ(0, run_uncond(m, 0x10));

	ASSERT_EQ(1, lin_recv(s, &f, LIN_RECV_MS));
	EXPECT_EQ(0x10, f.lin_id);
	close(m);		/* stop the 0x10 emitter */

	/* Prove the surviving filter is still 0x10-specific, not silently
	 * reset to the default match-all: drain queued 0x10 frames, then emit
	 * 0x20 and require silence (match-all would have delivered it).
	 * Bounded: if close() failed to stop the 0x10 emitter, fail fast here
	 * instead of looping until the kselftest timeout.
	 */
	for (i = 0; i < 16; i++) {
		if (lin_recv(s, &f, LIN_SLOT_US / 1000 + 5) != 1)
			break;
	}
	EXPECT_LT(i, 16);
	m = lin_open_bound(self->ifindex);
	ASSERT_GE(m, 0);
	ASSERT_EQ(0, lin_master(m, 1));
	ASSERT_EQ(0, run_uncond(m, 0x20));
	EXPECT_TRUE(lin_silent(s, LIN_SILENCE_MS));
	close(s);
	close(m);
}

TEST_F(lin_life, delete_unbinds_socket)
{
	int m = lin_open_bound(self->ifindex);
	int ret;

	ASSERT_GE(m, 0);
	ASSERT_EQ(0, lin_master(m, 1));

	ASSERT_EQ(0, lin_run("ip link del " LIN_IF));

	/* After the interface is gone the socket is unbound; per-bus ops are
	 * no longer supported.
	 */
	ret = lin_master(m, 1);
	EXPECT_EQ(-1, ret);
	EXPECT_EQ(EOPNOTSUPP, errno);
	close(m);
}

TEST_HARNESS_MAIN
