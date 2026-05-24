// SPDX-License-Identifier: GPL-2.0
/*
 * PF_LIN selftests: bus wakeup (LIN_RAW_WAKEUP / WAKEUP_FILTER) and the
 * sleep command (LIN_RAW_SLEEP).
 */
#include "lin_harness.h"

static int activate_dummy(int m)
{
	union lin_sched_buf buf = {};
	__u8 d[1] = { 1 };
	int ret;

	ret = lin_publish(m, 0x10, d, 1, 0);
	if (ret)
		return ret;
	buf.s.entry_count = 1;
	buf.s.default_slot_us = LIN_SLOT_US;
	buf.s.entry[0].type = LIN_SCHED_TYPE_UNCOND;
	buf.s.entry[0].member_count = 1;
	buf.s.entry[0].members[0] = 0x10;
	ret = lin_sched_load(m, &buf, 1);
	if (ret)
		return ret;
	return lin_sched_activate(m, 0);
}

FIXTURE(lin_wake) {
	int ifindex;
	__u32 caps;
};

FIXTURE_SETUP(lin_wake)
{
	self->ifindex = lin_setup_iface(LIN_IF);
	if (self->ifindex <= 0)
		SKIP(return, "need root and the vlin module (ip link add type vlin)");
	ASSERT_EQ(0, lin_query_caps(self->ifindex, &self->caps));
}

FIXTURE_TEARDOWN(lin_wake)
{
}

/* WAKEUP_FILTER gates wakeup delivery: a subscriber with the filter set
 * receives the wakeup frame (LIN_ID_NONE, LIN_F_WAKEUP), and a subscriber
 * with only the default data filter does not. Wakeup is role-agnostic: any
 * bound socket may drive it.
 */
TEST_F(lin_wake, wakeup_filter_toggles_delivery)
{
	LIN_SKIP_UNLESS_CAP(self->caps, LIN_CAP_WAKEUP);
	int filt = lin_open_bound(self->ifindex);
	int plain = lin_open_bound(self->ifindex);	/* default data filter only */
	int w = lin_open_bound(self->ifindex);
	struct lin_frame f;

	ASSERT_GE(filt, 0);
	ASSERT_GE(plain, 0);
	ASSERT_GE(w, 0);
	ASSERT_EQ(0, lin_setopt_int(filt, LIN_RAW_WAKEUP_FILTER, 1));

	ASSERT_EQ(0, lin_wakeup(w));

	ASSERT_EQ(1, lin_recv(filt, &f, LIN_RECV_MS));
	EXPECT_NE(0, f.flags & LIN_F_WAKEUP);
	EXPECT_EQ(LIN_ID_NONE, f.lin_id);

	EXPECT_TRUE(lin_silent(plain, LIN_SILENCE_MS));
	close(filt);
	close(plain);
	close(w);
}

/* Wakeup and sleep both refuse to fire while a schedule is active — wakeup
 * because the bus is awake, sleep because its one-shot header would collide
 * with a scheduled slot.
 */
TEST_F(lin_wake, busy_when_scheduled)
{
	int m = lin_open_bound(self->ifindex);

	ASSERT_GE(m, 0);
	ASSERT_EQ(0, lin_master(m, 1));
	ASSERT_EQ(0, activate_dummy(m));

	if (self->caps & LIN_CAP_WAKEUP) {
		EXPECT_EQ(-1, lin_wakeup(m));
		EXPECT_EQ(EBUSY, errno);
	}
	EXPECT_EQ(-1, lin_sleep(m));
	EXPECT_EQ(EBUSY, errno);
	close(m);
}

TEST_F(lin_wake, wakeup_when_down_netdown)
{
	LIN_REQUIRE_MANAGED_IFACE();
	LIN_SKIP_UNLESS_CAP(self->caps, LIN_CAP_WAKEUP);
	int w = lin_open_bound(self->ifindex);
	int ret;

	ASSERT_GE(w, 0);
	ASSERT_EQ(0, lin_run("ip link set " LIN_IF " down"));
	ret = lin_wakeup(w);
	EXPECT_EQ(-1, ret);
	EXPECT_EQ(ENETDOWN, errno);
	close(w);
}

TEST_F(lin_wake, sleep_command_emitted)
{
	int m = lin_open_bound(self->ifindex);
	int s = lin_open_bound(self->ifindex);
	struct lin_frame f;
	int i;

	ASSERT_GE(m, 0);
	ASSERT_GE(s, 0);
	ASSERT_EQ(0, lin_master(m, 1));

	ASSERT_EQ(0, lin_sleep(m));

	/* Per spec, the go-to-sleep command is a classic-checksum 0x3C frame
	 * carrying { 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF }.
	 */
	ASSERT_EQ(1, lin_recv(s, &f, LIN_RECV_MS));
	EXPECT_EQ(LIN_ID_DIAG_MASTER_REQ, f.lin_id);
	EXPECT_EQ(LIN_MAX_DLEN, f.len);
	EXPECT_EQ(0x00, f.data[0]);
	for (i = 1; i < LIN_MAX_DLEN; i++)
		EXPECT_EQ(0xff, f.data[i]);
	EXPECT_EQ(0, f.flags & LIN_F_CHK_ENH);	/* classic checksum */
	EXPECT_EQ(0, f.flags & LIN_F_ERR);
	close(m);
	close(s);
}

TEST_F(lin_wake, sleep_needs_master)
{
	int fd = lin_open_bound(self->ifindex);
	int ret;

	ASSERT_GE(fd, 0);
	ret = lin_sleep(fd);		/* not the master */
	EXPECT_EQ(-1, ret);
	EXPECT_EQ(EPERM, errno);
	close(fd);
}

/* Sleep emits the go-to-sleep command via the master header-send primitive on
 * LIN_ID_DIAG_MASTER_REQ. A master-supplied response on an ID that already has
 * a registered publisher would put two different responses on one slot, so
 * header_send (and thus LIN_RAW_SLEEP) is refused with EBUSY. vlin advertises
 * LIN_CAP_DIAG, so a classic-checksum publisher may be registered on 0x3C.
 */
TEST_F(lin_wake, sleep_busy_when_diag_published)
{
	LIN_SKIP_UNLESS_CAP(self->caps, LIN_CAP_DIAG);
	int m = lin_open_bound(self->ifindex);
	__u8 d[1] = { 0x00 };
	int ret;

	ASSERT_GE(m, 0);
	ASSERT_EQ(0, lin_master(m, 1));
	ASSERT_EQ(0, lin_publish(m, LIN_ID_DIAG_MASTER_REQ, d, 1, 0));

	ret = lin_sleep(m);
	EXPECT_EQ(-1, ret);
	EXPECT_EQ(EBUSY, errno);
	close(m);
}

TEST_F(lin_wake, wakeup_sleep_reject_nonzero_optlen)
{
	int fd = lin_open_bound(self->ifindex);
	int v = 1, ret;

	ASSERT_GE(fd, 0);

	/* Both are no-argument sockopts; a nonzero optlen is rejected. */
	ret = setsockopt(fd, SOL_LIN_RAW, LIN_RAW_WAKEUP, &v, sizeof(v));
	EXPECT_EQ(-1, ret);
	EXPECT_EQ(EINVAL, errno);

	ret = setsockopt(fd, SOL_LIN_RAW, LIN_RAW_SLEEP, &v, sizeof(v));
	EXPECT_EQ(-1, ret);
	EXPECT_EQ(EINVAL, errno);
	close(fd);
}

TEST_HARNESS_MAIN
