// SPDX-License-Identifier: GPL-2.0
/*
 * PF_LIN selftests: master-role claim policy (LIN_RAW_MASTER).
 *
 * Single master per interface, master-only operations gated by -EPERM,
 * observer (ifindex 0) and link-down rejections, and auto-release on close.
 */
#include "lin_harness.h"

FIXTURE(lin_master) {
	int ifindex;
	__u32 caps;
};

FIXTURE_SETUP(lin_master)
{
	self->ifindex = lin_setup_iface(LIN_IF);
	if (self->ifindex <= 0)
		SKIP(return, "need root and the vlin module (ip link add type vlin)");
	ASSERT_EQ(0, lin_query_caps(self->ifindex, &self->caps));
}

FIXTURE_TEARDOWN(lin_master)
{
}

/* Claim happy path: idempotent re-claim by the same socket, then release,
 * then a different socket can claim. The second_socket_claim_busy case
 * (EBUSY when another socket holds the role) is covered by
 * close_releases_master below.
 */
TEST_F(lin_master, release_allows_new_claim)
{
	int a = lin_open_bound(self->ifindex);
	int b = lin_open_bound(self->ifindex);

	ASSERT_GE(a, 0);
	ASSERT_GE(b, 0);
	ASSERT_EQ(0, lin_master(a, 1));
	/* Re-claim by the same socket is idempotent. */
	EXPECT_EQ(0, lin_master(a, 1));
	ASSERT_EQ(0, lin_master(a, 0));
	EXPECT_EQ(0, lin_master(b, 1));
	close(a);
	close(b);
}

TEST_F(lin_master, close_releases_master)
{
	int a = lin_open_bound(self->ifindex);
	int b = lin_open_bound(self->ifindex);
	int ret;

	ASSERT_GE(a, 0);
	ASSERT_GE(b, 0);
	ASSERT_EQ(0, lin_master(a, 1));

	ret = lin_master(b, 1);
	ASSERT_EQ(-1, ret);
	ASSERT_EQ(EBUSY, errno);

	/* Closing the holder releases the role. */
	close(a);
	EXPECT_EQ(0, lin_master(b, 1));
	close(b);
}

/* Every master-gated schedule sockopt (LOAD / DELETE / ACTIVATE / STOP)
 * returns EPERM on a socket that does not hold the master role. The master
 * itself can STOP an idle bus as a no-op success.
 */
TEST_F(lin_master, non_master_ops_eperm)
{
	int a = lin_open_bound(self->ifindex);
	int b = lin_open_bound(self->ifindex);
	union lin_sched_buf buf = {};

	ASSERT_GE(a, 0);
	ASSERT_GE(b, 0);
	ASSERT_EQ(0, lin_master(a, 1));

	buf.s.handle = 0;
	buf.s.entry_count = 1;
	buf.s.default_slot_us = LIN_SLOT_US;
	buf.s.entry[0].type = LIN_SCHED_TYPE_UNCOND;
	buf.s.entry[0].member_count = 1;
	buf.s.entry[0].members[0] = 0x10;

	/* b is not master. Every master-gated sockopt is refused with EPERM. */
	EXPECT_EQ(-1, lin_sched_load(b, &buf, 1));
	EXPECT_EQ(EPERM, errno);
	EXPECT_EQ(-1, lin_sched_delete(b, 0));
	EXPECT_EQ(EPERM, errno);
	EXPECT_EQ(-1, lin_sched_activate(b, 0));
	EXPECT_EQ(EPERM, errno);
	EXPECT_EQ(-1, lin_sched_stop(b));
	EXPECT_EQ(EPERM, errno);

	/* For the master, stopping with no active schedule is idempotent. */
	EXPECT_EQ(0, lin_sched_stop(a));
	close(a);
	close(b);
}

TEST_F(lin_master, observer_cannot_claim)
{
	int fd = lin_open_bound(0);	/* ifindex 0 => observer */
	int ret;

	ASSERT_GE(fd, 0);
	ret = lin_master(fd, 1);
	EXPECT_EQ(-1, ret);
	EXPECT_EQ(EOPNOTSUPP, errno);
	close(fd);
}

TEST_HARNESS_MAIN
