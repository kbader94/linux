// SPDX-License-Identifier: GPL-2.0
/*
 * PF_LIN selftests: socket creation, bind, rebind, and observer semantics.
 *
 * Basic socket()/bind() happy paths are exercised implicitly by every
 * other fixture's setup (lin_open_bound), so this file focuses on
 * behaviors not surfaced elsewhere: bind admission rejections, rebind
 * variants, observer-mode receive semantics, recvmsg truncation, and the
 * managed-vlin-only link-state corner cases.
 */
#include "lin_harness.h"

/* Start a looping uncond emitter for @id on @ifindex; returns master fd. */
static int sock_emitter(int ifindex, __u8 id)
{
	union lin_sched_buf buf = {};
	__u8 d[1] = { 0x5a };
	int m = lin_open_bound(ifindex);

	if (m < 0)
		return -1;
	if (lin_master(m, 1) || lin_publish(m, id, d, 1, 0)) {
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

FIXTURE(lin_sock) {
	int ifindex;
	__u32 caps;
};

FIXTURE_SETUP(lin_sock)
{
	self->ifindex = lin_setup_iface(LIN_IF);
	if (self->ifindex <= 0)
		SKIP(return, "need root and the vlin module (ip link add type vlin)");
	ASSERT_EQ(0, lin_query_caps(self->ifindex, &self->caps));
}

FIXTURE_TEARDOWN(lin_sock)
{
}

/* Bind admission rejections. */
TEST_F(lin_sock, bind_rejections)
{
	struct sockaddr_lin addr = {
		.lin_family = AF_INET,	/* wrong family */
		.lin_ifindex = self->ifindex,
	};
	int fd;
	int ret;

	/* Non-existent ifindex => ENODEV. */
	fd = lin_socket();
	ASSERT_GE(fd, 0);
	ret = lin_bind(fd, 0x7fffffff);
	EXPECT_EQ(-1, ret);
	EXPECT_EQ(ENODEV, errno);
	close(fd);

	/* Wrong address family => EINVAL. */
	fd = lin_socket();
	ASSERT_GE(fd, 0);
	ret = bind(fd, (struct sockaddr *)&addr, sizeof(addr));
	EXPECT_EQ(-1, ret);
	EXPECT_EQ(EINVAL, errno);
	close(fd);
}

/* A socket may bind to an interface that is already administratively down;
 * the bind succeeds and primes SO_ERROR with ENETDOWN so the application
 * observes the link state at bind time (the socket is a passive observer
 * until the interface comes up).
 */
TEST_F(lin_sock, bind_while_down)
{
	LIN_REQUIRE_MANAGED_IFACE();
	int fd = lin_socket();
	int soerr = 0;
	socklen_t l = sizeof(soerr);

	ASSERT_GE(fd, 0);
	ASSERT_EQ(0, lin_run("ip link set " LIN_IF " down"));

	EXPECT_EQ(0, lin_bind(fd, self->ifindex));
	ASSERT_EQ(0, getsockopt(fd, SOL_SOCKET, SO_ERROR, &soerr, &l));
	EXPECT_EQ(ENETDOWN, soerr);
	close(fd);
}

/* Rebind variants: observer -> concrete, and concrete -> observer.
 * getsockname must reflect the active binding after each transition.
 */
TEST_F(lin_sock, rebind_variants)
{
	struct sockaddr_lin addr = {};
	socklen_t len;
	int fd;

	/* observer -> concrete */
	fd = lin_socket();
	ASSERT_GE(fd, 0);
	ASSERT_EQ(0, lin_bind(fd, 0));
	EXPECT_EQ(0, lin_bind(fd, self->ifindex));
	len = sizeof(addr);
	ASSERT_EQ(0, getsockname(fd, (struct sockaddr *)&addr, &len));
	EXPECT_EQ(AF_LIN, addr.lin_family);
	EXPECT_EQ(self->ifindex, addr.lin_ifindex);
	close(fd);

	/* concrete -> observer */
	fd = lin_socket();
	ASSERT_GE(fd, 0);
	ASSERT_EQ(0, lin_bind(fd, self->ifindex));
	EXPECT_EQ(0, lin_bind(fd, 0));
	len = sizeof(addr);
	ASSERT_EQ(0, getsockname(fd, (struct sockaddr *)&addr, &len));
	EXPECT_EQ(AF_LIN, addr.lin_family);
	EXPECT_EQ(0, addr.lin_ifindex);
	close(fd);
}

/* An ifindex-0 observer receives traffic from every LIN interface in the
 * namespace, and recvmsg() reports the source interface.
 */
TEST_F(lin_sock, observer_receives_with_source_ifindex)
{
	int obs = lin_open_bound(0);
	struct lin_frame f;
	int src = -1, m;

	ASSERT_GE(obs, 0);
	m = sock_emitter(self->ifindex, 0x10);
	ASSERT_GE(m, 0);

	ASSERT_EQ(1, lin_recv_from(obs, &f, &src, LIN_RECV_MS));
	EXPECT_EQ(0x10, f.lin_id);
	EXPECT_EQ(self->ifindex, src);
	close(obs);
	close(m);
}

/* The ifindex-0 observer spans every LIN interface in the namespace: with two
 * vlin devices each emitting a distinct ID, the observer sees both and
 * recvmsg reports the correct source ifindex for each.
 */
TEST_F(lin_sock, observer_spans_multiple_ifaces)
{
	LIN_REQUIRE_MANAGED_IFACE();
	int obs = lin_open_bound(0);
	struct lin_frame f;
	int vlin1_idx, m0, m1, src;
	int saw0 = 0, saw1 = 0, i;

	ASSERT_GE(obs, 0);
	ASSERT_EQ(0, lin_run("ip link add vlin1 type vlin"));
	ASSERT_EQ(0, lin_run("ip link set vlin1 up"));
	vlin1_idx = if_nametoindex("vlin1");
	ASSERT_GT(vlin1_idx, 0);

	m0 = sock_emitter(self->ifindex, 0x10);	/* vlin0 emits 0x10 */
	ASSERT_GE(m0, 0);
	m1 = sock_emitter(vlin1_idx, 0x20);	/* vlin1 emits 0x20 */
	ASSERT_GE(m1, 0);

	/* Collect (bounded) until a frame from each interface has been seen. */
	for (i = 0; i < 32 && !(saw0 && saw1); i++) {
		src = -1;
		if (lin_recv_from(obs, &f, &src, LIN_RECV_MS) != 1)
			break;
		if (f.lin_id == 0x10 && src == self->ifindex)
			saw0 = 1;
		else if (f.lin_id == 0x20 && src == vlin1_idx)
			saw1 = 1;
	}
	EXPECT_TRUE(saw0);
	EXPECT_TRUE(saw1);
	close(obs);
	close(m0);
	close(m1);
}

/* Rebinding to a different interface releases the old interface's per-bus
 * policy (master claim, publisher ownership, schedules).
 */
TEST_F(lin_sock, rebind_away_drops_policy)
{
	LIN_REQUIRE_MANAGED_IFACE();
	struct sockaddr_lin addr = {};
	socklen_t len = sizeof(addr);
	union lin_sched_buf buf = {};
	int s = lin_open_bound(self->ifindex);
	int other = lin_open_bound(self->ifindex);
	__u8 d[1] = { 1 };
	int vlin1_idx, ret, h = -2;

	ASSERT_GE(s, 0);
	ASSERT_GE(other, 0);

	ASSERT_EQ(0, lin_run("ip link add vlin1 type vlin"));
	ASSERT_EQ(0, lin_run("ip link set vlin1 up"));
	vlin1_idx = if_nametoindex("vlin1");
	ASSERT_GT(vlin1_idx, 0);

	/* Claim master + publisher 0x10 and run a schedule on vlin0, so the
	 * rebind must tear down schedules too, not just master/publisher.
	 */
	ASSERT_EQ(0, lin_master(s, 1));
	ASSERT_EQ(0, lin_publish(s, 0x10, d, 1, 0));
	buf.s.entry_count = 1;
	buf.s.default_slot_us = LIN_SLOT_US;
	buf.s.entry[0].type = LIN_SCHED_TYPE_UNCOND;
	buf.s.entry[0].member_count = 1;
	buf.s.entry[0].members[0] = 0x10;
	ASSERT_EQ(0, lin_sched_load(s, &buf, 1));
	ASSERT_EQ(0, lin_sched_activate(s, 0));

	/* Held: another socket cannot take the master role. */
	ret = lin_master(other, 1);
	ASSERT_EQ(-1, ret);
	ASSERT_EQ(EBUSY, errno);

	/* Rebind s away to vlin1. */
	ASSERT_EQ(0, lin_bind(s, vlin1_idx));
	ASSERT_EQ(0, getsockname(s, (struct sockaddr *)&addr, &len));
	EXPECT_EQ(vlin1_idx, addr.lin_ifindex);

	/* The active schedule on vlin0 was torn down by the rebind. */
	ASSERT_EQ(0, lin_sched_active(other, &h));
	EXPECT_EQ(-1, h);

	/* vlin0's policy is free: master, publisher, and the handle reload +
	 * activate all succeed cleanly.
	 */
	EXPECT_EQ(0, lin_master(other, 1));
	EXPECT_EQ(0, lin_publish(other, 0x10, d, 1, 0));
	EXPECT_EQ(0, lin_sched_load(other, &buf, 1));
	EXPECT_EQ(0, lin_sched_activate(other, 0));
	close(s);
	close(other);
}

/* recvmsg() into a buffer shorter than a struct lin_frame returns the
 * truncated length and flags MSG_TRUNC.
 */
TEST_F(lin_sock, recvmsg_truncation)
{
	char small[8];
	struct iovec iov = { .iov_base = small, .iov_len = sizeof(small) };
	struct msghdr msg = { .msg_iov = &iov, .msg_iovlen = 1 };
	struct pollfd pfd;
	int s = lin_open_bound(self->ifindex);
	int m, ret;

	ASSERT_GE(s, 0);
	m = sock_emitter(self->ifindex, 0x10);
	ASSERT_GE(m, 0);

	pfd.fd = s;
	pfd.events = POLLIN;
	ASSERT_GT(poll(&pfd, 1, LIN_RECV_MS), 0);

	ret = recvmsg(s, &msg, 0);
	EXPECT_EQ(sizeof(small), ret);			/* truncated copy */
	EXPECT_NE(0, msg.msg_flags & MSG_TRUNC);
	close(s);
	close(m);
}

TEST_HARNESS_MAIN
