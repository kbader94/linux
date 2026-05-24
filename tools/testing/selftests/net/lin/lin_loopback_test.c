// SPDX-License-Identifier: GPL-2.0
/*
 * PF_LIN selftests: loopback model (LIN_RAW_LOOPBACK / RECV_OWN_MSGS).
 *
 * Like SocketCAN: emissions are visible to other sockets by default, an
 * owner does not receive its own unless RECV_OWN_MSGS is set, and an owner
 * that disables LOOPBACK suppresses local delivery of its emissions.
 */
#include "lin_harness.h"

/* @m runs a single-slot schedule for @id; @id's publisher is @pub_fd. */
static int run_uncond(int m, int pub_fd, __u8 id, const __u8 *d, __u8 len)
{
	union lin_sched_buf buf = {};
	int ret;

	ret = lin_publish(pub_fd, id, d, len, 0);
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

FIXTURE(lin_lb) {
	int ifindex;
	__u32 caps;
};

FIXTURE_SETUP(lin_lb)
{
	self->ifindex = lin_setup_iface(LIN_IF);
	if (self->ifindex <= 0)
		SKIP(return, "need root and the vlin module (ip link add type vlin)");
	ASSERT_EQ(0, lin_query_caps(self->ifindex, &self->caps));
}

FIXTURE_TEARDOWN(lin_lb)
{
}

/* RECV_OWN_MSGS toggle: off by default (owner silent, bystander receives),
 * on (owner sees its own emissions too).
 */
TEST_F(lin_lb, recv_own_toggle)
{
	__u8 d[1] = { 0x42 };
	int m = lin_open_bound(self->ifindex);
	int s = lin_open_bound(self->ifindex);
	struct lin_frame f;

	ASSERT_GE(m, 0);
	ASSERT_GE(s, 0);
	ASSERT_EQ(0, lin_master(m, 1));
	ASSERT_EQ(0, run_uncond(m, m, 0x10, d, 1));

	/* Default: owner does not see its own; bystander does. */
	EXPECT_TRUE(lin_silent(m, LIN_SILENCE_MS));
	ASSERT_EQ(1, lin_recv(s, &f, LIN_RECV_MS));
	EXPECT_EQ(0x10, f.lin_id);

	/* RECV_OWN_MSGS=1: owner sees its own subsequent emissions. */
	ASSERT_EQ(0, lin_setopt_int(m, LIN_RAW_RECV_OWN_MSGS, 1));
	ASSERT_EQ(0, lin_sched_stop(m));
	ASSERT_EQ(0, lin_sched_activate(m, 0));
	ASSERT_EQ(1, lin_recv(m, &f, LIN_RECV_MS));
	EXPECT_EQ(0x10, f.lin_id);
	close(m);
	close(s);
}

TEST_F(lin_lb, loopback_off_suppresses_locally)
{
	__u8 d[1] = { 0x42 };
	int m = lin_open_bound(self->ifindex);
	int s = lin_open_bound(self->ifindex);

	ASSERT_GE(m, 0);
	ASSERT_GE(s, 0);
	ASSERT_EQ(0, lin_master(m, 1));
	/* m is the sole owner (master + publisher); with its loopback off,
	 * the frame is not synthesised for any local socket.
	 */
	ASSERT_EQ(0, lin_setopt_int(m, LIN_RAW_LOOPBACK, 0));
	ASSERT_EQ(0, run_uncond(m, m, 0x10, d, 1));

	EXPECT_TRUE(lin_silent(s, LIN_SILENCE_MS));
	close(m);
	close(s);
}

TEST_F(lin_lb, split_master_and_publisher_owners)
{
	__u8 d[1] = { 0x42 };
	int m = lin_open_bound(self->ifindex);	/* master only */
	int p = lin_open_bound(self->ifindex);	/* publisher only */
	int s = lin_open_bound(self->ifindex);	/* subscriber */
	struct lin_frame f;

	ASSERT_GE(m, 0);
	ASSERT_GE(p, 0);
	ASSERT_GE(s, 0);
	ASSERT_EQ(0, lin_master(m, 1));
	ASSERT_EQ(0, lin_setopt_int(p, LIN_RAW_RECV_OWN_MSGS, 1));

	/* m schedules 0x10, but p owns the response. */
	ASSERT_EQ(0, run_uncond(m, p, 0x10, d, 1));

	/* Publisher owner (recv-own on) receives. */
	ASSERT_EQ(1, lin_recv(p, &f, LIN_RECV_MS));
	EXPECT_EQ(0x10, f.lin_id);
	/* Bystander receives. */
	EXPECT_EQ(1, lin_recv(s, &f, LIN_RECV_MS));
	/* Master owner (recv-own off) does not. */
	EXPECT_TRUE(lin_silent(m, LIN_SILENCE_MS));
	close(m);
	close(p);
	close(s);
}

/* Symmetric to the event suite's master-loopback-off case: the loopback gate
 * is an OR over stakeholders, so with the master's loopback on but the
 * publisher's off, the master's vote alone keeps the synthesised frame alive
 * and a bystander still receives the emission.
 */
TEST_F(lin_lb, publisher_loopback_off_master_on_delivers)
{
	__u8 d[1] = { 0x42 };
	int m = lin_open_bound(self->ifindex);	/* master only, loopback on */
	int p = lin_open_bound(self->ifindex);	/* publisher only, loopback off */
	int s = lin_open_bound(self->ifindex);	/* bystander */
	struct lin_frame f;

	ASSERT_GE(m, 0);
	ASSERT_GE(p, 0);
	ASSERT_GE(s, 0);
	ASSERT_EQ(0, lin_master(m, 1));
	ASSERT_EQ(0, lin_setopt_int(p, LIN_RAW_LOOPBACK, 0));

	/* m schedules 0x10; p owns the response with its loopback disabled. */
	ASSERT_EQ(0, run_uncond(m, p, 0x10, d, 1));

	/* The publisher's lone "no" vote does not suppress the synth — the
	 * master's "yes" carries it, so the bystander receives.
	 */
	ASSERT_EQ(1, lin_recv(s, &f, LIN_RECV_MS));
	EXPECT_EQ(0x10, f.lin_id);
	close(m);
	close(p);
	close(s);
}

TEST_HARNESS_MAIN
