// SPDX-License-Identifier: GPL-2.0
/*
 * PF_LIN selftests: interface statistics (sysfs).
 *
 * Driver-agnostic: every conforming LIN driver counts its emissions in
 * tx_packets / tx_bytes. vlin's "is the whole bus" symmetric accounting
 * (rx == tx for self-emission) is a vlin implementation detail rather
 * than a PF_LIN contract, so it is not exercised here.
 */
#include "lin_harness.h"

static int run_uncond(int m, __u8 id, __u8 len)
{
	union lin_sched_buf buf = {};
	__u8 d[LIN_MAX_DLEN] = { 0x11, 0x22, 0x33, 0x44 };
	int ret;

	ret = lin_publish(m, id, d, len, 0);
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

FIXTURE(lin_stats) {
	int ifindex;
	__u32 caps;
};

FIXTURE_SETUP(lin_stats)
{
	self->ifindex = lin_setup_iface(LIN_IF);
	if (self->ifindex <= 0)
		SKIP(return, "need root and the vlin module (ip link add type vlin)");
	ASSERT_EQ(0, lin_query_caps(self->ifindex, &self->caps));
}

FIXTURE_TEARDOWN(lin_stats)
{
}

/* Driver-agnostic: every conforming LIN driver counts its emissions in
 * tx_packets / tx_bytes. Verify a baseline-then-emit delta strictly
 * advances both counters, without asserting any particular per-frame
 * byte count (real drivers and vlin may differ on what they include).
 */
TEST_F(lin_stats, tx_counters_advance)
{
	const char *ifn = lin_active_ifname();
	long long txp0, txb0, txp1, txb1;
	int m = lin_open_bound(self->ifindex);
	int s = lin_open_bound(self->ifindex);
	struct lin_frame f;
	int i;

	ASSERT_GE(m, 0);
	ASSERT_GE(s, 0);
	ASSERT_EQ(0, lin_master(m, 1));

	txp0 = lin_stat(ifn, "tx_packets");
	txb0 = lin_stat(ifn, "tx_bytes");

	ASSERT_EQ(0, run_uncond(m, 0x10, 4));
	for (i = 0; i < 3; i++)
		ASSERT_EQ(1, lin_recv(s, &f, LIN_RECV_MS));
	ASSERT_EQ(0, lin_sched_stop(m));

	txp1 = lin_stat(ifn, "tx_packets");
	txb1 = lin_stat(ifn, "tx_bytes");

	EXPECT_GT(txp1, txp0);
	EXPECT_GT(txb1, txb0);
	close(m);
	close(s);
}

TEST_HARNESS_MAIN
