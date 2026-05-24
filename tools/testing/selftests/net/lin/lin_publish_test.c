// SPDX-License-Identifier: GPL-2.0
/*
 * PF_LIN selftests: publisher registry (LIN_RAW_PUBLISH / UNPUBLISH) and
 * the write()/sendmsg() response upsert.
 */
#include "lin_harness.h"

#include <linux/lin/error.h>

FIXTURE(lin_publish) {
	int ifindex;
	__u32 caps;
};

FIXTURE_SETUP(lin_publish)
{
	self->ifindex = lin_setup_iface(LIN_IF);
	if (self->ifindex <= 0)
		SKIP(return, "need root and the vlin module (ip link add type vlin)");
	ASSERT_EQ(0, lin_query_caps(self->ifindex, &self->caps));
	/* The whole suite uses LIN_RAW_PUBLISH from sockets that do NOT
	 * claim LIN_RAW_MASTER first — i.e. the slave-only publisher role.
	 * Drivers whose transport cannot meet the spec's header-RX →
	 * response-TX window do not advertise LIN_CAP_PUB_SLAVE and reject
	 * this path with -EOPNOTSUPP from the core. Skip the suite cleanly
	 * on those drivers; the master-with-publish path is exercised
	 * elsewhere.
	 */
	LIN_SKIP_UNLESS_CAP(self->caps, LIN_CAP_PUB_SLAVE);
}

FIXTURE_TEARDOWN(lin_publish)
{
}

TEST_F(lin_publish, unpublish_negative)
{
	__u8 d[1] = { 1 };
	int a = lin_open_bound(self->ifindex);
	int b = lin_open_bound(self->ifindex);
	int obs = lin_open_bound(0);
	int ret;

	ASSERT_GE(a, 0);
	ASSERT_GE(b, 0);
	ASSERT_GE(obs, 0);
	ASSERT_EQ(0, lin_publish(a, 0x10, d, 1, 0));

	/* Not the owner. */
	ret = lin_unpublish(b, 0x10);
	EXPECT_EQ(-1, ret);
	EXPECT_EQ(ENOENT, errno);

	/* Nothing registered at this ID. */
	ret = lin_unpublish(a, 0x20);
	EXPECT_EQ(-1, ret);
	EXPECT_EQ(ENOENT, errno);

	/* ID out of range. */
	ret = lin_unpublish(a, 0x40);
	EXPECT_EQ(-1, ret);
	EXPECT_EQ(EINVAL, errno);

	/* Observer has no bound device. */
	ret = lin_unpublish(obs, 0x10);
	EXPECT_EQ(-1, ret);
	EXPECT_EQ(EOPNOTSUPP, errno);

	/* Interface down (managed mode only — mutating link state on an
	 * external hardware interface would be unsafe).
	 */
	if (lin_iface_is_managed()) {
		ASSERT_EQ(0, lin_run("ip link set " LIN_IF " down"));
		ret = lin_unpublish(a, 0x10);
		EXPECT_EQ(-1, ret);
		EXPECT_EQ(ENETDOWN, errno);
	}
	close(a);
	close(b);
	close(obs);
}

TEST_F(lin_publish, second_owner_busy)
{
	__u8 data[1] = { 0x11 };
	int a = lin_open_bound(self->ifindex);
	int b = lin_open_bound(self->ifindex);
	int ret;

	ASSERT_GE(a, 0);
	ASSERT_GE(b, 0);
	ASSERT_EQ(0, lin_publish(a, 0x10, data, 1, 0));

	ret = lin_publish(b, 0x10, data, 1, 0);
	EXPECT_EQ(-1, ret);
	EXPECT_EQ(EBUSY, errno);
	close(a);
	close(b);
}

/* The data-plane upsert (write()/sendmsg()) reaches the same publisher
 * ownership check as LIN_RAW_PUBLISH, so a write() for an ID owned by another
 * socket is refused with EBUSY rather than stealing the entry.
 */
TEST_F(lin_publish, write_second_owner_busy)
{
	__u8 data[1] = { 0x11 };
	int a = lin_open_bound(self->ifindex);
	int b = lin_open_bound(self->ifindex);
	ssize_t ret;

	ASSERT_GE(a, 0);
	ASSERT_GE(b, 0);
	ASSERT_EQ(0, lin_publish(a, 0x10, data, 1, 0));

	ret = lin_write(b, 0x10, data, 1, 0);
	EXPECT_EQ(-1, ret);
	EXPECT_EQ(EBUSY, errno);
	close(a);
	close(b);
}

TEST_F(lin_publish, same_owner_update)
{
	__u8 a[1] = { 1 }, b[3] = { 1, 2, 3 };
	int fd = lin_open_bound(self->ifindex);

	ASSERT_GE(fd, 0);
	EXPECT_EQ(0, lin_publish(fd, 0x10, a, 1, 0));
	EXPECT_EQ(0, lin_publish(fd, 0x10, b, 3, 0));	/* update is fine */
	close(fd);
}

TEST_F(lin_publish, close_releases_owner)
{
	__u8 data[1] = { 0x11 };
	int a = lin_open_bound(self->ifindex);
	int b = lin_open_bound(self->ifindex);
	int ret;

	ASSERT_GE(a, 0);
	ASSERT_GE(b, 0);
	ASSERT_EQ(0, lin_publish(a, 0x10, data, 1, 0));
	ret = lin_publish(b, 0x10, data, 1, 0);
	ASSERT_EQ(-1, ret);
	ASSERT_EQ(EBUSY, errno);

	close(a);				/* releases the ID */
	EXPECT_EQ(0, lin_publish(b, 0x10, data, 1, 0));
	close(b);
}

/* LIN_RAW_PUBLISH admission rejections: one representative per parser class
 * (bad ID, bad length, bad flag, reserved bytes).
 */
TEST_F(lin_publish, publish_admission_rejections)
{
	int fd = lin_open_bound(self->ifindex);
	struct lin_publish pub;

	ASSERT_GE(fd, 0);

	/* ID above the 6-bit range. */
	memset(&pub, 0, sizeof(pub));
	pub.lin_id = 0x40;
	pub.len = 1;
	EXPECT_EQ(-1, lin_setopt(fd, LIN_RAW_PUBLISH, &pub, sizeof(pub)));
	EXPECT_EQ(EINVAL, errno);

	/* Over-long payload. */
	memset(&pub, 0, sizeof(pub));
	pub.lin_id = 0x10;
	pub.len = LIN_MAX_DLEN + 1;
	EXPECT_EQ(-1, lin_setopt(fd, LIN_RAW_PUBLISH, &pub, sizeof(pub)));
	EXPECT_EQ(EINVAL, errno);

	/* Unknown flag bit (only LIN_F_CHK_ENH is valid here). */
	memset(&pub, 0, sizeof(pub));
	pub.lin_id = 0x10;
	pub.len = 1;
	pub.flags = LIN_F_WAKEUP;
	EXPECT_EQ(-1, lin_setopt(fd, LIN_RAW_PUBLISH, &pub, sizeof(pub)));
	EXPECT_EQ(EINVAL, errno);

	/* Nonzero reserved bytes. */
	memset(&pub, 0, sizeof(pub));
	pub.lin_id = 0x10;
	pub.len = 1;
	pub.__res[0] = 1;
	EXPECT_EQ(-1, lin_setopt(fd, LIN_RAW_PUBLISH, &pub, sizeof(pub)));
	EXPECT_EQ(EINVAL, errno);
	close(fd);
}

TEST_F(lin_publish, diag_enhanced_checksum_rejected)
{
	LIN_SKIP_UNLESS_CAP(self->caps, LIN_CAP_DIAG);
	__u8 data[1] = { 1 };
	int fd = lin_open_bound(self->ifindex);
	int ret;

	ASSERT_GE(fd, 0);
	/* vlin advertises LIN_CAP_DIAG, so classic-checksum 0x3C is fine ... */
	EXPECT_EQ(0, lin_publish(fd, LIN_ID_DIAG_MASTER_REQ, data, 1, 0));
	/* ... but enhanced checksum on a diagnostic ID is rejected. */
	ret = lin_publish(fd, LIN_ID_DIAG_MASTER_REQ, data, 1, 1);
	EXPECT_EQ(-1, ret);
	EXPECT_EQ(EINVAL, errno);
	close(fd);
}

TEST_F(lin_publish, observer_cannot_publish)
{
	__u8 data[1] = { 1 };
	int fd = lin_open_bound(0);	/* observer */
	int ret;

	ASSERT_GE(fd, 0);
	ret = lin_publish(fd, 0x10, data, 1, 0);
	EXPECT_EQ(-1, ret);
	EXPECT_EQ(EOPNOTSUPP, errno);
	close(fd);
}

TEST_F(lin_publish, publish_when_down_netdown)
{
	LIN_REQUIRE_MANAGED_IFACE();
	__u8 data[1] = { 1 };
	int fd = lin_open_bound(self->ifindex);
	int ret;

	ASSERT_GE(fd, 0);
	ASSERT_EQ(0, lin_run("ip link set " LIN_IF " down"));
	ret = lin_publish(fd, 0x10, data, 1, 0);
	EXPECT_EQ(-1, ret);
	EXPECT_EQ(ENETDOWN, errno);
	close(fd);
}

/* write()/sendmsg() upserts the publisher response; the scheduled emission
 * carries whatever was last written.
 */
TEST_F(lin_publish, write_upsert_reflected)
{
	union lin_sched_buf buf = {};
	__u8 d1[2] = { 0x01, 0x02 };
	__u8 d2[2] = { 0xde, 0xad };
	int m = lin_open_bound(self->ifindex);
	int s = lin_open_bound(self->ifindex);
	struct lin_frame f;
	int i;

	ASSERT_GE(m, 0);
	ASSERT_GE(s, 0);
	ASSERT_EQ(0, lin_master(m, 1));
	ASSERT_EQ(sizeof(struct lin_frame), lin_write(m, 0x10, d1, 2, 0));

	buf.s.entry_count = 1;
	buf.s.default_slot_us = LIN_SLOT_US;
	buf.s.entry[0].type = LIN_SCHED_TYPE_UNCOND;
	buf.s.entry[0].member_count = 1;
	buf.s.entry[0].members[0] = 0x10;
	ASSERT_EQ(0, lin_sched_load(m, &buf, 1));
	ASSERT_EQ(0, lin_sched_activate(m, 0));

	ASSERT_EQ(1, lin_recv(s, &f, LIN_RECV_MS));
	EXPECT_EQ(0x10, f.lin_id);
	EXPECT_EQ(2, f.len);
	EXPECT_EQ(0x01, f.data[0]);

	/* Upsert new data; a later emission reflects it. */
	ASSERT_EQ(sizeof(struct lin_frame), lin_write(m, 0x10, d2, 2, 0));
	for (i = 0; i < 20; i++) {
		ASSERT_EQ(1, lin_recv(s, &f, LIN_RECV_MS));
		if (f.data[0] == 0xde)
			break;
	}
	EXPECT_EQ(0xde, f.data[0]);
	EXPECT_EQ(0xad, f.data[1]);
	close(m);
	close(s);
}

/* sendmsg/write malformed-frame rejections: one representative per parser
 * class (short message, bad ID, bad length, bad flag, reserved bytes).
 */
TEST_F(lin_publish, sendmsg_rejects_malformed)
{
	int fd = lin_open_bound(self->ifindex);
	struct lin_frame f;

	ASSERT_GE(fd, 0);

	/* Short frame. */
	memset(&f, 0, sizeof(f));
	f.lin_id = 0x10;
	f.len = 1;
	EXPECT_EQ(-1, write(fd, &f, sizeof(f) - 1));
	EXPECT_EQ(EINVAL, errno);

	/* ID above the 6-bit range. */
	memset(&f, 0, sizeof(f));
	f.lin_id = 0x40;
	f.len = 1;
	EXPECT_EQ(-1, write(fd, &f, sizeof(f)));
	EXPECT_EQ(EINVAL, errno);

	/* Out-of-range length. */
	memset(&f, 0, sizeof(f));
	f.lin_id = 0x10;
	f.len = LIN_MAX_DLEN + 1;
	EXPECT_EQ(-1, write(fd, &f, sizeof(f)));
	EXPECT_EQ(EINVAL, errno);

	/* Unknown / non-sendable flag bit (only LIN_F_CHK_ENH is valid here). */
	memset(&f, 0, sizeof(f));
	f.lin_id = 0x10;
	f.len = 1;
	f.flags = LIN_F_ERR;
	EXPECT_EQ(-1, write(fd, &f, sizeof(f)));
	EXPECT_EQ(EINVAL, errno);

	/* Nonzero reserved bytes. */
	memset(&f, 0, sizeof(f));
	f.lin_id = 0x10;
	f.len = 1;
	f.__res[0] = 1;
	EXPECT_EQ(-1, write(fd, &f, sizeof(f)));
	EXPECT_EQ(EINVAL, errno);
	close(fd);
}

TEST_F(lin_publish, sendmsg_bad_msg_name)
{
	struct lin_frame f = { .lin_id = 0x10, .len = 1, .data = { 1 } };
	struct iovec iov = { .iov_base = &f, .iov_len = sizeof(f) };
	struct msghdr msg = { .msg_iov = &iov, .msg_iovlen = 1 };
	struct sockaddr_lin sa;
	int fd = lin_open_bound(self->ifindex);
	int ret;

	ASSERT_GE(fd, 0);

	/* Wrong address family. */
	memset(&sa, 0, sizeof(sa));
	sa.lin_family = AF_INET;
	sa.lin_ifindex = self->ifindex;
	msg.msg_name = &sa;
	msg.msg_namelen = sizeof(sa);
	ret = sendmsg(fd, &msg, 0);
	EXPECT_EQ(-1, ret);
	EXPECT_EQ(EINVAL, errno);

	/* Too-short msg_namelen. */
	memset(&sa, 0, sizeof(sa));
	sa.lin_family = AF_LIN;
	sa.lin_ifindex = self->ifindex;
	msg.msg_name = &sa;
	msg.msg_namelen = 1;
	ret = sendmsg(fd, &msg, 0);
	EXPECT_EQ(-1, ret);
	EXPECT_EQ(EINVAL, errno);

	/* Mismatched ifindex: LIN does not support sendto() retargeting. */
	memset(&sa, 0, sizeof(sa));
	sa.lin_family = AF_LIN;
	sa.lin_ifindex = self->ifindex + 1000;
	msg.msg_name = &sa;
	msg.msg_namelen = sizeof(sa);
	ret = sendmsg(fd, &msg, 0);
	EXPECT_EQ(-1, ret);
	EXPECT_EQ(EINVAL, errno);
	close(fd);
}

/* A valid non-NULL sockaddr_lin (AF_LIN + the bound ifindex) is accepted:
 * sendmsg() upserts the publisher just like the no-name write() path,
 * exercising the address-validation success branch.
 */
TEST_F(lin_publish, sendmsg_valid_msg_name)
{
	struct lin_frame f = { .lin_id = 0x10, .len = 1, .data = { 0x5a } };
	struct iovec iov = { .iov_base = &f, .iov_len = sizeof(f) };
	struct msghdr msg = { .msg_iov = &iov, .msg_iovlen = 1 };
	struct sockaddr_lin sa;
	int fd = lin_open_bound(self->ifindex);
	ssize_t ret;

	ASSERT_GE(fd, 0);

	memset(&sa, 0, sizeof(sa));
	sa.lin_family = AF_LIN;
	sa.lin_ifindex = self->ifindex;
	msg.msg_name = &sa;
	msg.msg_namelen = sizeof(sa);

	ret = sendmsg(fd, &msg, 0);
	EXPECT_EQ(sizeof(struct lin_frame), ret);

	/* The upsert took effect: fd now owns the 0x10 entry and can release it. */
	EXPECT_EQ(0, lin_unpublish(fd, 0x10));
	close(fd);
}

TEST_F(lin_publish, sendmsg_observer_unsupported)
{
	struct lin_frame f = { .lin_id = 0x10, .len = 1, .data = { 1 } };
	int fd = lin_open_bound(0);	/* observer: no bound device */
	int ret;

	ASSERT_GE(fd, 0);
	ret = write(fd, &f, sizeof(f));
	EXPECT_EQ(-1, ret);
	EXPECT_EQ(EOPNOTSUPP, errno);
	close(fd);
}

TEST_F(lin_publish, sendmsg_when_down_netdown)
{
	LIN_REQUIRE_MANAGED_IFACE();
	struct lin_frame f = { .lin_id = 0x10, .len = 1, .data = { 1 } };
	int fd = lin_open_bound(self->ifindex);
	int ret;

	ASSERT_GE(fd, 0);
	ASSERT_EQ(0, lin_run("ip link set " LIN_IF " down"));
	ret = write(fd, &f, sizeof(f));
	EXPECT_EQ(-1, ret);
	EXPECT_EQ(ENETDOWN, errno);
	close(fd);
}

/* Releasing a publisher must clear the driver's response table, so the slot
 * stops delivering the stale bytes and falls back to NO_RESPONSE.
 */
TEST_F(lin_publish, unpublish_stops_response)
{
	union lin_sched_buf buf = {};
	__u8 d[2] = { 0xc0, 0xde };
	int m = lin_open_bound(self->ifindex);
	int p = lin_open_bound(self->ifindex);
	int s = lin_open_bound(self->ifindex);
	struct lin_frame f;
	int i, saw_data = 0, saw_noresp = 0;

	ASSERT_GE(m, 0);
	ASSERT_GE(p, 0);
	ASSERT_GE(s, 0);
	ASSERT_EQ(0, lin_master(m, 1));
	ASSERT_EQ(0, lin_publish(p, 0x10, d, 2, 0));	/* p owns 0x10 */
	ASSERT_EQ(0, lin_err_filter(s, LIN_ERR_NO_RESPONSE));

	buf.s.entry_count = 1;
	buf.s.default_slot_us = LIN_SLOT_US;
	buf.s.entry[0].type = LIN_SCHED_TYPE_UNCOND;
	buf.s.entry[0].member_count = 1;
	buf.s.entry[0].members[0] = 0x10;
	ASSERT_EQ(0, lin_sched_load(m, &buf, 1));
	ASSERT_EQ(0, lin_sched_activate(m, 0));

	/* Publisher present: the response is delivered. */
	for (i = 0; i < 16; i++) {
		if (lin_recv(s, &f, LIN_RECV_MS) != 1)
			break;
		if (f.lin_id == 0x10 && !(f.flags & LIN_F_ERR)) {
			saw_data = 1;
			break;
		}
	}
	ASSERT_TRUE(saw_data);

	/* Release it; later polls must yield NO_RESPONSE (skip frames already
	 * queued before the release took effect).
	 */
	ASSERT_EQ(0, lin_unpublish(p, 0x10));
	for (i = 0; i < 40; i++) {
		if (lin_recv(s, &f, LIN_RECV_MS) != 1)
			break;
		if (f.lin_id == 0x10 && (f.flags & LIN_F_ERR) &&
		    (f.err_mask & LIN_ERR_NO_RESPONSE)) {
			saw_noresp = 1;
			break;
		}
	}
	ASSERT_TRUE(saw_noresp);

	/* The stale response stays gone: only NO_RESPONSE from here on. */
	for (i = 0; i < 8; i++) {
		ASSERT_EQ(1, lin_recv(s, &f, LIN_RECV_MS));
		EXPECT_EQ(0x10, f.lin_id);
		EXPECT_NE(0, f.flags & LIN_F_ERR);
	}
	close(m);
	close(p);
	close(s);
}

TEST_HARNESS_MAIN
