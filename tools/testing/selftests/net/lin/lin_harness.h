/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Shared helpers for the PF_LIN selftests.
 *
 * Two run modes, distinguished by the LIN_IFNAME environment variable
 * (matching SocketCAN's CANIF convention):
 *
 *   * Managed mode (default, LIN_IFNAME unset). Each test forks into its
 *     own process and unshare(CLONE_NEWNET) gives it a private netns, in
 *     which lin_setup_iface() creates a fresh vlin interface. Teardown is
 *     automatic when the child exits. This is the CI default.
 *
 *   * External mode (LIN_IFNAME set, e.g. LIN_IFNAME=lin0). The harness
 *     uses the named pre-existing interface as-is — no unshare, no
 *     ip-link-add, no link-state mutation. Drop-in for hardware bring-up
 *     against any conforming LIN driver.
 *
 * Tests need root (CAP_NET_ADMIN); lin_setup_iface() returns -1 otherwise
 * and fixtures SKIP.
 *
 * How to tell whether a test is driver-agnostic
 * ---------------------------------------------
 * Each test self-documents via its gates at the top of the body:
 *
 *   * No gate           - driver-agnostic baseline. Every conforming
 *                         LIN driver passes; the test runs in both modes.
 *
 *   * LIN_SKIP_UNLESS_CAP(self->caps, LIN_CAP_<X>)
 *                       - driver-agnostic for drivers advertising
 *                         LIN_CAP_<X>; skipped cleanly otherwise.
 *
 *   * LIN_REQUIRE_MANAGED_IFACE()
 *                       - vlin-only (managed mode). The test mutates
 *                         link state (ip-link-set-down/up/delete) or
 *                         creates a sibling vlin, both of which would
 *                         be unsafe on a real hardware interface.
 *
 * Fixture-level gates apply the same to every test in that fixture.
 */
#ifndef LIN_HARNESS_H
#define LIN_HARNESS_H

#include <errno.h>
#include <sched.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <net/if.h>
#include <poll.h>
#include <sys/socket.h>
#include <sys/uio.h>
#include <sys/wait.h>

#include <linux/if_link.h>
#include <linux/lin.h>
#include <linux/lin/netlink.h>
#include <linux/lin/raw.h>
#include <linux/netlink.h>
#include <linux/rtnetlink.h>

#include "../../kselftest_harness.h"

/* AF_LIN / PF_LIN are new families that predate glibc's <sys/socket.h>
 * knowing about them; define them here if the C library headers do not.
 */
#ifndef AF_LIN
#define AF_LIN		46
#endif
#ifndef PF_LIN
#define PF_LIN		AF_LIN
#endif

#define LIN_IF		"vlin0"

/* A short slot duration keeps scheduled tests responsive without being so
 * small it rounds to a single jiffy on low-HZ kernels. recv timeouts are
 * comfortably larger so a slot reliably fires within the wait.
 */
#define LIN_SLOT_US	20000		/* 20 ms */
#define LIN_RECV_MS	500		/* wait up to 500 ms for a frame */
#define LIN_SILENCE_MS	200		/* "no frame" observation window */

/* Stack-friendly backing for a flexible struct lin_schedule. Tests fill
 * buf.s.entry[0..n-1] and load lin_sched_bytes(n) bytes.
 */
#define LIN_T_ENTRIES	8
union lin_sched_buf {
	struct lin_schedule s;
	char _pad[sizeof(struct lin_schedule) +
		  LIN_T_ENTRIES * sizeof(struct lin_schedule_entry)];
};

static inline size_t lin_sched_bytes(unsigned int n)
{
	return sizeof(struct lin_schedule) +
	       n * sizeof(struct lin_schedule_entry);
}

/* Run a shell command; return 0 only if it exited 0. */
static inline int lin_run(const char *cmd)
{
	int ret = system(cmd);

	return (ret != -1 && WIFEXITED(ret) && WEXITSTATUS(ret) == 0) ? 0 : -1;
}

/* True iff the suite is in "managed vlin" mode: no LIN_IFNAME env override,
 * so each test gets its own freshly-created vlin in a private netns. In
 * external mode (LIN_IFNAME set) the suite uses the named interface as-is
 * and must not manipulate its link state — tests that ip-link-set/del or
 * create a sibling vlin should call LIN_REQUIRE_MANAGED_IFACE() and SKIP.
 */
static inline int lin_iface_is_managed(void)
{
	return getenv("LIN_IFNAME") == NULL;
}

/* The interface name actually in use: LIN_IFNAME env override if set,
 * else the managed-vlin default. Tests that look up runtime state by
 * name (sysfs, ip command targets) should route through this rather than
 * hard-coding LIN_IF.
 */
static inline const char *lin_active_ifname(void)
{
	const char *n = getenv("LIN_IFNAME");

	return n ? n : LIN_IF;
}

#define LIN_REQUIRE_MANAGED_IFACE() do { \
	if (!lin_iface_is_managed()) \
		SKIP(return, "needs a managed vlin (unset LIN_IFNAME)"); \
} while (0)

/* Set up the LIN interface this test will use. In managed mode (default)
 * unshare a private netns and create a fresh vlin under @default_name; in
 * external mode (LIN_IFNAME env var set) use the named pre-existing
 * interface as-is — the caller is responsible for bringing it up. Returns
 * the ifindex, or -1 if not permitted / interface unavailable (caller
 * should SKIP).
 */
static inline int lin_setup_iface(const char *default_name)
{
	const char *name = getenv("LIN_IFNAME");
	char cmd[128];

	if (geteuid() != 0)
		return -1;

	if (name)
		return if_nametoindex(name);

	if (unshare(CLONE_NEWNET) != 0)
		return -1;
	if (lin_run("ip link set lo up") != 0)
		return -1;
	snprintf(cmd, sizeof(cmd), "ip link add %s type vlin", default_name);
	if (lin_run(cmd) != 0)
		return -1;
	snprintf(cmd, sizeof(cmd), "ip link set %s up", default_name);
	if (lin_run(cmd) != 0)
		return -1;
	return if_nametoindex(default_name);
}

/* Query the LIN_CAP_* bitmask the driver bound to @ifindex advertises via
 * IFLA_LIN_CAPS. Returns 0 on success (caps populated). Returns -1 with
 * errno set on netlink failure, or with errno == ENOMSG if the attribute
 * is absent — that case is a driver / kernel bug, since every conforming
 * LIN driver must advertise it; callers should hard-fail rather than mask
 * the bug by skipping silently.
 */
static inline int lin_query_caps(int ifindex, __u32 *caps)
{
	struct { struct nlmsghdr nh; struct ifinfomsg ifi; } req = {};
	char resp[4096];
	struct sockaddr_nl addr = { .nl_family = AF_NETLINK };
	struct nlmsghdr *nh;
	struct rtattr *rta, *linkinfo, *infodata, *cap;
	int sock, ret = -1, attrlen;
	ssize_t len;

	sock = socket(AF_NETLINK, SOCK_RAW, NETLINK_ROUTE);
	if (sock < 0)
		return -1;

	req.nh.nlmsg_len = NLMSG_LENGTH(sizeof(req.ifi));
	req.nh.nlmsg_type = RTM_GETLINK;
	req.nh.nlmsg_flags = NLM_F_REQUEST;
	req.ifi.ifi_family = AF_UNSPEC;
	req.ifi.ifi_index = ifindex;
	if (sendto(sock, &req, req.nh.nlmsg_len, 0,
		   (struct sockaddr *)&addr, sizeof(addr)) < 0)
		goto out;

	len = recv(sock, resp, sizeof(resp), 0);
	if (len < 0)
		goto out;

	for (nh = (struct nlmsghdr *)resp; NLMSG_OK(nh, len);
	     nh = NLMSG_NEXT(nh, len)) {
		if (nh->nlmsg_type != RTM_NEWLINK)
			continue;
		rta = IFLA_RTA(NLMSG_DATA(nh));
		attrlen = IFLA_PAYLOAD(nh);
		linkinfo = NULL;
		for (; RTA_OK(rta, attrlen); rta = RTA_NEXT(rta, attrlen))
			if (rta->rta_type == IFLA_LINKINFO)
				linkinfo = rta;
		if (!linkinfo)
			continue;

		rta = (struct rtattr *)RTA_DATA(linkinfo);
		attrlen = RTA_PAYLOAD(linkinfo);
		infodata = NULL;
		for (; RTA_OK(rta, attrlen); rta = RTA_NEXT(rta, attrlen))
			if (rta->rta_type == IFLA_INFO_DATA)
				infodata = rta;
		if (!infodata)
			continue;

		rta = (struct rtattr *)RTA_DATA(infodata);
		attrlen = RTA_PAYLOAD(infodata);
		cap = NULL;
		for (; RTA_OK(rta, attrlen); rta = RTA_NEXT(rta, attrlen))
			if (rta->rta_type == IFLA_LIN_CAPS)
				cap = rta;
		if (cap && RTA_PAYLOAD(cap) >= sizeof(__u32)) {
			memcpy(caps, RTA_DATA(cap), sizeof(__u32));
			ret = 0;
		}
		break;
	}

	if (ret)
		errno = ENOMSG;
out:
	close(sock);
	return ret;
}

#define LIN_SKIP_UNLESS_CAP(caps, mask) do { \
	if (!((caps) & (mask))) \
		SKIP(return, "driver does not advertise " #mask); \
} while (0)

static inline int lin_socket(void)
{
	return socket(PF_LIN, SOCK_RAW, LIN_RAW);
}

static inline int lin_bind(int fd, int ifindex)
{
	struct sockaddr_lin addr = {
		.lin_family = AF_LIN,
		.lin_ifindex = ifindex,
	};

	return bind(fd, (struct sockaddr *)&addr, sizeof(addr));
}

/* socket() + bind(); returns the fd, or -1 (errno set). */
static inline int lin_open_bound(int ifindex)
{
	int fd = lin_socket();

	if (fd < 0)
		return -1;
	if (lin_bind(fd, ifindex) != 0) {
		int err = errno;

		close(fd);
		errno = err;
		return -1;
	}
	return fd;
}

static inline int lin_setopt(int fd, int opt, const void *val, socklen_t len)
{
	return setsockopt(fd, SOL_LIN_RAW, opt, val, len);
}

static inline int lin_setopt_int(int fd, int opt, int val)
{
	return setsockopt(fd, SOL_LIN_RAW, opt, &val, sizeof(val));
}

static inline int lin_getopt_int(int fd, int opt, int *val)
{
	socklen_t len = sizeof(*val);

	return getsockopt(fd, SOL_LIN_RAW, opt, val, &len);
}

static inline int lin_master(int fd, int on)
{
	return lin_setopt_int(fd, LIN_RAW_MASTER, on);
}

static inline int lin_publish(int fd, __u8 id, const void *data, __u8 len,
			      int enhanced)
{
	struct lin_publish pub = {
		.lin_id = id,
		.flags = enhanced ? LIN_F_CHK_ENH : 0,
		.len = len,
	};

	if (data && len)
		memcpy(pub.data, data, len);
	return lin_setopt(fd, LIN_RAW_PUBLISH, &pub, sizeof(pub));
}

static inline int lin_unpublish(int fd, __u8 id)
{
	return lin_setopt_int(fd, LIN_RAW_UNPUBLISH, id);
}

static inline int lin_set_filter(int fd, const struct lin_filter *f, int n)
{
	return lin_setopt(fd, LIN_RAW_FILTER, f, n * sizeof(*f));
}

static inline int lin_clear_filter(int fd)
{
	return lin_setopt(fd, LIN_RAW_FILTER, NULL, 0);
}

static inline int lin_err_filter(int fd, __u32 mask)
{
	return lin_setopt(fd, LIN_RAW_ERR_FILTER, &mask, sizeof(mask));
}

static inline int lin_sched_load(int fd, const union lin_sched_buf *buf,
				 unsigned int n)
{
	return lin_setopt(fd, LIN_RAW_SCHEDULE_LOAD, buf, lin_sched_bytes(n));
}

static inline int lin_sched_activate(int fd, int handle)
{
	return lin_setopt_int(fd, LIN_RAW_SCHEDULE_ACTIVATE, handle);
}

static inline int lin_sched_active(int fd, int *handle)
{
	return lin_getopt_int(fd, LIN_RAW_SCHEDULE_ACTIVATE, handle);
}

/* LIN_RAW_SCHEDULE_STOP is a no-argument sockopt: optlen must be 0. */
static inline int lin_sched_stop(int fd)
{
	return setsockopt(fd, SOL_LIN_RAW, LIN_RAW_SCHEDULE_STOP, NULL, 0);
}

static inline int lin_sched_delete(int fd, int handle)
{
	return lin_setopt_int(fd, LIN_RAW_SCHEDULE_DELETE, handle);
}

/* LIN_RAW_WAKEUP / LIN_RAW_SLEEP are no-argument sockopts: optlen must be 0. */
static inline int lin_wakeup(int fd)
{
	return setsockopt(fd, SOL_LIN_RAW, LIN_RAW_WAKEUP, NULL, 0);
}

static inline int lin_sleep(int fd)
{
	return setsockopt(fd, SOL_LIN_RAW, LIN_RAW_SLEEP, NULL, 0);
}

/* Publisher-response update via the data plane (write == sendmsg upsert). */
static inline ssize_t lin_write(int fd, __u8 id, const void *data, __u8 len,
				int enhanced)
{
	struct lin_frame f = {
		.lin_id = id,
		.flags = enhanced ? LIN_F_CHK_ENH : 0,
		.len = len,
	};

	if (data && len)
		memcpy(f.data, data, len);
	return write(fd, &f, sizeof(f));
}

/* Wait up to @ms for a frame. Returns 1 and fills @out on success, 0 on
 * timeout, -1 on error.
 */
static inline int lin_recv(int fd, struct lin_frame *out, int ms)
{
	struct pollfd pfd = { .fd = fd, .events = POLLIN };
	int ret;

	ret = poll(&pfd, 1, ms);
	if (ret < 0)
		return -1;
	if (ret == 0)
		return 0;
	if (read(fd, out, sizeof(*out)) != (ssize_t)sizeof(*out))
		return -1;
	return 1;
}

/* Like lin_recv() but via recvmsg(), reporting the source interface index
 * from the delivered struct sockaddr_lin (useful for observer sockets).
 */
static inline int lin_recv_from(int fd, struct lin_frame *out, int *src_ifindex,
				int ms)
{
	struct sockaddr_lin sa = {};
	struct iovec iov = { .iov_base = out, .iov_len = sizeof(*out) };
	struct msghdr msg = {
		.msg_name = &sa,
		.msg_namelen = sizeof(sa),
		.msg_iov = &iov,
		.msg_iovlen = 1,
	};
	struct pollfd pfd = { .fd = fd, .events = POLLIN };
	int ret = poll(&pfd, 1, ms);

	if (ret < 0)
		return -1;
	if (ret == 0)
		return 0;
	if (recvmsg(fd, &msg, 0) != (ssize_t)sizeof(*out))
		return -1;
	if (src_ifindex)
		*src_ifindex = sa.lin_ifindex;
	return 1;
}

/* True if no frame arrives within @ms. */
static inline int lin_silent(int fd, int ms)
{
	struct lin_frame f;

	return lin_recv(fd, &f, ms) == 0;
}

/* Read a netdev statistics counter via rtnetlink (RTM_GETLINK / IFLA_STATS64).
 * Netlink runs in the caller's network namespace; sysfs does not — the
 * /sys/class/net view is tagged to the netns of the /sys mount, which under the
 * harness's unshare(CLONE_NEWNET) sandbox is still init_net, so a sysfs read
 * would miss the test's vlin entirely. @field is a struct rtnl_link_stats64
 * member name. Returns the counter value, or -1 on any failure / unknown field.
 */
static inline long long lin_stat(const char *ifname, const char *field)
{
	struct { struct nlmsghdr nh; struct ifinfomsg ifi; } req = {};
	struct sockaddr_nl addr = { .nl_family = AF_NETLINK };
	struct rtnl_link_stats64 st = {};
	char resp[8192];
	struct nlmsghdr *nh;
	struct rtattr *rta;
	int sock, ifindex, attrlen, found = 0;
	long long val = -1;
	ssize_t len;

	ifindex = if_nametoindex(ifname);
	if (!ifindex)
		return -1;

	sock = socket(AF_NETLINK, SOCK_RAW, NETLINK_ROUTE);
	if (sock < 0)
		return -1;

	req.nh.nlmsg_len = NLMSG_LENGTH(sizeof(req.ifi));
	req.nh.nlmsg_type = RTM_GETLINK;
	req.nh.nlmsg_flags = NLM_F_REQUEST;
	req.ifi.ifi_family = AF_UNSPEC;
	req.ifi.ifi_index = ifindex;
	if (sendto(sock, &req, req.nh.nlmsg_len, 0,
		   (struct sockaddr *)&addr, sizeof(addr)) < 0)
		goto out;

	len = recv(sock, resp, sizeof(resp), 0);
	if (len < 0)
		goto out;

	for (nh = (struct nlmsghdr *)resp; NLMSG_OK(nh, len);
	     nh = NLMSG_NEXT(nh, len)) {
		if (nh->nlmsg_type != RTM_NEWLINK)
			continue;
		rta = IFLA_RTA(NLMSG_DATA(nh));
		attrlen = IFLA_PAYLOAD(nh);
		for (; RTA_OK(rta, attrlen); rta = RTA_NEXT(rta, attrlen)) {
			if (rta->rta_type != IFLA_STATS64)
				continue;
			memcpy(&st, RTA_DATA(rta),
			       RTA_PAYLOAD(rta) < sizeof(st) ?
			       RTA_PAYLOAD(rta) : sizeof(st));
			found = 1;
			break;
		}
		break;
	}
	if (!found)
		goto out;

	if (!strcmp(field, "tx_packets"))
		val = st.tx_packets;
	else if (!strcmp(field, "tx_bytes"))
		val = st.tx_bytes;
	else if (!strcmp(field, "rx_packets"))
		val = st.rx_packets;
	else if (!strcmp(field, "rx_bytes"))
		val = st.rx_bytes;
	else if (!strcmp(field, "tx_dropped"))
		val = st.tx_dropped;
	else if (!strcmp(field, "rx_dropped"))
		val = st.rx_dropped;
out:
	close(sock);
	return val;
}

#endif /* LIN_HARNESS_H */
