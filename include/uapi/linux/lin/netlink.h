/* SPDX-License-Identifier: ((GPL-2.0-only WITH Linux-syscall-note) OR BSD-3-Clause) */
/*
 * linux/lin/netlink.h
 *
 * UAPI for LIN driver capability flags. Drivers populate
 * struct lin_dev.caps with the union of features they implement before
 * lin_register_netdev(); the core checks each user-facing feature
 * against these flags and returns -EOPNOTSUPP for unsupported requests.
 * The bitmask is reported to userspace under IFLA_LIN_CAPS by the
 * rtnetlink integration that lands later in this series.
 *
 * Basic features that every master-capable driver must support
 * (unconditional schedule slots, classic checksum) are not flagged -
 * they are implied by the presence of the master ops and
 * set_response/clear_response respectively.
 *
 * Author: Kyle Bader <kyle.bader94@gmail.com>
 * Copyright (c) 2026 Kyle Bader
 */

#ifndef _UAPI_LIN_NETLINK_H
#define _UAPI_LIN_NETLINK_H

#include <linux/types.h>

#define LIN_CAP_SPORADIC	0x01	/* TYPE_SPORADIC schedule slots. The
					 * driver keeps a per-member dirty flag
					 * ("updated since last emit"): set it
					 * when @set_response is called for the
					 * member, clear it when that member is
					 * emitted. On each TYPE_SPORADIC slot
					 * the driver emits the highest-priority
					 * dirty member - members are listed in
					 * priority order, index 0 highest - and
					 * clears its flag; the slot stays silent
					 * when no member is dirty. The member
					 * priority order is the userspace-to-
					 * driver contract carried in
					 * lin_schedule_entry.members[].
					 */
#define LIN_CAP_EVENT		0x02	/* TYPE_EVENT schedule slots
					 * (reserved for v2; v1 always
					 * rejects regardless of caps)
					 */
#define LIN_CAP_DIAG		0x04	/* diagnostic ID handling. Master-
					 * side: TYPE_DIAG schedule slots
					 * route 0x3C / 0x3D to the diagnostic
					 * transport path. Slave-side: driver
					 * accepts and answers diag headers
					 * for transport-protocol responders.
					 * Role-agnostic - slave-only drivers
					 * may advertise this without the
					 * master ops.
					 */
#define LIN_CAP_CHK_ENH		0x08	/* enhanced checksum (LIN 2.x)
					 * supported in addition to the
					 * mandatory classic checksum
					 */
#define LIN_CAP_PUB_SLAVE	0x20	/* driver can meet the LIN spec's
					 * header-RX -> response-TX timing
					 * for the slave-only publisher
					 * role (~40 character times, ~20 ms
					 * at 19200 baud / ~4 ms at 10417).
					 * Required for LIN_RAW_PUBLISH from
					 * a socket that does NOT hold
					 * LIN_RAW_MASTER; the LIN core
					 * returns -EOPNOTSUPP otherwise.
					 * Master-with-publish path (a socket
					 * holding LIN_RAW_MASTER also
					 * registering a publisher) sits on
					 * the same TX path as the schedule
					 * and is exempt from this cap.
					 */

#endif /* _UAPI_LIN_NETLINK_H */
