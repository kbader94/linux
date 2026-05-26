/* SPDX-License-Identifier: ((GPL-2.0-only WITH Linux-syscall-note) OR BSD-3-Clause) */
/*
 * linux/lin/netlink.h
 *
 * UAPI for LIN driver capability flags and the rtnetlink interface for
 * LIN network devices.
 *
 * Capability flags. Drivers populate struct lin_dev.caps with the union
 * of features they implement before lin_register_netdev(); the core
 * checks each user-facing feature against these flags and returns
 * -EOPNOTSUPP for unsupported requests. The bitmask is exported to
 * userspace under IFLA_LIN_CAPS.
 *
 * Basic features that every master-capable driver must support
 * (unconditional schedule slots, classic checksum) are not flagged -
 * they are implied by the presence of the master ops and
 * set_response/clear_response respectively.
 *
 * rtnetlink attributes. The IFLA_LIN_* attributes defined here are
 * carried inside IFLA_INFO_DATA on RTM_*LINK messages for links whose
 * info_kind belongs to a LIN driver. Userspace reads them with
 * RTM_GETLINK (or `ip -d link show <if>`).
 *
 *   IFLA_LIN_CAPS    - supported-capability bitmask (the LIN_CAP_*
 *                      values below). Read-only; advertised by every
 *                      conforming driver so userspace and the selftest
 *                      suite can discover which optional features the
 *                      device implements rather than probing each one
 *                      with attempt-and-EOPNOTSUPP.
 *
 *   IFLA_LIN_BITRATE - current bus bit rate in bits per second.
 *                      Read-write: a driver that supports runtime
 *                      bitrate changes accepts a u32 value here on
 *                      RTM_NEWLINK / RTM_SETLINK; drivers that do not
 *                      have set_bitrate hooked up return -EOPNOTSUPP.
 *                      The cached value is exported on RTM_GETLINK by
 *                      every driver, including those that took the
 *                      value at attach time and cannot change it later.
 *
 * Other LIN-specific settings - controller state, LIN xstats, etc. -
 * remain deferred to land alongside the first real consumer that needs
 * them. New attributes must be appended (next free slot) to preserve
 * ABI.
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
					 * (driver emits the event-trigger
					 * header, demuxes the responder via
					 * the first response data byte, and
					 * auto-switches to the slot's
					 * collision-resolving schedule on a
					 * collision)
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
#define LIN_CAP_WAKEUP		0x10	/* bus wakeup signalling (LIN 2.1+).
					 * Driver implements @wakeup_send for
					 * TX, and synthesizes wakeup frames
					 * (LIN_F_WAKEUP, lin_id=LIN_ID_NONE,
					 * len=0) on RX detection. Role-
					 * agnostic - any node may wake the
					 * bus per spec.
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
					 * publishing its own slot responses)
					 * is unaffected and always works.
					 *
					 * Drivers advertise this iff their
					 * transport can meet the timing.
					 * UART-based drivers (sllin, sdlin)
					 * do so iff they can program the
					 * UART for sub-frame interrupt
					 * latency (trigger=1 byte, or FIFO
					 * disabled). USB-CDC bridges
					 * (ft232, ch340, pl2303, cp210x)
					 * cannot. On-chip LIN devices that
					 * handle bus timing internally
					 * (Microchip LIN Serial Bus
					 * Analyzer, hardware LIN controllers)
					 * do.
					 */

/*
 * Per-LIN-link attributes carried inside IFLA_INFO_DATA on RTM_*LINK.
 */
enum {
	IFLA_LIN_UNSPEC,
	IFLA_LIN_CAPS,		/* __u32: supported LIN_CAP_* bitmask           */
	IFLA_LIN_BITRATE,	/* __u32: current bus bit rate in bits per sec  */
	IFLA_LIN_FORCE_PUB_SLAVE, /* __u8 (boolean): operator override of the
				 * LIN_CAP_PUB_SLAVE gate. When set, the LIN
				 * core admits LIN_RAW_PUBLISH from non-master
				 * sockets regardless of whether the driver
				 * advertised LIN_CAP_PUB_SLAVE. Intended for
				 * development against permissive masters and
				 * for exercising the slave-publisher code
				 * path on host UARTs whose driver has not
				 * been ported to the FC framework; the
				 * resulting bus timing is the operator's
				 * problem. Default 0.
				 */
	__IFLA_LIN_MAX,
};

#define IFLA_LIN_MAX (__IFLA_LIN_MAX - 1)

#endif /* !_UAPI_LIN_NETLINK_H */
