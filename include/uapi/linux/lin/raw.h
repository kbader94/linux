/* SPDX-License-Identifier: ((GPL-2.0-only WITH Linux-syscall-note) OR BSD-3-Clause) */
/*
 * linux/lin/raw.h
 *
 * Definitions for raw LIN sockets
 *
 * Author: Kyle Bader <kyle.bader94@gmail.com>
 * Copyright (c) 2026 Kyle Bader
 */

#ifndef _UAPI_LIN_RAW_H
#define _UAPI_LIN_RAW_H

#include <linux/lin.h>

#define SOL_LIN_RAW			288

/* Maximum number of filter entries set via LIN_RAW_FILTER. */
#define LIN_RAW_FILTER_MAX		64

/* Maximum number of schedules loaded per master socket. */
#define LIN_RAW_SCHEDULES_MAX		16

/* Maximum number of entries in a single schedule. */
#define LIN_RAW_SCHEDULE_ENTRIES_MAX	256

/*
 * Maximum frame slot duration in microseconds, applied to both
 * @lin_schedule.default_slot_us and per-entry @lin_schedule_entry.slot_us.
 * LIN clusters use slot times in the 5-100 ms range; the 1-second cap
 * is well above any realistic configuration and prevents userspace from
 * stalling kernel-side policy operations (LIN_RAW_SCHEDULE_ACTIVATE
 * blocks for up to one slot of the previously-active schedule).
 */
#define LIN_RAW_SCHEDULE_SLOT_MAX_US	1000000U

/*
 * LIN_RAW socket options. Socket level is SOL_LIN_RAW.
 *
 * The first block mirrors the equivalent CAN_RAW options so that applications
 * and tooling translate directly. The second block is LIN-specific: it
 * expresses the master/slave distinction, the single-publisher-per-frame-ID
 * rule, and the kernel-managed master schedule tables. Schedule execution
 * and the hardware response table are owned by the driver; the LIN core
 * validates userspace input, enforces cross-socket policy, and forwards
 * the resulting state to the bound driver.
 *
 * Most socket-state mutation goes through setsockopt(): filter setup,
 * master-role claim, schedule upload and switching. The one exception
 * is the data-plane update for publishers — sendmsg() (and write())
 * on a LIN_RAW socket upsert a publisher entry for frame.lin_id,
 * functionally identical to LIN_RAW_PUBLISH with the frame's content.
 * This keeps per-sample response updates ergonomic while preserving
 * sockopt as the mutation path for all structural configuration. See
 * LIN_RAW_PUBLISH below for semantics, error cases, and the
 * sendmsg/write contract.
 *
 * There is no "send this frame onto the bus now" path on LIN_RAW:
 * frame emission is driven by the master's schedule firing a slot
 * whose response was previously registered. sendmsg() never injects
 * arbitrary traffic, only updates the kernel-tracked publisher state.
 *
 * Device lifecycle
 *
 * A LIN_RAW socket may be bound (sockaddr_lin.lin_ifindex != 0) while
 * the interface is administratively down, but every sockopt that
 * drives a frame onto the bus or mutates kernel-tracked bus policy
 * requires IFF_UP on the bound netdev. While the bound interface is
 * down, the role-agnostic operations LIN_RAW_MASTER, LIN_RAW_PUBLISH,
 * LIN_RAW_UNPUBLISH, LIN_RAW_WAKEUP, and sendmsg() / write() (which
 * upserts a publisher entry) return -ENETDOWN. The master-gated
 * operations LIN_RAW_SCHEDULE_{LOAD,DELETE,ACTIVATE,STOP},
 * LIN_RAW_SEND_HEADER, and LIN_RAW_SLEEP instead return -EPERM while
 * down: the master-role check precedes the link-state check, and no
 * socket can hold the role across a down transition — the going-down
 * drain (below) force-releases the claim and LIN_RAW_MASTER itself
 * returns -ENETDOWN while down, so the role cannot be re-acquired
 * until the interface is back up. Subscription / state-only sockopts —
 * LIN_RAW_FILTER, LIN_RAW_ERR_FILTER, LIN_RAW_LOOPBACK,
 * LIN_RAW_RECV_OWN_MSGS, LIN_RAW_JOIN_FILTERS, LIN_RAW_WAKEUP_FILTER —
 * remain available regardless of link state.
 *
 * When the bound interface is brought down (`ip link set <if> down`),
 * the kernel quiesces LIN policy ahead of the driver's stop callback.
 * The drain runs at NETDEV_GOING_DOWN (before ndo_stop, while the
 * driver is still alive) — not at NETDEV_DOWN — so any sockopt
 * already in flight is either ordered before the quiesce or refused
 * with -ENETDOWN, and the driver never observes a policy op call
 * concurrent with its ndo_stop. During the drain the kernel:
 *
 *   - force-releases the master-role claim
 *   - releases all publisher entries owned by the socket
 *   - stops the active schedule and deletes loaded schedules
 *
 * Rx filters (LIN_RAW_FILTER / LIN_RAW_ERR_FILTER / LIN_RAW_WAKEUP_FILTER),
 * the binding itself, and the held netdev reference all survive across
 * down/up — the socket remains usable as a passive observer if the
 * interface returns. The socket's sk_err is set to ENETDOWN at
 * NETDEV_DOWN so a blocked recvmsg() or a subsequent send observes
 * the transition.
 *
 * Userspace must re-establish per-bus policy state after bringing the
 * interface back up: re-claim LIN_RAW_MASTER (if previously held),
 * re-register publishers via LIN_RAW_PUBLISH, and re-load + re-activate
 * any schedules. The kernel does not cache and replay these
 * registrations across a down/up cycle; doing so would race against
 * the driver's tx path and against any other process that races to
 * claim the master role between down and up.
 *
 * NETDEV_UNREGISTER (interface deletion) is the harder teardown: the
 * netdev is going away, so in addition to the above the socket's
 * filters and netdev reference are dropped and the socket is unbound.
 * After unregister, sockopts return -EOPNOTSUPP until the socket is
 * re-bound to a different interface.
 */
enum {
	/* CAN_RAW parity: */
	LIN_RAW_FILTER = 1,		/* 0..n struct lin_filter; 0 clears
					 * (receive no data frames)
					 */
	LIN_RAW_ERR_FILTER,		/* error frame subscription mask       */
	LIN_RAW_LOOPBACK,		/* core synthesises rx for own master /
					 * publisher emissions (default on)
					 */
	LIN_RAW_RECV_OWN_MSGS,		/* deliver loopback to the originator
					 * (default off)
					 */
	LIN_RAW_JOIN_FILTERS,		/* AND-combine filters (default: OR)   */

	/* LIN-specific role / ownership: */
	LIN_RAW_MASTER,			/* claim(1)/release(0) master role     */
	LIN_RAW_PUBLISH,		/* register/update a publisher entry   */
	LIN_RAW_UNPUBLISH,		/* release a publisher entry by LIN ID */

	/* LIN-specific scheduling (master socket only): */
	LIN_RAW_SCHEDULE_LOAD,		/* upload/replace a schedule           */
	LIN_RAW_SCHEDULE_DELETE,	/* remove a schedule by handle         */
	LIN_RAW_SCHEDULE_ACTIVATE,	/* switch active schedule              */
	LIN_RAW_SCHEDULE_STOP,		/* stop the active schedule            */
};

/*
 * Error subscription mask (LIN_RAW_ERR_FILTER)
 *
 *   __u32 mask = LIN_ERR_CHECKSUM | LIN_ERR_NO_RESPONSE;
 *   setsockopt(sock, SOL_LIN_RAW, LIN_RAW_ERR_FILTER,
 *              &mask, sizeof(mask));
 *
 * Default mask is zero, meaning error frames are not delivered to this
 * socket. Set to LIN_ERR_MASK_ALL (see <linux/lin/error.h>) to receive
 * every class, or OR specific LIN_ERR_* values for a subset. A frame's
 * struct lin_frame.err_mask is AND'd against the subscription mask;
 * delivery occurs when the result is non-zero.
 */

/*
 * Loopback model (LIN_RAW_LOOPBACK and LIN_RAW_RECV_OWN_MSGS)
 *
 * LIN drivers do not natively echo their own transmissions onto rx.
 * Most LIN transceivers (UART/SPI bridges, half-duplex single-wire
 * transceivers) read back the line internally to validate the byte
 * stream but do not surface their own transmissions as received
 * frames. To preserve a SocketCAN-style "every emission is visible
 * to every socket on the bus" abstraction, the LIN core synthesises
 * a tagged loopback skb at master/publisher emission time and feeds
 * it through the normal rx path. The skb carries pointers to the
 * originating master and/or publisher socket so subscribers can
 * honour LIN_RAW_RECV_OWN_MSGS.
 *
 * Two independent sockopts shape this:
 *
 *   LIN_RAW_LOOPBACK — default ON. Controls whether emissions
 *     sourced by this socket are made visible to OTHER sockets on
 *     the same interface via the loopback path. Mirrors
 *     CAN_RAW_LOOPBACK semantics.
 *
 *     For frames with multiple local stakeholders (a master who
 *     fired the header AND a publisher whose data formed the
 *     response), the synth happens if ANY stakeholder has LOOPBACK
 *     on. To suppress local visibility entirely, all local
 *     stakeholders must opt out. This permissive policy avoids
 *     situations where one socket's preference silently breaks
 *     unrelated subscribers; sockets that want to filter their own
 *     emissions on the receive side should use LIN_RAW_FILTER.
 *
 *   LIN_RAW_RECV_OWN_MSGS — default OFF. Controls whether the
 *     synthesised skb is delivered to the originator's own recvmsg
 *     queue. RECV_OWN_MSGS and LOOPBACK are independent: with
 *     LOOPBACK off but another stakeholder voting yes, your
 *     emissions may still be synthesised, and your RECV_OWN_MSGS
 *     setting still controls whether they reach your queue.
 *
 * Provenance is recorded at emission time. The skb tags name the
 * exact sockets that sourced the frame, so role transitions during
 * observation cannot misclassify a frame as own / not-own.
 */

/*
 * Master-role claim (LIN_RAW_MASTER)
 *
 *   setsockopt(sock, SOL_LIN_RAW, LIN_RAW_MASTER, &on, sizeof(int));
 *
 * At most one socket on a given LIN interface may hold the master role.
 * Claiming while another socket holds it returns -EBUSY. The role is
 * released automatically when the socket is closed, or explicitly by
 * setting @on = 0. Any number of subscriber-only (non-master, non-
 * publisher) sockets may coexist with the master claim — this is what
 * allows diagnostic tools to observe traffic alongside an active master.
 *
 * There is no corresponding LIN_RAW_SLAVE sockopt: a socket that has not
 * claimed the master role is a slave by default. Slave nodes are not
 * mutually exclusive (any number may coexist on one bus), and the
 * capabilities a slave exercises — publishing responses and subscribing
 * to frames — are already expressed via LIN_RAW_PUBLISH and
 * LIN_RAW_FILTER respectively. A socket that neither publishes nor
 * claims master is a passive observer (e.g. a sniffer); this is a valid
 * and supported configuration.
 *
 * The master claim requires the socket to be bound to a specific
 * interface (lin_ifindex != 0 in sockaddr_lin). Claiming the master
 * role on a zero-bound (observer) socket returns -EOPNOTSUPP.
 */

/**
 * struct lin_publish - argument for LIN_RAW_PUBLISH
 * @lin_id: 6-bit frame ID (0..LIN_ID_MASK). Bits 6 and 7 must be
 *          zero — pass the raw ID, not the on-wire Protected ID
 *          (the kernel computes the two parity bits itself).
 *          Out-of-range values are rejected with -EINVAL.
 * @flags:  LIN_F_CHK_ENH for enhanced checksum, else classic
 * @len:    response payload length (1..LIN_MAX_DLEN)
 * @data:   initial response payload
 * @__res:  trailing reserved bytes, must be all zero on write. Sized
 *          to absorb one future u64-scale field (e.g. response-delay
 *          override, per-publish sequence number, priority hint) plus
 *          a couple of flag bytes without growing the struct or
 *          requiring a v2 sockopt; the kernel rejects nonzero bytes
 *          with -EINVAL.
 *
 * Registers the socket as the publisher for @lin_id on its bound
 * interface, and stores @data in the kernel's response table for that
 * interface (the LIN core forwards the response to the driver, which
 * owns the actual transceiver-facing response storage).
 *
 * Only one socket may publish a given frame ID on a given interface at
 * a time. Attempting to register a publisher for an ID that is already
 * owned by a different socket returns -EBUSY. Re-registering the same
 * (socket, ID) pair updates the stored response in place.
 *
 * IDs 0x3E and 0x3F are reserved by the LIN specification and are
 * rejected with -EINVAL. IDs 0x3C (master request) and 0x3D (slave
 * response) require the bound driver to advertise LIN_CAP_DIAG (else
 * -EOPNOTSUPP) and use classic checksum per LIN spec — LIN_F_CHK_ENH
 * on @flags is rejected with -EINVAL for 0x3C / 0x3D regardless of
 * LIN_CAP_CHK_ENH.
 *
 * Publisher registration requires the socket to be bound to a specific
 * interface (lin_ifindex != 0 in sockaddr_lin). LIN_RAW_PUBLISH on a
 * zero-bound (observer) socket returns -EOPNOTSUPP.
 *
 * sendmsg() / write() as an alternative update path:
 *
 *   write(fd, &frame, sizeof(frame));
 *
 * NOTE - deliberate ABI departure from CAN_RAW: write()/sendmsg() does
 * NOT put a frame on the bus. On a LIN bus the master schedule decides
 * when a frame's header is sent; a publisher only supplies the response
 * bytes. So write() here installs or refreshes this socket's sticky
 * response for frame.lin_id (identical to LIN_RAW_PUBLISH), and those
 * bytes are transmitted later, when the running schedule reaches that
 * ID. CAN_RAW-style tooling that assumes one write() == one frame on the
 * wire must be adapted to LIN's scheduler-owned data plane.
 *
 * On success, registers this socket as publisher for frame.lin_id
 * with the frame's data (if no publisher exists), or updates the
 * stored response bytes in place (if this socket already owns it).
 * Returns sizeof(struct lin_frame) on success. Error cases:
 *
 *   -EINVAL       frame size wrong, frame.__pad / frame.__res non-zero,
 *                 flags other than LIN_F_CHK_ENH set, LIN_F_ERR set,
 *                 len == 0 or len > LIN_MAX_DLEN, or lin_id reserved (0x3E/0x3F)
 *   -EOPNOTSUPP   socket unbound, or bound to ifindex == 0
 *   -ENODEV       bound netdev has gone away
 *   -EBUSY        frame.lin_id already owned by a different socket
 *
 * The master-role claim and publisher registration are independent: a
 * master socket that wishes to supply response data for some IDs must
 * register those IDs via LIN_RAW_PUBLISH, exactly as a slave socket does.
 */
struct lin_publish {
	__u8	lin_id;
	__u8	flags;
	__u8	len;
	__u8	data[LIN_MAX_DLEN];
	__u8	__res[9];
};

/*
 * Release a publisher entry (LIN_RAW_UNPUBLISH)
 *
 *   int id = lin_id;
 *   setsockopt(sock, SOL_LIN_RAW, LIN_RAW_UNPUBLISH, &id, sizeof(int));
 *
 * Only the owning socket may release its own publisher entry. A socket's
 * publisher entries are also released automatically when the socket is
 * closed.
 *
 * The kernel does not enforce schedule-publisher cross-consistency past
 * the LIN_RAW_SCHEDULE_LOAD validation: unpublishing an ID currently
 * referenced as a sporadic member in a loaded schedule is permitted.
 * The corresponding sporadic slot then fires silently (no header
 * emission) for that member until either the schedule is reloaded or
 * the publisher is re-registered. Applications that care about this
 * invariant should release publishers only after deleting the
 * referencing schedule.
 */

/*
 * Schedule entry frame type. The @type byte of struct
 * lin_schedule_entry selects a mutually-exclusive LIN frame type.
 *
 * Validation enforced by the kernel on LIN_RAW_SCHEDULE_LOAD:
 *   - Each entry's member IDs must be < LIN_ID_RESERVED_FIRST (0x3E)
 *     except where the type explicitly permits a reserved ID.
 *   - TYPE_UNCOND requires member_count == 1.
 *   - TYPE_DIAG requires member_count == 1 and members[0] is one of
 *     LIN_ID_DIAG_MASTER_REQ (0x3C) or LIN_ID_DIAG_SLAVE_RESP (0x3D).
 *     Requires the driver to advertise LIN_CAP_DIAG.
 *   - TYPE_SPORADIC accepts member_count >= 1 and requires the driver
 *     to advertise LIN_CAP_SPORADIC. Every member ID must already
 *     have a publisher registered on the interface (load-time
 *     validation: -EINVAL otherwise).
 *   - TYPE_EVENT is reserved (v2). Returns -EOPNOTSUPP regardless of
 *     driver capability.
 */
#define LIN_SCHED_TYPE_UNCOND	0x00	/* single unconditional frame */
#define LIN_SCHED_TYPE_SPORADIC	0x01	/* emit one of @members whose
					 * publisher has been updated since
					 * last emission. @member_count == 1
					 * is the simple single-publisher
					 * case; >1 expresses LIN's grouped
					 * sporadic semantics.
					 */
#define LIN_SCHED_TYPE_EVENT	0x02	/* event-triggered (reserved; v1
					 * returns -EOPNOTSUPP). Will use
					 * @members[0] as the trigger ID
					 * and @members[1..] as the
					 * fallback unconditional members.
					 */
#define LIN_SCHED_TYPE_DIAG	0x03	/* diagnostic frame slot (0x3C/0x3D) */

/* Maximum frame IDs per schedule slot (covers grouped sporadic and
 * event-triggered groups). LIN cluster configurations rarely exceed
 * 4 members; 8 caps the slot at 16 bytes.
 */
#define LIN_SLOT_MAX_MEMBERS	8

/**
 * struct lin_schedule_entry - one slot in a master schedule
 * @type:         LIN_SCHED_TYPE_* (one of the values above)
 * @flags:        reserved, must be zero
 * @member_count: number of valid IDs in @members (1..LIN_SLOT_MAX_MEMBERS)
 * @cr_handle:    reserved for a future TYPE_EVENT collision-resolving
 *                schedule handle (this commit defines only the byte;
 *                TYPE_EVENT itself is rejected as -EOPNOTSUPP at the
 *                schedule-API commit). The byte is compiler-mandated
 *                alignment padding for @slot_us made explicit and
 *                validated as zero for every non-TYPE_EVENT entry so
 *                it stays repurposable; TYPE_EVENT entries will name
 *                their collision-resolving table here when that type
 *                is enabled.
 * @slot_us:      frame slot duration in microseconds; 0 = inherit
 *                struct lin_schedule.default_slot_us
 * @members:      6-bit frame IDs participating in this slot.
 *                Trailing entries beyond @member_count must be zero.
 *                Per-type semantics:
 *                  TYPE_UNCOND:   one ID; the slot fires its header
 *                                 every cycle.
 *                  TYPE_DIAG:     one ID (LIN_ID_DIAG_MASTER_REQ or
 *                                 LIN_ID_DIAG_SLAVE_RESP).
 *                  TYPE_SPORADIC: one or more IDs; master fires the
 *                                 header for whichever member's
 *                                 publisher is dirty, in priority
 *                                 order (lowest @members index
 *                                 wins). Slot stays silent when no
 *                                 member is dirty.
 *                  TYPE_EVENT:    @members[0] is the event-triggered
 *                                 trigger ID; @members[1..] are the
 *                                 unconditional fallback members
 *                                 (reserved for v2).
 * @__res:        trailing reserved bytes, must be all zero on write.
 *                Sized to absorb one future u64-scale per-entry field
 *                (e.g. per-entry priority, retry count, slot variant)
 *                without growing the struct or requiring a v2 schedule
 *                entry shape; the kernel rejects nonzero bytes with
 *                -EINVAL.
 *
 * LIN schedules typically use a fixed slot time across all entries
 * (configured via struct lin_schedule.default_slot_us), but the LIN
 * specification permits per-frame slot variation. Setting @slot_us on
 * individual entries allows that mixed-timing case; leaving it zero
 * lets every entry inherit the schedule default.
 */
struct lin_schedule_entry {
	__u8	type;
	__u8	flags;
	__u8	member_count;
	__u8	cr_handle;
	__u32	slot_us;
	__u8	members[LIN_SLOT_MAX_MEMBERS];
	__u8	__res[8];
};

/**
 * struct lin_schedule - argument for LIN_RAW_SCHEDULE_LOAD
 * @handle:          caller-assigned schedule handle
 *                   (0..LIN_RAW_SCHEDULES_MAX - 1)
 * @flags:           reserved, must be zero
 * @entry_count:     number of entries following this header
 *                   (1..LIN_RAW_SCHEDULE_ENTRIES_MAX)
 * @default_slot_us: default frame slot duration in microseconds, applied
 *                   to any entry whose slot_us is zero
 * @__res:           trailing schedule-header reserved bytes, must be
 *                   all zero on write. Sized to absorb one future
 *                   u64-scale field (e.g. timeout, priority, schedule
 *                   reference) without growing the header or shifting
 *                   @entry's offset; the kernel rejects nonzero bytes
 *                   with -EINVAL so the space stays available to
 *                   repurpose. The placement between @default_slot_us
 *                   and @entry is deliberate: the flexible array's
 *                   offset is part of the ABI, so any future header
 *                   field has to be reserved up-front here rather than
 *                   appended.
 * @entry:           flexible array of @entry_count schedule slots
 *
 * Uploads (or replaces, if @handle is already loaded) a schedule on the
 * master socket. Schedules are identified by caller-assigned handle so
 * an application can pre-load several schedules (e.g. "normal" and
 * "diagnostic") and switch between them with LIN_RAW_SCHEDULE_ACTIVATE.
 *
 * The LIN core validates the schedule and forwards it to the bound
 * driver, which stores it in its own (often hardware-backed) schedule
 * table and executes it. Schedule ownership is tracked by the LIN core
 * per loading socket: entries are released when the socket is closed
 * or the master role is released. Loading a schedule while one is
 * active is permitted; the currently-running schedule is unaffected
 * until LIN_RAW_SCHEDULE_ACTIVATE selects a different handle.
 *
 * Validation at LOAD time (fail-fast):
 *   - Structural: handle / entry_count / member_count ranges,
 *     reserved fields zero, type-specific member shape.
 *   - Capability: TYPE_SPORADIC requires LIN_CAP_SPORADIC, TYPE_DIAG
 *     requires LIN_CAP_DIAG; missing capability returns -EOPNOTSUPP.
 *     TYPE_EVENT is reserved for v2 and always returns -EOPNOTSUPP.
 *   - Publisher existence: every TYPE_SPORADIC member must have a
 *     publisher registered on the interface (LIN_RAW_PUBLISH). Missing
 *     publisher returns -EINVAL. Per-cluster setup order: register
 *     publishers first, then load schedules.
 */
struct lin_schedule {
	__u8	handle;
	__u8	flags;
	__u16	entry_count;
	__u32	default_slot_us;
	__u8	__res[8];
	struct lin_schedule_entry entry[];
};

/*
 * Activate a loaded schedule (LIN_RAW_SCHEDULE_ACTIVATE)
 *
 *   int handle = 0;
 *   setsockopt(sock, SOL_LIN_RAW, LIN_RAW_SCHEDULE_ACTIVATE,
 *              &handle, sizeof(int));
 *
 * Begins running the previously-loaded schedule identified by @handle.
 * If another schedule is currently active, it is stopped at the next slot
 * boundary and replaced. Returns -ENOENT if @handle is not loaded.
 *
 * Synchronous: the call BLOCKS until the swap has taken effect — the
 * previously-active schedule (if any) has finished its in-flight slot
 * and the new schedule is running on the wire. Bounded by one slot
 * duration of the previous schedule (typically 10-50 ms, capped by
 * the schedule's slot_us). On successful return, a subsequent
 * LIN_RAW_SCHEDULE_LOAD on the previously-active handle is race-free:
 * by the time ACTIVATE returns, that handle is fully inactive and
 * safe to modify. Idempotent: activating the currently-active handle
 * returns 0 immediately without blocking.
 *
 * Schedule contents (frame types, member IDs, sporadic publisher
 * existence) were validated at LOAD time; ACTIVATE itself only
 * verifies the handle is currently loaded. If a publisher referenced
 * by a sporadic slot has been unregistered between LOAD and ACTIVATE,
 * the slot fires silently — no error from this sockopt.
 *
 * getsockopt(LIN_RAW_SCHEDULE_ACTIVATE)
 *
 *   int handle;
 *   socklen_t len = sizeof(handle);
 *   getsockopt(sock, SOL_LIN_RAW, LIN_RAW_SCHEDULE_ACTIVATE,
 *              &handle, &len);
 *
 * Reads the currently-active schedule handle on the bound interface,
 * or -1 if no schedule is active. Available to any socket bound to a
 * specific interface (no master claim required for observation;
 * useful for diagnostic tools attaching to an in-flight session).
 * Returns -EOPNOTSUPP on an unbound or zero-bound socket.
 *
 * Stop the active schedule (LIN_RAW_SCHEDULE_STOP)
 *
 *   setsockopt(sock, SOL_LIN_RAW, LIN_RAW_SCHEDULE_STOP, NULL, 0);
 *
 * Delete a loaded schedule (LIN_RAW_SCHEDULE_DELETE)
 *
 *   int handle = 0;
 *   setsockopt(sock, SOL_LIN_RAW, LIN_RAW_SCHEDULE_DELETE,
 *              &handle, sizeof(int));
 *
 * Returns -EBUSY if @handle is the currently-active schedule; stop it
 * first.
 */

#endif /* !_UAPI_LIN_RAW_H */
