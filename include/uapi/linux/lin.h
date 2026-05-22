/* SPDX-License-Identifier: ((GPL-2.0-only WITH Linux-syscall-note) OR BSD-3-Clause) */
/*
 * linux/lin.h
 *
 * Definitions for LIN network layer (socket addr / LIN frame / LIN filter)
 *
 * Author: Kyle Bader <kyle.bader94@gmail.com>
 * Copyright (c) 2026 Kyle Bader
 *
 * The shape of struct sockaddr_lin, struct lin_filter, and the socket API
 * conventions follow SocketCAN so that tooling and patterns translate
 * directly. The frame structure does *not* reuse the 32-bit id+flags
 * packing from struct can_frame: LIN identifiers are 6-bit and fit in a
 * single byte, so ID and flags are exposed as separate fields for clarity.
 *
 * LIN-specific semantics (header/response split, single-master claim,
 * single-publisher-per-ID rule, kernel-managed schedule tables) are
 * expressed via LIN_RAW socket options; see <linux/lin/raw.h>. The
 * userspace contract is "the kernel drives the bus according to
 * configuration you upload"; internally, the LIN core validates and
 * forwards schedule and response-table state to the bound driver,
 * which owns the actual schedule execution and hardware response
 * table (this matches typical LIN controller capability).
 */

#ifndef _UAPI_LINUX_LIN_H
#define _UAPI_LINUX_LIN_H

#include <linux/types.h>
#include <linux/socket.h>
#include <linux/stddef.h>	/* for offsetof */

/*
 * LIN payload length per ISO 17987-3. On-wire classic LIN data frames
 * carry 1..LIN_MAX_DLEN bytes; this is enforced for data frames and
 * for publisher registrations. Zero-length data is only permitted in
 * the error-frame context field of struct lin_frame.
 */
#define LIN_MAX_DLEN	8

/* Valid bits in a LIN frame ID. LIN uses a 6-bit frame ID (without the two
 * parity bits that form the protected identifier on the wire). Parity is
 * computed and verified by the kernel; userspace only sees the 6-bit ID.
 */
#define LIN_ID_MASK	0x3FU

/* LIN bus default bitrate. 19200 bps is the LIN 1.x classic default and
 * the most common deployed speed. Drivers that cannot derive a bitrate
 * from firmware or operator configuration fall back to this; userspace
 * tooling can use it as the default when no value is specified.
 */
#define LIN_DEFAULT_BITRATE	19200

/*
 * Named 6-bit LIN frame IDs with specification-defined meanings.
 *
 *   LIN_ID_DIAG_MASTER_REQ (0x3C) / LIN_ID_DIAG_SLAVE_RESP (0x3D) are
 *   the diagnostic-transport-layer frame IDs. The LIN core requires
 *   the driver to advertise LIN_CAP_DIAG to publish or fire headers
 *   on these IDs, and per LIN spec they use classic checksum
 *   unconditionally — enhanced checksum on 0x3C / 0x3D is rejected
 *   with -EINVAL.
 *
 *   LIN_ID_RESERVED_FIRST (0x3E) is the first reserved ID; the kernel
 *   rejects publisher registration and schedule entries with lin_id
 *   >= LIN_ID_RESERVED_FIRST.
 */
#define LIN_ID_DIAG_MASTER_REQ	0x3CU
#define LIN_ID_DIAG_SLAVE_RESP	0x3DU
#define LIN_ID_RESERVED_FIRST	0x3EU

/*
 * Sentinel placed in struct lin_frame.lin_id on an error frame whose
 * associated LIN ID is unknown to the driver. The two high bits of a
 * valid LIN ID must be zero, so 0xFF never aliases a real data frame.
 */
#define LIN_ID_NONE	0xFFU

/*
 * Flag bits carried in struct lin_frame.flags. These bits are also valid
 * in struct lin_filter.flags / flags_mask for filter matching.
 */
#define LIN_F_ERR	0x01	/* error frame: err_mask carries the error
				 * class(es) and data[] carries optional
				 * class-specific context (see
				 * <linux/lin/error.h>).
				 */
#define LIN_F_CHK_ENH	0x02	/* enhanced checksum was used on the wire
				 * (as opposed to the classic checksum).
				 */
#define LIN_F_EVENT_COLLISION	0x08	/* event-triggered slot collision: the
					 * master polled an event-triggered
					 * frame and two or more slaves
					 * answered. A non-error notification
					 * (mutually exclusive with LIN_F_ERR)
					 * carrying no data — lin_id is the
					 * event-trigger ID, len is 0, err_mask
					 * is 0. The master resolves it by
					 * running the slot's collision-resolving
					 * schedule. Delivered through the normal
					 * filter path, so subscribe by matching
					 * the trigger ID (or match-all).
					 * (0x04 is LIN_F_WAKEUP, added with bus
					 * wakeup support.)
					 */

/**
 * struct lin_frame - LIN frame exchanged over PF_LIN sockets
 * @lin_id:   6-bit frame ID (without parity). Only the low 6 bits are
 *            meaningful; the top two bits must be zero on transmit and
 *            will be zero on receive, except on error frames where
 *            LIN_ID_NONE indicates the driver could not associate the
 *            error with a specific ID.
 * @flags:    LIN_F_* flag bits describing properties of the frame.
 * @len:      payload length in bytes. For data frames (LIN_F_ERR clear
 *            in @flags), 1..LIN_MAX_DLEN per the LIN specification. For
 *            error frames (LIN_F_ERR set), 0..LIN_MAX_DLEN — the number
 *            of meaningful bytes of class-specific context in @data,
 *            which may be zero when the driver has no context to report.
 * @__pad:    compiler-mandated alignment padding for @err_mask, made
 *            explicit and validated as zero so the byte stays available
 *            for repurposing into a small future field.
 * @err_mask: on error frames (LIN_F_ERR set in @flags), OR of one or
 *            more LIN_ERR_* class bits. Must be zero on non-error frames.
 * @data:     frame payload (up to LIN_MAX_DLEN bytes). For error frames
 *            carries optional, class-specific context (e.g. the bytes
 *            that failed checksum); see <linux/lin/error.h>.
 * @__res:    trailing reserved bytes, must be all zero on write. Sized
 *            to absorb one future u64-scale field (or several smaller
 *            fields) without growing the struct or requiring a v2
 *            sockopt; the kernel rejects nonzero bytes with -EINVAL so
 *            the space stays available to repurpose.
 *
 * Frames observed on the bus are delivered to subscribed sockets as a
 * single struct lin_frame, regardless of whether the response came from
 * the master or a slave. The header/response split of the wire protocol
 * is resolved by the kernel and is not visible on the socket interface.
 *
 * The checksum byte on the wire is also not exposed here: on receive the
 * kernel either validates it (delivering a normal frame) or reports the
 * failure (delivering an error frame); on transmit the kernel computes it
 * from the registered publisher's checksum type.
 */
struct lin_frame {
	__u8	lin_id;
	__u8	flags;
	__u8	len;
	__u8	__pad;
	__u32	err_mask;
	__u8	data[LIN_MAX_DLEN];
	__u8	__res[8];
};

#define LIN_MTU		(sizeof(struct lin_frame))

/* particular protocols of the protocol family PF_LIN */
#define LIN_RAW		1	/* RAW sockets */
#define LIN_NPROTO	2

/**
 * struct sockaddr_lin - the sockaddr structure for LIN sockets
 * @lin_family:  address family number AF_LIN
 * @lin_ifindex: LIN network interface index, or zero to bind across
 *               every LIN interface in the socket's network namespace.
 *               A zero-bound socket is observer-only: receive filters
 *               and error subscriptions apply across all interfaces,
 *               but operations that claim a per-bus resource
 *               (LIN_RAW_MASTER, LIN_RAW_PUBLISH) require a non-zero
 *               ifindex and return -EOPNOTSUPP otherwise.
 * @lin_addr:    protocol specific address information (reserved)
 */
struct sockaddr_lin {
	__kernel_sa_family_t lin_family;
	int		lin_ifindex;
	union {
		/* Reserved for future LIN protocols (e.g. a transport layer
		 * per ISO 17987-2). The size matches the 16-byte reservation
		 * in struct sockaddr_can so that future protocols can add
		 * addressing members without growing struct sockaddr_lin and
		 * breaking its ABI.
		 */
		__u8 __reserved[16];
	} lin_addr;
};

/*
 * Filter flags with special meaning inside struct lin_filter.flags.
 * These bits do not overlap LIN_F_* frame flag bits.
 */
#define LIN_FILT_INV	0x80	/* invert the filter match */

/**
 * struct lin_filter - LIN frame filter for subscribe sockets
 * @lin_id:     6-bit frame ID bits of interest (0..LIN_ID_MASK).
 *              Bits 6 and 7 must be zero — pass the raw ID, not
 *              the on-wire Protected ID. Filters with bits set
 *              above LIN_ID_MASK are rejected with -EINVAL.
 * @id_mask:    mask over @lin_id (LIN_ID_MASK to match a specific
 *              ID, 0x00 to match any ID). Bits above LIN_ID_MASK
 *              must also be zero and are similarly rejected.
 * @flags:      LIN_F_* bits of interest; optionally LIN_FILT_INV to
 *              invert the match
 * @flags_mask: mask over the LIN_F_* portion of @flags (LIN_FILT_INV
 *              is not affected by this mask)
 * @__res:      trailing reserved bytes, must be all zero on write.
 *              Sized to absorb a future small per-filter field (e.g.
 *              priority, callback id, match counter) without growing
 *              the struct or requiring a v2 filter shape; the kernel
 *              rejects nonzero bytes with -EINVAL.
 *
 * Description:
 * A filter matches when both of the following hold, and the result is
 * then inverted if LIN_FILT_INV is set in @flags:
 *
 *   (received_lin_id & id_mask)    == (lin_id & id_mask)
 *   (received_flags  & flags_mask) == (flags  & flags_mask)
 *
 * struct lin_filter is for data frames only. Error frames are
 * subscribed separately via LIN_RAW_ERR_FILTER (an error-class
 * bitmask), not through this filter shape — the kernel routes data
 * and error frames to disjoint subscriber lists.
 *
 * To receive every data frame for a specific ID regardless of
 * checksum type, set lin_id to the ID, id_mask to 0x3F, and
 * flags_mask to 0. To match only enhanced-checksum data frames for
 * that ID, set flags to LIN_F_CHK_ENH and flags_mask to LIN_F_CHK_ENH.
 */
struct lin_filter {
	__u8	lin_id;
	__u8	id_mask;
	__u8	flags;
	__u8	flags_mask;
	__u8	__res[4];
};

#endif /* !_UAPI_LINUX_LIN_H */
