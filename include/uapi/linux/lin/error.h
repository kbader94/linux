/* SPDX-License-Identifier: ((GPL-2.0-only WITH Linux-syscall-note) OR BSD-3-Clause) */
/*
 * linux/lin/error.h
 *
 * Error frame class definitions for LIN sockets.
 *
 * When a driver observes a bus or protocol fault it delivers a frame with
 * LIN_F_ERR set in struct lin_frame.flags. The classification is carried
 * in struct lin_frame.err_mask as an OR of one or more LIN_ERR_* bits;
 * a single frame may report multiple concurrent classes (for example a
 * checksum mismatch that was also framed incorrectly). struct
 * lin_frame.data carries optional class-specific context — for example
 * the received bytes that failed checksum — and struct lin_frame.len
 * indicates how many bytes of context are present.
 *
 * struct lin_frame.lin_id carries the PID the error was associated with
 * if known, or LIN_ID_NONE (see <linux/lin.h>) if the driver could not
 * attribute the error to a specific ID.
 *
 * The LIN_ERR_* values are single-bit masks in a 32-bit field so they
 * can be OR'd together both on the wire and when building a
 * LIN_RAW_ERR_FILTER subscription mask. Drivers should prefer an
 * existing class over inventing a new one; new classes must be appended
 * (next free bit) to preserve ABI compatibility.
 *
 * Author: Kyle Bader <kyle.bader94@gmail.com>
 * Copyright (c) 2026 Kyle Bader
 */

#ifndef _UAPI_LIN_ERROR_H
#define _UAPI_LIN_ERROR_H

#include <linux/lin.h>
#include <linux/types.h>

/*
 * Error class bits for struct lin_frame.err_mask and for the mask
 * argument to LIN_RAW_ERR_FILTER.
 */
#define LIN_ERR_SYNC		0x00000001U	/* break or sync field malformed       */
#define LIN_ERR_PID_PARITY	0x00000002U	/* received PID has invalid parity     */
#define LIN_ERR_CHECKSUM	0x00000004U	/* checksum byte mismatch on response  */
#define LIN_ERR_NO_RESPONSE	0x00000008U	/* header emitted; no response arrived
						 * within the expected slot
						 */
#define LIN_ERR_READBACK	0x00000010U	/* TX readback mismatch; likely bus
						 * collision or physical-layer issue
						 */
#define LIN_ERR_FRAMING		0x00000020U	/* UART framing error on a frame byte  */
#define LIN_ERR_OVERRUN		0x00000040U	/* UART / DMA overrun                  */
#define LIN_ERR_BUS		0x00000080U	/* generic bus error, transport-defined*/
/* 0x00000100U .. 0x80000000U reserved for future error classes */

/* Convenience mask matching every defined (and future) error class. */
#define LIN_ERR_MASK_ALL	((__u32)~0U)

#endif /* !_UAPI_LIN_ERROR_H */
