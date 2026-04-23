/* SPDX-License-Identifier: (GPL-2.0 OR BSD-3-Clause) */
/*
 * linux/lin/skb.h
 *
 * SKB shape helpers for LIN frames.
 *
 * Author: Kyle Bader <kyle.bader94@gmail.com>
 * Copyright (c) 2026 Kyle Bader
 */

#ifndef _LIN_SKB_H
#define _LIN_SKB_H

#include <linux/if_ether.h>
#include <linux/skbuff.h>
#include <linux/string.h>
#include <linux/lin.h>
#include <linux/lin/error.h>

/**
 * lin_is_lin_skb - validate that an skb carries a well-formed LIN frame
 * @skb: the skb to inspect
 *
 * Checks total length, flag-bit validity, reserved-field zeroing,
 * and the data/error-frame invariants expressed by the UAPI:
 *
 *   * Data frames (LIN_F_ERR clear): err_mask must be zero, lin_id
 *     must be a 6-bit ID (0..LIN_ID_MASK), len must be 1..LIN_MAX_DLEN
 *     per the LIN specification.
 *   * Error frames (LIN_F_ERR set): err_mask must be non-zero, lin_id
 *     must be either a 6-bit ID or LIN_ID_NONE, len may be 0..LIN_MAX_DLEN
 *     (context bytes, zero when no context).
 *
 * Called by the core on rx (lin_rcv) to keep buggy drivers and
 * forwarding paths from reaching subscriber dispatch with malformed
 * frames. Drivers do not need to call this themselves — the core
 * validates every skb arriving via netif_rx() before dispatch.
 *
 * Return: true if the skb is a well-formed LIN frame, false otherwise.
 */
static inline bool lin_is_lin_skb(const struct sk_buff *skb)
{
	const struct lin_frame *lf;

	if (skb->len != LIN_MTU)
		return false;

	if (skb_headlen(skb) < sizeof(struct lin_frame))
		return false;

	lf = (const struct lin_frame *)skb->data;

	if (lf->__pad)
		return false;

	if (memchr_inv(lf->__res, 0, sizeof(lf->__res)))
		return false;

	if (lf->flags & ~(LIN_F_ERR | LIN_F_CHK_ENH))
		return false;

	if (lf->len > LIN_MAX_DLEN)
		return false;

	if (lf->flags & LIN_F_ERR) {
		if (!lf->err_mask)
			return false;
		if (lf->lin_id != LIN_ID_NONE &&
		    (lf->lin_id & ~LIN_ID_MASK))
			return false;
	} else {
		if (lf->err_mask)
			return false;
		if (lf->lin_id & ~LIN_ID_MASK)
			return false;
		if (lf->len < 1)
			return false;
	}

	return true;
}

#endif /* _LIN_SKB_H */
