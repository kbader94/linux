/* SPDX-License-Identifier: ((GPL-2.0-only WITH Linux-syscall-note) OR BSD-3-Clause) */
/*
 * linux/lin.h
 *
 * Definitions for LIN network layer (socket addr / LIN frame / LIN filter)
 *
 * Author: Kyle Bader <kyle.bader94@example.com>
 * 
 * Based off of the existing linux CAN implementation by Oliver Hartkopp <oliver.hartkopp@volkswagen.de>
 * 
 * Inspired by the linux-lin project at https://github.com/lin-bus/linux-lin by Pavel Pisa <pisa@cmp.felk.cvut.cz>
 * 
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the author nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * Alternatively, provided that this notice is retained in full, this
 * software may be distributed under the terms of the GNU General
 * Public License ("GPL") version 2, in which case the provisions of the
 * GPL apply INSTEAD OF those given above.
 *
 * The provided data structures and external interfaces from this code
 * are not restricted to be used by modules with a GPL compatible license.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
 * DAMAGE.
 */

#ifndef _UAPI_LIN_H
#define _UAPI_LIN_H

#include <linux/types.h>
#include <linux/socket.h>
#include <linux/stddef.h> /* for offsetof */

/* Local Interconnect Network (LIN) kernel definitions */

/* LIN frame identifier */
typedef __u8 lin_pid_t;

/* LIN error mask */
typedef __u32 lin_err_mask_t;

/* LIN payload length and DLC definitions */
#define LIN_MAX_DLC 8
#define LIN_MAX_DLEN 8

/**
 * struct lin_frame - LIN frame structure
 * @lin_id:  LIN ID of the frame
 * @len:     LIN frame payload length in bytes (0 .. 8)
 * @data:    LIN frame payload (up to 8 bytes)
 */
struct lin_frame {
	lin_pid_t lin_pid;  /* 8 bit LIN_ID */
	__u8    len;     /* frame payload length in bytes */
	__u8    data[LIN_MAX_DLEN] __attribute__((aligned(8)));
};

/* LIN protocol versions */
#define LIN_PROTOCOL_1_3 1 /* protocol version 1.3 */
#define LIN_PROTOCOL_2_0 2 /* protocol version 2.0 */
#define LIN_PROTOCOL_2_1 3 /* protocol version 2.1 */
#define LIN_PROTOCOL_2_2 4 /* protocol version 2.2 */
#define LIN_PROTOCOL_J2602 5 /* SAE J2602 */

/* LIN node types */
#define LIN_NODE_TYPE_MASTER 1 /* Node is master */
#define LIN_NODE_TYPE_SLAVE 2 /* Node is slave */

/**
 * struct sockaddr_lin - the sockaddr structure for LIN sockets
 * @lin_family:  address family number AF_LIN.
 * @lin_ifindex: LIN network interface index.
 * @lin_addr:    protocol specific address information
 */
struct sockaddr_lin {
	__kernel_sa_family_t lin_family;
	int         lin_ifindex;
	union {
		/* reserved for future LIN protocols address information */
	} lin_addr;
};

/**
 * struct lin_filter - LIN ID based filter in lin_register().
 * @lin_id:   relevant bits of LIN ID which are not masked out.
 * @lin_mask: LIN mask (see description)
 *
 * Description:
 * A filter matches, when
 *
 *          <received_lin_id> & mask == lin_id & mask
 *
 * The filter can be inverted (LIN_INV_FILTER bit set in lin_id) or it can
 * filter for error message frames.
 */
struct lin_filter {
	linid_t lin_id;
	linid_t lin_mask;
};

#define LIN_INV_FILTER 0x80U /*  to be set in lin_filter.lin_id */

#endif /* !_UAPI_LIN_H */
