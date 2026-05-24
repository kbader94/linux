// SPDX-License-Identifier: GPL-2.0
/*
 * PF_LIN selftests: event-triggered slots and collision resolution.
 *
 * Includes regression coverage for the responder-tagging fix: an event
 * response is owner-tagged by the answering frame's ID (resp_id), not the
 * trigger, so RECV_OWN_MSGS works for the answering publisher and the
 * response survives even when the master has loopback disabled.
 */
#include "lin_harness.h"

/* Load CR schedule (handle 1) of unconditional @members, an event schedule
 * (handle 2) whose slot triggers @trigger -> cr_handle 1, and activate 2.
 */
static int setup_event(int m, __u8 trigger, const __u8 *members, unsigned int n)
{
	union lin_sched_buf cr = {}, ev = {};
	unsigned int i;
	int ret;

	cr.s.handle = 1;
	cr.s.entry_count = n;
	cr.s.default_slot_us = LIN_SLOT_US;
	for (i = 0; i < n; i++) {
		cr.s.entry[i].type = LIN_SCHED_TYPE_UNCOND;
		cr.s.entry[i].member_count = 1;
		cr.s.entry[i].members[0] = members[i];
	}
	ret = lin_sched_load(m, &cr, n);
	if (ret)
		return ret;

	ev.s.handle = 2;
	ev.s.entry_count = 1;
	ev.s.default_slot_us = LIN_SLOT_US;
	ev.s.entry[0].type = LIN_SCHED_TYPE_EVENT;
	ev.s.entry[0].member_count = 1;
	ev.s.entry[0].members[0] = trigger;
	ev.s.entry[0].cr_handle = 1;
	ret = lin_sched_load(m, &ev, 1);
	if (ret)
		return ret;
	return lin_sched_activate(m, 2);
}

#define TRIGGER	0x30

FIXTURE(lin_event) {
	int ifindex;
	__u32 caps;
	int m;		/* master */
	int s;		/* subscriber */
};

FIXTURE_SETUP(lin_event)
{
	self->ifindex = lin_setup_iface(LIN_IF);
	if (self->ifindex <= 0)
		SKIP(return, "need root and the vlin module (ip link add type vlin)");
	ASSERT_EQ(0, lin_query_caps(self->ifindex, &self->caps));
	LIN_SKIP_UNLESS_CAP(self->caps, LIN_CAP_EVENT);
	self->m = lin_open_bound(self->ifindex);
	ASSERT_GE(self->m, 0);
	ASSERT_EQ(0, lin_master(self->m, 1));
	self->s = lin_open_bound(self->ifindex);
	ASSERT_GE(self->s, 0);
}

FIXTURE_TEARDOWN(lin_event)
{
	if (self->m > 0)
		close(self->m);
	if (self->s > 0)
		close(self->s);
}

TEST_F(lin_event, silent_without_fresh_data)
{
	__u8 members[1] = { 0x10 };

	/* No publisher for 0x10 => nothing is fresh => event slot is silent. */
	ASSERT_EQ(0, setup_event(self->m, TRIGGER, members, 1));
	EXPECT_TRUE(lin_silent(self->s, LIN_SILENCE_MS));
}

TEST_F(lin_event, single_responder_under_trigger)
{
	__u8 members[1] = { 0x10 };
	__u8 d[2] = { 0x10, 0x99 };	/* byte0 carries the answerer's PID */
	struct lin_frame f;

	ASSERT_EQ(0, lin_publish(self->m, 0x10, d, 2, 0));
	ASSERT_EQ(0, setup_event(self->m, TRIGGER, members, 1));

	/* Exactly one fresh member => emitted under the trigger ID. */
	ASSERT_EQ(1, lin_recv(self->s, &f, LIN_RECV_MS));
	EXPECT_EQ(TRIGGER, f.lin_id);
	EXPECT_EQ(0, f.flags & (LIN_F_ERR | LIN_F_EVENT_COLLISION));
	EXPECT_EQ(0x10, f.data[0]);

	/* The dirty flag was consumed: further event polls stay silent rather
	 * than re-emitting the stale response.
	 */
	EXPECT_TRUE(lin_silent(self->s, LIN_SILENCE_MS));

	/* Refreshing the response re-arms the responder. */
	ASSERT_EQ(sizeof(struct lin_frame), lin_write(self->m, 0x10, d, 2, 0));
	ASSERT_EQ(1, lin_recv(self->s, &f, LIN_RECV_MS));
	EXPECT_EQ(TRIGGER, f.lin_id);
	EXPECT_EQ(0x10, f.data[0]);
}

/* Regression: the answering publisher is owner-tagged by its own ID, so it
 * receives its own event response with RECV_OWN_MSGS enabled.
 */
TEST_F(lin_event, responder_recv_own_msgs)
{
	__u8 members[1] = { 0x10 };
	__u8 d[2] = { 0x10, 0x99 };
	struct lin_frame f;
	int p = lin_open_bound(self->ifindex);

	ASSERT_GE(p, 0);
	ASSERT_EQ(0, lin_setopt_int(p, LIN_RAW_RECV_OWN_MSGS, 1));
	ASSERT_EQ(0, lin_publish(p, 0x10, d, 2, 0));	/* p owns 0x10 */
	ASSERT_EQ(0, setup_event(self->m, TRIGGER, members, 1));

	/* p must see its own response, carried under the trigger ID. */
	ASSERT_EQ(1, lin_recv(p, &f, LIN_RECV_MS));
	EXPECT_EQ(TRIGGER, f.lin_id);
	EXPECT_EQ(0x10, f.data[0]);
	close(p);
}

/* Regression: with the master's loopback disabled, the answering
 * publisher's vote still keeps the synthesised frame alive, so a third
 * subscriber receives the event response.
 */
TEST_F(lin_event, master_loopback_off_still_delivers)
{
	__u8 members[1] = { 0x10 };
	__u8 d[2] = { 0x10, 0x99 };
	struct lin_frame f;
	int p = lin_open_bound(self->ifindex);

	ASSERT_GE(p, 0);
	ASSERT_EQ(0, lin_setopt_int(self->m, LIN_RAW_LOOPBACK, 0));
	ASSERT_EQ(0, lin_publish(p, 0x10, d, 2, 0));	/* p loopback on (default) */
	ASSERT_EQ(0, setup_event(self->m, TRIGGER, members, 1));

	ASSERT_EQ(1, lin_recv(self->s, &f, LIN_RECV_MS));
	EXPECT_EQ(TRIGGER, f.lin_id);
	EXPECT_EQ(0x10, f.data[0]);
	close(p);
}

TEST_F(lin_event, collision_then_resolution)
{
	union lin_sched_buf cr = {}, ev = {};
	__u8 d0[2] = { 0x10, 0xa0 };
	__u8 d1[2] = { 0x11, 0xb0 };
	__u8 d2[1] = { 0x20 };
	struct lin_frame f;

	ASSERT_EQ(0, lin_publish(self->m, 0x10, d0, 2, 0));
	ASSERT_EQ(0, lin_publish(self->m, 0x11, d1, 2, 0));
	ASSERT_EQ(0, lin_publish(self->m, 0x20, d2, 1, 0));

	/* CR schedule (handle 1): the two colliding members, polled in order. */
	cr.s.handle = 1;
	cr.s.entry_count = 2;
	cr.s.default_slot_us = LIN_SLOT_US;
	cr.s.entry[0].type = LIN_SCHED_TYPE_UNCOND;
	cr.s.entry[0].member_count = 1;
	cr.s.entry[0].members[0] = 0x10;
	cr.s.entry[1].type = LIN_SCHED_TYPE_UNCOND;
	cr.s.entry[1].member_count = 1;
	cr.s.entry[1].members[0] = 0x11;
	ASSERT_EQ(0, lin_sched_load(self->m, &cr, 2));

	/* Event schedule (handle 2): the event trigger slot, then a plain
	 * unconditional 0x20 slot. After the collision diversion resolves, the
	 * engine must resume at the saved slot (this 0x20 slot) — proving the
	 * interrupted schedule continues rather than merely halting.
	 */
	ev.s.handle = 2;
	ev.s.entry_count = 2;
	ev.s.default_slot_us = LIN_SLOT_US;
	ev.s.entry[0].type = LIN_SCHED_TYPE_EVENT;
	ev.s.entry[0].member_count = 1;
	ev.s.entry[0].members[0] = TRIGGER;
	ev.s.entry[0].cr_handle = 1;
	ev.s.entry[1].type = LIN_SCHED_TYPE_UNCOND;
	ev.s.entry[1].member_count = 1;
	ev.s.entry[1].members[0] = 0x20;
	ASSERT_EQ(0, lin_sched_load(self->m, &ev, 2));
	ASSERT_EQ(0, lin_sched_activate(self->m, 2));

	/* 1. The collision notification comes first, under the trigger ID —
	 *    before any CR response (an engine emitting CR frames ahead of the
	 *    collision would fail here).
	 */
	ASSERT_EQ(1, lin_recv(self->s, &f, LIN_RECV_MS));
	EXPECT_EQ(TRIGGER, f.lin_id);
	EXPECT_NE(0, f.flags & LIN_F_EVENT_COLLISION);

	/* 2. The CR schedule resolves the collision, each member answering in
	 *    its own unconditional slot, in CR order: 0x10 then 0x11.
	 */
	ASSERT_EQ(1, lin_recv(self->s, &f, LIN_RECV_MS));
	EXPECT_EQ(0x10, f.lin_id);
	EXPECT_EQ(0, f.flags & LIN_F_EVENT_COLLISION);
	ASSERT_EQ(1, lin_recv(self->s, &f, LIN_RECV_MS));
	EXPECT_EQ(0x11, f.lin_id);
	EXPECT_EQ(0, f.flags & LIN_F_EVENT_COLLISION);

	/* 3. The engine resumes the interrupted schedule at the saved slot: the
	 *    0x20 slot that followed the event slot fires next. Silence here
	 *    would also pass if the engine simply stopped, and a stuck CR cycle
	 *    would re-emit 0x10 — requiring 0x20 proves a real resume.
	 */
	ASSERT_EQ(1, lin_recv(self->s, &f, LIN_RECV_MS));
	EXPECT_EQ(0x20, f.lin_id);
	EXPECT_EQ(0, f.flags & LIN_F_EVENT_COLLISION);
}

/* Regression for the activation-while-diverted timeout: during a collision
 * diversion the armed delay is the (long) event slot while the active
 * schedule is the (short) CR table. A per-active-schedule activate timeout
 * would expire before the boundary; the engine must instead bound the wait
 * by the maximum slot, so activating another handle still succeeds.
 */
TEST_F(lin_event, activation_during_diversion)
{
	union lin_sched_buf cr = {}, ev = {}, tgt = {};
	__u8 d0[2] = { 0x10, 0xa0 }, d1[2] = { 0x11, 0xb0 }, d4[1] = { 0x40 };
	struct lin_frame f;
	int i, got_collision = 0, got_target = 0, h = -2;

	/* Two fresh members force a collision on the event poll. */
	ASSERT_EQ(0, lin_publish(self->m, 0x10, d0, 2, 0));
	ASSERT_EQ(0, lin_publish(self->m, 0x11, d1, 2, 0));
	ASSERT_EQ(0, lin_publish(self->m, 0x21, d4, 1, 0));

	/* CR schedule (handle 1): short slots. */
	cr.s.handle = 1;
	cr.s.entry_count = 2;
	cr.s.default_slot_us = LIN_SLOT_US;
	cr.s.entry[0].type = LIN_SCHED_TYPE_UNCOND;
	cr.s.entry[0].member_count = 1;
	cr.s.entry[0].members[0] = 0x10;
	cr.s.entry[1].type = LIN_SCHED_TYPE_UNCOND;
	cr.s.entry[1].member_count = 1;
	cr.s.entry[1].members[0] = 0x11;
	ASSERT_EQ(0, lin_sched_load(self->m, &cr, 2));

	/* Plain target schedule (handle 0). */
	tgt.s.handle = 0;
	tgt.s.entry_count = 1;
	tgt.s.default_slot_us = LIN_SLOT_US;
	tgt.s.entry[0].type = LIN_SCHED_TYPE_UNCOND;
	tgt.s.entry[0].member_count = 1;
	tgt.s.entry[0].members[0] = 0x21;
	ASSERT_EQ(0, lin_sched_load(self->m, &tgt, 1));

	/* Event schedule (handle 2) with a deliberately long event slot. */
	ev.s.handle = 2;
	ev.s.entry_count = 1;
	ev.s.default_slot_us = 300000;		/* 300 ms keeps the divert open */
	ev.s.entry[0].type = LIN_SCHED_TYPE_EVENT;
	ev.s.entry[0].member_count = 1;
	ev.s.entry[0].members[0] = TRIGGER;
	ev.s.entry[0].cr_handle = 1;
	ASSERT_EQ(0, lin_sched_load(self->m, &ev, 1));
	ASSERT_EQ(0, lin_sched_activate(self->m, 2));

	/* Wait for the collision; the engine is now diverted and armed for the
	 * long event slot.
	 */
	for (i = 0; i < 8; i++) {
		if (lin_recv(self->s, &f, LIN_RECV_MS) != 1)
			break;
		if (f.lin_id == TRIGGER && (f.flags & LIN_F_EVENT_COLLISION)) {
			got_collision = 1;
			break;
		}
	}
	ASSERT_TRUE(got_collision);

	/* Switching to another handle now must succeed, not time out. */
	EXPECT_EQ(0, lin_sched_activate(self->m, 0));

	/* And the engine must actually be running the target schedule, not
	 * still stuck in the diverted CR table: it should now emit 0x21.
	 */
	ASSERT_EQ(0, lin_sched_active(self->m, &h));
	EXPECT_EQ(0, h);
	for (i = 0; i < 16; i++) {
		if (lin_recv(self->s, &f, LIN_RECV_MS) != 1)
			break;
		if (f.lin_id == 0x21) {
			got_target = 1;
			break;
		}
	}
	EXPECT_TRUE(got_target);
}

TEST_HARNESS_MAIN
