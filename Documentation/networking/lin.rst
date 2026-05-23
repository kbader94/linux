.. SPDX-License-Identifier: GPL-2.0

=========================================
PF_LIN - Local Interconnect Network (LIN)
=========================================

Overview
========

LIN (Local Interconnect Network, ISO 17987) is a low-cost, single-master
serial bus widely used in automotive body electronics for subsystems where
CAN is oversized: door and seat modules, window lifts, climate flaps, rain
and light sensors, and similar. One master node drives the bus by sending
frame *headers* according to a *schedule*; slave nodes supply the *response*
that follows a header.

PF_LIN provides socket-based access to LIN interfaces, following the same
model as SocketCAN (PF_CAN): LIN controllers are network interfaces of type
``ARPHRD_LIN``, and applications exchange frames and configure bus policy
through the BSD socket API. Programmers familiar with SocketCAN should find
the API immediately recognisable; the LIN-specific differences are called
out throughout this document and summarised at the end.

The kernel side splits cleanly: the LIN core validates userspace requests,
enforces cross-socket policy (single master, single publisher per ID,
schedule reference integrity), and forwards the resulting state to the bound
driver. The *driver* owns the timing-sensitive work - executing the schedule
and holding the hardware response table - because real LIN controllers
implement that autonomously. See `Network drivers`_ below.

Addressing and sockets
======================

A raw LIN socket is created with::

    int s = socket(PF_LIN, SOCK_RAW, LIN_RAW);

and bound to an interface with ``struct sockaddr_lin``::

    struct sockaddr_lin addr = {
        .lin_family  = AF_LIN,
        .lin_ifindex = if_nametoindex("vlin0"),
    };
    bind(s, (struct sockaddr *)&addr, sizeof(addr));

A ``lin_ifindex`` of 0 binds across every LIN interface in the socket's
network namespace as a passive *observer*: receive filters and error/wakeup
subscriptions apply to all interfaces, but operations that claim a per-bus
resource (master role, publisher ownership, schedules) require a non-zero
ifindex and otherwise return ``-EOPNOTSUPP``.

LIN frames
==========

Frames are exchanged as ``struct lin_frame``::

    struct lin_frame {
        __u8  lin_id;       /* 6-bit frame ID (no parity) */
        __u8  flags;        /* LIN_F_* */
        __u8  len;          /* payload length */
        __u8  __pad;
        __u32 err_mask;     /* LIN_ERR_* on error frames */
        __u8  data[LIN_MAX_DLEN];
        __u8  __res[8];
    };

The on-wire header/response split is resolved by the kernel and is not
visible here: a received slot is delivered as one ``lin_frame`` regardless of
whether the master or a slave supplied the response. The protected-identifier
parity bits and the checksum byte are likewise handled below the socket -
parity is computed/verified by the kernel, and the checksum is validated (or
its failure reported as an error frame) by the driver.

``lin_id`` is the 6-bit ID (``0..LIN_ID_MASK``). ``0x3C``/``0x3D`` are the
diagnostic IDs, ``0x3E``/``0x3F`` are reserved. ``flags`` carries:

``LIN_F_ERR``
    Error frame. ``err_mask`` is an OR of ``LIN_ERR_*`` classes (see
    ``<linux/lin/error.h>``); ``data`` carries optional class context.
``LIN_F_CHK_ENH``
    Enhanced checksum was/should be used (vs. classic).
``LIN_F_WAKEUP``
    Bus wakeup signal (see `Wakeup and sleep`_).
``LIN_F_EVENT_COLLISION``
    Event-triggered slot collision notification (see
    `Event-triggered frames`_).

Receiving frames
================

A newly bound socket receives every non-error data frame on its interface
(a default match-all filter). Subscriptions are configured per socket:

``LIN_RAW_FILTER``
    Install an array of ``struct lin_filter`` (max ``LIN_RAW_FILTER_MAX``).
    A frame matches when ``(lin_id & id_mask) == (filter.lin_id & id_mask)``
    and the ``flags_mask`` bits match; ``LIN_FILT_INV`` inverts a filter.
    Setting ``optlen == 0`` **clears** all data filters - the socket then
    receives no data frames (matching ``CAN_RAW_FILTER``). This is the way a
    master/publisher socket opts out of the default match-all subscription.

``LIN_RAW_ERR_FILTER``
    A ``__u32`` mask of ``LIN_ERR_*`` classes to receive as error frames.
    Default 0 (no error frames).

``LIN_RAW_WAKEUP_FILTER``
    Boolean; subscribe to ``LIN_F_WAKEUP`` frames. Wakeup frames route
    through a dedicated list, disjoint from data and error frames.

``LIN_RAW_JOIN_FILTERS``
    Boolean; AND-combine the filter set (a frame must match every filter)
    instead of the default OR.

``LIN_RAW_LOOPBACK`` / ``LIN_RAW_RECV_OWN_MSGS``
    See `Loopback`_.

The master and publisher model
==============================

LIN is single-master. Exactly one socket per interface may hold the master
role; it is claimed and released with ``LIN_RAW_MASTER``::

    int on = 1;
    setsockopt(s, SOL_LIN_RAW, LIN_RAW_MASTER, &on, sizeof(on));

Slaves (and a master that also sources data) register *publisher* responses.
A publisher owns a frame ID and supplies the bytes transmitted whenever that
ID's header is seen. At most one socket may publish a given ID on a given
interface (``-EBUSY`` otherwise)::

    struct lin_publish pub = {
        .lin_id = 0x10, .len = 2, .data = { 0x12, 0x34 },
    };
    setsockopt(s, SOL_LIN_RAW, LIN_RAW_PUBLISH, &pub, sizeof(pub));

Note: an event-triggered frame's trigger ID is not publisher-owned - no
socket registers a response for it; it is only a poll header. Each responder
in the group publishes its own associated unconditional frame ID, under the
normal one-publisher-per-ID rule (see `Event-triggered frames`_).

``LIN_RAW_UNPUBLISH`` releases an owned ID.

.. _sendmsg-upsert:

write()/sendmsg() updates a response, does not transmit a frame
---------------------------------------------------------------

This is the most important deviation from ``CAN_RAW``. On a LIN bus the
*schedule* decides when a frame's header goes out; a publisher only supplies
the response bytes. Accordingly, ``write()``/``sendmsg()`` of a
``struct lin_frame`` does **not** put a frame on the bus. It upserts this
socket's sticky publisher response for ``frame.lin_id`` (identical to
``LIN_RAW_PUBLISH`` with the frame's content), and the bytes are transmitted
later, when the running schedule reaches that ID::

    /* refresh the response for ID 0x10 at the sensor's sample rate */
    struct lin_frame f = { .lin_id = 0x10, .len = 2 };
    while (read_sensor(f.data)) write(s, &f, sizeof(f));


Schedules
=========

The master drives the bus by running a *schedule table*: an ordered list of
slots, each firing one header and waiting a per-slot duration (``slot_us``).
An application may pre-load several schedules (identified by caller-assigned
handle, ``0..LIN_RAW_SCHEDULES_MAX-1``) and switch between them - e.g. a
"normal" schedule and a "diagnostic" schedule.

Each ``struct lin_schedule_entry`` has a ``type``:

``LIN_SCHED_TYPE_UNCOND``
    One ID; its header fires every cycle.

``LIN_SCHED_TYPE_SPORADIC``
    One or more member IDs in priority order (``members[0]`` highest). The
    master fires the header for the highest-priority member whose publisher
    response has been *updated* since it was last sent; the slot stays silent
    when no member has fresh data.

``LIN_SCHED_TYPE_DIAG``
    A diagnostic slot (``0x3C``/``0x3D``). Requires the driver to advertise
    ``LIN_CAP_DIAG``.

``LIN_SCHED_TYPE_EVENT``
    An event-triggered frame; see `Event-triggered frames`_.

The schedule sockopts are::

    LIN_RAW_SCHEDULE_LOAD      /* upload/replace a schedule by handle */
    LIN_RAW_SCHEDULE_ACTIVATE  /* begin executing a loaded handle     */
    LIN_RAW_SCHEDULE_STOP      /* stop the active schedule            */
    LIN_RAW_SCHEDULE_DELETE    /* remove a loaded schedule by handle  */

Loading is permitted while a *different* handle is active; loading over the
active handle returns ``-EBUSY`` (stop it first). ``LIN_RAW_SCHEDULE_ACTIVATE``
with no argument can be read back with ``getsockopt`` to obtain the active
handle. Drivers will switch schedules at the end of the current slot,
so ``LIN_RAW_SCHEDULE_ACTIVATE`` will not interrupt the current frame transmission.

Sporadic frames
---------------

A sporadic slot models "send this frame only when its value changed." The
driver keeps a per-member *dirty* flag, set when the member's publisher
response is updated (``LIN_RAW_PUBLISH`` or ``write()``) and cleared when the
member is emitted. On each sporadic slot the highest-priority (lowest index)
dirty member is sent;
if none is dirty the slot is silent. The ``members[]`` index order is
the priority contract from userspace to the driver. Every sporadic member
must have a registered publisher at load time (``-EINVAL`` otherwise).

Event-triggered frames
----------------------

An event-triggered frame lets a group of slaves share one trigger header,
saving schedule bandwidth when updates are infrequent. A slave answers only
when its associated unconditional frame has fresh data; identifying itself
via the first response data byte (the protected ID of its own frame, a
cluster-design convention). Usually at most one slave answers, but **two or
more may answer in the same slot and collide** - that possibility is
intrinsic to event-triggered frames, and resolving it is what the
collision-resolving schedule is for.

An event slot therefore carries:

* ``members[0]`` - the event-trigger ID, and
* ``cr_handle`` - the handle of a separate *collision-resolving schedule*: a
  designer-authored schedule that lists (at least) the group's unconditional
  frames so each can be polled individually. It must contain **only**
  unconditional (``LIN_SCHED_TYPE_UNCOND``) entries - the core rejects an
  event slot whose ``cr_handle`` names a schedule with any other entry type
  (see below).

On a collision the master delivers a ``LIN_F_EVENT_COLLISION`` notification
(a non-error frame on the trigger ID), switches to the collision-resolving
schedule, runs it once so every member answers in its own unconditional
slot, then resumes the interrupted schedule.

.. _schedule-lifecycle:

Schedule reference ordering
--------------------------------------------

Because an event slot *references* another schedule by handle, the two must
be set up and torn down in dependency order. The rules:

* **Setup (load order):** the collision-resolving schedule must be loaded
  before the schedule that references it. Loading a schedule whose event
  slot names an unloaded ``cr_handle`` - or one that is not
  unconditional-only - returns ``-EINVAL``. (The unconditional-only rule
  also means a collision-resolving schedule can never contain an event slot,
  so a collision can never nest.)
* **Teardown (delete order):** a collision-resolving schedule cannot be
  deleted while any loaded schedule references it -
  ``LIN_RAW_SCHEDULE_DELETE`` returns ``-EBUSY``. Delete the referencing
  schedule first. (There is no cascade delete: a collision-resolving
  schedule may be shared by several referrers, so each handle is managed
  explicitly.)
* A schedule that is currently active cannot be deleted (``-EBUSY``; stop
  it first). A collision-resolving schedule referenced by *any* loaded
  schedule is pinned: it can be neither deleted nor replaced by reloading
  its handle (``-EBUSY`` either way), so its unconditional-only property
  cannot change behind a referrer. Drop the referrer first to edit it.

Putting it together, with handle 0 a normal schedule, handle 1 a
collision-resolving schedule, and handle 2 a schedule containing an event
slot that references handle 1::

    /* setup: collision-resolving schedule (1) before its referrer (2) */
    load(handle=1, <unconditional frames of the event group>);
    load(handle=2, <... event slot with cr_handle=1 ...>);   /* needs 1 loaded */
    load(handle=0, <normal schedule>);

    activate(handle=2);     /* run the event schedule */
    ...
    activate(handle=0);     /* switch schedules at any time */
    ...

    /* teardown: stop, then delete referrer (2) before its CR schedule (1) */
    stop();                 /* or activate() a different handle */
    delete(handle=2);       /* delete(1) here would return -EBUSY */
    delete(handle=1);
    delete(handle=0);

For the common "I am finished" case none of this is required: closing the
socket (or releasing the master role, or bringing the interface down)
releases every schedule the socket loaded, in the right order, automatically.

Wakeup and sleep
================

``LIN_RAW_WAKEUP`` drives a bus wakeup pulse (any node may wake the bus, so
this needs no master role, only ``LIN_CAP_WAKEUP`` on the driver).
``LIN_RAW_WAKEUP_FILTER`` subscribes to wakeup events. A wakeup frame carries
no data (``lin_id == LIN_ID_NONE``, ``len == 0``).

``LIN_RAW_SLEEP`` sends the LIN go-to-sleep command - the master request
frame ``0x3C`` with payload ``{0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
0xFF}``. It requires the master role and that no schedule is active; it does
**not** require diagnostic-transport support.

Device lifecycle
================

A socket may be bound while the interface is administratively down. While it
is down, the role-agnostic operations (master, publish, unpublish, wakeup,
and ``write()``) return ``-ENETDOWN``. The master-gated operations (schedule
load/delete/activate/stop and sleep) instead return ``-EPERM``: the
master-role check runs before the link-state check, and no socket can hold
the role while down - the going-down drain (below) force-releases the claim
and ``LIN_RAW_MASTER`` itself returns ``-ENETDOWN`` while down, so the role
cannot be re-acquired until the interface is back up. Subscription-only
sockopts (filters, loopback flags) work regardless of link state.

When the interface goes down, the kernel quiesces LIN policy at
``NETDEV_GOING_DOWN`` - before the driver's stop callback - force-releasing
the master claim and publisher entries and stopping/deleting schedules, so a
policy operation never races the driver's ``ndo_stop()``. Filters, the
binding, and the netdev reference survive a down/up cycle (the socket remains
a passive observer), but per-bus policy state is **not** replayed: after the
interface returns, userspace must re-claim the master role, re-register
publishers, and re-load/activate schedules.

Loopback
========

Like SocketCAN, every emission is observable from every socket on the bus.
Because LIN frames are generated by the driver's schedule engine rather than
by a socket ``write()``, the LIN core synthesises a tagged loopback frame at
emission time and feeds it through the normal receive path. The synthesised
frame is owner-tagged so ``LIN_RAW_RECV_OWN_MSGS`` (default off - do not
deliver a socket its own emissions) and ``LIN_RAW_LOOPBACK`` (default on -
make emissions visible to other sockets) behave as in ``CAN_RAW``. Drivers
must not also deliver their own hardware readback/echo; the core synth is the
single source of owner-tagged loopback.

Network drivers
===============

A LIN driver registers a network interface of type ``ARPHRD_LIN`` and
implements ``struct lin_dev_ops`` - the contract through which the core
forwards validated policy (master start/stop, response table updates,
schedule load/activate/stop/delete, the one-shot header used for the sleep
command, and the wakeup pulse). The driver advertises the optional features
it supports via ``LIN_CAP_*`` flags. Hardware drivers allocate with
``alloc_lindev()`` and register with ``lin_register_netdev()`` (mirroring
``alloc_candev()`` / ``register_candev()``).

Capability advertisement (rtnetlink)
------------------------------------

Drivers advertise their supported ``LIN_CAP_*`` set to userspace via
rtnetlink: the per-link uapi header ``<linux/lin/netlink.h>`` defines an
``IFLA_LIN_*`` attribute namespace carried inside ``IFLA_INFO_DATA``.
Today the only attribute is ``IFLA_LIN_CAPS`` - the supported-capability
bitmask. Userspace reads it with ``RTM_GETLINK`` (or ``ip -d link show
<if>``) and gates on it; the PF_LIN selftest suite uses this to skip
cap-gated cases cleanly when a driver does not implement the feature.
Every conforming LIN driver is expected to advertise this attribute.

Hardware drivers going through the standard ``alloc_lindev()`` /
``lin_register_netdev()`` path inherit advertisement automatically: the
LIN core registers a shared ``lin_link_ops`` at module init and
``alloc_lindev()`` assigns it to each new device, the same model
SocketCAN uses with ``can_link_ops``. Drivers that register their own
``rtnl_link_ops`` to support a userspace-creatable link type (vlin's
case, for ``ip link add type vlin``) plug
``lin_link_get_size()`` / ``lin_link_fill_info()`` into their own ops
instead - the rtnl core's per-kind ops take precedence at link-creation
time.

Bitrate, controller state, LIN xstats, and other per-link settings are
deliberately deferred - they will land alongside the first real (hardware)
driver that needs them, so the UAPI shape can be designed against a
concrete consumer rather than in the abstract.

Slave-publisher timing (``LIN_CAP_PUB_SLAVE``)
----------------------------------------------

A LIN slave that publishes a response to a header it observed on the wire
has only a small latency budget to put its first response byte on the
bus. The LIN spec requires the slave to respond within the frame slot.
However, the duration of the frame slot is not fixed by the spec;
it is determined by the cluster designer and can vary by implementation.
According to the LIN spec the maximum frame slot duration is
``T_Frame_Max = 1.4 * T_Frame_Nominal``. The actual slot duration
is typically rounded up from ``1.4 * T_Frame_Nominal`` to the nearest
integer multiple of the timebase tick, with typical timebase ticks of 5 or 10ms.

With a 5ms timebase tick, the worst case scenario is a frame with DLC of 2.
At 19200 baud:

T_Frame_Nominal = bits x bit_time
T_Frame_Nominal = 64 x 52.08us = 3.33ms
T_Frame_Max = 1.4 x 3.33ms = 4.66ms
T_Slot = 5ms (4.66ms rounded up to the nearest 5ms tick)

T_Frame_Allowable_Response_Delay = T_Slot - T_Frame_Nominal
T_Frame_Allowable_Response_Delay = 5ms - 3.33ms = 1.67ms

On UART-based LIN interfaces, the RX FIFO will timeout before triggering
the RX fill level interrupt. Most UARTS have a FIFO RX timeout equal to
4x the character time, which is 2.08ms at 19200 baud. This already exceeds
the LIN spec's maximum response delay for a 2 byte response on a 5ms timebase
tick(1.67ms). Therefore, it is necessary for UART-based LIN drivers to disable
the RX FIFO or set the fill level to 1 byte, so that the RX interrupt is
triggered immediately when the first response byte arrives,
giving the driver the best chance to meet the LIN spec's timing requirements.

Furthermore, CPU constrained devices (single core or heavy workload),
can have considerable scheduling latency(100s to 1000s of ms). Therefore,
the LIN core enforces a policy gate for slave publishers:
the ``LIN_CAP_PUB_SLAVE`` capability bit.

Drivers advertise the ``LIN_CAP_PUB_SLAVE`` flag when their transport
can realistically meet the slave response timing requirements;
the LIN core enforces it by rejecting ``LIN_RAW_PUBLISH`` from sockets that do
not also hold ``LIN_RAW_MASTER`` with ``-EOPNOTSUPP`` on devices without the cap.
i.e., slave responses are NOT allowed without the ``LIN_CAP_PUB_SLAVE`` flag.

The master-with-publish path (a socket that holds ``LIN_RAW_MASTER`` and
also publishes its own slot responses) is always allowed regardless of
this cap - master-side response timing is governed by the master's own
schedule, not by reception of an external header.

What's still supported on devices without ``LIN_CAP_PUB_SLAVE``:

* full master role (drives schedule, fires headers, publishes own slot
  responses)
* logger master (drives schedule for stimulus, observes externally
  sourced responses without publishing)
* bus observer (no master, no publish, just receive)

UART-based drivers (``sllin``, ``sdlin``) advertise this cap if
they can program the UART for single-byte RX interrupt latency, or FIFO disabled
outright, which together with the ``SCHED_FIFO`` schedule engine gives the
kthread the best chance to react inside the budget. This is queried through the
Universal FIFO Control framework (see
``Documentation/ABI/testing/sysfs-tty`` and the API in
``include/linux/serial_core.h``). UART drivers not yet ported to that
framework return ``-EOPNOTSUPP`` from ``uart_get_fifo_control()``;
sllin treats this as "transport cannot guarantee sub-frame RX latency"
and withholds the cap. To enable ``LIN_CAP_PUB_SLAVE`` on such a UART,
extend its driver to support the FIFO control framework, or see below
for override options.

USB serial bridges (FT232, CH340, PL2303, CP210x, etc.) have
inherent host-to-device latency on the order of 1-2 ms even in
low-latency mode and cannot meet the LIN timing window. They do not advertise
the cap, so they can only be used as master publishers or passive observers,
not as slave publishers.

On-chip LIN devices that handle bus timing internally advertise the cap
unconditionally; the timing constraint is satisfied by the device's
firmware, not by host-side interrupt latency.

Operator override
~~~~~~~~~~~~~~~~~

The ``LIN_CAP_PUB_SLAVE`` gate can be overridden on a per-link basis via
the ``IFLA_LIN_FORCE_PUB_SLAVE`` rtnetlink attribute::

    ip link set sllin0 type lin force_pub_slave on
    ip -d link show sllin0     # reports the override state

When set, the LIN core admits ``LIN_RAW_PUBLISH`` from non-master
sockets regardless of whether the driver advertised the cap. This is
intended for:

* **Use with a permissive master.** If a user determines that the master
  has sufficient slot timing to accommodate their high-latency slave publisher,
  they can enable this override to allow the slave to operate without the cap.
* **Exercising the slave-publisher code path** on host hardware whose
  UART has not yet been ported to the FIFO control framework.
  i.e., older 8250 or 16450 based UARTs that have no FIFO but have not been
  implemented into the FIFO control framework.
* **Custom protocols layered on the LIN PHY** that intentionally
  relax the spec's slot timing.

When the override is on, the slave's response may exceed the LIN spec's
slot-time bound. A strict master will discard the late frame, log a
``LIN_ERR_NO_RESPONSE`` for the missed slot, and possibly increment
its own slave-deadline-miss counter - none of which are observable from
the slave side. **Forcing the cap on against a strict master will
silently produce out-of-spec frames; the resulting behaviour is the
operator's problem.** The LIN core logs each transition at INFO level
so the choice is auditable in dmesg.

vlin - the virtual LIN interface
--------------------------------

``vlin`` is the software-only LIN interface, the analogue of ``vcan``. It has
no hardware: the interface is its own bus. A master socket's schedule drives
headers, registered publishers supply responses, and every resulting frame is
looped back to the sockets bound to the interface. The full master schedule
surface (unconditional, sporadic, diagnostic, and event-triggered slots,
including collision resolution), publisher responses, wakeup, and sleep are
all simulated in software; there is no baud-rate (byte) timing, but the
schedule's ``slot_us`` cadence is honoured.

Create an instance with::

    ip link add dev vlin0 type vlin
    ip link set vlin0 up

vlin is the recommended way to develop and test PF_LIN applications without
LIN hardware.

Interface statistics
--------------------

``ip -s link`` reports per-interface counters. Because ``vlin`` *is* the bus
rather than a single node, it counts every frame its schedule engine emits as
both one transmit and one receive - the same convention ``vcan`` uses for a
virtual loopback interface - so ``rx_packets``/``tx_packets`` track total bus
traffic. The count is taken at emission, so a frame the loopback gate drops
(no interested socket) still registers as having been on the wire.
``tx_dropped`` counts frames pushed down the ``ndo_start_xmit`` path (for
example an ``AF_PACKET`` injection), which LIN does not use for emission.

A real LIN node sits on the bus in a single role, and a per-node driver
attributes counters by role:

* **as master** it transmits headers and its own published responses, and
  receives the responses published by slaves;
* **as slave** it transmits only the responses it publishes, and receives the
  headers and other responses on the bus.

``vlin`` cannot make that split because one interface hosts the master and
every slave at once, so the aggregate count is the meaningful one for a
virtual bus.

Differences from SocketCAN
==========================

* ``write()``/``sendmsg()`` updates a sticky publisher response; it does not immediately
  transmit a frame (see :ref:`sendmsg-upsert`). The frame will be transmitted whenever
  a master transmits a header for that ID, according to the running schedule.
* ``struct lin_frame`` exposes ``lin_id``/``flags``/``len`` as separate
  fields rather than packing them, since LIN IDs are 6-bit.
* Cross-socket bus policy is first-class: a single-master claim, one
  publisher per ID, and kernel-managed, reference-tracked schedule tables -
  none of which CAN has.
* Frame emission timing is owned by the driver's schedule engine, not by the
  qdisc/``ndo_start_xmit`` path.
