# Design: device-initiated events

> **Status: design, ready to argue with. Nothing built yet.**
> Follows [Research: device-initiated events](RESEARCH_DEVICE_INITIATED_EVENTS.md),
> which established that the protocol cannot express a vehicle-originated
> message. This document proposes the shape that fixes it.
> Raised: 2026-08-13.

## Two questions, and the second one comes first

The first question is how an event interface should look. The second, asked
alongside it, is whether this protocol can be used on a robot that has no ROS 2
anywhere. The second is answered first, because if the answer were no, the whole
arrangement would be a detour and micro-ROS would deserve another look.

## This protocol does not need ROS 2, and that is checkable

Nothing in this repository knows ROS 2 exists.

- `src/robot_wire_protocol.c` is freestanding C11: `stddef.h`, `stdint.h`, and
  no allocation.
- `python/robot_wire_protocol.py` imports `struct`, `dataclasses`, `enum` and
  `typing`. All of that is the standard library.
- `COLCON_IGNORE` at the root exists specifically to stop this becoming a ROS
  package by accident.

The proof is not the file listing, it is a working path that has no ROS in it.
The route A bench script drives the vehicle's lamps from plain Python with
`import zenoh` and `import robot_wire_protocol`, over the same bytes the
firmware decodes. A machine running that script needs no ROS installation, no
distribution, no workspace.

So the layering is:

```text
  ROS 2 nodes  <-->  gateway  <-->  Zenoh  <-->  vehicle
  \_________optional__________/     \______always present______/
```

A robot without ROS 2 is not a special case that has to be engineered. It is
the ordinary case with the leftmost box absent. Remove the gateway and the
vehicle keeps its full command and event surface; what disappears is only the
translation into ROS topics. Python scripts, a web dashboard, a test rig or a
different middleware can all consume the same frames, because the frames were
never ROS messages in the first place.

This is exactly the property micro-ROS trades away, and it is worth stating in
those terms because it is the sharpest form of the argument in
[Evaluation: micro-ROS](EVALUATION_MICRO_ROS.md). Under micro-ROS, ROS 2
interface definitions are compiled into the firmware binary. A robot without
ROS 2 would then be a robot whose firmware is built from ROS 2 packages, is
tied to a distribution's release cycle, and cannot talk to anything without an
agent. Not impossible, but absurd. The current split keeps "no ROS on this
robot" as a configuration, not a rewrite.

**Answer: yes, the wire protocol is usable with no ROS 2 present, and the
gateway is what makes ROS optional rather than required.**

## Decisions taken

### 1. The timestamp is vehicle uptime

Settled: no comparability with ROS time is required.

That removes the largest open question in the research. No clock
synchronisation, no time translation in the protocol, no dependence on a time
source the vehicle may not have. The field answers *how old is this and in what
order did these happen*, both of which are answerable from a monotonic counter
that starts at boot.

The consequence belongs in the gateway, so write it down: **the gateway stamps
`std_msgs/Header.stamp` at receipt**, using its own ROS clock, and treats the
vehicle uptime as an ordering and staleness field carried alongside. That is
honest about what is known. A gateway that wanted a better estimate could
subtract the transport delay it measures, but it should not pretend the vehicle
knows what time it is.

Width: `uint64` milliseconds. A `uint32` of milliseconds wraps after 49.7 days,
which is inside the plausible uptime of a vehicle left powered, and wrap
handling in the gateway is precisely the kind of rarely-executed code that is
wrong when it finally runs. Eight bytes buys the problem's removal.
`esp_timer_get_time()` already returns 64-bit microseconds, so the vehicle side
is a division rather than a cast.

### 2. `boot_id` is opaque and random

A `uint32` drawn from the hardware RNG at startup, **compared for equality
only, never ordered**.

The alternative is a counter persisted in NVS, which would let a host tell
which session is newer. That ordering is not needed: within a session, uptime
orders events; across a session boundary, the only question a host has is "is
my cached view from a previous life", and equality answers it. A persisted
counter would mean a flash write on every boot for information nobody uses.

Documented as opaque so that nobody later infers meaning from its value.

### 3. One event sequence counter for the whole vehicle

A `uint32`, incremented once per published event regardless of subsystem,
independent of the command sequence in the header.

Reusing the command sequence is exactly what failed on the bench and produced
`Ignored unmatched Lights state sequence 1`. A separate counter says plainly
that this is not a reply.

One counter rather than one per subsystem: with snapshot semantics a lost event
repairs itself, so this counter is diagnostics, and its useful question is "did
we lose anything at all". A single counter answers that across the whole stream
with one place in the firmware that increments. The cost is that a detected gap
does not say which subsystem it came from, which is acceptable for a field
whose job is to make loss visible rather than to recover it.

### 4. One message type per subsystem, sharing a common prefix

Not one general `STATE_EVENT` with a subsystem selector. The reason is in the
existing decoder:

```c
if (input_size != ROBOT_WIRE_HEADER_SIZE + (size_t)expected_payload_size) {
    return ROBOT_WIRE_INVALID_LENGTH;
}
```

Length is validated against a per-type constant **before any field is read**,
and `ARCHITECTURE.md` names that validation order as part of the contract. A
subsystem selector inside the payload would invert it: the decoder would have
to read a field to learn how long the payload should be, which means trusting
bytes it has not yet validated. Per-subsystem types keep every payload a fixed,
known size, keep each one independently versionable, and cost nothing but
numbers out of a `uint16` space that has used six.

They share a **16-byte event prefix** at the start of every event payload, so
common handling stays common:

```text
 0.. 7   uptime_ms        uint64   milliseconds since vehicle boot
 8..11   event_sequence   uint32   monotonic, all subsystems, one counter
12..15   boot_id          uint32   opaque; equality only
```

Sixteen bytes, naturally aligned after the 16-byte header.

### 5. No protocol version bump

The header does not change, and adding message types is backward compatible.
This is not an assumption; it is asserted by a test that already exists:

```python
(GOLDEN[:6] + bytes([0x09, 0x00]) + GOLDEN[8:], "INVALID_MESSAGE_TYPE"),
```

An unknown type is rejected cleanly and loudly. An old gateway meeting a new
event logs a rejection rather than misreading it, which is the behaviour the
version field was put there for.

**One trap that follows from this**: that test uses type **9** as its stand-in
for "unknown". The moment the number space reaches 9, the test stops testing
what it claims to. It must move to a number that is unassigned and will stay
unassigned. `tests/test_robot_wire_protocol.c` should be checked for the same
pattern before any type is added.

Types 1 and 2 are unassigned and are left that way. An unexplained gap is not
worth reclaiming; the safe rule is that a number is never reused.

### 6. Snapshots, published on change and periodically

Restated from the research because it drives the payloads: events carry
complete state, not edges. A lost snapshot is repaired by the next one; a lost
edge is lost permanently.

Proposed rates, which is question 5 from the research:

- **On change**, rate-limited to at most one event per subsystem per **100 ms**.
- **Unconditionally every 1000 ms**, whether or not anything changed.

The ceiling bounds staleness after a loss to about a second. The floor stops a
chattering input flooding a shared router: worst case ten frames per second per
subsystem, of tens of bytes each, which is nothing.

### 7. A `changed` mask, which is how snapshots keep short pulses

This is the answer to the one research question that snapshots do not settle on
their own: *which inputs need edge accuracy?*

Rather than deciding that per input now, the pinout payload carries a **sticky
mask of everything that changed since the previous event**, cleared each time an
event is published. A pulse that rises and falls entirely between two
publications still shows up: the level bits look unchanged, and the changed bit
is set.

That gives "something happened here" for every input at four extra bytes,
without per-input counters and without committing to which inputs are special.
It does not give timing or a count of pulses. If some input eventually needs
those, that is a counter on the vehicle reported in its own payload, and the
mask will have told you which input deserved one.

## Proposed message types

### Type 7, `PINOUT_EVENT`

The GPIO case that started this. Fixed 64-bit masks: the vehicle carries three
MCP23017 expanders at `0x25`, `0x26`, `0x27`, which is 48 lines before any
native GPIO is counted, so 32 bits would have been too narrow.

```text
 0..15   event prefix                     as above
16..23   inputs           uint64          level at publication
24..31   changed          uint64          changed since the previous event
32..39   outputs          uint64          commanded output level
40       input_count      uint8           meaningful bits, <= 64
41       output_count     uint8           meaningful bits, <= 64
42..43   reserved         uint16          zero
```

Payload 44 bytes, frame 60. A count above 64 is rejected, in the same style as
`ROBOT_WIRE_INVALID_LIGHT_COUNT`.

Bit order is fixed by the contract and must be written down in the protocol
document, not inferred from the firmware's expander scan order. That is the
same mistake the lamp ordering already made once.

Clearpath's `PinoutState` also carries `rails` and `output_periods`. Both are
omitted: this vehicle has no rail concept and periods are a command-side
detail. Adding them later is a new type, not a longer payload, because payload
size is fixed per type by design.

### Type 8, `LIGHTS_EVENT`

Resolves [Proposal: unsolicited LIGHTS_STATE](PROPOSAL_UNSOLICITED_LIGHTS_STATE.md)
in favour of its option 4, now with a proper prefix rather than a bare new type.

```text
 0..15   event prefix                     as above
16..19   last_command_sequence  uint32    command this state resulted from, 0 if none
20       light_count            uint8     exactly 4
21       result                 uint8     robot_lights_result_t
22       active_source          uint8     robot_lights_source_t
23       reserved               uint8     zero
24..35   lights                 3 * 4     RGB, front-right, front-left, rear-left, rear-right
```

Payload 36 bytes, frame 52.

`last_command_sequence` is deliberately named differently from the header's
`sequence`. It is information, not correlation: *this situation resulted from
that command*. The ambiguity that produced the bench warning was that one field
was being asked to mean both. Now `LIGHTS_STATE` answers "here is the reply to
your request", `LIGHTS_EVENT` announces "the situation changed on its own", and
neither has to guess which it is.

The RC transmitter being switched off, which is what produced the original
warning, becomes a `LIGHTS_EVENT` with `result = OVERRIDDEN`, `source =
SAFETY`. When the link returns, another event with `result = OK`. Both are
delivered, and neither is unmatched, because neither claims to match anything.

## Transport: one key per subsystem

Events get their own Zenoh keys, following the existing route table:

| Direction | Zenoh key |
| --- | --- |
| Vehicle to host | `<prefix>/io/pinout/event` |
| Vehicle to host | `<prefix>/io/lights/event` |

The alternative is a single `<prefix>/events` key with type dispatch on
receipt. That would need a header-peek function the codec does not have today,
because `decode_header()` takes the expected type as a parameter and is
type-directed by design.

Key-per-subsystem needs no such function: the key tells the subscriber which
decoder to call, exactly as `io/lights/cmd` does now. It also lets a consumer
subscribe to only what it cares about, and keeps `<prefix>/io/**` a useful
wildcard for debugging.

## What this costs

- Two message types, two encode/decode pairs in C, the same in Python.
- Golden vectors for both, and the cross-implementation test extended, per rule
  5 of `ARCHITECTURE.md`.
- Firmware: an event publisher with a rate limiter and change detection, a boot
  id at startup, one shared counter.
- Gateway: two subscribers, receipt-time stamping, and dropping a cached view
  when `boot_id` changes.
- No version bump, no header change, no change to any existing message.

## Still open

1. **Bit assignment for the pinout masks.** Which physical line is bit 0. This
   must be decided here and written into the protocol document before either
   consumer implements it.
2. **Whether `LIGHTS_STATE` survives.** With `LIGHTS_EVENT` carrying the
   spontaneous case, `LIGHTS_STATE` becomes purely a reply, which is what the
   gateway already assumes. That is a clean outcome, but it should be confirmed
   rather than allowed to happen quietly.
3. **Whether the pinout ROS route stays a service.** The gateway's plan makes
   pinout a Zenoh query with a 1000 ms timeout. Reads become unnecessary once
   state is published; writes stay transactional. Worth revisiting there.

## What was verified rather than assumed

- The Python codec's imports are standard library only, and the route A bench
  script's imports contain no ROS: read from `python/robot_wire_protocol.py`
  and `apps/sdc2026/test_scripts/lights_route_a.py`.
- Length is validated before any field is read, and the decoder is
  type-directed: read from `decode_header()` in `src/robot_wire_protocol.c`.
- An unknown message type is rejected cleanly, and the test uses type 9 to
  prove it: read from `tests/test_python_matches_golden.py`.
- Three MCP23017 expanders, so 48 expander lines: read from the firmware's
  `include/config.h`.
- Message types 3 to 6 are the only assigned numbers: read from
  `include/robot_wire_protocol.h`.

Not verified: whether the 100 ms floor is low enough for any input the vehicle
actually has, since no input is wired to anything time-critical yet. Worth
re-checking when the e-stop switch is on the board, as that is the first input
where latency has a safety argument attached to it.
