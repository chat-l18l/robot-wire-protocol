# Research: device-initiated events

> **Status: research, answered.** Written before building anything, to
> establish whether the protocol can express a message the vehicle originates.
> Raised: 2026-08-13.
>
> The questions at the end are now settled in
> [Design: device-initiated events](DESIGN_DEVICE_EVENTS.md). The timestamp
> does not need to be comparable to ROS time, which was the largest of them.

## The question

A GPIO input changes on the ESP32-P4. Nobody asked. That fact has to reach the
ROS 2 nodes on the far side of the gateway.

Can the protocol express that today?

## Finding: no. Every message type is a reply

The four defined types are two request/response pairs:

| Type | Direction | Shape |
| ---: | --- | --- |
| 3 `LIGHTS_COMMAND` | host to vehicle | request |
| 4 `LIGHTS_STATE` | vehicle to host | reply |
| 5 `ESTOP_CLEAR_RESET_REQUEST` | host to vehicle | request |
| 6 `ESTOP_CLEAR_RESET_RESULT` | vehicle to host | reply |

The planned pinout route follows the same shape and commits to it harder: the
gateway's implementation plan describes a Zenoh *query* with a 1000 ms timeout,
a deferred ROS service response, and request-ID deduplication on the ESP32.
That is a transaction, and a transaction cannot carry news.

Every vehicle-to-host message in the protocol is therefore an answer to
something the host said first. There is no category for a message the vehicle
originates, and a GPIO input is exactly that: the host cannot ask about a
change it does not know happened, and polling for edges loses them between
polls.

The unsolicited `LIGHTS_STATE` problem observed on the bench, recorded in
[the LIGHTS_STATE proposal](PROPOSAL_UNSOLICITED_LIGHTS_STATE.md), is the first
symptom of this gap rather than a lights-specific defect.

## The JSON route already solved it

The education JSON route publishes vehicle-originated state today:
`io/toggles/state`, `io/steer/state`, `io/drive/state`, `system/status`, on
change and periodically. It has no correlation problem because it never claimed
to be a reply.

So the capability exists in the system, on the informal route, and is missing
from the formal one. That is worth naming plainly: the strict, versioned
contract is the one that cannot express a common case.

## Clearpath models this as state publication, with a timestamp

`clearpath_platform_msgs/msg/PinoutState`:

```text
std_msgs/Header header      # <- timestamp

bool[] rails
bool[] inputs               # <- read-only; only the vehicle knows these
bool[] outputs
uint32[] output_periods
```

Three things to take from it.

**It is a topic, not a service.** Alongside it, `PinoutCommand` is a command
topic and `SetPinout.srv` is a service. Clearpath uses all three shapes and
picks per direction: commands and transactions are services, vehicle-originated
facts are publications.

**It carries a header with a stamp.** Our protocol has no time field at all.
For a reply that is fine, because the request just happened and `sequence`
carries the correlation. For an event there is no request, and *when* is the
primary fact rather than *which command*.

**It is a full snapshot, not an edge.** `inputs` is the complete input state,
not "input 3 went high". That distinction matters more than it looks; see
below.

## What an event needs that a reply does not

### 1. A time, not a correlation

`sequence` answers "which request is this about". An event has no request. A
timestamp answers the question an event actually raises, and lets the host
order events and detect staleness.

### 2. Its own numbering, or none

Reusing the command sequence is precisely what failed on the bench: the gateway
could not match a state to a transaction it had closed. An event needs either a
monotonic event counter of its own or an explicit statement that it carries no
correlation.

### 3. Snapshot semantics, because a lost event is lost forever

This is the important one.

A dropped reply is recoverable: the requester notices the timeout and retries.
A dropped event is simply gone, and if the event was "input 3 went high", the
host's view stays wrong indefinitely.

Publishing full state instead of edges makes a loss self-correcting: the next
publication repairs the host's picture whether or not the previous one arrived.
Clearpath's `bool[] inputs` is a snapshot for this reason, and it is why the
answer here should be "publish state on change and periodically" rather than
"notify on edge".

The cost is that a fast pulse between two publications is invisible. If any
input needs edge-accurate capture, that is a latch or a counter on the vehicle
side reported within the snapshot, not a faster event rate.

### 4. Something that survives a restart

Sequence numbers restart at boot, so a host cannot tell a fresh sequence 1 from
a stale one. A boot or session counter in the event would let the host discard
a view built before a vehicle restart. The protocol has no such field today.

## What this means for the LIGHTS_STATE proposal

It changes the recommendation there.

That proposal offered a flag bit in a reserved byte as the cheapest fix, and
noted the choice would set the pattern for GPIO, AUX and odometry. This research
answers that: GPIO needs vehicle-originated messages, odometry is pure
publication, and AUX will follow GPIO. At least three of them need it.

So the general answer is the right one, and the flag is not it. A per-message
flag would leave the timestamp, the numbering and the restart problem unsolved
in each case separately.

## The alternative that was considered and rejected

micro-ROS would remove this problem entirely by making the microcontroller a
ROS 2 node, so a GPIO change becomes an ordinary topic publication with a
standard header. It was evaluated and not adopted: interface definitions are
compiled into the firmware, so every message change and every distribution
upgrade means reflashing every vehicle, and ESP32-P4 is not among its supported
chips. See [Evaluation: micro-ROS](EVALUATION_MICRO_ROS.md).

That evaluation is why the work below is worth doing rather than avoided.

## Shape of a possible answer

Not a decision, a sketch to argue about.

Add a message category for vehicle-originated state, with a common payload
prefix carrying what all events need:

```text
event_sequence   uint32   monotonic per vehicle, independent of commands
boot_id          uint16   changes on restart, so a stale view is detectable
timestamp_ms     uint32   vehicle uptime, or a synced clock if one exists
```

Then per-subsystem payloads: pinout state, lights state, odometry.

Open in that sketch: whether the header grows for everyone or only for events,
whether the timestamp is uptime or wall clock, and whether one general
`STATE_EVENT` type with a subsystem selector beats one type per subsystem.

## Questions that decide it

1. **Does the timestamp need to be comparable to ROS time?** If yes it needs a
   synced clock or a gateway-side translation, which is a much larger question
   than a field.
2. **Which inputs, if any, need edge accuracy?** If none, snapshots settle it.
   If some do, they need a latch or counter on the vehicle, decided before the
   payload is fixed.
3. **One event type or one per subsystem?** One type keeps the number space
   small and the gateway's routing simple; per-subsystem keeps each payload
   honest and independently versionable.
4. **Does this force a protocol version bump?** Adding message types does not
   break existing decoders, which reject unknown types cleanly. Changing the
   16-byte header would break everything and should be avoided.
5. **Publication rate and change detection.** Periodic-plus-on-change needs a
   floor and a ceiling: too fast floods a shared router, too slow makes the
   self-healing property useless.

## What was verified rather than assumed

- The four message types and their request/response shape: read from
  `include/robot_wire_protocol.h` and `src/robot_wire_protocol.c`.
- The pinout route's transactional design: read from the gateway's
  `PINOUT_SERVICES_IMPLEMENTATION_PLAN.md`.
- `PinoutState`, `PinoutCommand` and `SetPinout.srv`: read from
  `clearpath_platform_msgs` in the gateway repository.
- The JSON route's vehicle-originated publications: read from the SDC2026
  firmware's Zenoh topic list.

Not verified: whether Clearpath publishes `PinoutState` periodically, on
change, or both. That is worth confirming from a running A300 before copying
the rate policy.
