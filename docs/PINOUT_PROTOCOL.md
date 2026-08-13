# Pinout Protocol

Vehicle-originated digital I/O state. The reasoning behind the shape is in
[Design: device-initiated events](DESIGN_DEVICE_EVENTS.md); this document is
the contract.

> **Codec and vehicle publisher implemented; no gateway subscriber yet.** The
> SDC2026 firmware publishes this message; nothing on the ROS side consumes it,
> so it is observable today only with a Zenoh subscriber.

## Route

| Direction | Zenoh key | Encoding |
|---|---|---|
| Motion core to gateway | `<zenoh_prefix>/io/pinout/event` | `application/octet-stream` |

There is no command route here. Writing outputs stays a separate transaction;
this route only answers what the vehicle currently sees and drives.

The key implies the message type, which is why the codec needs no header-peek
function: `decode_header()` is type-directed and the subscriber already knows
what it subscribed to.

## Frame

Message type 7, payload 64 bytes, frame 80 bytes, little-endian throughout,
behind the usual 16-byte `RBC1` header.

```text
header 12..15   sequence         reserved, zero: an event answers no request

payload
 0.. 7   uptime_ms        uint64   milliseconds since vehicle boot
 8..11   event_sequence   uint32   monotonic per vehicle, all event types
12..15   boot_id          uint32   opaque session identifier
16..23   inputs_level     uint64
24..31   inputs_changed   uint64
32..39   inputs_valid     uint64
40..47   outputs_level    uint64
48..55   outputs_changed  uint64
56..63   outputs_valid    uint64
```

Bytes 0..15 of the payload are the **event prefix**, shared by every
vehicle-originated message type.

## What the fields mean

**`uptime_ms`** is monotonic since boot and is **not comparable to ROS time**.
It orders events and detects staleness; it does not say when in world time
anything happened. A gateway publishing into ROS stamps `Header.stamp` from its
own clock at receipt.

**`event_sequence`** counts published events, one counter for the whole
vehicle, incremented across all event types. It is independent of the command
sequence used by request/response messages. A gap means an event was lost,
which is diagnostics rather than something to recover: see snapshots below.

**`boot_id`** changes when the vehicle restarts. **Compare it for equality
only, never order it.** A consumer that sees a different `boot_id` discards
whatever view it had built. It carries no other meaning and is not a count.

**`*_level`** is the state at the moment of publication.

**`*_changed`** is sticky since the previous event and cleared each time one is
published. A signal that changed and changed back between two publications
still appears here, with its level bit unchanged. This is how a snapshot keeps
short pulses visible without publishing edges.

**`*_valid`** is which bits this vehicle implements. A bit that is 0 in
`valid` says **nothing at all** about the corresponding level bit. This is the
field that keeps "not wired" distinguishable from "not asserted", which matters
most for the e-stop.

## Two rules for bit numbers

**1. A bit names a signal, not a pin.** Where a signal is wired is the
firmware's private business. A physical mapping such as `expander * 16 + pin`
would make the contract a function of one vehicle's wiring, so moving a signal
during a board revision would silently change what every consumer reports, with
no decode error anywhere. The e-brake settles it: it is native GPIO, not an
expander line at all.

**2. A bit is 1 when the named condition holds**, never when the pin reads
high. Buttons read low when pressed, through a pull-up, and the e-brake is
active-low; neither fact reaches the wire. Pull-ups and the expander's `IPOL`
register are firmware concerns.

Consequently each bit is named for what 1 means, hence `THROTTLE_DIR_REVERSE`
rather than `THROTTLE_DIR`.

Inputs and outputs are numbered in **separate spaces**, both starting at 0.
Output bit 0 is the horn even though the horn is on physical pin 8.

## Bit assignment

Numbers are handed out in blocks of 16, so related signals stay together as
hardware is added. Unassigned numbers are free; assigned numbers are permanent.

### Inputs

| Bit | Name | Block |
|---:|---|---|
| 0 | `BUTTON_UP` | operator panel, 0–15 |
| 1 | `BUTTON_DOWN` | |
| 2 | `BUTTON_ESC` | |
| 3 | `BUTTON_ENTER` | |
| 4 | `BUTTON_TEST` | |
| 5 | `TOGGLE_1` | |
| 6 | `TOGGLE_2` | |
| 7 | `TOGGLE_3` | |
| 16 | `ESTOP` | safety chain, 16–31 |

Bits 32–47 are reserved for drivetrain feedback and 48–63 for the auxiliary
connector. Neither has assignments yet.

`ESTOP` has a number but no hardware: it is expected to be absent from
`inputs_valid` until the switch is wired. Assigning it early is deliberate, and
is what the valid mask makes safe.

### Outputs

| Bit | Name | Block |
|---:|---|---|
| 0 | `HORN` | signalling and indication, 0–15 |
| 1 | `HEARTBEAT` | |
| 2 | `FULLSTOP_LED` | |
| 3 | `ZENOH_LED` | |
| 4 | `INDICATOR_LEFT` | |
| 5 | `INDICATOR_RIGHT` | |
| 6 | `AUTONOMOUS_LED` | |
| 16 | `THROTTLE_DIR_REVERSE` | drivetrain control, 16–31 |
| 17 | `EBRAKE` | |
| 18 | `SKID_DIR_FL_REVERSE` | |
| 19 | `SKID_DIR_FR_REVERSE` | |
| 20 | `SKID_DIR_RL_REVERSE` | |
| 21 | `SKID_DIR_RR_REVERSE` | |

Bits 32–47 are reserved for the auxiliary connector and 48–63 for expansion.

The WS2812 lamps are deliberately absent. They have colour, an arbitration
result and a source, and are reported by the lights messages; repeating them
here as on/off bits would be a second, poorer answer.

## Validation

Rejection order is length, magic, version, message type, payload size, then the
mask consistency check, matching every other message in this protocol.

A `level` or `changed` bit set outside its `valid` mask is rejected as
`INVALID_PINOUT_MASK` (11) by **both the encoder and the decoder**. Rejecting
rather than silently masking keeps encode and decode inverses of each other,
matches how this codec treats every other out-of-range field, and guarantees no
conforming frame can ever claim a signal the vehicle does not implement.

A decoder populates the caller's struct only after every check has passed, so a
rejected frame never leaves a half-written message behind.

## Publication policy

Snapshots, not edges, because a dropped reply is retried while a dropped edge is
gone forever and would leave a consumer's view wrong indefinitely.

- **On change**, rate-limited to at most one event per **100 ms**.
- **Unconditionally every 1000 ms**, whether or not anything changed.

The ceiling bounds staleness after a loss to about a second; the floor stops a
chattering input flooding a shared router. Worst case is ten 80-byte frames per
second.

If an input ever needs true edge accuracy rather than "something happened here",
that is a latch or counter on the vehicle reported in its own payload. The
`changed` mask is what tells you which input deserved one.
