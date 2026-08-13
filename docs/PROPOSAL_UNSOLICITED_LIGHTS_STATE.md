# Proposal: unsolicited `LIGHTS_STATE`

> **Status: open.** Needs a decision before either consumer changes.
> Raised: 2026-08-13, from bench observation on `k300_12345`.

## What happened

The motion core reported an arbitration outcome that changed without a new
command, and the gateway discarded it:

```text
[WARN] Lights command sequence 1 returned result 3 from source 1
[WARN] Ignored unmatched Lights state sequence 1        # ~10 s later
```

The first line is correct behaviour on both sides. The command arrived, was
decoded, and a local safety condition owned the lamps, so the motion core
answered `OVERRIDDEN` from `SAFETY`. In this case the RC transmitter was
switched off, which the vehicle treats as a link fault. Exactly what the
priority model is for.

The second line is the problem. Ten seconds later the RC link returned, the
outcome changed from `OVERRIDDEN` to `OK`, and the motion core published an
updated `LIGHTS_STATE` still carrying sequence 1, the last command it had
accepted. The gateway had closed that transaction and dropped the message.

Neither side is misbehaving. They disagree about what the message is.

## Root cause: is `LIGHTS_STATE` a reply or a state publication?

The name says state. The gateway treats it as a reply.

- **As a reply**, `sequence` correlates a response to a request. One command,
  one state; a second state for the same sequence is a duplicate and dropping
  it is right.
- **As a state publication**, `sequence` records *which command the current
  situation resulted from*. The vehicle may publish whenever that situation
  changes, and a repeat sequence is normal.

The specification does not currently say which it is, so both implementations
picked a reading and both are defensible. That ambiguity is the actual defect;
everything below is a way of removing it.

Why this matters beyond tidiness: the outcome genuinely can change with no new
command. A fault reclaiming the lamps, or releasing them, is precisely the
event an operator needs to see, and it is the reason the motion core reports
`result` and `active_source` at all. If unsolicited updates cannot be
delivered, the gateway's view goes stale exactly when the vehicle has something
important to say.

## Options

### 1. Motion core publishes only on a new sequence

The narrowest change: drop the outcome-change trigger.

- No protocol change, no gateway change.
- Loses the capability the field exists for. A gateway would keep showing `OK`
  while the vehicle displays a fault pattern, and nothing would correct it until
  the next command happened to arrive.

Rejecting a message is at least visible; going quietly stale is not. This
option trades a warning for a silent wrong answer.

### 2. Gateway accepts an updated state for a sequence it has already seen

Treat `LIGHTS_STATE` as a state publication and say so in the specification.

- No protocol change. Semantically honest: the outcome for that command really
  did change.
- Needs a rule the gateway does not have today: how long does a sequence stay
  matchable? Indefinitely is unbounded state, and a state arriving from before
  a vehicle restart would match something unrelated.
- Sequence numbers restart at boot, so "matchable forever" is not safe without
  a session marker.

### 3. Flag unsolicited states in the existing reserved byte

`LIGHTS_STATE` payload byte 3 is reserved and written zero. Use bit 0 to mean
"not a reply to a command; the outcome changed on its own".

```text
payload offset 0   light_count (4)
payload offset 1   result
payload offset 2   active_source
payload offset 3   flags, bit 0 = unsolicited      <-- currently reserved
```

- Frame size unchanged, so no message-type allocation and no version bump.
- Degrades gracefully. The current decoders do not validate that byte, so an
  older gateway sees exactly what it sees today rather than failing.
- The reserved byte exists for this kind of extension, and spending one bit of
  it is cheap.
- Against: silently reinterpreting a reserved byte is only safe because nothing
  validates it, and that is luck rather than design. It should be written down
  as an explicit protocol version 1 addendum, not slipped in.

### 4. A separate message type for unsolicited state

Add `LIGHTS_STATE_EVENT` alongside `LIGHTS_STATE`.

- Cleanest semantics: a reply and an event are different things and would look
  different on the wire.
- Costs a message type, requires both consumers to implement it, and an older
  gateway rejects the new type outright rather than degrading.

## Superseded in part

[Research: device-initiated events](RESEARCH_DEVICE_INITIATED_EVENTS.md) answers
the question this proposal deferred. GPIO inputs, AUX and odometry all need
vehicle-originated messages, so at least three subsystems require the same
category rather than one. That settles the "what decides it" question below in
favour of a general answer, and rules out the flag: a per-message flag would
leave the timestamp, the numbering and the restart problem unsolved separately
in each case.

The recommendation below is kept as written for the record. Read it as the
lights-only view, taken before the general shape was understood.

## Recommendation

Option 3, written up as an explicit addendum to version 1, with option 2's
clarification alongside it: state the specification's position that
`LIGHTS_STATE` is a state publication rather than a reply, and let the flag
distinguish the two cases for consumers that care.

Option 4 is the better shape if the protocol grows more event-like traffic. If
GPIO, AUX or odometry turn out to need the same distinction, the right move is
one general answer for all of them rather than a flag per message.

That is the reason not to decide this in isolation: whichever way it goes here
sets the pattern the other message types will follow.

## What decides it

1. Does any other planned message need to distinguish a reply from a
   spontaneous update? Odometry is pure publication, GPIO and AUX are likely
   command-and-confirm like lights. If two or more need it, prefer option 4.
2. Can the gateway hold a sequence open safely across a vehicle restart? If
   not, option 2 alone is insufficient regardless of what else is chosen.
3. Is a version bump acceptable now, or must version 1 stay wire-compatible
   with what is deployed? Option 3 is the only one that keeps it.
