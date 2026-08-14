# Architecture and Separation of Concerns

This document explains what this repository is, what it deliberately is not,
and why that boundary is drawn where it is. The layout looks like extra work
until you have watched a protocol drift apart across two codebases; the
reasoning is written down here so the arrangement survives the next person who
wonders why a four-file library needs its own repository.

## The three concerns

One robot, three codebases, each owning exactly one concern:

| Repository | Owns | Knows nothing about |
| --- | --- | --- |
| `robot_ros2_compat` | ROS 2 message types, the ROS graph, the gateway | GPIO, RMT, motor drivers |
| `omnibot` (`sdc2026` firmware) | Hardware: motors, lights, I2C expanders, the safety FSM | ROS 2, the ROS graph |
| **this repository** | The bytes on the wire | Both of the above |

The two outer repositories never include each other's headers. Their only
common ground is this one, and that is on purpose: it is the whole point of the
arrangement.

## Why the contract needs its own repository

Because a contract that lives inside one participant is not a contract, it is
that participant's preference.

Put the codec in the gateway and the firmware becomes a follower: it has to
track whatever the gateway does, and a change is announced by things breaking.
Put it in the firmware and the gateway inherits the same problem. Put a copy in
both and you have two implementations that agree only until someone edits one
of them, which is the failure mode that is hardest to see, because both sides
compile, both sides run, and only the bytes disagree.

Giving the contract its own repository makes changing it a deliberate act.
Neither consumer can move it unilaterally, a change is a commit with a version
tag, and both consumers update against a named version rather than against
whatever `main` happened to be that afternoon.

That is why the README states the rule bluntly: **do not duplicate the codecs
or hand-copy definitions into the consumer repositories.** Every shortcut past
that rule reintroduces exactly the drift the split was meant to prevent.

## Three concerns inside the message path

"Separation of concerns" is easy to nod at and easy to get wrong, so it helps
to name the three layers that a message actually passes through:

```text
application   what the values mean         a red lamp means a fault
    |
transport     getting bytes from A to B    Zenoh, a socket, a file
    |
encoding      messages <-> bytes           THIS REPOSITORY
```

This repository implements the bottom layer and nothing else. It has no
dependency on Zenoh, ROS 2, ESP-IDF or any driver, and that is not modesty
about scope, it is what makes the layer testable. Encoding is a pure function:
the same message always produces the same bytes, so it can be verified against
a golden vector on a laptop with no robot, no network and no router.

The moment a codec also opens a socket, that property is gone. You can no
longer test the encoding without standing up the transport, and a bug in either
looks like a bug in the other.

The consumers supply the other two layers. The firmware picks Zenoh for
transport and decides that a fault outranks a colour command; the gateway picks
Zenoh and decides how a ROS topic maps onto a message. Neither decision belongs
here.

### Where the boundary is subtle

The protocol defines `LightsResult` and `LightsSource`, which look like policy:

```c
ROBOT_LIGHTS_RESULT_OVERRIDDEN
ROBOT_LIGHTS_SOURCE_SAFETY
```

It is not policy. The protocol supplies the *vocabulary* for reporting an
outcome; it does not decide what the outcome should be. Whether a fault
outranks a remote colour command is the firmware's decision, made in the
firmware, for reasons that have to do with that vehicle. The protocol only
guarantees both ends can say "overridden, by safety" and mean the same thing.

That distinction is worth holding on to. A protocol that dictated the policy
would force every consumer into one behaviour; a protocol with no vocabulary
for the outcome would leave every consumer inventing its own words for it.

## Why two implementations, and how they stay honest

The contract is the **bytes**, not the code. So there is nothing wrong with
implementing it twice, as long as "twice" means two implementations of one
spec rather than two specs.

- `src/` is C11 for the embedded target: allocation-free, freestanding.
- `python/` is for tooling and bench scripts, which would otherwise hand-pack a
  struct and quietly lie the first time a field moves.

What makes this safe rather than reckless is
`tests/test_python_matches_golden.py`. It asserts the Python encoder against
the same byte-for-byte frame the C test asserts on, and checks that corrupted
frames are rejected for the same reason on both sides:

```python
GOLDEN = bytes([0x52, 0x42, 0x43, 0x31, 0x01, 0x00, ...])   # "RBC1", version 1
assert encode_lights_command(distinct_command()) == GOLDEN
```

The point of that test is not that Python works. It is that a divergence
between the two implementations becomes a failing test instead of a frame that
only one end understands. Without it, two implementations would be exactly the
duplication this repository exists to prevent; with it, they are two views of
one verified spec.

## How a frame is built

Every message is a fixed 16-byte header followed by a fixed-size payload, all
little-endian:

```text
 0..3   magic         0x31434252, "RBC1"
 4..5   version       1
 6..7   message_type
 8..9   payload_size
10..11  reserved      0
12..15  sequence
16..    payload
```

Four properties, each deliberate:

- **Magic** catches a payload that arrived from something else entirely.
- **Version** turns a protocol change into a clean rejection rather than a
  misread. This is why a version bump must reach both consumers.
- **Explicit length**, checked against the actual buffer, so a truncated frame
  is rejected rather than read past its end.
- **Sequence**, echoed in the corresponding state message, which is what lets a
  sender tell "applied" from "never arrived".

Decoders validate in a fixed order: length, magic, version, message type,
payload size, then field ranges. The order is part of the contract, so a given
bad frame is rejected for the same stated reason by every implementation. That
is why the Python `WireError.reason` carries the C enumerator name.

Nothing in a message is optional, and no field is inferred. A decoder either
returns a fully populated message or an error; it never returns something
partially filled, because a caller that missed the error would then act on
whatever the uninitialised half contained.

## How consumers integrate

Neither consumer copies anything. Both point at this repository.

**ESP-IDF firmware** uses `espidf/CMakeLists.txt` as a component entry point,
usually via a git submodule and a thin wrapper component that gives it a proper
name. The wrapper exists because ESP-IDF takes a component's name from its
directory and treats any directory holding a `CMakeLists.txt` as the component
itself, so pointing the build at this repository's root would pick up the
standalone CMake project instead.

**Host tooling** adds `python/` to `sys.path`, or installs it. No build step,
which matters: a bench script that needs a compiler first is a bench script
nobody runs.

**A plain CMake consumer** adds this directory with `add_subdirectory()` and
links `robot_wire_protocol`.

`COLCON_IGNORE` at the root keeps colcon from treating this as a ROS package,
since it is not one and must not become one.

## What this buys, concretely

- A protocol change is one commit in one place, with a version, and both
  consumers move against it deliberately.
- The encoding is testable with no hardware, no network and no ROS.
- A bench script produces the same bytes as the firmware because both are
  driven by the same definition, so a script that works proves something about
  the vehicle rather than about the script.
- Neither consumer can quietly redefine the contract to suit itself.
- **ROS 2 is optional.** A robot built without it is this system minus the
  gateway: the vehicle keeps its full command and event surface, and plain
  Python, a dashboard or a different middleware consume the same frames. See
  [Design: device-initiated events](DESIGN_DEVICE_EVENTS.md) for why that
  survives only as long as the firmware stays free of ROS build dependencies.

## The alternative to all of this

Two alternatives remove the need for a separate protocol, and both have been
evaluated. Read the relevant one before proposing either again.

micro-ROS makes the microcontroller a ROS 2 node. Interface definitions compile
into the firmware, so a message change or a distribution upgrade reflashes
every vehicle, and the separation described above is exactly what gets traded
away. See [Evaluation: micro-ROS](EVALUATION_MICRO_ROS.md).

`rmw_zenoh` lets ROS 2 speak Zenoh natively, so a vehicle could publish onto
ROS topics directly with no ROS package in its build. The coupling is milder —
type identity is a string, not generated code — but it means replicating four
undocumented internal formats that the project explicitly declines to support.
See [Evaluation: rmw_zenoh](EVALUATION_RMW_ZENOH.md), including the spike that
would settle it.

## Rules that follow

1. No transport, no ROS 2, no ESP-IDF, no drivers in this repository. If a
   change needs any of them, it belongs in a consumer.
2. No codec, no message struct and no constant is copied into a consumer.
3. A protocol change is a tagged release, and both consumers are updated to
   compatible versions.
4. New message types are defined here first, never in a consumer "for now".
5. Any second implementation of the codec must be proven against the golden
   vectors in the same commit that introduces it.

Rule 5 is the one that is tempting to skip, and it is the one holding the rest
up. Two implementations without a test that compares them are just duplication
wearing a nicer name.
