# Evaluation: speaking rmw_zenoh directly from the vehicle

> **Status: evaluated, not adopted, but materially better than micro-ROS on the
> axis that matters here.** Recorded because it is a genuine third option that
> was not on the table when [Evaluation: micro-ROS](EVALUATION_MICRO_ROS.md)
> was written, and because two of the four things commonly said about it are
> wrong in ways that change the decision.
> Raised: 2026-08-14.

## The question

ROS 2 Jazzy can run Zenoh as its middleware, through `rmw_zenoh_cpp`. If ROS
nodes are already publishing and subscribing over Zenoh, could the ESP32-P4
publish onto those same topics directly, and would that remove the gateway
without taking on micro-ROS's build-time coupling?

## Four claims, checked

| Claim | Verdict |
| --- | --- |
| Jazzy can use Zenoh as its middleware | **Correct** |
| The router is integrated, so no daemon to start | **No.** It ships with ROS, and you still start it |
| ROS topics map onto Zenoh keys by minimal name mangling | **Partly.** The mangling includes a type hash |
| CDR plus a 4-byte header makes a message ROS-compatible | **Correct for the payload, incomplete for the sample** |

### It is a real, packaged option

`sudo apt install ros-<DISTRO>-rmw-zenoh-cpp` installs it, selected with
`RMW_IMPLEMENTATION=rmw_zenoh_cpp`. It is an alternative RMW rather than the
default, and binary packages exist for supported distributions on Tier-1
platforms.

### The router did not go away

From the design document, twice over:

> It is assumed that a Zenoh router is running on the local system. This router
> will be used for discovery and host-to-host communication.

> `rmw_zenoh_cpp` requires the Zenoh router to be running.

It is launched as `ros2 run rmw_zenoh_cpp rmw_zenohd`. So the daemon is now a
ROS-shipped binary instead of a separately installed `zenohd`, and discovery
depends on it: sessions connect to the router and learn about peers through
gossip scouting. Multicast scouting is off by default and only helps
same-host peers.

This is a packaging improvement, not a topology change. Our router does not
disappear; it changes owner.

### The mangling is not minimal, and that is the interesting part

The key expression for topic data is:

```text
<domain_id>/<fully_qualified_name>/<type_name>/<type_hash>
```

For example:

```text
0/chatter/std_msgs::msg::dds_::String_/RIHS01_df668c740482bbd48fb39d76a70dfd4bd59db1288021743503259e948f6b1a18
```

That trailing field is a REP-2016 type hash. It is **derived from the type
description**, not from the distribution, which is the single most important
fact in this document and the reason the coupling story differs from
micro-ROS's. More on that below.

### The 4-byte header is real, but a sample needs more than a payload

Confirmed in the source rather than inferred. `rmw_zenoh_cpp` constructs its
CDR object in `DDS_CDR` mode:

```cpp
: cdr_(fastbuffer, eprosima::fastcdr::Cdr::DEFAULT_ENDIAN,
       eprosima::fastcdr::CdrVersion::DDS_CDR)
```

and writes the encapsulation explicitly:

```cpp
// Serialize encapsulation
ser.serialize_encapsulation();
```

with `deser.read_encapsulation()` on the way in. So the payload really is a
4-byte CDR encapsulation header followed by the CDR body, and nothing
ROS-specific beyond that. A hand-rolled encoder, or eProsima's standalone
Micro-CDR, can produce it.

**But the payload is not the whole sample.** Every publication also carries a
Zenoh attachment:

```text
 8 bytes   sequence number      int64, little-endian
 8 bytes   source timestamp     int64, nanoseconds since UNIX epoch
 1 byte    GID length           currently 16
16 bytes   publisher GID
```

and the subscriber treats its absence as a failure, not as a default:

```cpp
auto attachment = sample.get_attachment();
if (!attachment.has_value()) {
  RMW_ZENOH_LOG_ERROR_NAMED(
    "rmw_zenoh_cpp",
    "Unable to obtain attachment for topic '%s'", ...)
  return;
}
```

A sample without an attachment is logged and dropped. So "wrap it in four
bytes" is genuinely most of the payload story and none of the sample story.

Note also that this timestamp is real wall-clock nanoseconds. Our own design
settled on vehicle uptime precisely because no clock synchronisation exists;
this route would put that question back on the table.

## Why the coupling really is different from micro-ROS

This deserves stating carefully, because it is where the intuition behind the
question is right.

Under micro-ROS, interface definitions are **generated into the firmware
binary**. The type identity and the serialisation code are both build products
of a ROS workspace pinned to a distribution.

Under this route, the type identity is a **string in a key expression**. It is
data, not code. It could live in NVS, or in a configuration field, and change
without recompiling anything. The serialisation is CDR that we write and own,
with no ROS package in the firmware build at all.

That gives a different table from the micro-ROS one:

| Change | micro-ROS | Direct rmw_zenoh interop |
| --- | --- | --- |
| Node logic on the Orin | No reflash | No reflash |
| ROS distribution upgrade | **Reflash** | **No reflash**, if the definitions are unchanged |
| A `.msg` field added | **Reflash** | Reflash: the CDR layout is hand-written |
| A new topic | **Reflash** | Reflash |

The distribution row is the one that made micro-ROS unacceptable for vehicles
that cannot be casually recalled, and this route does not have it. Type hashes
are content-derived, so an unchanged `std_msgs/String` hashes the same on Jazzy
and on whatever follows it.

**Caveat, and it is not small**: that reasoning is sound but unverified. It
should be tested by comparing the hash of one message across two distributions
before anyone relies on it.

## The coupling that replaces it

Nothing is free. This trades a coupling to ROS's build system for a coupling to
`rmw_zenoh`'s internal design, and that one comes with no promise at all. From
the project's own README:

> While it is possible for an application using any Zenoh API to interoperate
> with rmw_zenoh, supporting such use cases is beyond the scope of this
> repository's goals. If you aim to develop such a Zenoh application, you must
> follow the same design than rmw_zenoh for key expressions, data serialization
> format, attachments, and liveliness tokens.

Four things to replicate, none of them a published contract, all of them free
to change in a patch release. The key expression format and the attachment
layout have already differed across branches. Tracking an undocumented internal
format is a maintenance obligation that behaves exactly like protocol drift,
which is the failure this repository exists to prevent.

The maintainers have also said on Discourse that they are unlikely to work on
zenoh-pico support themselves, while being willing to help others.

## The existing microcontroller attempt

`esol-community/rmw_zenoh_pico` is a real project targeting this, and worth
knowing about before anyone starts from scratch. Three findings:

- It states plainly: *"This software is not ready for production use. It has
  neither been developed nor tested for a specific use case."*
- Targets are Linux and Raspberry Pi OS. **No ESP32 of any kind**, and
  bare-metal support is described as unavailable.
- It is built on the micro-ROS stack, with `micro_ros_setup` and compiled-in
  message type support. So it reintroduces exactly the build-time coupling that
  made micro-ROS unattractive, while adding an unstable interop surface.

It is a useful reference implementation. It is not a shortcut.

## What this means for the current architecture

It does not invalidate it, and it strengthens the case against micro-ROS rather
than weakening it: there is now a way to reach ROS 2 without compiling ROS 2
into the firmware.

What it changes is what the gateway is *for*. Today the gateway translates our
protocol into ROS messages. This route would move that translation into the
firmware, where it would be written by hand against an unsupported internal
format, on a chip nobody has tried it on.

The trade is: delete one process on the Orin, take on four undocumented formats
in the firmware, and lose the property that a robot without ROS 2 is the
ordinary case rather than a stripped-down special case. Publishing CDR onto
ROS-shaped keys is still consumable by a non-ROS client, but the vocabulary
becomes ROS's, and the education JSON route would then be the only thing left
that a student can read without a type description.

Not adopted, for now. The gateway stays.

## When to revisit

1. **rmw_zenoh publishes a stable interop specification**, or otherwise commits
   to the key/attachment/liveliness formats. This is the big one: the technical
   obstacles are all mechanical, and only the absence of a contract makes them
   risky.
2. **A ROS-side subscriber is needed for a topic our gateway would only pass
   through unchanged.** Odometry at rate is the likely first case, where the
   translation hop earns nothing.
3. **The gateway becomes the bottleneck**, in latency or in maintenance.
4. **`rmw_zenoh_pico` grows ESP32 support and drops the micro-ROS stack.**
   Either alone is not enough.

## A spike worth doing before any of that

Small, bounded, and it answers the riskiest unknown first: **can zenoh-pico
even produce the attachment?**

`rmw_zenoh_cpp` builds it with the zenoh-cpp serialisation helpers, and
zenoh-pico is a different implementation with a different API surface. If the
attachment cannot be byte-matched from pico, the whole route is closed and the
rest of the investigation is moot.

The order to answer the unknowns:

1. Can zenoh-pico attach the exact 33-byte attachment layout?
2. Does an `ros2 topic echo` receive a hand-built sample with a correct key,
   encapsulated CDR payload and attachment, but **no liveliness token**?
3. If not, what is the minimum liveliness token that makes it visible?
4. Is the type hash of one unchanged message identical across two
   distributions?

Question 2 is worth isolating, because liveliness affects graph introspection
and it is not established here whether data flow requires it at all.

## What was verified rather than assumed

- Key expression format and the router requirement: `docs/design.md` on the
  `jazzy` branch of `ros2/rmw_zenoh`.
- Attachment contents: the same document, cross-checked against
  `AttachmentData::serialize_to_zbytes()` in
  `rmw_zenoh_cpp/src/detail/attachment_helpers.cpp`.
- `DDS_CDR` mode: `rmw_zenoh_cpp/src/detail/cdr.cpp`.
- `serialize_encapsulation()` and `read_encapsulation()` being called:
  `rmw_zenoh_cpp/src/detail/type_support.cpp`.
- A missing attachment being logged and dropped:
  `rmw_zenoh_cpp/src/detail/rmw_subscription_data.cpp`.
- Interop being out of scope, verbatim: the `ros2/rmw_zenoh` README.
- `rmw_zenoh_pico`'s maturity, targets and micro-ROS dependency: its README.

Not verified, and each one could change the conclusion:

- Whether data flows to an `rmw_zenoh` subscriber with no liveliness token.
- Whether a type hash is identical across two ROS distributions in practice.
- Whether zenoh-pico can produce a byte-identical attachment.
- The flash and RAM cost of CDR encoding plus the larger key expressions.

## Sources

- <https://github.com/ros2/rmw_zenoh>
- <https://github.com/ros2/rmw_zenoh/blob/jazzy/docs/design.md>
- <https://github.com/esol-community/rmw_zenoh_pico>
- <https://discourse.openrobotics.org/t/integrating-ros-2-with-microcontrollers-when-using-zenoh/43463>
- <https://docs.ros.org/en/jazzy/p/rmw_zenoh_cpp/>
