# Evaluation: speaking rmw_zenoh directly from the vehicle

> **Status: proven to work on hardware; still not adopted.** The spike ran on
> an ESP32-P4 against ROS 2 with `rmw_zenoh`, and the vehicle appeared as a
> full participant: data, topic and node. See "The spike ran" below. Adoption
> is now a design question rather than a feasibility one. Recorded because it is a genuine third option that
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

## The spike was answered by reading, not building: Pico-ROS

A spike was planned to answer the riskiest unknown — whether zenoh-pico can
even produce the attachment — because a "no" would close the route and make the
rest moot.

It does not need writing. [Pico-ROS](https://github.com/Pico-ROS) is exactly
this, already built: *"a lightweight ROS client implementation designed for
resource-constrained devices. Built on top of zenoh-pico and working in
conjunction with rmw-zenoh"*. BSD-3-Clause, and it ships an ESP-IDF component.

Every unknown on the list above is answered in its source.

### 1. Yes, zenoh-pico can produce the attachment

`src/picoros.h` declares it as a packed struct:

```c
typedef struct __attribute__((__packed__)) {
    int64_t  sequence_number;
    int64_t  time;
    uint8_t  rmw_gid_size;
    uint8_t  rmw_gid[RMW_GID_SIZE];
} rmw_attachment_t;
```

8 + 8 + 1 + 16 = 33 bytes, matching the rmw_zenoh design document byte for
byte, attached with `z_bytes_from_static_buf` into
`z_publisher_put_options_t::attachment`. No API gap, no serialisation helper
missing from pico. The route is open.

### 2. The 4-byte header is literally one line

```c
*((uint32_t*)pBUF) =  0x0100; /*Little endian header*/
ucdr_init_buffer(&writer, pBUF + sizeof(uint32_t), MAX - sizeof(uint32_t));
```

CDR encapsulation, then Micro-CDR for the body. Exactly as predicted, and the
whole of it.

### 3. The type hash is a runtime string, which was the crux

```c
snprintf(keyexpr, KEYEXPR_SIZE, "%" PRIu32 "/%s/%s_/RIHS01_%s",
         node->domain_id, topic->name, topic->type, topic->rihs_hash);
```

`rmw_topic_t` carries `const char* rihs_hash`. The type identity is a string
field, not generated code. This is the concrete form of the coupling argument
made above: a type identity that can be changed without recompiling is a
different kind of dependency from one that is compiled in.

### 4. Liveliness is handled, so the question does not arise

Pico-ROS declares a node token and a per-entity token, `"MP"` for a message
publisher, via `z_liveliness_declare_token`. Whether data would flow without
one is now academic.

### 5. Type generation runs without ROS installed

This is the finding that matters most, and it is stated outright in
`tools/type-gen/readme.md`: *"Runs on host without ROS installation with
virtual environment"*. The script vendors `rosidl` and mocks the parts that
would otherwise require a workspace:

```python
"""Mock ament_index_python module for standalone rosidl usage.
...provides a minimal implementation... without requiring a full ROS2
installation."""
```

Dependencies are `lark`, `empy`, `catkin-pkg`, `argcomplete` — pip, in a venv.
It reads `.msg` files and emits a header of **strings and field lists**:

```c
CTYPE(ros_Point,
    "geometry_msgs::msg::dds_::Point",
    "6963084842a9b04494d6b2941d11444708d892da2f4b09843b9c43f42a7f6881",
    FIELD(double, x) ...
```

Generated once, checked in, no ROS in the firmware build and none in CI. That
is precisely the decoupling this evaluation hypothesised, implemented.

## What Pico-ROS changes, and what it does not

It removes the "reimplement four undocumented formats" objection, because the
reimplementation exists and is maintained by someone tracking rmw_zenoh. We
would track Pico-ROS instead of tracking rmw_zenoh's internals directly, which
is a considerably better position.

It does not remove the argument for this repository. Pico-ROS is a way to
*also* be a ROS participant; it says nothing about whether the vehicle's own
semantics should be ROS message types.

### Four concerns, in order of how much they would cost us

**It vendors its own zenoh-pico, and we already have one.** The ESP-IDF
component globs `picoros/thirdparty/zenoh-pico/src/**` straight into the build.
We pull zenoh-pico 1.9.0 from upstream through PlatformIO; the submodule pins
1.8.0. Two copies of zenoh-pico in one binary is duplicate symbols, so the
component cannot be adopted as-is — it would have to be pointed at ours.

That looks feasible: the submodule is `git@github.com:g4sp3r/zenoh-pico.git`, a
personal mirror whose visible history is upstream commits merged from
`eclipse-zenoh:main`, with no divergence found. It is also an SSH URL, so
`git submodule update --init --recursive` fails for anyone without keys.

**The ESP-IDF component names no chips and no IDF version.** Unlike micro-ROS,
this is not a blocker: the platform layer is zenoh-pico's `system/espidf`, and
we already run zenoh-pico 1.9.0 on an ESP32-P4 in production firmware. The
question that killed micro-ROS does not exist here.

**The timestamp is wrong, and that is informative.**

```c
pub->attachment.time = z_clock_now().tv_nsec;
```

`tv_nsec` is the nanoseconds *field* of a timespec, 0 to 999999999, not
nanoseconds since the epoch. So the attachment timestamp cycles once a second
and means nothing. That it works anyway says rmw_zenoh does not validate the
field — useful for us, since our own design deliberately avoided wall-clock
time. It also says the project is young enough to have a bug in a
33-byte struct, which is worth weighing.

**`*((uint32_t*)pBUF) = 0x0100`** assumes a little-endian host. True on
RISC-V, and a type-punned unaligned store either way.

## The spike ran, and it works

`tests/bringup/ros_native/esp_picoros_talker` in the omnibot repository, on an
ESP32-P4 against ROS 2 with `rmw_zenoh` on an AGX Orin. All three levels:

| Check | Result |
| --- | --- |
| `ros2 topic echo /picoros/chatter std_msgs/msg/String` | receives |
| `ros2 topic list --no-daemon` | `/picoros/chatter` |
| `ros2 node list --no-daemon` | `/picoros_p4` |

So the whole chain holds in practice: the key expression, the CDR payload with
its 4-byte encapsulation, the mandatory 33-byte attachment, and both liveliness
tokens. **An ESP32-P4 can be a first-class ROS 2 participant with no ROS
package anywhere in its firmware build.**

Two findings from doing it, neither of them about Pico-ROS:

**Pico-ROS builds against upstream zenoh-pico 1.9.0**, not only its own
vendored 1.8.0 fork. That was the concern that would have made it unusable
alongside our existing stack, and it is not one.

**The ROS daemon caches the graph and will report an empty one indefinitely.**
Every listing was empty until `--no-daemon` was passed, with nothing wrong on
the vehicle. Worth knowing before anyone else spends an afternoon on it.

Untested and load-bearing for a fielded vehicle: whether the vehicle rejoins
the graph when the **router alone** restarts. The tokens only appeared after
both ends were restarted. zenoh-pico re-announces `_local_tokens` on interest
and has auto-reconnect enabled, so it ought to recover unaided, but that is an
inference rather than an observation.

## Recommendation

Still not a migration. Feasibility is settled; desirability is not, and they
are different questions.

What has not changed is the objection in "The coupling that replaces it": this
works by matching four formats the rmw_zenoh project explicitly declines to
support. Working today is not the same as a contract. What has changed is who
carries that burden — Pico-ROS tracks rmw_zenoh, and we would track Pico-ROS,
which is a considerably better position than reimplementing it ourselves.

The question worth asking now is not "replace the wire protocol" but **which
topics, if any, are better published as native ROS messages than translated by
the gateway**. Odometry at rate is the obvious candidate: high rate, standard
type, and the gateway adds nothing but a hop.

The vehicle's own semantics have no reason to move. Neither does the education
JSON route, which students read without a type description and which would
become CDR nobody can inspect with a text editor.

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

- Pico-ROS's attachment struct, key expression construction, CDR encapsulation
  write, liveliness token declaration and timestamp bug: read from a clone of
  `Pico-ROS/Pico-ROS-software` at `master`, files `src/picoros.c`,
  `src/picoros.h` and `src/picoserdes.h`.
- Type generation without a ROS installation: `tools/type-gen/readme.md`,
  `tools/type-gen/requirements.txt` and the mock in
  `tools/type-gen/ament_index_python.py`.
- The vendored zenoh-pico's origin, version and history: the submodule in
  `Pico-ROS/picoros-espidf-component`.
- That our own zenoh-pico 1.9.0 already exposes `z_liveliness_declare_token`:
  read from the PlatformIO dependency in this project.

Not verified, and each one could change the conclusion:

- Whether a type hash is identical across two ROS distributions in practice.
- Whether Pico-ROS builds and runs on ESP32-P4 at all, and against our
  zenoh-pico rather than its own.
- Whether `ros2 topic echo` actually receives from it, end to end.
- The flash and RAM cost of Micro-CDR plus a second set of key expressions.

## Sources

- <https://github.com/ros2/rmw_zenoh>
- <https://github.com/ros2/rmw_zenoh/blob/jazzy/docs/design.md>
- <https://github.com/esol-community/rmw_zenoh_pico>
- <https://discourse.openrobotics.org/t/integrating-ros-2-with-microcontrollers-when-using-zenoh/43463>
- <https://docs.ros.org/en/jazzy/p/rmw_zenoh_cpp/>
- <https://github.com/Pico-ROS/Pico-ROS-software>
- <https://github.com/Pico-ROS/picoros-espidf-component>
- <https://github.com/eProsima/Micro-CDR>
