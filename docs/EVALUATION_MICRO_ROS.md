# Evaluation: micro-ROS instead of this protocol

> **Status: evaluated, not adopted.** Recorded so the reasoning survives and
> the question is not re-argued from scratch. Revisit conditions are at the end.
> Raised: 2026-08-13, while designing device-initiated events.

## The question

[Research: device-initiated events](RESEARCH_DEVICE_INITIATED_EVENTS.md) found
that this protocol cannot express a message the vehicle originates. micro-ROS
would make the microcontroller a ROS 2 node outright. Does that solve the
problem, and what does it cost in version coupling?

## It solves the problem completely

By dissolving the boundary that created it.

With micro-ROS the ESP32 runs `rclc` over Micro XRCE-DDS and publishes to real
ROS 2 topics through an agent. A GPIO change becomes `rcl_publish()`. There is
no event category to design, no timestamp field to invent, no boot identifier
to add: `std_msgs/Header` and the whole ROS type system come with it, and the
publish/subscribe direction is symmetric by construction.

That is a real strength and should not be talked down. Every problem the events
research raised is a problem this protocol has *because* it is not ROS.

## The cost is coupling, and it is as hard as it looks

Type support is generated at build time. From `rosidl_typesupport_microxrcedds`:

> Support for serialization / deserialization code, **generated during the
> build process**, for each ROS 2 interface.

Interface definitions therefore live in the firmware binary. The ESP-IDF
component's `extra_packages/` directory is where custom message packages are
dropped to be compiled into `libmicroros`, which confirms the model.

What that means in practice:

| Change | Rebuild and reflash every vehicle? |
| --- | --- |
| Node logic on the Orin | No |
| A `.msg` definition the vehicle uses | **Yes** |
| ROS 2 distribution upgrade | **Yes** |
| A new topic the vehicle must publish | **Yes** |

The distribution coupling is explicit throughout the stack: the agent image is
pinned per distro (`microros/micro-ros-agent:jazzy` with Jazzy), and both
`micro_ros_setup` and `micro_ros_espidf_component` carry per-distro branches
(humble, jazzy, kilted, rolling). `micro_ros_setup` currently marks Humble as
end of life.

So the honest answer to "must I rebuild the ESP32 for every ROS change" is: not
for every change, but for every *interface* change and every distribution
upgrade. Those are exactly the changes hardest to perform on vehicles that are
already with customers.

## A direct blocker today: ESP32-P4 is not supported

`micro_ros_espidf_component` states it is tested on ESP32, ESP32-S2, ESP32-S3,
ESP32-C3 and ESP32-C6. **ESP32-P4 is not listed.**

P4 is RISC-V like the C3 and C6, so this is unlikely to be a fundamental
obstacle, but untested and unsupported is not a basis for a vehicle platform.
This alone settles the question for the current hardware.

## What the current architecture buys instead

The separation this repository exists to enforce pays off precisely here. The
ESP32 does not know ROS 2 exists. The wire protocol is versioned on its own
schedule, by us. A distribution upgrade on the Orin touches the gateway and
nothing else, and **no vehicle is reflashed**.

micro-ROS trades that away for convenience. For a single prototype on a bench
that is a good trade: less code, no protocol to design, no events research to
write. For vehicles at customer sites it means every fleet update is gated on
another project's release cycle and on distribution end-of-life dates nobody
here controls.

That is the trade, stated plainly. It is not that micro-ROS is worse; it is
that it optimises for a different problem than the one this fleet has.

## Conclusion

Not adopted. The P4 gap makes it moot for now, and the coupling would be the
wrong trade even without it, given vehicles that cannot be casually recalled.

The events problem is solved in this protocol instead: a message category for
vehicle-originated state, costing a timestamp, a boot identifier and an event
sequence. That is real work, but it is done once and the decoupling is kept.

## When to revisit

Any of these should reopen it:

1. **A subsystem lands on a supported chip.** If a C6 co-processor takes on ROS
   traffic of its own, micro-ROS on that part is a fair question, and the answer
   need not match the P4's.
2. **ESP32-P4 becomes supported and tested** by the component.
3. **The interface starts changing faster than the firmware ships.** If the
   protocol needs a new message type every sprint, the argument that we control
   the schedule stops being worth much.
4. **A single vehicle, never fielded.** For a bench-only research platform the
   coupling costs nothing and the saved work is substantial.

## What was verified rather than assumed

- Build-time type generation: `rosidl_typesupport_microxrcedds` README.
- Supported chips and ESP-IDF versions, and the absence of P4:
  `micro_ros_espidf_component` README.
- Per-distro agent images and setup branches, and the Humble EOL note:
  `micro_ros_setup` and the micro-ROS agent documentation.

Not verified, and worth checking before acting on any revisit condition:
whether P4 works unofficially, the flash and RAM footprint of `libmicroros` on
this class of part, and whether an interface change can ever be made
backward-compatible enough to avoid reflashing in practice.

## Sources

- <https://github.com/micro-ROS/rosidl_typesupport_microxrcedds>
- <https://github.com/micro-ROS/micro_ros_espidf_component>
- <https://github.com/micro-ROS/micro_ros_setup>
- <https://micro.ros.org/docs/concepts/middleware/Micro_XRCE-DDS/>
