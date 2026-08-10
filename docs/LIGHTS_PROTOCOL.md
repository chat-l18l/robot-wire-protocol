# Lights Protocol

## Routes

| Direction | Zenoh key | Encoding |
|---|---|---|
| Gateway to motion core | `robot/v1/io/lights/cmd` | `application/octet-stream` |
| Motion core to gateway | `robot/v1/io/lights/state` | `application/octet-stream` |

## Version 1

Every frame uses the 16-byte little-endian `RBC1` header declared in
`robot_wire_protocol.h`. `LIGHTS_COMMAND` is message type 3 and `LIGHTS_STATE`
is type 4. Both frames are 32 bytes.

The payload starts with an exact light count of four. RGB values follow in this
order: front-right, front-left, rear-left, rear-right. State frames also carry a
result code and active source. The receiver rejects an invalid magic, version,
message type, length, light count, result or source without applying hardware.

The motion core owns arbitration: local safety/fault state has priority over the
ROS binary route and education JSON route. The state frame echoes the command
sequence and reports the actually applied values.
