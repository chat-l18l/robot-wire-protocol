# E-stop Reset Protocol

## Route

| Direction | Zenoh key | Encoding |
|---|---|---|
| Gateway to motion core | `robot/v1/safety/estop/clear_reset` | `application/octet-stream` |

The gateway issues a Zenoh query on this key. The ESP32-P4 motion core is the
queryable and returns exactly one result frame.

## Version 1

Both frames use the 16-byte little-endian `RBC1` header declared in
`robot_wire_protocol.h`. The request is message type 5 and the result is message
type 6. Both frames are 20 bytes.

The request payload is four reserved zero bytes. The result payload contains a
result code, `needs_reset`, `estop_active`, and one reserved zero byte. The
gateway must only acknowledge the ROS `std_srvs/srv/Empty` request when the
result code is `OK`.

The ESP32-P4 owns all acceptance rules. It must reject a request while the
physical e-stop is active, when the safety sequence is incomplete, or while a
safety fault is active. An accepted request clears only the volatile
`needs_reset` latch and leaves the drive state at `STOPPED`; it never enables
motion directly.
