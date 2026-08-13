#ifndef ROBOT_WIRE_PROTOCOL_H_
#define ROBOT_WIRE_PROTOCOL_H_

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ROBOT_WIRE_MAGIC UINT32_C(0x31434252)
#define ROBOT_WIRE_PROTOCOL_VERSION UINT16_C(1)
#define ROBOT_WIRE_HEADER_SIZE ((size_t)16)

#define ROBOT_WIRE_MESSAGE_LIGHTS_COMMAND UINT16_C(3)
#define ROBOT_WIRE_MESSAGE_LIGHTS_STATE UINT16_C(4)
#define ROBOT_WIRE_MESSAGE_ESTOP_CLEAR_RESET_REQUEST UINT16_C(5)
#define ROBOT_WIRE_MESSAGE_ESTOP_CLEAR_RESET_RESULT UINT16_C(6)
#define ROBOT_WIRE_MESSAGE_PINOUT_EVENT UINT16_C(7)

#define ROBOT_LIGHT_COUNT ((size_t)4)
#define ROBOT_LIGHTS_PAYLOAD_SIZE ((size_t)16)
#define ROBOT_LIGHTS_MESSAGE_SIZE (ROBOT_WIRE_HEADER_SIZE + ROBOT_LIGHTS_PAYLOAD_SIZE)

#define ROBOT_ESTOP_CLEAR_RESET_PAYLOAD_SIZE ((size_t)4)
#define ROBOT_ESTOP_CLEAR_RESET_MESSAGE_SIZE \
    (ROBOT_WIRE_HEADER_SIZE + ROBOT_ESTOP_CLEAR_RESET_PAYLOAD_SIZE)

/* Every vehicle-originated event payload starts with this prefix. It is 16
 * bytes so the fields that follow stay 8-byte aligned relative to the frame. */
#define ROBOT_EVENT_PREFIX_SIZE ((size_t)16)

#define ROBOT_PINOUT_EVENT_PAYLOAD_SIZE ((size_t)64)
#define ROBOT_PINOUT_EVENT_MESSAGE_SIZE \
    (ROBOT_WIRE_HEADER_SIZE + ROBOT_PINOUT_EVENT_PAYLOAD_SIZE)

#define ROBOT_PINOUT_BIT(index) (UINT64_C(1) << (index))

typedef enum robot_wire_result {
    ROBOT_WIRE_OK = 0,
    ROBOT_WIRE_NULL_ARGUMENT = 1,
    ROBOT_WIRE_BUFFER_TOO_SMALL = 2,
    ROBOT_WIRE_INVALID_MAGIC = 3,
    ROBOT_WIRE_INVALID_VERSION = 4,
    ROBOT_WIRE_INVALID_MESSAGE_TYPE = 5,
    ROBOT_WIRE_INVALID_LENGTH = 6,
    ROBOT_WIRE_INVALID_LIGHT_COUNT = 7,
    ROBOT_WIRE_INVALID_RESULT = 8,
    ROBOT_WIRE_INVALID_SOURCE = 9,
    ROBOT_WIRE_INVALID_ESTOP_CLEAR_RESET_RESULT = 10,
    ROBOT_WIRE_INVALID_PINOUT_MASK = 11
} robot_wire_result_t;

/* Pinout bit assignment.
 *
 * A bit names a signal, never an expander pin: where a signal is wired is the
 * firmware's private business, so a board revision cannot silently change what
 * a bit means. The e-brake is native GPIO rather than an expander line, which
 * is the case that settles it.
 *
 * A bit is 1 when the named condition holds, never when the pin reads high.
 * Buttons read low when pressed and the e-brake is active-low; neither fact
 * reaches the wire. Bits are named for what 1 means, hence the _REVERSE
 * suffixes.
 *
 * Inputs and outputs are numbered in separate spaces, both starting at 0, and
 * numbers are handed out in blocks of 16 so related signals stay together as
 * hardware is added. A number is permanent and a retired signal's number is
 * never reused. A bit that is 0 in the corresponding valid mask says nothing
 * about its level bit; see docs/PINOUT_PROTOCOL.md.
 */
typedef enum robot_pinout_input_bit {
    /* 0..15 operator panel */
    ROBOT_PINOUT_IN_BUTTON_UP = 0,
    ROBOT_PINOUT_IN_BUTTON_DOWN = 1,
    ROBOT_PINOUT_IN_BUTTON_ESC = 2,
    ROBOT_PINOUT_IN_BUTTON_ENTER = 3,
    ROBOT_PINOUT_IN_BUTTON_TEST = 4,
    ROBOT_PINOUT_IN_TOGGLE_1 = 5,
    ROBOT_PINOUT_IN_TOGGLE_2 = 6,
    ROBOT_PINOUT_IN_TOGGLE_3 = 7,
    /* 16..31 safety chain */
    ROBOT_PINOUT_IN_ESTOP = 16
    /* 32..47 drivetrain feedback, 48..63 auxiliary connector: unassigned */
} robot_pinout_input_bit_t;

typedef enum robot_pinout_output_bit {
    /* 0..15 signalling and indication */
    ROBOT_PINOUT_OUT_HORN = 0,
    ROBOT_PINOUT_OUT_HEARTBEAT = 1,
    ROBOT_PINOUT_OUT_FULLSTOP_LED = 2,
    ROBOT_PINOUT_OUT_ZENOH_LED = 3,
    ROBOT_PINOUT_OUT_INDICATOR_LEFT = 4,
    ROBOT_PINOUT_OUT_INDICATOR_RIGHT = 5,
    ROBOT_PINOUT_OUT_AUTONOMOUS_LED = 6,
    /* 16..31 drivetrain control */
    ROBOT_PINOUT_OUT_THROTTLE_DIR_REVERSE = 16,
    ROBOT_PINOUT_OUT_EBRAKE = 17,
    ROBOT_PINOUT_OUT_SKID_DIR_FL_REVERSE = 18,
    ROBOT_PINOUT_OUT_SKID_DIR_FR_REVERSE = 19,
    ROBOT_PINOUT_OUT_SKID_DIR_RL_REVERSE = 20,
    ROBOT_PINOUT_OUT_SKID_DIR_RR_REVERSE = 21
    /* 32..47 auxiliary connector, 48..63 expansion: unassigned */
} robot_pinout_output_bit_t;

typedef enum robot_lights_result {
    ROBOT_LIGHTS_RESULT_OK = 0,
    ROBOT_LIGHTS_RESULT_INVALID_COMMAND = 1,
    ROBOT_LIGHTS_RESULT_APPLY_FAILED = 2,
    ROBOT_LIGHTS_RESULT_OVERRIDDEN = 3,
    ROBOT_LIGHTS_RESULT_BUSY_SOURCE = 4
} robot_lights_result_t;

typedef enum robot_lights_source {
    ROBOT_LIGHTS_SOURCE_NONE = 0,
    ROBOT_LIGHTS_SOURCE_SAFETY = 1,
    ROBOT_LIGHTS_SOURCE_ROS_BINARY = 2,
    ROBOT_LIGHTS_SOURCE_EDUCATION_JSON = 3
} robot_lights_source_t;

typedef enum robot_estop_clear_reset_result {
    ROBOT_ESTOP_CLEAR_RESET_OK = 0,
    ROBOT_ESTOP_CLEAR_RESET_ESTOP_ACTIVE = 1,
    ROBOT_ESTOP_CLEAR_RESET_NO_RESET_PENDING = 2,
    ROBOT_ESTOP_CLEAR_RESET_SAFETY_SEQUENCE_INCOMPLETE = 3,
    ROBOT_ESTOP_CLEAR_RESET_FAULT_ACTIVE = 4,
    ROBOT_ESTOP_CLEAR_RESET_INTERNAL_ERROR = 5
} robot_estop_clear_reset_result_t;

typedef struct robot_rgb
{
    uint8_t red;
    uint8_t green;
    uint8_t blue;
} robot_rgb_t;

typedef struct robot_lights_command
{
    uint32_t sequence;
    robot_rgb_t lights[ROBOT_LIGHT_COUNT];
} robot_lights_command_t;

typedef struct robot_lights_state
{
    uint32_t sequence;
    uint8_t result;
    uint8_t active_source;
    robot_rgb_t lights[ROBOT_LIGHT_COUNT];
} robot_lights_state_t;

/* Vehicle-originated pinout snapshot.
 *
 * Not a reply: nothing was asked. The header sequence is therefore reserved and
 * zero, and the counter that matters is event_sequence in the payload prefix.
 *
 * The masks are snapshots rather than edges, because a dropped reply is retried
 * while a dropped edge is gone forever. The *_changed masks are sticky since
 * the previous event, so a pulse that came and went between two publications is
 * still visible: the level looks unchanged and the changed bit is set.
 *
 * A level or changed bit outside its valid mask is rejected by both encoder and
 * decoder, so no conforming frame can ever report a signal the vehicle does not
 * implement. That is what keeps "not wired" distinguishable from "not asserted",
 * which matters most for the e-stop.
 */
typedef struct robot_pinout_event
{
    uint64_t uptime_ms;       /* since vehicle boot; not comparable to ROS time */
    uint32_t event_sequence;  /* monotonic per vehicle, independent of commands */
    uint32_t boot_id;         /* opaque; compare for equality only, never order */
    uint64_t inputs_level;
    uint64_t inputs_changed;
    uint64_t inputs_valid;
    uint64_t outputs_level;
    uint64_t outputs_changed;
    uint64_t outputs_valid;
} robot_pinout_event_t;

typedef struct robot_estop_clear_reset_request
{
    uint32_t sequence;
} robot_estop_clear_reset_request_t;

typedef struct robot_estop_clear_reset_state
{
    uint32_t sequence;
    uint8_t result;
    uint8_t needs_reset;
    uint8_t estop_active;
} robot_estop_clear_reset_state_t;

robot_wire_result_t robot_wire_encode_lights_command(const robot_lights_command_t* command,
                                                      uint8_t* output, size_t output_size);
robot_wire_result_t robot_wire_decode_lights_command(const uint8_t* input, size_t input_size,
                                                      robot_lights_command_t* command);
robot_wire_result_t robot_wire_encode_lights_state(const robot_lights_state_t* state,
                                                    uint8_t* output, size_t output_size);
robot_wire_result_t robot_wire_decode_lights_state(const uint8_t* input, size_t input_size,
                                                     robot_lights_state_t* state);
robot_wire_result_t robot_wire_encode_estop_clear_reset_request(
    const robot_estop_clear_reset_request_t* request, uint8_t* output, size_t output_size);
robot_wire_result_t robot_wire_decode_estop_clear_reset_request(
    const uint8_t* input, size_t input_size, robot_estop_clear_reset_request_t* request);
robot_wire_result_t robot_wire_encode_estop_clear_reset_state(
    const robot_estop_clear_reset_state_t* state, uint8_t* output, size_t output_size);
robot_wire_result_t robot_wire_decode_estop_clear_reset_state(
    const uint8_t* input, size_t input_size, robot_estop_clear_reset_state_t* state);
robot_wire_result_t robot_wire_encode_pinout_event(const robot_pinout_event_t* event,
                                                    uint8_t* output, size_t output_size);
robot_wire_result_t robot_wire_decode_pinout_event(const uint8_t* input, size_t input_size,
                                                    robot_pinout_event_t* event);

#ifdef __cplusplus
}
#endif

#endif  // ROBOT_WIRE_PROTOCOL_H_
