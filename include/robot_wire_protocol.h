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

#define ROBOT_LIGHT_COUNT ((size_t)4)
#define ROBOT_LIGHTS_PAYLOAD_SIZE ((size_t)16)
#define ROBOT_LIGHTS_MESSAGE_SIZE (ROBOT_WIRE_HEADER_SIZE + ROBOT_LIGHTS_PAYLOAD_SIZE)

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
    ROBOT_WIRE_INVALID_SOURCE = 9
} robot_wire_result_t;

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

robot_wire_result_t robot_wire_encode_lights_command(const robot_lights_command_t* command,
                                                      uint8_t* output, size_t output_size);
robot_wire_result_t robot_wire_decode_lights_command(const uint8_t* input, size_t input_size,
                                                      robot_lights_command_t* command);
robot_wire_result_t robot_wire_encode_lights_state(const robot_lights_state_t* state,
                                                    uint8_t* output, size_t output_size);
robot_wire_result_t robot_wire_decode_lights_state(const uint8_t* input, size_t input_size,
                                                    robot_lights_state_t* state);

#ifdef __cplusplus
}
#endif

#endif  // ROBOT_WIRE_PROTOCOL_H_
