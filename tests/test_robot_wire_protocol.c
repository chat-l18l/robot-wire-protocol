#include <assert.h>
#include <stdint.h>
#include <string.h>

#include "robot_wire_protocol.h"

static robot_lights_command_t distinct_command(void)
{
    robot_lights_command_t command = {0};
    command.sequence = UINT32_C(0x01020304);
    command.lights[0] = (robot_rgb_t){UINT8_C(1), UINT8_C(2), UINT8_C(3)};
    command.lights[1] = (robot_rgb_t){UINT8_C(4), UINT8_C(5), UINT8_C(6)};
    command.lights[2] = (robot_rgb_t){UINT8_C(7), UINT8_C(8), UINT8_C(9)};
    command.lights[3] = (robot_rgb_t){UINT8_C(10), UINT8_C(11), UINT8_C(12)};
    return command;
}

int main(void)
{
    const uint8_t golden[ROBOT_LIGHTS_MESSAGE_SIZE] = {
        0x52, 0x42, 0x43, 0x31, 0x01, 0x00, 0x03, 0x00, 0x10, 0x00, 0x00,
        0x00, 0x04, 0x03, 0x02, 0x01, 0x04, 0x00, 0x00, 0x00, 0x01, 0x02,
        0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c};
    const robot_lights_command_t command = distinct_command();
    uint8_t encoded[ROBOT_LIGHTS_MESSAGE_SIZE] = {0};
    robot_lights_command_t decoded = {0};
    robot_lights_state_t state = {0};
    robot_lights_state_t decoded_state = {0};
    robot_estop_clear_reset_request_t reset_request = {UINT32_C(0x11223344)};
    robot_estop_clear_reset_request_t decoded_reset_request = {0};
    robot_estop_clear_reset_state_t reset_state = {
        UINT32_C(0x11223344), ROBOT_ESTOP_CLEAR_RESET_OK, UINT8_C(0), UINT8_C(0)};
    robot_estop_clear_reset_state_t decoded_reset_state = {0};
    uint8_t reset_encoded[ROBOT_ESTOP_CLEAR_RESET_MESSAGE_SIZE] = {0};

    assert(robot_wire_encode_lights_command(&command, encoded, sizeof(encoded)) == ROBOT_WIRE_OK);
    assert(memcmp(encoded, golden, sizeof(encoded)) == 0);
    assert(robot_wire_decode_lights_command(encoded, sizeof(encoded), &decoded) == ROBOT_WIRE_OK);
    assert(memcmp(&command, &decoded, sizeof(command)) == 0);
    assert(robot_wire_decode_lights_command(encoded, sizeof(encoded) - 1U, &decoded) == ROBOT_WIRE_INVALID_LENGTH);
    encoded[0] = 0U;
    assert(robot_wire_decode_lights_command(encoded, sizeof(encoded), &decoded) == ROBOT_WIRE_INVALID_MAGIC);
    assert(robot_wire_encode_lights_command(NULL, encoded, sizeof(encoded)) == ROBOT_WIRE_NULL_ARGUMENT);

    state.sequence = command.sequence;
    state.result = ROBOT_LIGHTS_RESULT_OK;
    state.active_source = ROBOT_LIGHTS_SOURCE_ROS_BINARY;
    memcpy(state.lights, command.lights, sizeof(state.lights));
    assert(robot_wire_encode_lights_state(&state, encoded, sizeof(encoded)) == ROBOT_WIRE_OK);
    assert(robot_wire_decode_lights_state(encoded, sizeof(encoded), &decoded_state) == ROBOT_WIRE_OK);
    assert(memcmp(&state, &decoded_state, sizeof(state)) == 0);

    assert(robot_wire_encode_estop_clear_reset_request(&reset_request, reset_encoded,
                                                        sizeof(reset_encoded)) == ROBOT_WIRE_OK);
    assert(robot_wire_decode_estop_clear_reset_request(reset_encoded, sizeof(reset_encoded),
                                                        &decoded_reset_request) == ROBOT_WIRE_OK);
    assert(memcmp(&reset_request, &decoded_reset_request, sizeof(reset_request)) == 0);
    assert(robot_wire_encode_estop_clear_reset_state(&reset_state, reset_encoded,
                                                      sizeof(reset_encoded)) == ROBOT_WIRE_OK);
    assert(robot_wire_decode_estop_clear_reset_state(reset_encoded, sizeof(reset_encoded),
                                                      &decoded_reset_state) == ROBOT_WIRE_OK);
    assert(memcmp(&reset_state, &decoded_reset_state, sizeof(reset_state)) == 0);
    return 0;
}
