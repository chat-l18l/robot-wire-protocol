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

/* Distinct in every field, so a swapped offset shows up rather than cancelling
 * out. Levels and changed bits stay inside their valid masks, which is what a
 * conforming vehicle publishes. */
static robot_pinout_event_t distinct_pinout_event(void)
{
    robot_pinout_event_t event = {0};
    event.uptime_ms = UINT64_C(0x0102030405060708);
    event.event_sequence = UINT32_C(0x11223344);
    event.boot_id = UINT32_C(0x55667788);
    event.inputs_valid = ROBOT_PINOUT_BIT(ROBOT_PINOUT_IN_BUTTON_UP) |
                         ROBOT_PINOUT_BIT(ROBOT_PINOUT_IN_BUTTON_DOWN) |
                         ROBOT_PINOUT_BIT(ROBOT_PINOUT_IN_BUTTON_ESC) |
                         ROBOT_PINOUT_BIT(ROBOT_PINOUT_IN_BUTTON_ENTER) |
                         ROBOT_PINOUT_BIT(ROBOT_PINOUT_IN_BUTTON_TEST) |
                         ROBOT_PINOUT_BIT(ROBOT_PINOUT_IN_TOGGLE_1) |
                         ROBOT_PINOUT_BIT(ROBOT_PINOUT_IN_TOGGLE_2) |
                         ROBOT_PINOUT_BIT(ROBOT_PINOUT_IN_TOGGLE_3) |
                         ROBOT_PINOUT_BIT(ROBOT_PINOUT_IN_ESTOP);
    event.inputs_level = ROBOT_PINOUT_BIT(ROBOT_PINOUT_IN_BUTTON_UP) |
                         ROBOT_PINOUT_BIT(ROBOT_PINOUT_IN_TOGGLE_1);
    event.inputs_changed = ROBOT_PINOUT_BIT(ROBOT_PINOUT_IN_TOGGLE_1);
    event.outputs_valid = ROBOT_PINOUT_BIT(ROBOT_PINOUT_OUT_HORN) |
                          ROBOT_PINOUT_BIT(ROBOT_PINOUT_OUT_HEARTBEAT) |
                          ROBOT_PINOUT_BIT(ROBOT_PINOUT_OUT_FULLSTOP_LED) |
                          ROBOT_PINOUT_BIT(ROBOT_PINOUT_OUT_ZENOH_LED) |
                          ROBOT_PINOUT_BIT(ROBOT_PINOUT_OUT_INDICATOR_LEFT) |
                          ROBOT_PINOUT_BIT(ROBOT_PINOUT_OUT_INDICATOR_RIGHT) |
                          ROBOT_PINOUT_BIT(ROBOT_PINOUT_OUT_AUTONOMOUS_LED) |
                          ROBOT_PINOUT_BIT(ROBOT_PINOUT_OUT_THROTTLE_DIR_REVERSE) |
                          ROBOT_PINOUT_BIT(ROBOT_PINOUT_OUT_EBRAKE) |
                          ROBOT_PINOUT_BIT(ROBOT_PINOUT_OUT_SKID_DIR_FL_REVERSE) |
                          ROBOT_PINOUT_BIT(ROBOT_PINOUT_OUT_SKID_DIR_FR_REVERSE) |
                          ROBOT_PINOUT_BIT(ROBOT_PINOUT_OUT_SKID_DIR_RL_REVERSE) |
                          ROBOT_PINOUT_BIT(ROBOT_PINOUT_OUT_SKID_DIR_RR_REVERSE);
    event.outputs_level = ROBOT_PINOUT_BIT(ROBOT_PINOUT_OUT_HEARTBEAT);
    event.outputs_changed = ROBOT_PINOUT_BIT(ROBOT_PINOUT_OUT_THROTTLE_DIR_REVERSE);
    return event;
}

static void test_pinout_event(void)
{
    const uint8_t golden[ROBOT_PINOUT_EVENT_MESSAGE_SIZE] = {
        0x52, 0x42, 0x43, 0x31, 0x01, 0x00, 0x07, 0x00, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x08, 0x07, 0x06, 0x05, 0x04, 0x03, 0x02, 0x01, 0x44, 0x33, 0x22, 0x11,
        0x88, 0x77, 0x66, 0x55, 0x21, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x7f, 0x00, 0x3f, 0x00, 0x00, 0x00, 0x00, 0x00};
    const robot_pinout_event_t event = distinct_pinout_event();
    robot_pinout_event_t rejected = distinct_pinout_event();
    robot_pinout_event_t decoded = {0};
    uint8_t encoded[ROBOT_PINOUT_EVENT_MESSAGE_SIZE] = {0};

    assert(robot_wire_encode_pinout_event(&event, encoded, sizeof(encoded)) == ROBOT_WIRE_OK);
    assert(memcmp(encoded, golden, sizeof(encoded)) == 0);
    assert(robot_wire_decode_pinout_event(encoded, sizeof(encoded), &decoded) == ROBOT_WIRE_OK);
    assert(memcmp(&event, &decoded, sizeof(event)) == 0);

    assert(robot_wire_encode_pinout_event(&event, encoded, sizeof(encoded) - 1U) ==
           ROBOT_WIRE_BUFFER_TOO_SMALL);
    assert(robot_wire_decode_pinout_event(encoded, sizeof(encoded) - 1U, &decoded) ==
           ROBOT_WIRE_INVALID_LENGTH);
    assert(robot_wire_encode_pinout_event(NULL, encoded, sizeof(encoded)) ==
           ROBOT_WIRE_NULL_ARGUMENT);

    /* A level bit the vehicle does not implement must not survive either
     * direction, or "not wired" would read as "not asserted". */
    rejected.inputs_level |= ROBOT_PINOUT_BIT(40);
    assert(robot_wire_encode_pinout_event(&rejected, encoded, sizeof(encoded)) ==
           ROBOT_WIRE_INVALID_PINOUT_MASK);
    encoded[37] = 0x01U; /* bit 40 of inputs_level, at payload offset 16 + 5 */
    assert(robot_wire_decode_pinout_event(encoded, sizeof(encoded), &decoded) ==
           ROBOT_WIRE_INVALID_PINOUT_MASK);
    assert(memcmp(&event, &decoded, sizeof(event)) == 0); /* untouched by the rejection */

    encoded[6] = 0x08U; /* an unassigned message type is rejected, not guessed */
    assert(robot_wire_decode_pinout_event(encoded, sizeof(encoded), &decoded) ==
           ROBOT_WIRE_INVALID_MESSAGE_TYPE);
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

    test_pinout_event();
    return 0;
}
