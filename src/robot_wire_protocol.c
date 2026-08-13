#include "robot_wire_protocol.h"

static void write_u16_le(uint8_t* output, uint16_t value)
{
    output[0] = (uint8_t)(value & UINT16_C(0x00ff));
    output[1] = (uint8_t)((value >> 8U) & UINT16_C(0x00ff));
}

static void write_u32_le(uint8_t* output, uint32_t value)
{
    output[0] = (uint8_t)(value & UINT32_C(0x000000ff));
    output[1] = (uint8_t)((value >> 8U) & UINT32_C(0x000000ff));
    output[2] = (uint8_t)((value >> 16U) & UINT32_C(0x000000ff));
    output[3] = (uint8_t)((value >> 24U) & UINT32_C(0x000000ff));
}

static void write_u64_le(uint8_t* output, uint64_t value)
{
    size_t index;
    for (index = 0U; index < 8U; ++index) {
        output[index] = (uint8_t)((value >> (8U * index)) & UINT64_C(0xff));
    }
}

static uint16_t read_u16_le(const uint8_t* input)
{
    return (uint16_t)((uint16_t)input[0] | ((uint16_t)input[1] << 8U));
}

static uint32_t read_u32_le(const uint8_t* input)
{
    return (uint32_t)((uint32_t)input[0] | ((uint32_t)input[1] << 8U) |
                      ((uint32_t)input[2] << 16U) | ((uint32_t)input[3] << 24U));
}

static uint64_t read_u64_le(const uint8_t* input)
{
    uint64_t value = UINT64_C(0);
    size_t index;
    for (index = 0U; index < 8U; ++index) {
        value |= (uint64_t)input[index] << (8U * index);
    }
    return value;
}

static void encode_header(uint8_t* output, uint16_t message_type, uint16_t payload_size,
                          uint32_t sequence)
{
    write_u32_le(output, ROBOT_WIRE_MAGIC);
    write_u16_le(&output[4], ROBOT_WIRE_PROTOCOL_VERSION);
    write_u16_le(&output[6], message_type);
    write_u16_le(&output[8], payload_size);
    write_u16_le(&output[10], UINT16_C(0));
    write_u32_le(&output[12], sequence);
}

static robot_wire_result_t decode_header(const uint8_t* input, size_t input_size,
                                          uint16_t expected_message_type,
                                          uint16_t expected_payload_size, uint32_t* sequence)
{
    if (input == NULL || sequence == NULL) return ROBOT_WIRE_NULL_ARGUMENT;
    if (input_size != ROBOT_WIRE_HEADER_SIZE + (size_t)expected_payload_size) {
        return ROBOT_WIRE_INVALID_LENGTH;
    }
    if (read_u32_le(input) != ROBOT_WIRE_MAGIC) return ROBOT_WIRE_INVALID_MAGIC;
    if (read_u16_le(&input[4]) != ROBOT_WIRE_PROTOCOL_VERSION) return ROBOT_WIRE_INVALID_VERSION;
    if (read_u16_le(&input[6]) != expected_message_type) return ROBOT_WIRE_INVALID_MESSAGE_TYPE;
    if (read_u16_le(&input[8]) != expected_payload_size) return ROBOT_WIRE_INVALID_LENGTH;
    *sequence = read_u32_le(&input[12]);
    return ROBOT_WIRE_OK;
}

static void encode_rgb_values(const robot_rgb_t* lights, uint8_t* output)
{
    size_t index;
    for (index = 0U; index < ROBOT_LIGHT_COUNT; ++index) {
        const size_t offset = index * 3U;
        output[offset] = lights[index].red;
        output[offset + 1U] = lights[index].green;
        output[offset + 2U] = lights[index].blue;
    }
}

static void decode_rgb_values(const uint8_t* input, robot_rgb_t* lights)
{
    size_t index;
    for (index = 0U; index < ROBOT_LIGHT_COUNT; ++index) {
        const size_t offset = index * 3U;
        lights[index].red = input[offset];
        lights[index].green = input[offset + 1U];
        lights[index].blue = input[offset + 2U];
    }
}

robot_wire_result_t robot_wire_encode_lights_command(const robot_lights_command_t* command,
                                                      uint8_t* output, size_t output_size)
{
    if (command == NULL || output == NULL) return ROBOT_WIRE_NULL_ARGUMENT;
    if (output_size < ROBOT_LIGHTS_MESSAGE_SIZE) return ROBOT_WIRE_BUFFER_TOO_SMALL;
    encode_header(output, ROBOT_WIRE_MESSAGE_LIGHTS_COMMAND, (uint16_t)ROBOT_LIGHTS_PAYLOAD_SIZE,
                  command->sequence);
    output[16] = (uint8_t)ROBOT_LIGHT_COUNT;
    output[17] = UINT8_C(0);
    output[18] = UINT8_C(0);
    output[19] = UINT8_C(0);
    encode_rgb_values(command->lights, &output[20]);
    return ROBOT_WIRE_OK;
}

robot_wire_result_t robot_wire_decode_lights_command(const uint8_t* input, size_t input_size,
                                                      robot_lights_command_t* command)
{
    robot_wire_result_t result;
    uint32_t sequence;
    if (command == NULL) return ROBOT_WIRE_NULL_ARGUMENT;
    result = decode_header(input, input_size, ROBOT_WIRE_MESSAGE_LIGHTS_COMMAND,
                           (uint16_t)ROBOT_LIGHTS_PAYLOAD_SIZE, &sequence);
    if (result != ROBOT_WIRE_OK) return result;
    if (input[16] != (uint8_t)ROBOT_LIGHT_COUNT) return ROBOT_WIRE_INVALID_LIGHT_COUNT;
    command->sequence = sequence;
    decode_rgb_values(&input[20], command->lights);
    return ROBOT_WIRE_OK;
}

robot_wire_result_t robot_wire_encode_lights_state(const robot_lights_state_t* state,
                                                    uint8_t* output, size_t output_size)
{
    if (state == NULL || output == NULL) return ROBOT_WIRE_NULL_ARGUMENT;
    if (output_size < ROBOT_LIGHTS_MESSAGE_SIZE) return ROBOT_WIRE_BUFFER_TOO_SMALL;
    if (state->result > (uint8_t)ROBOT_LIGHTS_RESULT_BUSY_SOURCE) return ROBOT_WIRE_INVALID_RESULT;
    if (state->active_source > (uint8_t)ROBOT_LIGHTS_SOURCE_EDUCATION_JSON) return ROBOT_WIRE_INVALID_SOURCE;
    encode_header(output, ROBOT_WIRE_MESSAGE_LIGHTS_STATE, (uint16_t)ROBOT_LIGHTS_PAYLOAD_SIZE,
                  state->sequence);
    output[16] = (uint8_t)ROBOT_LIGHT_COUNT;
    output[17] = state->result;
    output[18] = state->active_source;
    output[19] = UINT8_C(0);
    encode_rgb_values(state->lights, &output[20]);
    return ROBOT_WIRE_OK;
}

robot_wire_result_t robot_wire_decode_lights_state(const uint8_t* input, size_t input_size,
                                                    robot_lights_state_t* state)
{
    robot_wire_result_t result;
    uint32_t sequence;
    if (state == NULL) return ROBOT_WIRE_NULL_ARGUMENT;
    result = decode_header(input, input_size, ROBOT_WIRE_MESSAGE_LIGHTS_STATE,
                           (uint16_t)ROBOT_LIGHTS_PAYLOAD_SIZE, &sequence);
    if (result != ROBOT_WIRE_OK) return result;
    if (input[16] != (uint8_t)ROBOT_LIGHT_COUNT) return ROBOT_WIRE_INVALID_LIGHT_COUNT;
    if (input[17] > (uint8_t)ROBOT_LIGHTS_RESULT_BUSY_SOURCE) return ROBOT_WIRE_INVALID_RESULT;
    if (input[18] > (uint8_t)ROBOT_LIGHTS_SOURCE_EDUCATION_JSON) return ROBOT_WIRE_INVALID_SOURCE;
    state->sequence = sequence;
    state->result = input[17];
    state->active_source = input[18];
    decode_rgb_values(&input[20], state->lights);
    return ROBOT_WIRE_OK;
}

/* A level or changed bit outside the valid mask would claim a signal the
 * vehicle does not implement, which is exactly the confusion the valid mask
 * exists to prevent. Rejecting rather than silently masking keeps encode and
 * decode inverses of each other, and matches how this codec treats every other
 * out-of-range field. */
static robot_wire_result_t check_pinout_masks(uint64_t level, uint64_t changed, uint64_t valid)
{
    if ((level & ~valid) != UINT64_C(0)) return ROBOT_WIRE_INVALID_PINOUT_MASK;
    if ((changed & ~valid) != UINT64_C(0)) return ROBOT_WIRE_INVALID_PINOUT_MASK;
    return ROBOT_WIRE_OK;
}

robot_wire_result_t robot_wire_encode_pinout_event(const robot_pinout_event_t* event,
                                                    uint8_t* output, size_t output_size)
{
    robot_wire_result_t result;
    if (event == NULL || output == NULL) return ROBOT_WIRE_NULL_ARGUMENT;
    if (output_size < ROBOT_PINOUT_EVENT_MESSAGE_SIZE) return ROBOT_WIRE_BUFFER_TOO_SMALL;
    result = check_pinout_masks(event->inputs_level, event->inputs_changed, event->inputs_valid);
    if (result != ROBOT_WIRE_OK) return result;
    result = check_pinout_masks(event->outputs_level, event->outputs_changed, event->outputs_valid);
    if (result != ROBOT_WIRE_OK) return result;
    /* Header sequence is zero: an event answers no request. */
    encode_header(output, ROBOT_WIRE_MESSAGE_PINOUT_EVENT,
                  (uint16_t)ROBOT_PINOUT_EVENT_PAYLOAD_SIZE, UINT32_C(0));
    write_u64_le(&output[16], event->uptime_ms);
    write_u32_le(&output[24], event->event_sequence);
    write_u32_le(&output[28], event->boot_id);
    write_u64_le(&output[32], event->inputs_level);
    write_u64_le(&output[40], event->inputs_changed);
    write_u64_le(&output[48], event->inputs_valid);
    write_u64_le(&output[56], event->outputs_level);
    write_u64_le(&output[64], event->outputs_changed);
    write_u64_le(&output[72], event->outputs_valid);
    return ROBOT_WIRE_OK;
}

robot_wire_result_t robot_wire_decode_pinout_event(const uint8_t* input, size_t input_size,
                                                    robot_pinout_event_t* event)
{
    robot_wire_result_t result;
    uint32_t header_sequence;
    robot_pinout_event_t decoded;
    if (event == NULL) return ROBOT_WIRE_NULL_ARGUMENT;
    result = decode_header(input, input_size, ROBOT_WIRE_MESSAGE_PINOUT_EVENT,
                           (uint16_t)ROBOT_PINOUT_EVENT_PAYLOAD_SIZE, &header_sequence);
    if (result != ROBOT_WIRE_OK) return result;
    decoded.uptime_ms = read_u64_le(&input[16]);
    decoded.event_sequence = read_u32_le(&input[24]);
    decoded.boot_id = read_u32_le(&input[28]);
    decoded.inputs_level = read_u64_le(&input[32]);
    decoded.inputs_changed = read_u64_le(&input[40]);
    decoded.inputs_valid = read_u64_le(&input[48]);
    decoded.outputs_level = read_u64_le(&input[56]);
    decoded.outputs_changed = read_u64_le(&input[64]);
    decoded.outputs_valid = read_u64_le(&input[72]);
    result = check_pinout_masks(decoded.inputs_level, decoded.inputs_changed, decoded.inputs_valid);
    if (result != ROBOT_WIRE_OK) return result;
    result =
        check_pinout_masks(decoded.outputs_level, decoded.outputs_changed, decoded.outputs_valid);
    if (result != ROBOT_WIRE_OK) return result;
    /* Populated only once every field has passed, so a rejected frame never
     * leaves the caller's struct half written. */
    *event = decoded;
    return ROBOT_WIRE_OK;
}

robot_wire_result_t robot_wire_encode_estop_clear_reset_request(
    const robot_estop_clear_reset_request_t* request, uint8_t* output, size_t output_size)
{
    if (request == NULL || output == NULL) return ROBOT_WIRE_NULL_ARGUMENT;
    if (output_size < ROBOT_ESTOP_CLEAR_RESET_MESSAGE_SIZE) return ROBOT_WIRE_BUFFER_TOO_SMALL;
    encode_header(output, ROBOT_WIRE_MESSAGE_ESTOP_CLEAR_RESET_REQUEST,
                  (uint16_t)ROBOT_ESTOP_CLEAR_RESET_PAYLOAD_SIZE, request->sequence);
    write_u32_le(&output[16], UINT32_C(0));
    return ROBOT_WIRE_OK;
}

robot_wire_result_t robot_wire_decode_estop_clear_reset_request(
    const uint8_t* input, size_t input_size, robot_estop_clear_reset_request_t* request)
{
    if (request == NULL) return ROBOT_WIRE_NULL_ARGUMENT;
    return decode_header(input, input_size, ROBOT_WIRE_MESSAGE_ESTOP_CLEAR_RESET_REQUEST,
                         (uint16_t)ROBOT_ESTOP_CLEAR_RESET_PAYLOAD_SIZE, &request->sequence);
}

robot_wire_result_t robot_wire_encode_estop_clear_reset_state(
    const robot_estop_clear_reset_state_t* state, uint8_t* output, size_t output_size)
{
    if (state == NULL || output == NULL) return ROBOT_WIRE_NULL_ARGUMENT;
    if (output_size < ROBOT_ESTOP_CLEAR_RESET_MESSAGE_SIZE) return ROBOT_WIRE_BUFFER_TOO_SMALL;
    if (state->result > (uint8_t)ROBOT_ESTOP_CLEAR_RESET_INTERNAL_ERROR) {
        return ROBOT_WIRE_INVALID_ESTOP_CLEAR_RESET_RESULT;
    }
    encode_header(output, ROBOT_WIRE_MESSAGE_ESTOP_CLEAR_RESET_RESULT,
                  (uint16_t)ROBOT_ESTOP_CLEAR_RESET_PAYLOAD_SIZE, state->sequence);
    output[16] = state->result;
    output[17] = state->needs_reset;
    output[18] = state->estop_active;
    output[19] = UINT8_C(0);
    return ROBOT_WIRE_OK;
}

robot_wire_result_t robot_wire_decode_estop_clear_reset_state(
    const uint8_t* input, size_t input_size, robot_estop_clear_reset_state_t* state)
{
    robot_wire_result_t result;
    uint32_t sequence;
    if (state == NULL) return ROBOT_WIRE_NULL_ARGUMENT;
    result = decode_header(input, input_size, ROBOT_WIRE_MESSAGE_ESTOP_CLEAR_RESET_RESULT,
                           (uint16_t)ROBOT_ESTOP_CLEAR_RESET_PAYLOAD_SIZE, &sequence);
    if (result != ROBOT_WIRE_OK) return result;
    if (input[16] > (uint8_t)ROBOT_ESTOP_CLEAR_RESET_INTERNAL_ERROR) {
        return ROBOT_WIRE_INVALID_ESTOP_CLEAR_RESET_RESULT;
    }
    if (input[17] > UINT8_C(1) || input[18] > UINT8_C(1)) return ROBOT_WIRE_INVALID_LENGTH;
    state->sequence = sequence;
    state->result = input[16];
    state->needs_reset = input[17];
    state->estop_active = input[18];
    return ROBOT_WIRE_OK;
}
