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

static uint16_t read_u16_le(const uint8_t* input)
{
    return (uint16_t)((uint16_t)input[0] | ((uint16_t)input[1] << 8U));
}

static uint32_t read_u32_le(const uint8_t* input)
{
    return (uint32_t)((uint32_t)input[0] | ((uint32_t)input[1] << 8U) |
                      ((uint32_t)input[2] << 16U) | ((uint32_t)input[3] << 24U));
}

static void encode_header(uint8_t* output, uint16_t message_type, uint32_t sequence)
{
    write_u32_le(output, ROBOT_WIRE_MAGIC);
    write_u16_le(&output[4], ROBOT_WIRE_PROTOCOL_VERSION);
    write_u16_le(&output[6], message_type);
    write_u16_le(&output[8], (uint16_t)ROBOT_LIGHTS_PAYLOAD_SIZE);
    write_u16_le(&output[10], UINT16_C(0));
    write_u32_le(&output[12], sequence);
}

static robot_wire_result_t decode_header(const uint8_t* input, size_t input_size,
                                         uint16_t expected_message_type, uint32_t* sequence)
{
    if (input == NULL || sequence == NULL) return ROBOT_WIRE_NULL_ARGUMENT;
    if (input_size != ROBOT_LIGHTS_MESSAGE_SIZE) return ROBOT_WIRE_INVALID_LENGTH;
    if (read_u32_le(input) != ROBOT_WIRE_MAGIC) return ROBOT_WIRE_INVALID_MAGIC;
    if (read_u16_le(&input[4]) != ROBOT_WIRE_PROTOCOL_VERSION) return ROBOT_WIRE_INVALID_VERSION;
    if (read_u16_le(&input[6]) != expected_message_type) return ROBOT_WIRE_INVALID_MESSAGE_TYPE;
    if (read_u16_le(&input[8]) != (uint16_t)ROBOT_LIGHTS_PAYLOAD_SIZE) return ROBOT_WIRE_INVALID_LENGTH;
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
    encode_header(output, ROBOT_WIRE_MESSAGE_LIGHTS_COMMAND, command->sequence);
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
    result = decode_header(input, input_size, ROBOT_WIRE_MESSAGE_LIGHTS_COMMAND, &sequence);
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
    encode_header(output, ROBOT_WIRE_MESSAGE_LIGHTS_STATE, state->sequence);
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
    result = decode_header(input, input_size, ROBOT_WIRE_MESSAGE_LIGHTS_STATE, &sequence);
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
