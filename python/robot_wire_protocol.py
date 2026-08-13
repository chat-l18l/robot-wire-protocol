"""Versioned wire codecs, Python mirror of the C11 implementation in ``src/``.

This module exists so tooling and test scripts encode the same bytes as the
firmware and the gateway, without anyone hand-packing a struct in a consumer
repository. It is the same protocol expressed twice, in one repository, which
is the only arrangement where the two can be tested against each other.

Like the C code it is deliberately independent of ROS 2, Zenoh, ESP-IDF and
hardware drivers: it turns messages into bytes and back, nothing more. The
transport belongs to the caller.

``tests/test_python_matches_golden.py`` checks this module against the same
golden frame the C test asserts on, so a divergence between the two
implementations is a failing test rather than a mystery on the bench.
"""

from __future__ import annotations

import struct
from dataclasses import dataclass, field
from enum import IntEnum
from typing import List

__all__ = [
    "MAGIC",
    "PROTOCOL_VERSION",
    "HEADER_SIZE",
    "LIGHT_COUNT",
    "MessageType",
    "WireError",
    "LightsResult",
    "LightsSource",
    "EstopClearResetResult",
    "PinoutInput",
    "PinoutOutput",
    "Rgb",
    "LightsCommand",
    "LightsState",
    "EstopClearResetRequest",
    "EstopClearResetState",
    "PinoutEvent",
    "encode_pinout_event",
    "decode_pinout_event",
    "encode_lights_command",
    "decode_lights_command",
    "encode_lights_state",
    "decode_lights_state",
    "encode_estop_clear_reset_request",
    "decode_estop_clear_reset_request",
    "encode_estop_clear_reset_state",
    "decode_estop_clear_reset_state",
]

MAGIC = 0x31434252  # "RBC1" little-endian
PROTOCOL_VERSION = 1
HEADER_SIZE = 16

LIGHT_COUNT = 4
LIGHTS_PAYLOAD_SIZE = 16
LIGHTS_MESSAGE_SIZE = HEADER_SIZE + LIGHTS_PAYLOAD_SIZE

ESTOP_CLEAR_RESET_PAYLOAD_SIZE = 4
ESTOP_CLEAR_RESET_MESSAGE_SIZE = HEADER_SIZE + ESTOP_CLEAR_RESET_PAYLOAD_SIZE

EVENT_PREFIX_SIZE = 16
PINOUT_EVENT_PAYLOAD_SIZE = 64
PINOUT_EVENT_MESSAGE_SIZE = HEADER_SIZE + PINOUT_EVENT_PAYLOAD_SIZE

_HEADER = struct.Struct("<IHHHHI")
_PINOUT_BODY = struct.Struct("<QIIQQQQQQ")

_UINT64_MAX = 0xFFFFFFFFFFFFFFFF


class MessageType(IntEnum):
    LIGHTS_COMMAND = 3
    LIGHTS_STATE = 4
    ESTOP_CLEAR_RESET_REQUEST = 5
    ESTOP_CLEAR_RESET_RESULT = 6
    PINOUT_EVENT = 7


class LightsResult(IntEnum):
    OK = 0
    INVALID_COMMAND = 1
    APPLY_FAILED = 2
    OVERRIDDEN = 3
    BUSY_SOURCE = 4


class LightsSource(IntEnum):
    NONE = 0
    SAFETY = 1
    ROS_BINARY = 2
    EDUCATION_JSON = 3


class EstopClearResetResult(IntEnum):
    OK = 0
    ESTOP_ACTIVE = 1
    NO_RESET_PENDING = 2
    SAFETY_SEQUENCE_INCOMPLETE = 3
    FAULT_ACTIVE = 4
    INTERNAL_ERROR = 5


class PinoutInput(IntEnum):
    """Input bit numbers. See ``robot_pinout_input_bit_t`` and PINOUT_PROTOCOL.md.

    A bit names a signal rather than an expander pin, and is 1 when the named
    condition holds regardless of how the line is wired electrically.
    """

    # 0..15 operator panel
    BUTTON_UP = 0
    BUTTON_DOWN = 1
    BUTTON_ESC = 2
    BUTTON_ENTER = 3
    BUTTON_TEST = 4
    TOGGLE_1 = 5
    TOGGLE_2 = 6
    TOGGLE_3 = 7
    # 16..31 safety chain
    ESTOP = 16


class PinoutOutput(IntEnum):
    """Output bit numbers. See ``robot_pinout_output_bit_t``."""

    # 0..15 signalling and indication
    HORN = 0
    HEARTBEAT = 1
    FULLSTOP_LED = 2
    ZENOH_LED = 3
    INDICATOR_LEFT = 4
    INDICATOR_RIGHT = 5
    AUTONOMOUS_LED = 6
    # 16..31 drivetrain control
    THROTTLE_DIR_REVERSE = 16
    EBRAKE = 17
    SKID_DIR_FL_REVERSE = 18
    SKID_DIR_FR_REVERSE = 19
    SKID_DIR_RL_REVERSE = 20
    SKID_DIR_RR_REVERSE = 21


class WireError(ValueError):
    """Raised when a frame cannot be decoded.

    The C API returns a result enum; Python raises, because a caller that
    ignores a return value here would act on an uninitialised message. The
    ``reason`` matches the C enumerator name so both sides report a rejection
    with the same word.
    """

    def __init__(self, reason: str) -> None:
        super().__init__(reason)
        self.reason = reason


@dataclass
class Rgb:
    red: int = 0
    green: int = 0
    blue: int = 0

    def as_tuple(self) -> tuple:
        return (self.red, self.green, self.blue)


def _default_lights() -> List[Rgb]:
    return [Rgb() for _ in range(LIGHT_COUNT)]


@dataclass
class LightsCommand:
    sequence: int = 0
    lights: List[Rgb] = field(default_factory=_default_lights)


@dataclass
class LightsState:
    sequence: int = 0
    result: int = LightsResult.OK
    active_source: int = LightsSource.NONE
    lights: List[Rgb] = field(default_factory=_default_lights)


@dataclass
class EstopClearResetRequest:
    sequence: int = 0


@dataclass
class EstopClearResetState:
    sequence: int = 0
    result: int = EstopClearResetResult.OK
    needs_reset: int = 0
    estop_active: int = 0


@dataclass
class PinoutEvent:
    """A vehicle-originated pinout snapshot.

    Not a reply, so there is no header sequence to correlate with; the counter
    that matters is ``event_sequence``. ``uptime_ms`` is milliseconds since the
    vehicle booted and is not comparable to ROS time, and ``boot_id`` is opaque:
    compare it for equality to notice a restart, never order it.

    The ``_changed`` masks are sticky since the previous event, so a pulse that
    came and went between two publications is still visible. A bit outside its
    ``_valid`` mask is rejected, which is what keeps "not wired" distinct from
    "not asserted".
    """

    uptime_ms: int = 0
    event_sequence: int = 0
    boot_id: int = 0
    inputs_level: int = 0
    inputs_changed: int = 0
    inputs_valid: int = 0
    outputs_level: int = 0
    outputs_changed: int = 0
    outputs_valid: int = 0


def _check_pinout_masks(level: int, changed: int, valid: int) -> None:
    for value, what in ((level, "level"), (changed, "changed"), (valid, "valid")):
        if not isinstance(value, int) or isinstance(value, bool):
            raise WireError(f"pinout {what} mask must be an integer")
        if not 0 <= value <= _UINT64_MAX:
            raise WireError(f"pinout {what} mask out of range")
    if (level & ~valid) or (changed & ~valid):
        raise WireError("INVALID_PINOUT_MASK")


def _check_channel(value: int, what: str) -> int:
    if not isinstance(value, int) or isinstance(value, bool):
        raise WireError(f"{what} must be an integer")
    if not 0 <= value <= 255:
        raise WireError(f"{what} out of range 0..255")
    return value


def _encode_header(message_type: int, payload_size: int, sequence: int) -> bytes:
    if not 0 <= sequence <= 0xFFFFFFFF:
        raise WireError("sequence out of range")
    return _HEADER.pack(MAGIC, PROTOCOL_VERSION, int(message_type), payload_size, 0, sequence)


def _decode_header(data: bytes, expected_type: int, expected_payload: int) -> int:
    """Validate a header and return its sequence.

    Checks run in the same order as the C decoder, so an identical frame is
    rejected for an identical reason on either side.
    """
    if len(data) != HEADER_SIZE + expected_payload:
        raise WireError("INVALID_LENGTH")
    magic, version, message_type, payload_size, _reserved, sequence = _HEADER.unpack(
        data[:HEADER_SIZE]
    )
    if magic != MAGIC:
        raise WireError("INVALID_MAGIC")
    if version != PROTOCOL_VERSION:
        raise WireError("INVALID_VERSION")
    if message_type != expected_type:
        raise WireError("INVALID_MESSAGE_TYPE")
    if payload_size != expected_payload:
        raise WireError("INVALID_LENGTH")
    return sequence


def _encode_rgb(lights: List[Rgb]) -> bytes:
    if len(lights) != LIGHT_COUNT:
        raise WireError("INVALID_LIGHT_COUNT")
    out = bytearray()
    for index, colour in enumerate(lights):
        out.append(_check_channel(colour.red, f"lights[{index}].red"))
        out.append(_check_channel(colour.green, f"lights[{index}].green"))
        out.append(_check_channel(colour.blue, f"lights[{index}].blue"))
    return bytes(out)


def _decode_rgb(payload: bytes) -> List[Rgb]:
    return [
        Rgb(payload[i * 3], payload[i * 3 + 1], payload[i * 3 + 2])
        for i in range(LIGHT_COUNT)
    ]


def encode_lights_command(command: LightsCommand) -> bytes:
    body = bytes([LIGHT_COUNT, 0, 0, 0]) + _encode_rgb(command.lights)
    return _encode_header(MessageType.LIGHTS_COMMAND, LIGHTS_PAYLOAD_SIZE, command.sequence) + body


def decode_lights_command(data: bytes) -> LightsCommand:
    sequence = _decode_header(data, MessageType.LIGHTS_COMMAND, LIGHTS_PAYLOAD_SIZE)
    if data[HEADER_SIZE] != LIGHT_COUNT:
        raise WireError("INVALID_LIGHT_COUNT")
    return LightsCommand(sequence=sequence, lights=_decode_rgb(data[HEADER_SIZE + 4 :]))


def encode_lights_state(state: LightsState) -> bytes:
    if not 0 <= int(state.result) <= int(LightsResult.BUSY_SOURCE):
        raise WireError("INVALID_RESULT")
    if not 0 <= int(state.active_source) <= int(LightsSource.EDUCATION_JSON):
        raise WireError("INVALID_SOURCE")
    body = bytes([LIGHT_COUNT, int(state.result), int(state.active_source), 0])
    body += _encode_rgb(state.lights)
    return _encode_header(MessageType.LIGHTS_STATE, LIGHTS_PAYLOAD_SIZE, state.sequence) + body


def decode_lights_state(data: bytes) -> LightsState:
    sequence = _decode_header(data, MessageType.LIGHTS_STATE, LIGHTS_PAYLOAD_SIZE)
    if data[HEADER_SIZE] != LIGHT_COUNT:
        raise WireError("INVALID_LIGHT_COUNT")
    result = data[HEADER_SIZE + 1]
    source = data[HEADER_SIZE + 2]
    if result > int(LightsResult.BUSY_SOURCE):
        raise WireError("INVALID_RESULT")
    if source > int(LightsSource.EDUCATION_JSON):
        raise WireError("INVALID_SOURCE")
    return LightsState(
        sequence=sequence,
        result=result,
        active_source=source,
        lights=_decode_rgb(data[HEADER_SIZE + 4 :]),
    )


def encode_pinout_event(event: PinoutEvent) -> bytes:
    _check_pinout_masks(event.inputs_level, event.inputs_changed, event.inputs_valid)
    _check_pinout_masks(event.outputs_level, event.outputs_changed, event.outputs_valid)
    if not 0 <= event.uptime_ms <= _UINT64_MAX:
        raise WireError("uptime_ms out of range")
    if not 0 <= event.event_sequence <= 0xFFFFFFFF:
        raise WireError("event_sequence out of range")
    if not 0 <= event.boot_id <= 0xFFFFFFFF:
        raise WireError("boot_id out of range")
    # Header sequence is zero: an event answers no request.
    header = _encode_header(MessageType.PINOUT_EVENT, PINOUT_EVENT_PAYLOAD_SIZE, 0)
    return header + _PINOUT_BODY.pack(
        event.uptime_ms,
        event.event_sequence,
        event.boot_id,
        event.inputs_level,
        event.inputs_changed,
        event.inputs_valid,
        event.outputs_level,
        event.outputs_changed,
        event.outputs_valid,
    )


def decode_pinout_event(data: bytes) -> PinoutEvent:
    _decode_header(data, MessageType.PINOUT_EVENT, PINOUT_EVENT_PAYLOAD_SIZE)
    fields = _PINOUT_BODY.unpack(data[HEADER_SIZE:])
    event = PinoutEvent(*fields)
    _check_pinout_masks(event.inputs_level, event.inputs_changed, event.inputs_valid)
    _check_pinout_masks(event.outputs_level, event.outputs_changed, event.outputs_valid)
    return event


def encode_estop_clear_reset_request(request: EstopClearResetRequest) -> bytes:
    header = _encode_header(
        MessageType.ESTOP_CLEAR_RESET_REQUEST, ESTOP_CLEAR_RESET_PAYLOAD_SIZE, request.sequence
    )
    return header + struct.pack("<I", 0)


def decode_estop_clear_reset_request(data: bytes) -> EstopClearResetRequest:
    sequence = _decode_header(
        data, MessageType.ESTOP_CLEAR_RESET_REQUEST, ESTOP_CLEAR_RESET_PAYLOAD_SIZE
    )
    return EstopClearResetRequest(sequence=sequence)


def encode_estop_clear_reset_state(state: EstopClearResetState) -> bytes:
    if not 0 <= int(state.result) <= int(EstopClearResetResult.INTERNAL_ERROR):
        raise WireError("INVALID_ESTOP_CLEAR_RESET_RESULT")
    header = _encode_header(
        MessageType.ESTOP_CLEAR_RESET_RESULT, ESTOP_CLEAR_RESET_PAYLOAD_SIZE, state.sequence
    )
    return header + bytes(
        [int(state.result), int(state.needs_reset), int(state.estop_active), 0]
    )


def decode_estop_clear_reset_state(data: bytes) -> EstopClearResetState:
    sequence = _decode_header(
        data, MessageType.ESTOP_CLEAR_RESET_RESULT, ESTOP_CLEAR_RESET_PAYLOAD_SIZE
    )
    result = data[HEADER_SIZE]
    if result > int(EstopClearResetResult.INTERNAL_ERROR):
        raise WireError("INVALID_ESTOP_CLEAR_RESET_RESULT")
    return EstopClearResetState(
        sequence=sequence,
        result=result,
        needs_reset=data[HEADER_SIZE + 1],
        estop_active=data[HEADER_SIZE + 2],
    )
