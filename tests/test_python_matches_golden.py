"""Check the Python codec against the same golden frame the C test asserts on.

The point of this file is not that the Python encoder works. It is that the two
implementations agree byte for byte. The vector below is copied from
``tests/test_robot_wire_protocol.c``; if either side drifts, this fails instead
of producing a frame that only one end understands.

Run:  python3 tests/test_python_matches_golden.py
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "python"))

import robot_wire_protocol as rwp  # noqa: E402

# Identical to `golden` in tests/test_robot_wire_protocol.c
GOLDEN = bytes(
    [
        0x52, 0x42, 0x43, 0x31, 0x01, 0x00, 0x03, 0x00, 0x10, 0x00, 0x00,
        0x00, 0x04, 0x03, 0x02, 0x01, 0x04, 0x00, 0x00, 0x00, 0x01, 0x02,
        0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C,
    ]
)

# Identical to `golden` in test_pinout_event() in the C test.
GOLDEN_PINOUT = bytes(
    [
        0x52, 0x42, 0x43, 0x31, 0x01, 0x00, 0x07, 0x00, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x08, 0x07, 0x06, 0x05, 0x04, 0x03, 0x02, 0x01, 0x44, 0x33, 0x22, 0x11,
        0x88, 0x77, 0x66, 0x55, 0x21, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x7F, 0x00, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x00,
    ]
)


def distinct_command() -> rwp.LightsCommand:
    """Mirrors distinct_command() in the C test."""
    return rwp.LightsCommand(
        sequence=0x01020304,
        lights=[
            rwp.Rgb(1, 2, 3),
            rwp.Rgb(4, 5, 6),
            rwp.Rgb(7, 8, 9),
            rwp.Rgb(10, 11, 12),
        ],
    )


def distinct_pinout_event() -> rwp.PinoutEvent:
    """Mirrors distinct_pinout_event() in the C test.

    Built from the bit enums rather than literals, so the two implementations
    agree on the assignment and not merely on the arithmetic.
    """
    return rwp.PinoutEvent(
        uptime_ms=0x0102030405060708,
        event_sequence=0x11223344,
        boot_id=0x55667788,
        inputs_valid=_bits(
            rwp.PinoutInput.BUTTON_UP,
            rwp.PinoutInput.BUTTON_DOWN,
            rwp.PinoutInput.BUTTON_ESC,
            rwp.PinoutInput.BUTTON_ENTER,
            rwp.PinoutInput.BUTTON_TEST,
            rwp.PinoutInput.TOGGLE_1,
            rwp.PinoutInput.TOGGLE_2,
            rwp.PinoutInput.TOGGLE_3,
            rwp.PinoutInput.ESTOP,
        ),
        inputs_level=_bits(rwp.PinoutInput.BUTTON_UP, rwp.PinoutInput.TOGGLE_1),
        inputs_changed=_bits(rwp.PinoutInput.TOGGLE_1),
        outputs_valid=_bits(
            rwp.PinoutOutput.HORN,
            rwp.PinoutOutput.HEARTBEAT,
            rwp.PinoutOutput.FULLSTOP_LED,
            rwp.PinoutOutput.ZENOH_LED,
            rwp.PinoutOutput.INDICATOR_LEFT,
            rwp.PinoutOutput.INDICATOR_RIGHT,
            rwp.PinoutOutput.AUTONOMOUS_LED,
            rwp.PinoutOutput.THROTTLE_DIR_REVERSE,
            rwp.PinoutOutput.EBRAKE,
            rwp.PinoutOutput.SKID_DIR_FL_REVERSE,
            rwp.PinoutOutput.SKID_DIR_FR_REVERSE,
            rwp.PinoutOutput.SKID_DIR_RL_REVERSE,
            rwp.PinoutOutput.SKID_DIR_RR_REVERSE,
        ),
        outputs_level=_bits(rwp.PinoutOutput.HEARTBEAT),
        outputs_changed=_bits(rwp.PinoutOutput.THROTTLE_DIR_REVERSE),
    )


def _bits(*indices: int) -> int:
    mask = 0
    for index in indices:
        mask |= 1 << int(index)
    return mask


def check_pinout_event() -> None:
    event = distinct_pinout_event()

    encoded = rwp.encode_pinout_event(event)
    assert encoded == GOLDEN_PINOUT, f"pinout encoder drifted:\n{encoded.hex()}"
    assert rwp.decode_pinout_event(GOLDEN_PINOUT) == event

    # An event answers no request, so the header sequence is reserved and zero.
    assert GOLDEN_PINOUT[12:16] == bytes(4)

    # A level bit outside the valid mask must be rejected on both sides, or
    # "not wired" would read as "not asserted".
    unimplemented = distinct_pinout_event()
    unimplemented.inputs_level |= 1 << 40
    for produce in (
        lambda: rwp.encode_pinout_event(unimplemented),
        lambda: rwp.decode_pinout_event(
            GOLDEN_PINOUT[:37] + bytes([0x01]) + GOLDEN_PINOUT[38:]
        ),
    ):
        try:
            produce()
        except rwp.WireError as err:
            assert err.reason == "INVALID_PINOUT_MASK", f"got {err.reason}"
        else:  # pragma: no cover
            raise AssertionError("a bit outside the valid mask was accepted")


def main() -> int:
    command = distinct_command()

    encoded = rwp.encode_lights_command(command)
    assert encoded == GOLDEN, f"encoder drifted from the C golden frame:\n{encoded.hex()}"

    decoded = rwp.decode_lights_command(GOLDEN)
    assert decoded.sequence == command.sequence
    assert [c.as_tuple() for c in decoded.lights] == [c.as_tuple() for c in command.lights]

    # Rejections must fail for the same reason the C decoder gives.
    for corrupt, reason in (
        (GOLDEN[:-1], "INVALID_LENGTH"),
        (bytes([0]) + GOLDEN[1:], "INVALID_MAGIC"),
        (GOLDEN[:4] + bytes([0x02, 0x00]) + GOLDEN[6:], "INVALID_VERSION"),
        (GOLDEN[:6] + bytes([0x09, 0x00]) + GOLDEN[8:], "INVALID_MESSAGE_TYPE"),
    ):
        try:
            rwp.decode_lights_command(corrupt)
        except rwp.WireError as err:
            assert err.reason == reason, f"expected {reason}, got {err.reason}"
        else:  # pragma: no cover
            raise AssertionError(f"expected {reason}, frame was accepted")

    # Round-trips for the remaining message types.
    state = rwp.LightsState(
        sequence=command.sequence,
        result=rwp.LightsResult.OK,
        active_source=rwp.LightsSource.ROS_BINARY,
        lights=command.lights,
    )
    assert rwp.decode_lights_state(rwp.encode_lights_state(state)) == state

    request = rwp.EstopClearResetRequest(sequence=0x11223344)
    assert rwp.decode_estop_clear_reset_request(
        rwp.encode_estop_clear_reset_request(request)
    ) == request

    reset_state = rwp.EstopClearResetState(sequence=0x11223344)
    assert rwp.decode_estop_clear_reset_state(
        rwp.encode_estop_clear_reset_state(reset_state)
    ) == reset_state

    check_pinout_event()

    print("python codec matches the C golden vectors")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
