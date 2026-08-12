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

    print("python codec matches the C golden vector")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
