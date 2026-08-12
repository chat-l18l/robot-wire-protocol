# Robot Wire Protocol

Versioned C11 wire codecs shared by the ROS 2-Zenoh gateway and the ESP32-P4
motion core. This repository is deliberately independent of ROS 2, Zenoh,
ESP-IDF and hardware drivers.

## Contents

- `include/`: public C11 protocol API;
- `src/`: allocation-free codec implementation;
- `docs/`: architecture rationale, plus protocol and Zenoh route contracts;
- `tests/`: host golden-vector and decoder tests;
- `espidf/`: ESP-IDF component entry point; copy this directory layout into an
  ESP-IDF component or include this repository as a component dependency;
- `python/`: pure Python mirror of the same codecs, for tooling and test
  scripts that would otherwise hand-pack a struct.

## Why this is a separate repository

The short version: a contract that lives inside one participant is not a
contract, it is that participant's preference. The gateway and the firmware
never include each other's headers; this repository is their only common
ground, which is what lets either evolve without silently breaking the other.

[`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md) explains the separation of
concerns in full: the three repositories and what each owns, why encoding is
kept apart from transport and application, where the boundary is subtler than
it looks, and how two implementations of one spec are kept honest rather than
being duplication under a nicer name.

## Build and test

```bash
cmake -S . -B build -DBUILD_TESTING=ON
cmake --build build
ctest --test-dir build --output-on-failure
```

## Python

`python/robot_wire_protocol.py` encodes and decodes the same messages as the C
code, with no third-party dependencies. It exists so tooling and bench scripts
do not reimplement the protocol in a consumer repository, which is the same
rule the versioning section states for the C side.

`tests/test_python_matches_golden.py` asserts the Python encoder against the
byte-for-byte golden frame the C test uses. The point is not that Python works;
it is that the two implementations agree, so a drift between them fails a test
rather than surfacing as a frame only one end understands.

```bash
python3 tests/test_python_matches_golden.py
```

Like the C code, this module is transport-agnostic: it turns messages into
bytes and back, and the caller supplies Zenoh, a socket, or a file.

## Versioning

Protocol changes require a release tag and compatible updates in both consumer
repositories. Do not duplicate codecs or hand-copy definitions into the gateway
or firmware repositories.

License: BSD-3-Clause. See `LICENSE`.
