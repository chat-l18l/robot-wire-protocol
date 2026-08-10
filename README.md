# Robot Wire Protocol

Versioned C11 wire codecs shared by the ROS 2-Zenoh gateway and the ESP32-P4
motion core. This repository is deliberately independent of ROS 2, Zenoh,
ESP-IDF and hardware drivers.

## Contents

- `include/`: public C11 protocol API;
- `src/`: allocation-free codec implementation;
- `docs/`: protocol and Zenoh route contracts;
- `tests/`: host golden-vector and decoder tests;
- `espidf/`: ESP-IDF component entry point; copy this directory layout into an
  ESP-IDF component or include this repository as a component dependency.

## Build and test

```bash
cmake -S . -B build -DBUILD_TESTING=ON
cmake --build build
ctest --test-dir build --output-on-failure
```

## Versioning

Protocol changes require a release tag and compatible updates in both consumer
repositories. Do not duplicate codecs or hand-copy definitions into the gateway
or firmware repositories.

License: BSD-3-Clause. See `LICENSE`.
