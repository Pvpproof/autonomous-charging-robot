# Command watchdog components

Project-authored C++14 watchdog state machine, Linux bounded serial writer and
standalone pseudoterminal tests. This directory contains no manufacturer driver,
ROS dependency source, firmware, device configuration, binary or field data.
The repository's project-wide license status also applies here; no additional
license or permission for third-party components is asserted.

The worker integration uses steady time, a 250 ms command/feedback expiry
threshold, latched zero output, explicit parked reset and fresh session/status
acknowledgements. See [integration contract](docs/INTEGRATION.md).

The test only allocates its own PTY. It never selects a hardware tty and requires
no ROS installation. On Linux with CMake, a C++14 compiler and pthread/libutil:

```sh
cmake -S . -B build
cmake --build build
ctest --test-dir build --output-on-failure
```

Run hardware-facing integration tests only in an isolated network/device
namespace with generated test devices. These tests do not prove MCU behavior,
physical braking, rear obstacle detection, process-crash stopping or deployment
readiness. This component provides no vehicle startup or deployment command.
