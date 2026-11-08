# Hardware-free testing

[中文](TESTING.md)

Run `bash tests/run_local.sh` with system Python, ROS 2 Humble Python message/launch packages, PyQt5, NumPy, Pillow and PyYAML. Qt runs offscreen; ROS logs use a temporary directory. Tests use synthetic traces, fake publishers/processes and offline windows. No ROS nodes or physical devices are started. Generating a launch description does not execute it.

Coverage includes return, cancellation/stale callbacks, source/session identity, stale feedback, leases, ordinary stop, actual faults, independent arm tools, RViz ownership and disabled public configuration. Layout compatibility checks cover original entry points, package paths and provenance. Historical test filenames do not imply removed classifiers remain active.

The native watchdog has a synthetic PTY test:

```bash
cmake -S native/driver-watchdog -B /tmp/charging-watchdog-build
cmake --build /tmp/charging-watchdog-build
ctest --test-dir /tmp/charging-watchdog-build --output-on-failure
```

`run_isolated.sh` requires a separately prepared ROS sandbox and is not a normal local test. Do not run it on a connected field system. Historical [validation](VALIDATION.md) is not current hardware acceptance. Run targeted regressions for ordinary changes and the full local suite for cross-module relocation. Report measured software results without inventing physical performance.

This reorganization passed 172 existing cases plus 4 compatibility cases (176 total). Native components were unchanged; native and ROS integration tests were not rerun.
