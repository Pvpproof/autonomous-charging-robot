# Architecture and boundaries

Pvpproof-owned historical arm workflow, ICP/V3 and planning sources plus limited reconstructions are included in `arm_stack/`. They are not a complete deployable environment; current field identity is unverified. MoveIt, SDKs and vendor models remain external. [Source status](ARM_SOURCE_SCOPE.md).

[中文](ARCHITECTURE.md)

The README workflow represents user choices, not a mandatory sequence. Arm test/home outcomes do not erase a complete outbound record or authorize return. External tools keep their stage confirmations.

`ui` renders Qt views. `control/ros_bridge.py` handles ROS callbacks and backend control with arbitration and leases. `returning` manages source identity, sessions, complete records and reverse pose tracking. Raw odometry is driver-integrated feedback, not independent ground truth. Cancellation and stale callbacks must not resurrect motion.

`arm` wraps external tools. Independent test closes only an identifiable standalone navigation RViz, retaining arm RViz. The panel map is a Qt canvas. This path does not automatically stop Nav2; retained lifecycle helpers do not imply automatic pause.

Implementations live in `src/charging_robot`. Root app, arbiter, arm terminal, docking console and navigation launch files remain compatibility entry points. Internal imports use full package names; development scripts use `PYTHONPATH="$PWD/src"`. Arbitrary old short module imports are not a supported compatibility API.

`charging_robot.paths.PROJECT_ROOT` keeps configuration, logs, maps and native binaries at their original repository-relative locations. Environment variables are unchanged. The source manifest preserves field baseline hashes separately from reorganized public hashes.

Ordinary stop cancels intent without a new software lock; actual arbiter faults still require handling. Return targets 0.15 m/s with a 0.4 m/s feedback bound. Recorded-route mode has no legacy one-shot 4 m/90 s authorization, but retains route-dependent watchdogs and output limits. Removing uncalibrated curvature classification does not remove these controls.

GUI stalls do not necessarily stop healthy backend autonomous control. Hardware emergency stop is independent. Offline tests do not establish stopping distance, obstacle detection or electrical charging safety.
