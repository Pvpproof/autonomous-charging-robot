# Agent onboarding

This repository is a ROS 2 Humble / PyQt5 automatic charging robot project: mobile navigation and recorded-route return, Nova2 arm tools, ICP/V3 vision, calibration, and MoveIt/RViz demonstrations. Begin with this file; no chat history is required.

## First pass

1. Run `git status --short --branch` and preserve unrelated changes.
2. Read [README.md](README.md), [How to use](docs/HOW_TO_USE.en.md), and [development guide](docs/DEVELOPMENT.md).
3. Find the module in the table below. Inspect its current code and relevant tests before proposing changes.
4. Check [original-source environment notes](robot_originals/ENVIRONMENT.md) for what is included versus what still needs installation or runtime validation. Historical documents are not proof of the current state.

## Source map and authority

| Area | Start here | Notes |
| --- | --- | --- |
| Application entry | `start.sh`, `app.py`, `src/charging_robot/paths.py` | Root Python files are compatibility shims. Implement changes under `src/charging_robot/`; preserve old import/CLI entries. |
| Panel/map/manual control | `src/charging_robot/ui/` | `app.py`, `canvas.py`, `teleop.py`, `arm_tools_ui.py`. |
| Nav2/localization | `src/charging_robot/navigation/` | Launch, map/TF helpers, display and optional navigation pause. |
| Route recording/return | `src/charging_robot/returning/` | Start with `raw_odom_mode.py`, `route_cycle.py`, `return_bridge.py`, `return_runtime.py`; see `docs/RAW_ODOM_RETURN.md`. |
| Command ownership/stop/feedback | `src/charging_robot/control/` | ROS bridge, arbiter, leases, stop causes and driver-watchdog client. |
| Independent arm terminals | `src/charging_robot/arm/`, `src/charging_robot/ui/arm_tools_ui.py` | Panel adapters, not the actual arm planner. |
| Exact field-original arm workflow | `robot_originals/nova2_tools/charge_robot.sh` | Original `test`, `home`, planning/execution helpers, readiness and feedback. |
| Exact vision/template/calibration | `robot_originals/charging_socket_vision/`, `robot_originals/charging_robot_tools/calibration/` | ICP, V3, PLY and robot-specific hand-eye calibration. |
| Vendor models/MoveIt and project simulation | `robot_originals/nova2_ros2_ws/src/` | `dobot_ros2_v4` retains upstream notices; `charging_dock_sim` retains Apache-2.0. |
| Adapted source reading view | `arm_stack/` | Derived copies with path/config adaptations, not the authority for restoring the robot. |
| Native helpers | `native/raw_odom_relay.cpp`, `native/driver-watchdog/` | Relay and project-authored watchdog components, not a complete base vendor driver. |
| Source evidence | `robot_originals/source-manifest.json`, `arm_stack/field-source-manifest.json`, `docs/source-layout.json` | Original hashes, adapted hashes, and compatibility path mapping. |

`robot_originals/` contains 599 byte-verified field originals captured on 2026-11-09. Earlier reconstructed workflow/runtime implementations have been replaced. Do not fabricate missing originals or present a newly written substitute as recovered source. When modifying a derived copy, identify which copy is affected; do not silently change the original snapshot or regenerate its provenance hashes to hide a change. New arm behavior should be described as a patch, not a field original.

## Existing behavior to preserve

- The three panel modules are independently operated. Arrival does not automatically start `test` or `home`.
- Outbound navigation starts recording automatically. Only a completed, valid outbound run becomes a return candidate; cancelling outbound is not equivalent to arrival.
- Return uses reverse-order measured poses in the same raw-odometry session with closed-loop linear/angular commands. It is not timed velocity playback. A restart/reset can invalidate a route.
- Return starts with one explicit click. Stop cancels the current intent; recovery must not restart an old task automatically.
- Arm `test`, `home`, and base return are separately initiated. Arm success/failure must not erase a completed route or become a workflow prerequisite for return. Existing active-motion conflicts and tool ownership remain separate from workflow success.
- The uncalibrated curvature eligibility model was deliberately removed. Do not reintroduce it or add unrelated interlocks while fixing another feature.
- `test` retains terminal confirmation for each motion; `home` is arm homing, not base return. Real insertion uses 75 mm; the project simulation YAML uses a separate 35 mm example.
- Normal GUI delay does not cancel a healthy autonomous backend. Keyboard control still depends on fresh input. Do not conflate GUI delay, control loss, operator takeover, and user stop.

## Commands and verification

Run from the repository root. Prefer focused existing tests for changed behavior; do not rerun every suite without a reason. Full requirements and examples are in [the development guide](docs/DEVELOPMENT.md).

```bash
# Disconnected UI; needs system Python, PyQt5, NumPy, Pillow and PyYAML.
bash start.sh --offline

# Source/manifest, feedback-readiness and synthetic V3 checks; no hardware.
bash tests/run_arm.sh

# Full workstation regression; needs ROS 2 Humble Python packages and Qt.
bash tests/run_local.sh
```

`tests/run_isolated.sh` is different: it requires an already-established network/device/PID isolation environment. Do not run it as an ordinary local check. A ROS domain ID alone is not device isolation.

## Scope, deployment and reporting

- Reading/editing/testing repository files does not authorize starting ROS hardware services, opening a camera, executing `test`/`home`, sending navigation goals or moving the robot. Use the task's actual scope; never start devices merely to validate documentation.
- Keep public `config/return_commissioning.json` disabled/uncommissioned unless the user explicitly requests a separately reviewed deployment. Do not confuse source recovery, software tests, simulation and physical acceptance.
- Do not stop/restart field services or deploy changes while doing a source-only task. Use `robot_originals/deploy_originals.py` only for a requested same-robot restore; it defaults to preview and does not build or start services.
- The owner explicitly authorized the included template, calibration, home pose, compensation and site configuration for publication. Preserve them and their provenance. Passwords, keys, tokens, unrelated logs/maps/routes and build caches stay out of Git.
- Keep Dobot and package-level license notices. Do not relabel vendor assets or the Apache-2.0 simulation package as project MIT.
- Keep Chinese/English README and usage instructions aligned when changing public behavior. Check links and shell syntax for documentation changes; do not invent new behavior just to make an example runnable.
- Report changed files/behavior, exact checks performed, missing dependencies, and whether any runtime/hardware validation occurred. Say when a check was not run; do not reuse an earlier test count as a new result.
