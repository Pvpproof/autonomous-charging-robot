# Autonomous Charging Robot for GB/T Electric Vehicles

**Arm sources partially included:** charge_robot, ICP, V3, arm simulation and planning applications are authored by Pvpproof. [arm_stack](arm_stack/README.md) contains 40 available historical source/display files and two inactive algorithm examples. Runtime modules and complete simulation assets remain missing; current field identity and deployability are not claimed.

[中文](README.md) · [Architecture](docs/ARCHITECTURE.en.md) · [Deployment](docs/DEPLOYMENT.en.md) · [Testing](docs/TESTING.en.md)

A robotics R&D project integrating mobile navigation, recorded-odometry return and arm docking tools in a PyQt5 / ROS 2 workstation. It demonstrates asynchronous control, state management, modular integration and fault diagnosis, with reusable components for further development.

The title describes the application domain, not standards certification or a validated unattended electrical charging system. Thesis text, site maps, trajectories, calibration and vendor implementations are excluded.

Added [eight traceable historical source files](arm_stack/recovered/README.md), including planning and bridges; completeness and current field identity remain unverified.

## Workflow

The panel has three independent sections: maps/targets/manual control, localization/outbound/return, and arm tools. Arm success is not a prerequisite for return.

```mermaid
flowchart LR
  A[Map and charging target] --> B[Localize]
  B --> C[Navigate and record]
  C --> D[Arrive completely and stop]
  D --> E[User chooses]
  E --> F[test with stage confirmations]
  E --> G[Independent home]
  E --> H[Click to return]
  H --> I[Recorded origin]
```

Cancelled outbound runs are not eligible return records. Return tracks reversed poses within the same raw-odometry session using linear/angular commands; it is not timed command replay. Ordinary stop cancels intent without automatically resuming old tasks.

## Architecture

```mermaid
flowchart TB
  UI[PyQt5 panel] --> BR[ROS bridge and backend state]
  BR --> NAV[External Nav2]
  BR --> REC[Raw recording and return tracker]
  NAV --> ARB[Velocity arbiter]
  REC --> ARB
  ARB --> DRIVER[External gated driver and watchdog]
  DRIVER --> BASE[Mobile base]
  BASE --> RELAY[Raw odometry relay]
  RELAY --> REC
  UI --> TERM[Independent test / home terminals]
  TERM --> ARM[Owned charge_robot / ICP / V3 with external MoveIt]
```

## Arm and vision

[arm_stack](arm_stack/README.md) contains project-owned test/home workflow, ICP point-cloud matching, V3 RGB PnP, feedback, camera alignment, gun prepose, Cartesian insertion planning and execution wrappers. Panel buttons still use a separately installed tool environment; this archive does not replace field software.

There are 40 historical source/display files with differentiated provenance, plus five explicitly reconstructed files for file status/writing, model display and plan-only startup. This is not an accepted current hardware release. The owner selected 75 mm; public planning and execution policy now agree. Field files were not changed and the update is not field-tested.

## Simulation and planning

```mermaid
flowchart LR
  U[Explicit local URDF] --> M[Reconstructed model display]
  J[Existing real or isolated test joint feedback] --> M
  M --> V[RViz model and TF]
  D[External DOBOT Nova2 MoveIt configuration] --> P[Plan-only entry: execution disabled]
  P --> R[Plan for inspection]
  G[Official Gazebo and virtual controllers] --> S[Isolated simulation pending setup and validation]
```

Model-display and plan-only wrappers are included; fake joint publishers, site calibration and a complete Gazebo project are not. Six official model/MoveIt files match historical field hashes, but the complete field revision, meshes and execution adapter still need verification. Pin external dependencies and preserve Dobot notices; see [upstream audit](arm_stack/DOBOT_DEPENDENCY.md).

Source inspection and synthetic tests work without hardware when test dependencies are installed. Actual model/planning visualization requires the reviewed upstream stack. No RViz/MoveIt/Gazebo or hardware run was performed in this update. There are 61 passing targeted tests; these do not establish end-to-end simulation or hardware acceptance.

## Evidence and limits

| Capability | Evidence | Remaining limits |
| --- | --- | --- |
| Mapping, manual control, outbound navigation | Historical field use | No general accuracy claim |
| Click-to-return | User reported complete return on earlier versions; offline regressions | Latest changes and source reorganization need new field acceptance |
| ICP alignment and V3 pre-insertion pose | User-provided successful stage logs | Later insertion timed out; full electrical charging is unverified |
| Stop, disconnect, cancellation | Synthetic input, fake publishers, offline Qt tests | Not hardware acceptance |
| Hardware emergency stop and controller disconnect stop | User confirmation | Not an automated test result |

## Start without hardware

With system Python, PyQt5, NumPy, Pillow and PyYAML installed:

```bash
bash start.sh --offline
```

This does not connect to devices. `bash tests/run_local.sh` runs hardware-free regression tests in a ROS 2 Humble development environment. Public configuration disables hardware operation. See the deployment and testing guides before integration.

## Source layout

`src/charging_robot/` contains `ui`, `navigation`, `returning`, `arm`, `control` and `shared`. `native/` contains project-owned C++ helpers; `tests/` contains regression fixtures. Original `start.sh`, `app.py` and launch entry points remain available. See [path mapping](docs/source-layout.json) and [provenance](docs/source-manifest.json).

Recorded routes cannot detect new obstacles or eliminate wheel slip. Uncalibrated curvature classification has been removed, while controller output limits and communication/state checks remain. Healthy autonomous control may continue while the GUI is stalled; its stop button may be unavailable, requiring an independent hardware emergency stop.

Pvpproof-owned code is [MIT licensed](LICENSE). Third-party dependencies retain their terms; see [notices](THIRD_PARTY_NOTICES.md) and [dependency audit](DEPENDENCIES.md), including PyQt GPL/commercial options and unresolved vendor licensing.

[Contributing](CONTRIBUTING.en.md) · [Changelog](CHANGELOG.md)
