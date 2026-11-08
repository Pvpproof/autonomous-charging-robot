# Deployment

charge_robot, ICP/V3 and the arm planning application are project-owned. Their latest files are pending import, not excluded as third-party implementations. Until dependency completeness is verified, configure them as a separate installation; this repository is not yet a complete arm deployment package.

[中文](DEPLOYMENT.md)

Use a ROS 2 Humble environment with PyQt5, NumPy, Pillow, PyYAML and gnome-terminal. Dependency versions in [DEPENDENCIES](../DEPENDENCIES.md) are historical observations, not a reproducible lockfile.

For a disconnected UI preview run `bash start.sh --offline`. Hardware use additionally requires externally supplied vendor drivers, reviewed navigation parameters, maps, calibration and the charge_robot toolchain. None are bundled.

| Variable | Meaning |
| --- | --- |
| CHARGING_ROBOT_SETUP | Reviewed robot workspace setup.bash |
| CHARGING_GATED_DRIVER_PREFIX | Installation prefix of the reviewed gated base driver |
| CHARGING_NAV2_PARAMS | Reviewed local Nav2 YAML |
| CHARGING_DOCKING_COMMAND | External executable path; test/home are passed as arguments; default ~/bin/charge_robot |
| CHARGING_NAV_RVIZ_CONFIGS | Optional colon-separated absolute navigation RViz config paths |

Build the raw relay following [raw odometry documentation](RAW_ODOM_RETURN.md). Integrate the separately supplied driver against the [watchdog contract](../native/driver-watchdog/README.md); the stock vendor driver is not a substitute. The placeholder Nav2 YAML is deliberately rejected for navigation.

Public `config/return_commissioning.json` stays disabled and uncommissioned. A single boolean does not establish hardware acceptance. Review exclusive command ownership, source/session identity, disconnect behavior and controlled short returns on the actual platform. The repository has no automatic hardware deployment/enabling script.

After external integration and acceptance, `bash start.sh` opens the live workstation: buttons can start services and motion. Never use that command as an offline preview. Keep field runtime data and credentials outside version control. Source reorganization is local-only until separately deployed and accepted.

Arm tool completion does not establish physical insertion, lock engagement or electrical charging. Recorded routes do not detect new obstacles. GUI stop is unavailable if the GUI stalls; independent emergency stop remains necessary.
