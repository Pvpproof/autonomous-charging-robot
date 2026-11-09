# Deployment

2026-11-09 更新 / Update: 原版源码已从现场恢复，昨日重建实现已替换；本页此前“缺失/重建”状态属于历史记录。当前范围与缺项以 [恢复说明](../arm_stack/README.md) 为准。 Field originals now supersede reconstructions; older missing-source statements are historical.

Use the 599 hash-verified field originals in `robot_originals/` for same-robot restoration. `arm_stack/` is a path-adapted reading view, and originals replace the earlier reconstructed implementations. The panel calls separately deployed arm tools. Complete source inclusion does not establish a configured or accepted runtime. Start with [How to use](HOW_TO_USE.en.md).

[中文](DEPLOYMENT.md)

Use a ROS 2 Humble environment with PyQt5, NumPy, Pillow, PyYAML and gnome-terminal. Dependency versions in [DEPENDENCIES](DEPENDENCIES.md) are historical observations, not a reproducible lockfile.

For a disconnected UI preview run `bash start.sh --offline`. `robot_originals/` includes the arm/vision toolchain, current DOBOT sources/models/configuration, project simulation, template and authorized robot-specific calibration. Hardware use still needs base/lidar drivers, navigation parameters, maps, ROS/camera/Python dependencies and workspace builds. See [environment notes](../robot_originals/ENVIRONMENT.md).

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
