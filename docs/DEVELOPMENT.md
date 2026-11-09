# 开发接手 / Development handoff

[AGENTS.md](../AGENTS.md) 是编码 Agent 的首读入口；[How to use](HOW_TO_USE.md) 面向实际使用。本页帮助开发者定位代码并选择检查，不依赖历史聊天。

For coding agents, start with [AGENTS.md](../AGENTS.md). This page maps implementation tasks to source files and runnable checks. [English usage](HOW_TO_USE.en.md) covers operation.

## 先建立正确的项目视图 / Establish the source model

本仓库有三种源码角色，不要把它们当成相互独立的最新版：

| 目录 / Directory | 角色 / Role | 改动方式 / Editing approach |
| --- | --- | --- |
| `src/charging_robot/` | 面板、导航桥接、实际轨迹返程与控制的维护实现 / Maintained workstation implementation | 在职责模块中改动，保留根目录兼容入口 / Edit modules, preserve compatibility shims |
| `robot_originals/` | 从原车恢复的 599 个原文件、模型与配置 / Hash-verified field snapshot | 原件与证据保持可追溯；新实现明确标为后续补丁 / Keep provenance; identify new work as a patch |
| `arm_stack/` | 由机械臂/视觉原件整理的路径适配阅读视图 / Adapted reading view | 查看 `field-source-manifest.json` 的原件/派生哈希，不冒充原车安装 / Consult source/adapted hashes, do not call it a field install |

根目录 `app.py`、`navigation.launch.py` 等保留旧导入和命令；映射见 [source-layout.json](source-layout.json)。不要直接在兼容入口重复实现同一逻辑。`robot_originals/deploy_originals.py` 是复制工具，不是工作流或运动控制器。

## 从任务定位代码 / Find the owning module

| 任务 / Task | 入口 / Source | 相关测试 / Relevant tests |
| --- | --- | --- |
| 按钮、地图、键控 / Panel, map, manual control | `ui/app.py`、`ui/canvas.py`、`ui/teleop.py` | `test_panel.py`、`test_layout.py` |
| 定位导航启动与显示 / Navigation startup/display | `navigation/navigation_launch.py`、`navigation/navigation_display.py` | `test_launch.py`、`test_startup_lock.py` |
| 去程记录与每轮资格 / Recording and route eligibility | `returning/raw_odom_mode.py`、`returning/route_cycle.py` | `test_raw_mode.py`、`test_per_route.py`、`test_recording_freshness.py` |
| 返程执行与单击状态 / Return and one-click state | `returning/return_runtime.py`、`returning/return_bridge.py` | `test_return.py`、`test_oneclick.py` |
| 停止、心跳、控制权 / Stop, heartbeat, ownership | `control/control_arbiter.py`、`control/ros_bridge.py`、`control/run_lease.py` | `test_stop_behavior.py`、`test_control_liveness.py`、`test_heartbeat_clock.py` |
| 面板机械臂工具终端 / Independent tool terminals | `ui/arm_tools_ui.py`、`arm/arm_tool_terminal.py` | `test_arm_tools.py` |
| 机械臂就绪 / Arm readiness | `robot_originals/nova2_tools/charge_startup_checks.py`；阅读视图 `arm_stack/tools/` | `test_arm_source_alignment.py` |
| ICP / V3 / 模板 / Vision/template | `robot_originals/charging_socket_vision/`；阅读视图 `arm_stack/vision/` | `test_field_arm_sources.py`、`test_arm_v3_timing.py` |
| 原件、标定与复制 / Originals, calibration, copying | `robot_originals/source-manifest.json`、`deploy_originals.py` | `test_robot_original_bundle.py` |

前六行源码路径相对 `src/charging_robot/`，测试均在 `tests/`。这些测试覆盖的是软件与合成输入；是否实车有效需另行验证。

The first six source rows are relative to `src/charging_robot/`; all tests are in `tests/`. Software tests are not physical acceptance.

## 本地开发命令 / Local commands

先查看改动，不覆盖已有工作：

```bash
git status --short --branch
git diff --stat
```

无硬件机械臂源码检查：

```bash
bash tests/run_arm.sh
```

在已有 ROS 2 Humble、PyQt5 等依赖的终端，可只运行相关工作台测试。例如改返程单击逻辑时：

```bash
source /opt/ros/humble/setup.bash
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$PWD/src:$PWD:$PWD/tests:${PYTHONPATH:-}"
export QT_QPA_PLATFORM=offscreen
/usr/bin/python3 tests/test_oneclick.py
```

`bash tests/run_local.sh` 是完整工作台回归；按变更影响选择，不必每次运行全部。`tests/run_isolated.sh` 需要已经建立的设备/网络/PID 隔离，不能用普通本地终端或仅换 ROS 域来代替。

原生看门狗独立 PTY 检查（Linux、CMake、C++ 编译器，不连接真实串口）：

```bash
cmake -S native/driver-watchdog -B build/watchdog
cmake --build build/watchdog
ctest --test-dir build/watchdog --output-on-failure
```

文档改动检查相对链接、代码块与 `git diff --check`；不要为确认 README 的运行命令而直接启动相机、test/home 或实车服务。

## 提交前核对 / Before submitting

- 说明改动属于底盘工作台、机械臂派生补丁、原件恢复工具还是文档；不要隐式同步多个来源副本。
- 保持去返程与机械臂结果解耦，保留单击启动、显式取消、不自动续行，以及现有命令所有权语义；不顺手引入新的无关门槛。
- 新功能更新 How to use 与中英文 README。标明原始来源、软件检查、仿真和现场实测各自的证据，不沿用旧测试数量充当本次结果。
- 不提交密码/密钥、地图/轨迹/日志和构建缓存。现有本车标定/模板是已授权发布资产；保留来源和第三方许可。

Describe the edited source layer, run focused checks, update both README languages when behavior changes, and distinguish code review, synthetic tests, simulation and field evidence. Preserve user changes, source provenance and third-party notices.
