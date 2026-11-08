# 测试入口 / Test entry points

从仓库根目录运行。避免对整个目录做不加选择的 unittest discovery：这里同时保留了需要严格隔离的 ROS 集成用例和历史基线。

Run from the repository root. Select a runner rather than discovering every file: ROS integration cases require a separate isolated environment, and historical baselines are retained.

| 入口 / Entry | 内容 / Scope | 环境 / Environment |
| --- | --- | --- |
| `bash tests/run_local.sh` | 工作台、返程、控制、Qt、入口兼容 / Workstation, return, control, Qt, compatibility | 系统 Python + ROS 2 Humble Python 包 + GUI/数值依赖；假发布器与 offscreen Qt，无 ROS 节点 |
| `bash tests/run_arm.sh` | 归档来源、几何、合成时序、有限重建、模型包装 / Provenance, geometry, synthetic timing, reconstructions, model wrappers | 系统 Python + NumPy，无 ROS 节点/相机/设备；模型启动测试使用假的 `ros2` |
| `tests/run_isolated.sh` | ROS 桥接与驱动契约 / ROS bridge and driver contracts | 必须先隔离网络、PID、IPC 与设备；不是普通本地入口 |
| `native/driver-watchdog` CTest | 纯 C++ 命令看门狗与模拟串口 / C++ watchdog and synthetic PTY | CMake / C++14，模拟 PTY，无真实串口 |

`baseline/`、`legacy_ros_harness.py` 是回归参照；文件名中的旧参数不代表当前仍启用旧策略。当前 75 mm 策略有专门的无硬件测试。现有用例路径保留，避免破坏审查记录与脚本调用。

Baseline paths and historical filenames are preserved. They do not enable old policies. Passing these checks does not establish hardware, simulation, locking or electrical charging acceptance. [中文说明](../docs/TESTING.md) · [English guide](../docs/TESTING.en.md)
