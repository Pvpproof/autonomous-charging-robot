# 无硬件测试

[English](TESTING.en.md)

```bash
bash tests/run_local.sh
```

脚本使用系统 Python、ROS 2 Humble 的消息/launch Python 包，以及 PyQt5、NumPy、Pillow、PyYAML。Qt 使用 offscreen，ROS 日志写入临时目录。测试使用合成轨迹、假发布器、假进程和离线窗口，不启动 ROS 节点、不连接相机或运动设备。导入 launch 描述不等于执行 launch。

覆盖记录与返程、取消与旧回调、来源/会话、里程过期、控制租约、普通停止、真实故障、独立机械臂工具、导航 RViz 归属和公开配置。新增目录兼容测试验证旧入口、包路径和来源清单。历史测试文件名可能保留旧模型名称，不表示该模型仍被启用。

原生看门狗可独立测试（仅模拟 PTY，不打开真实串口）：

```bash
cmake -S native/driver-watchdog -B /tmp/charging-watchdog-build
cmake --build /tmp/charging-watchdog-build
ctest --test-dir /tmp/charging-watchdog-build --output-on-failure
```

`tests/run_isolated.sh` 是另行准备的 ROS 沙箱路径，不是常规本地测试；不要在连接机器人现场直接运行。历史结果见 [VALIDATION.md](VALIDATION.md)，不可当成本轮实车验收。

改动后优先运行相关测试；源码整体搬迁需完整跑本地套件。请报告环境、命令和结果，不将合成误差或软件通过率写成实车精度。

本次目录整理验证：原有 172 项加 4 项兼容测试，共 176 项通过。原生组件未改变，本轮未重跑原生或 ROS 集成测试。

机械臂归档与重建代码另用 `bash tests/run_arm.sh`（系统 Python 与 NumPy）检查；只运行静态/合成用例，包含来源哈希、V3 时序、75 mm 几何和假的模型启动命令。入口与测试分组见 [tests/README.md](../tests/README.md)。
