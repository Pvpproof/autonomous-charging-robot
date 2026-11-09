# 本车原版源码与标定 / Originals and calibration for this robot

这是 2026-11-09 从机器人只读获取的 599 个原始文件，按 /home/nvidia 的目录结构组织。源码、模型、配置与标定均逐文件核对 SHA-256，未把昨日重建版放入此目录。完整来源见 [source-manifest.json](source-manifest.json)。

This bundle contains 599 byte-verified original files arranged relative to the robot's /home/nvidia. It includes original workflow/runtime, camera applications, templates, calibration and the vendor source dependencies. It contains no reconstructed workflow implementation.

## 内容

- nova2_tools：原版 charge_robot test/home、workflow/runtime、执行、规划、反馈、启动与显示文件。
- charging_socket_vision：ICP、V3、桥接、预览、模板与实际算法配置。V3 相机参数由实际设备 SDK 获取；test_data/camera.json 是现场保留的测试参数，不强行覆盖实时内参。
- charging_robot_tools/calibration：实际代码引用的手眼标定 NPZ；工具目录包含初始关节姿态和枪头补偿。
- nova2_ros2_ws/src/charging_dock_sim：原版自有仿真包，保留原 Apache-2.0 声明。
- nova2_ros2_ws/src/dobot_ros2_v4：509 个现用 DOBOT 源码、模型、网格和配置，保留 Dobot LICENSE 与包级许可声明。现场源目录没有 Git 版本元数据，本地修改没有被冒充上游某个提交。

## 在同一台车恢复

原文件写死的 /home/nvidia 布局和原环境入口被保留。这里面存在控制器地址和**仅适用于此车**的标定、回位姿态及补偿，用户已明确要求包含；不要直接套用到别的机械臂。

先保持设备停稳并关闭相关服务，预览将复制的文件：

```bash
/usr/bin/python3 robot_originals/deploy_originals.py
```

明确需要恢复时执行：

```bash
/usr/bin/python3 robot_originals/deploy_originals.py --apply
```

工具只校验和复制；覆盖的旧文件存入 /home/nvidia/.charging-source-backups。它不启动服务、不构建工作空间、不发送运动指令。不能把复制完成当作运行验收。

现有同车环境还需要保留 /opt/ros/humble、已构建 ROS 工作空间的 install、charging_robot_env 和系统相机驱动。源目录变化后须按现场依赖重新构建；恢复包不包含 OS、ROS 安装包、虚拟环境二进制或缓存。Python 版本记录见 [python-environment.json](python-environment.json)。此记录不是跨平台 pip 一键安装清单。

On the same robot, preview first; --apply copies verified sources/configuration and backs up replaced files. It does not start hardware, build ROS or install dependencies. Existing ROS Humble, workspace builds, camera drivers and virtual environment remain required. For a fresh OS these must be recreated separately. Calibration is specific to this robot.

## 当前验证边界

原件传输前后 470 个文件哈希无变化；另 4 个配置/标定原件逐文件核对（排除开发约定文件 AGENTS.md 后，总公开原件为 599）。恢复包的 8 项检查通过，包含原件哈希、标定矩阵形式、原许可、无写入预览以及临时目录备份复制。另有 33 项应用源码/语法/时序检查通过。没有在车上运行相机、MoveIt、仿真或执行 test/home；当前尚不能宣称 GitHub 下载后整机闭环已经验收。

原车命令入口已只读核实：/home/nvidia/bin/charge_robot 是指向 /home/nvidia/nova2_tools/charge_robot.sh 的符号链接。恢复工具不重建该链接；同车保留即可，也可直接用 bash /home/nvidia/nova2_tools/charge_robot.sh 调用原入口（实际命令可能启动设备，未在本轮执行）。

环境安装、已确认版本、静态缺件核查与待验证项见 [ENVIRONMENT.md](ENVIRONMENT.md)。126 个 ROS 接口定义已补齐，当前原文件总数为 599。

本地压缩包的中文逐步恢复说明见 [本车恢复说明](../docs/本车恢复说明.md)，包含解压、校验、恢复、构建、test/home 与未验收事项。
