# 环境准备与文件完整性 / Environment and completeness

## 已确认的文件

当前有 **599 个已核对哈希的原文件**，包括原版 charge_robot、ICP/V3、模板、实际策略配置、手眼/相机标定、初始姿态/枪头补偿、自有仿真包，以及 DOBOT 驱动、消息接口、MoveIt、Nova2 URDF/SRDF/网格与执行适配器。补齐了 126 个 ROS .srv/.msg/.action 定义；CMake 中明确列出的接口文件无缺失。工具目录中写死的本地 .py/.sh/.npz/.yaml/.rviz 引用也未发现缺件。

这是静态引用核查，不是动态全链测试。日志、待执行轨迹、测量结果等由运行时生成，不应伪造来填满目录。ICP 初始化报告可选，源码在不存在时继续搜索。

## 环境：同车已有，新系统需安装

原启动脚本明确使用 **ROS 2 Humble**、/home/nvidia/nova2_ros2_ws 和 /home/nvidia/charging_robot_env。读取到的现用 Python 是 **3.10.12 / aarch64 / Linux 5.15.148-tegra**。Ubuntu 22.04 是 Humble 的常规部署目标；本轮没有重新刷机或验证系统安装方案。

需要 ROS Humble desktop/RViz、MoveIt、ros2_control/controller_manager、robot_state_publisher、xacro、colcon、rosdep、C++/CMake/Eigen、gnome-terminal。先按 [ROS 官方 Humble 安装说明](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)配置软件源，再按系统架构安装；不能把 x86 安装包套到 Jetson。

RealSense 相机通过 **librealsense + pyrealsense2** 使用；原视觉程序直接读取 SDK，不要求 realsense2_camera ROS 节点来代替。现用 Python 包元数据为 pyrealsense2 **2.56.5.9235**。本轮未查询相机固件、JetPack 具体版本或 SDK 的 native/RSUSB 构建选择，不假定 pip 在每种 Jetson 上都有合适 wheel。按 [RealSense 官方 Jetson 指南](https://github.com/realsenseai/librealsense/blob/master/doc/installation_jetson.md)与[Python bindings](https://github.com/realsenseai/librealsense/blob/master/wrappers/python/readme.md)选择匹配内核和 Python 的安装方式。

读取到的其他依赖版本：NumPy 1.21.5、SciPy 1.8.0、Open3D 0.20.0、PyYAML 5.4.1、PyQt5 5.15.6。OpenCV 的 cv2 模块需要存在，但本轮 distribution metadata 未给出可靠版本。Pillow 元数据同时出现不同大小写/版本记录，因此 [python-environment.json](python-environment.json) 是环境观察，不是可直接 pip install 的锁文件。不要不加选择地升级现用数值库或用这份元数据覆盖能工作的环境。

## 新系统恢复步骤（准备说明，未实际安装验证）

1. 安装与设备相符的 Jetson/Ubuntu、ROS Humble、系统构建工具与相机 SDK；现有同车环境可复用，不必重装。
2. 查看 `deploy_originals.py` 的默认预览，确认目标为 /home/nvidia；确需复制时再 --apply。被覆盖的文件有备份，不启动任何服务。
3. 在已配置 ROS 的 shell 中解析依赖并构建源码工作空间：

```bash
source /opt/ros/humble/setup.bash
cd /home/nvidia/nova2_ros2_ws
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble
colcon build --symlink-install
source install/local_setup.bash
```

以上命令是后续部署步骤，本轮没有执行。原仿真 package.xml 有 moveit_commander 依赖声明，但现有仿真 Python 使用 ROS 2 Action/Service；该声明能否由当前 rosdep 解析需在新系统核对，不擅自改原文件或声称构建已经通过。

4. 若原 Python 环境不存在，可建立同路径环境，例如 `python3 -m venv --system-site-packages /home/nvidia/charging_robot_env`，再按 aarch64 可用渠道配置 NumPy、SciPy、OpenCV、Open3D、PyYAML、Pillow、PyQt5 和 pyrealsense2。这不是已验证的一键 pip 安装路线，尤其 Open3D/RealSense 可能依赖本地构建或对应 wheel。
5. 在不启动设备的前提下检查 Python 模块可导入、ROS 包可发现和配置文件可读取。之后另做现场原地显示、反馈和运动验收；不得把编译完成当作实际运行通过。

## 还有什么未包含或未确认

- 主 test/home 调用链没有发现新的必需源码、模型或标定缺件；仍需真实安装/构建及全流程验收。
- 独立 Gazebo 物理仿真的 dobot_gazebo 未收入本恢复范围；dobot_demo、cr6al_moveit 也未收入，少数旧显示/仿真脚本的 AMENT_PREFIX_PATH 仍列出它们。这不等同于 Nova2 test/home 主流程缺文件。
- 自有 charging_dock_sim 与 Nova2 MoveIt demo 已有，Gazebo 物理仿真和 MoveIt/RViz 规划演示须区分。
- 原现场 DOBOT 源目录无可用 Git HEAD/remote，文件哈希已保存，但整套对应哪一个上游提交、有哪些本地修改仍待进一步比对。
- 此恢复包聚焦机械臂/视觉。新系统重装底盘、雷达、Nav2 时，还需对应硬件驱动及现场导航地图、参数；不能将机械臂资料齐全扩大为整机系统全部已封装。

## 实际验证

470 个首批原文件在远端复制后重算哈希无变化，新增标定和接口定义有逐文件传输哈希。41 项无硬件检查通过。本轮没有安装环境、构建工作空间、启动相机/ROS/仿真，未发送动作。
