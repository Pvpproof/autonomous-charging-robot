# How to use · 使用指南

[English](HOW_TO_USE.en.md) · [首页](../README.md) · [Agent 接手入口](../AGENTS.md) · [开发指南](DEVELOPMENT.md)

## 1. 下载与选择入口

下载仓库 ZIP 并解压，进入含 `start.sh`、`README.md`、`robot_originals/` 的目录；也可克隆：

```bash
git clone https://github.com/Pvpproof/autonomous-charging-robot.git
cd autonomous-charging-robot
```

| 想做什么 | 使用入口 |
| --- | --- |
| 先看面板、编辑已有地图与点位 | `bash start.sh --offline` |
| 开发底盘面板、去程与返程 | `src/charging_robot/`，见[开发指南](DEVELOPMENT.md) |
| 在原车恢复机械臂、相机、模板与标定 | `robot_originals/`，见[恢复说明](本车恢复说明.md) |
| 独立使用机械臂 | 部署原版后运行 `charge_robot test` / `charge_robot home` |
| 无硬件检查源码或运行规划演示 | 本页第 6、7 节 |

这是完整仓库源码包；ROS、Python 环境、相机驱动、底盘驱动和真实场地图仍按设备准备。`arm_stack/` 是路径适配后的阅读视图，同车恢复以 `robot_originals/` 原件为准。

## 2. 先离线打开面板

系统 Python 需要 PyQt5、NumPy、Pillow、PyYAML。在桌面终端、仓库根目录运行：

```bash
bash start.sh --offline
```

此入口不连接 ROS 或设备。可查看三分区布局、导入自己的地图 YAML、编辑并保存充电点；定位、真实位置反馈和运动按钮需要实车模式。地图用中键拖动、滚轮缩放、右键取消当前标点。

## 3. 准备原车环境

原件按 `/home/nvidia` 布局组织，使用 ROS 2 Humble、原 Nova2 工作空间、RealSense SDK/Python 绑定和 `charging_robot_env`。已知版本和安装边界见[环境说明](../robot_originals/ENVIRONMENT.md)。本车标定、工具补偿与回位姿态不能直接用于另一台机器人。

需要恢复原车文件时，在设备停稳、相关服务关闭后先预览：

```bash
/usr/bin/python3 robot_originals/deploy_originals.py
```

确认目标正确后，用 `--apply` 才会复制并备份旧文件；详细恢复和 `colcon` 构建步骤见[恢复说明](本车恢复说明.md)。复制完成不会自动安装依赖或启动车辆。

## 4. 使用底盘面板

### 配置并打开实车界面

除了机械臂环境，还需底盘/雷达驱动、已集成的门禁驱动、原始里程 relay、有效 Nav2 参数与场地图。以下路径是**需要替换的示例**，不是下载后已存在的安装位置：

```bash
export CHARGING_ROBOT_SETUP=/absolute/path/to/robot_ws/install/setup.bash
export CHARGING_GATED_DRIVER_PREFIX=/absolute/path/to/gated_driver/install
export CHARGING_NAV2_PARAMS=/absolute/path/to/nav2.yaml
export CHARGING_DOCKING_COMMAND=/home/nvidia/nova2_tools/charge_robot.sh
bash start.sh
```

relay 构建和底盘接口见[原始里程返程说明](RAW_ODOM_RETURN.md)。公开配置默认 `disabled`、`commissioned=false`，实车运动需使用完成现场配置和验收的运行配置；修改一个开关不能补齐驱动与参数。

### 地图、去程和返程

1. **地图与点位：** 加载或导入地图，选择已有充电点；没有点位时点击“点击地图新增充电停靠点”，设置停靠位置与朝向，再“保存点位”。重新建图是可选操作，需要真实设备反馈。
2. **连接与定位：** 点击“连接底盘与雷达”及“启动定位与导航”。用“设置初始位置”在地图上标记实车位置和朝向，待定位节点就绪后点“应用初始位置”，核对雷达与地图重合。
3. **去程：** 点击“去程：充电点”；或者“设置导航目标”后点击“去程：地图目标”。记录会自动开始。到达并停稳后查看记录结果；“停止去程”中断的轨迹不作为完整返程记录。
4. **返程：** 保持同一原始里程计会话，车仍在记录终点时，点击一次“开始返程”并确认。无需持续按住；“停止返程”或“停止底盘”可取消本次底盘任务。重启底盘/里程计后应重新录制。
5. **可选机械臂：** 到点不会自动启动机械臂。可分别点击 `charge_robot test`、`charge_robot home`，在独立终端按提示操作。工具结束并确认收拢后，点击“已收拢，释放底盘”再进行底盘运动；不要求 test/home 成功才能保留或使用已完成的去程记录。

键盘操控通过“开启键盘控制”启用，W/S 前后、A/D 转向，松键停车；用“停车并退出键盘”结束。底盘停止按钮不控制机械臂；记录路线不代表返程时没有新增障碍，实车操作仍需现场观察和独立急停。

## 5. 单独运行机械臂 test / home

在已完成原版部署和环境配置的**原车**终端中运行，两个命令分别调用：

```bash
charge_robot test
```

`test` 自动衔接 **ICP 定位 → 相机预对齐 → V3 测量 → 50 mm 枪头预备位 → 75 mm 直线插入**。每段真实运动仍需终端确认；机械臂 RViz 用于显示模型、点云和姿态。未就绪或规划失败时按具体原因排查，不继续确认后续运动。

```bash
charge_robot home
```

`home` 回机械臂保存的初始姿态；如果本轮已插入且未撤回，会先处理撤回。它不让底盘回起点，`test` 也不会自动调用它。完整状态分支和架构见[机械臂工作流](ARM_WORKFLOWS.md)。

原车命令链接应指向 `/home/nvidia/nova2_tools/charge_robot.sh`。若链接不存在，也可显式调用 `bash /home/nvidia/nova2_tools/charge_robot.sh test` 或 `home`；不要在 GitHub 源码目录直接执行这些命令来检查安装，因为它们可能连接和驱动真机。

## 6. 不连接设备的检查

在仓库根目录运行：

```bash
bash tests/run_arm.sh
```

该入口使用现有四组源码/原件哈希、语法、反馈就绪和合成 V3 检查，需要系统 Python 与 NumPy，不启动相机或 ROS 控制节点。工作台完整回归另需 ROS 2 Humble 与 Qt，入口为 `bash tests/run_local.sh`。开发时优先挑选改动相关测试，见[开发指南](DEVELOPMENT.md)。

## 7. MoveIt / RViz 规划演示

已有 `nova2_moveit` 和自有 `charging_dock_sim`，不是完整 Gazebo 物理仿真。先在不连接真实驱动的独立开发环境安装 ROS/MoveIt 并构建这两个包，再加载该工作空间：

```bash
source /opt/ros/humble/setup.bash
source /absolute/path/to/simulation_ws/install/setup.bash
ros2 launch charging_dock_sim charging_dock_demo.launch.py
```

此 launch 引入 Nova2 MoveIt demo 和充电口场景；收录的 Nova2 demo 控制配置使用 `mock_components/GenericSystem`。另开相同仿真环境的终端，可做只规划检查：

```bash
ros2 run charging_dock_sim plan_docking --ros-args --params-file "$(ros2 pkg prefix charging_dock_sim)/share/charging_dock_sim/config/docking_params.yaml"
```

`plan_docking` 使用 plan-only 请求，不提交真实轨迹执行。仿真 YAML 的插入示例为 35 mm，不是实机 test 的 75 mm。新环境的构建、RViz 与完整演示尚未在本次发布中运行验收；原仿真包的 `moveit_commander` 依赖声明需核对，详见[环境说明](../robot_originals/ENVIRONMENT.md)。
