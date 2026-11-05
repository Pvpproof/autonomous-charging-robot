# 依赖与发布范围

本记录来自现有工作台的导入和调用关系，供后续发布源码时逐项复核。当前提交仅含项目文档及忽略规则，不分发下列软件包或其源码、二进制、配置。

| 依赖 | 用途 | 本轮核实情况 |
| --- | --- | --- |
| ROS 2 Humble / rclpy / launch / launch_ros | ROS 节点、消息与启动编排 | 外部依赖；未在本轮核实安装版本及各包许可 |
| Nav2 / nav2_msgs / nav2_bringup | 导航目标及导航启动 | 外部依赖；未在本轮核实安装版本及各包许可 |
| PyQt5 | 桌面界面 | 外部依赖；未核实实际安装版本与适用许可，不推定可任意再分发 |
| NumPy / Pillow / PyYAML | 数组、地图图像与 YAML | 外部依赖；未核实实际安装版本与许可文本 |
| turn_on_ychd_robot | 底盘驱动 | 本机安装包许可字段为 `TODO: License declaration`，无法据此确定许可 |
| ychd_robot_keyboard | 键盘方向映射接口 | 本机安装包许可字段为 `TODO: License declaration`，无法据此确定许可 |
| ychd_nav2 | 厂家导航参数来源 | 本机安装包许可字段为 `TODO: License declaration`；参数副本排除发布 |
| lslidar_driver | 雷达驱动 | 安装包声明 `GNU General Public License V3.0`；未复制或分发该包 |
| slam_gmapping | 建图 | 外部依赖；未在本轮核实实际安装包许可 |
| 外部 charge_robot 命令 | 人工对接流程 | 不属于当前候选源码目录，未复制或发布；不推定许可 |

后续分发源码前，应补齐适用版本、上游来源与许可文本。用户的项目公开授权不替代第三方许可条件。当前未因依赖关系而将整个项目自动指定为某一种许可证。
