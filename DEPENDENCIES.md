# 依赖与许可核查

归属更正：用户确认 charge_robot 工具链、ICP、V3、机械臂仿真与路径规划应用均为 Pvpproof 自写。下文“外部”表示当前仓库未收录；不代表这些应用是第三方。其调用的 SDK、MoveIt、模型、相机驱动和专用标定仍需分别核查。当前补全状态见 [ARM_SOURCE_SCOPE](docs/ARM_SOURCE_SCOPE.md)。

本仓库发布维护者已授权公开的项目自有 Python 源码、测试及文档；不包含第三方库、驱动源码、二进制或厂家参数副本。下表是 2026-11-05 对实际设备已安装包元数据的只读核查，不表示已完成全部依赖的法律兼容性判断，也不是完整依赖锁定文件。

| 依赖 | 已安装版本 | 元数据中的许可声明 | 发布范围 |
| --- | --- | --- | --- |
| PyQt5 | 5.15.6 | GPL v3 | 外部 GUI 依赖，不复制分发 |
| NumPy | 1.26.4 | BSD License 分类；元数据还包含 bundled libraries 的各自条款 | 外部依赖，不复制分发 |
| Pillow | 12.0.0 | MIT-CMU（License-Expression） | 外部依赖，不复制分发 |
| PyYAML | 6.0.3 | MIT | 外部依赖，不复制分发 |
| rclpy | 3.3.19 | Apache License 2.0 | 外部 ROS 依赖 |
| launch | 1.0.13 | Apache License 2.0 | 外部 ROS 依赖 |
| launch_ros | 0.19.13 | Apache License 2.0 | 外部 ROS 依赖 |
| nav2_bringup | 1.1.6 | Apache-2.0 | 外部导航依赖 |
| nav2_msgs | 1.1.20 | Apache-2.0 | 外部消息依赖 |
| slam_gmapping | 0.0.0 | CreativeCommons-by-nc-sa-2.0 | 非商业条款不能据“已安装”视为任意可再分发；不复制分发 |
| turn_on_ychd_robot | 0.0.0 | TODO: License declaration | 无法确定许可；不复制分发 |
| ychd_robot_keyboard | 0.0.0 | TODO: License declaration | 仅调用已安装方向映射接口，不复制其源码；许可待核实 |
| ychd_nav2 | 0.0.0 | TODO: License declaration | 厂家参数副本排除，改为显式指定本地参数 |
| lslidar_driver | 1.2.0 | GNU General Public License V3.0 | 不复制分发 |
| 自有 charge_robot 工具链 | 最新版本待现场核对 | 用户确认自有；拟按项目 MIT 发布 | 当前待补齐，不是第三方 SDK |

工作台还使用 geometry_msgs、nav_msgs、sensor_msgs、std_msgs、action_msgs、tf2_ros、tf2_msgs、ament_index_python 等 ROS 包。本轮未逐项核实这些包的安装版本与许可文本，仓库不包含它们。

元数据来源为本机 Python distribution metadata 与各 ROS 安装包的 package.xml。厂家包中占位许可声明不是有效许可证明。本轮未获取或验证缺失的上游授权，不把用户对自有项目的公开授权扩大到第三方材料。

Pvpproof 自有代码现采用 MIT，见 LICENSE 与 THIRD_PARTY_NOTICES.md；该授权不覆盖外部依赖，也不声称组合发行已完成许可兼容审查。特别是 PyQt5 的 GPL 声明、建图包的非商业条款及未明确的厂家许可，仍需在打包依赖、分发整套软件或商业部署前核实适用条件；本次公开不包含这些第三方材料。

本次新增项目自有C++原始里程relay源码，构建外部依赖ament_cmake、rclcpp、nav_msgs、std_msgs与C++17工具链；不附带这些依赖源码或编译二进制。专用底盘驱动门禁实现仍作为外部接口契约，不将许可未明确的厂家驱动复制进仓库。先前版本表为既有核查记录，本次没有重新锁定全部依赖版本。

单击返程新增std_srvs作为外部ROS依赖，以及项目自有C++14命令看门狗核心、Linux有界串口writer和独立PTY测试，位于native/driver-watchdog。其构建使用CMake、pthread和libutil，不复制这些依赖；不包含厂家集成驱动或含厂家实现上下文的开发补丁。外部驱动需实现文档中的v2契约，不能直接用原厂驱动替代。


本次新增的是项目自有独立工具面板、工具终端包装、进程域检查、导航显示选择及Nav2生命周期辅助模块。外部依赖还包括gnome-terminal和lifecycle_msgs；测试仅使用假的进程、发布器及状态。机械臂charge_robot、Nova2/MoveIt环境、ICP/PnP程序、厂家模型和相机/机械臂标定仍由使用者另行提供，不在本次源码同步中分发。此前依赖版本表是历史核查，本轮没有重新锁定全部依赖版本。

机械臂源码补充：arm_stack 已加入可得自有历史源码。其外部导入还包括 OpenCV、Open3D、SciPy、RealSense Python SDK、MoveIt/control_msgs、Dobot 消息、moveit_configs_utils 等；本轮未重新锁定其版本或核实全部适用许可，不附带这些库、SDK 或模型。完整运行依赖与当前缺件见 [arm_stack](arm_stack/README.md)。

DOBOT Nova2 模型/MoveIt 已完成固定上游提交与 6 项历史哈希比对，根许可 MIT、部分包元数据另有 BSD/TODO；具体边界见 [上游核对](arm_stack/DOBOT_DEPENDENCY.md)，未将厂家资产纳入自有 MIT。
