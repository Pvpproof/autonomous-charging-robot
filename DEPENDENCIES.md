# 依赖与许可核查

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
| 外部 charge_robot 工具 | 未核实 | 未核实 | 不包含；使用者需自行提供有权使用的工具 |

工作台还使用 geometry_msgs、nav_msgs、sensor_msgs、std_msgs、action_msgs、tf2_ros、tf2_msgs、ament_index_python 等 ROS 包。本轮未逐项核实这些包的安装版本与许可文本，仓库不包含它们。

元数据来源为本机 Python distribution metadata 与各 ROS 安装包的 package.xml。厂家包中占位许可声明不是有效许可证明。本轮未获取或验证缺失的上游授权，不把用户对自有项目的公开授权扩大到第三方材料。

项目尚未选择整体开源许可证，未添加 LICENSE 或声称 MIT/Apache/GPL 兼容。特别是 PyQt5 的 GPL 声明、建图包的非商业条款及未明确的厂家许可，仍需在打包依赖、分发整套软件或商业部署前核实适用条件；本次公开不包含这些第三方材料。

本次新增项目自有C++原始里程relay源码，构建外部依赖ament_cmake、rclcpp、nav_msgs、std_msgs与C++17工具链；不附带这些依赖源码或编译二进制。专用底盘驱动门禁实现仍作为外部接口契约，不将许可未明确的厂家驱动复制进仓库。先前版本表为既有核查记录，本次没有重新锁定全部依赖版本。
