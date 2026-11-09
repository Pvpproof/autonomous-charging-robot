# 许可范围 / License scope

归属更正 / Ownership correction：charge_robot、ICP/V3、机械臂仿真和规划应用为 Pvpproof 自有代码；本次纳入的历史源码适用项目 MIT，见 arm_stack 的来源清单。SDK、MoveIt、厂家模型与其他依赖的授权不因此改变。Included project-owned historical applications are MIT; third-party dependencies retain their terms.

[LICENSE](LICENSE) 的 MIT 授权适用于 Pvpproof 自有的源码、测试、原生辅助组件和文档。用户确认有权公开自有项目；源码来源见 [manifest](docs/source-manifest.json)。这不是对所有外部依赖完成许可兼容审查的声明。

MIT covers Pvpproof-owned source, tests, native helpers and documentation. It does not relicense external components or certify compatibility of a bundled distribution.

| 外部材料 / External component | 边界 / Boundary |
| --- | --- |
| PyQt5 / Qt | PyQt 为 GPL v3 或商业许可，不能把自有代码 MIT 当作打包 PyQt 的授权。PyQt is GPL v3 or commercially licensed; packaging obligations remain separate. |
| ROS / Nav2 / message packages | 依各包许可，未随仓库分发。Individual upstream terms apply; not bundled. |
| slam_gmapping | 历史本机元数据声明非商业许可，需核实准确上游版本和适用条款。Historical metadata contains a noncommercial declaration requiring review. |
| 厂家底盘、雷达、Nova2、SDK、模型 | `robot_originals` 收录现用 DOBOT 源码/模型/配置并保留 Dobot LICENSE 与包级声明；底盘/雷达驱动与 SDK 二进制等不在本次恢复范围。Bundled DOBOT originals retain their notices; base/lidar drivers and SDK binaries are outside this recovery scope. |
| charge_robot、ICP/PnP 应用与标定 | 自有原版应用源码已收录，本车标定/配置经作者授权附带；仅适用于该车。Owned original applications and owner-authorized robot-specific calibration/configuration are included. |

PyQt 官方许可说明：[Riverbank](https://www.riverbankcomputing.com/software/pyqt/intro)。项目 MIT 文本：[OSI](https://opensource.org/license/mit)。历史版本及逐项来源见 [DEPENDENCIES.md](docs/DEPENDENCIES.md)。Commercial use or redistribution of a combined system requires reviewing the actual dependencies and their licenses.

本次未发现已跟踪自有源码内的第三方版权头；这不替代上游权属调查。后续引入第三方文件必须保留原许可并单独标识。论文正文与无关实拍/运行数据未纳入；本车模板、标定与配置属于用户明确授权发布范围。

## 原版恢复例外 / Original-source licensing exception

arm_stack/simulation/charging_dock_sim 保留现场原 package.xml 的 Apache-2.0 声明，不被根 MIT 统一改授权。DOBOT 依赖经用户要求收入 `robot_originals/`，保留原版权与包级声明。

## 用户要求的原版恢复包 / Owner-requested original bundle

robot_originals/nova2_ros2_ws/src/dobot_ros2_v4 内为现用第三方副本，保留原 Dobot LICENSE 与 package.xml 的 BSD/TODO 等原声明，不以根 Pvpproof MIT 重署。原仿真包保留 Apache-2.0；本车标定和参数经用户明确要求附带，仅适用于此车。
