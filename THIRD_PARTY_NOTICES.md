# 许可范围 / License scope

归属更正 / Ownership correction：charge_robot、ICP/V3、机械臂仿真和规划应用为 Pvpproof 自有代码；本次纳入的历史源码适用项目 MIT，见 arm_stack 的来源清单。SDK、MoveIt、厂家模型与其他依赖的授权不因此改变。Included project-owned historical applications are MIT; third-party dependencies retain their terms.

[LICENSE](LICENSE) 的 MIT 授权适用于 Pvpproof 自有的源码、测试、原生辅助组件和文档。用户确认有权公开自有项目；源码来源见 [manifest](docs/source-manifest.json)。这不是对所有外部依赖完成许可兼容审查的声明。

MIT covers Pvpproof-owned source, tests, native helpers and documentation. It does not relicense external components or certify compatibility of a bundled distribution.

| 外部材料 / External component | 边界 / Boundary |
| --- | --- |
| PyQt5 / Qt | PyQt 为 GPL v3 或商业许可，不能把自有代码 MIT 当作打包 PyQt 的授权。PyQt is GPL v3 or commercially licensed; packaging obligations remain separate. |
| ROS / Nav2 / message packages | 依各包许可，未随仓库分发。Individual upstream terms apply; not bundled. |
| slam_gmapping | 历史本机元数据声明非商业许可，需核实准确上游版本和适用条款。Historical metadata contains a noncommercial declaration requiring review. |
| 厂家底盘、雷达、Nova2、SDK、模型 | 不包含厂家源码、二进制或参数；部分许可未知。Vendor sources, binaries and parameters are excluded; some terms remain unresolved. |
| charge_robot、ICP/PnP 应用与标定 | 可得历史应用源码已部分纳入 MIT；专用标定排除。Available historical application sources are partially included under MIT; site calibration stays excluded. |

PyQt 官方许可说明：[Riverbank](https://www.riverbankcomputing.com/software/pyqt/intro)。项目 MIT 文本：[OSI](https://opensource.org/license/mit)。历史版本及逐项来源见 [DEPENDENCIES.md](docs/DEPENDENCIES.md)。Commercial use or redistribution of a combined system requires reviewing the actual dependencies and their licenses.

本次未发现已跟踪自有源码内的第三方版权头；这不替代上游权属调查。后续引入第三方文件必须保留原许可并单独标识。论文正文、实拍数据和现场材料未纳入本次授权发布。
