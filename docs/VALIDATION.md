# 验证记录 / Validation evidence

## 2026-11-08 首页与文档整理：本轮离线验证

只修改公开副本的双语首页、原创 SVG、文档布局/说明、配置忽略例外和机械臂测试入口；运行代码、入口、公开配置值与来源哈希未改。未连接或启动现场 ROS、机械臂、底盘或相机，也未部署至设备。

This update changes public presentation, documentation layout, the configuration README ignore exception and an arm test runner. Runtime implementations, entry points, public configuration values and source hashes are unchanged. No field connection, ROS nodes, motion, camera or deployment was performed.

| 命令 / Check | 本轮结果 / Result | 范围 / Scope |
| --- | --- | --- |
| `bash tests/run_local.sh` | 176 passed / 22 suites | 兼容入口、导入/路径、Qt、返程/控制、公开配置、来源清单 |
| `bash tests/run_arm.sh` | 61 passed / 7 suites | 归档来源、合成视觉时序、有限重建、75 mm 几何、假的模型命令 |
| Python AST + `bash -n` | Passed | 全部公开 Python 与 shell 语法；不执行控制入口 |
| 本地 Markdown 链接与原图比较 / Links and diagram comparison | Passed | 文件链接存在；中英文 test/home Mermaid 与 `1f118a1` 一致 |
| 中英文 SVG / SVG assets | Passed | XML/Qt SVG 渲染并人工查看，原创矢量图，无第三方图片 |
| 当前发布文件扫描 / Publication scan | No matches | 未见私钥/令牌、现场私网地址、个人账号路径或设备数据扩展名；扫描不是权属或安全保证 |

未重跑 ROS 隔离集成、原生看门狗或完整仿真，相关代码未变。软件通过不证明实体位移、插接、锁止、电气连接或实际充电。Native watchdog, isolated ROS integration and full simulation were not rerun; these results do not establish hardware acceptance.

许可复核未引入第三方材料；原创 SVG 沿用自有文档的 MIT。PyQt 官方仍声明 GPL v3/商业双许可，见 [Riverbank](https://www.riverbankcomputing.com/software/pyqt/intro)；DOBOT 固定上游及其他历史依赖缺项见 [第三方说明](../THIRD_PARTY_NOTICES.md)、[依赖记录](DEPENDENCIES.md)与 [DOBOT 核对](../arm_stack/DOBOT_DEPENDENCY.md)。没有补发或假定未知厂家授权。

No third-party assets were introduced. Dependency licensing remains separate; unresolved vendor permissions and historical noncommercial metadata remain recorded rather than assumed resolved.

## 历史现场证据 / Historical field evidence

下表保留自 `1f118a1` 首页，未在本轮重新实测。Preserved from the published home page; not rerun in this update.

| 功能 | 已有证据 | 边界 |
| --- | --- | --- |
| 地图、键控、导航去程 | 历史现场使用记录 | 不承诺通用导航精度 |
| 单击原始里程返程 | 用户报告早期版本完整返程成功；本地回归 | 最新改动及目录整理未完成新一轮实车验收 |
| ICP 对齐、V3 枪头预备位 | 用户提供逐段执行成功日志 | 后续插入曾超时，完整电气充电未验证 |
| 停止、失联、任务取消 | 合成数据、假发布器和离线 Qt 回归 | 不替代真实设备验收 |
| 物理急停与下位机断连停车 | 用户报告验证 | 不表述为本仓库自动测试结果 |

| Capability | Evidence | Remaining limits |
| --- | --- | --- |
| Mapping, manual control, outbound navigation | Historical field use | No general accuracy claim |
| Click-to-return | User reported complete return on earlier versions; offline regressions | Latest changes and source reorganization need new field acceptance |
| ICP alignment and V3 pre-insertion pose | User-provided successful stage logs | Later insertion timed out; full electrical charging is unverified |
| Stop, disconnect, cancellation | Synthetic input, fake publishers, offline Qt tests | Not hardware acceptance |
| Hardware emergency stop and controller disconnect stop | User confirmation | Not an automated test result |

## 既有发布阶段记录 / Earlier publication records

以下为此前核查与测试记录，其中“本轮”仅指对应的历史阶段，不代表本次连接过现场。The following notes describe prior stages, not field work during this documentation update.

## 2026-11-08 公开目录整理 / Public source reorganization

本轮将实现迁至 src/charging_robot，保留根启动入口。原有 172 项本地回归通过；另加 4 项入口、路径及清单兼容测试。未启动现场服务、未部署、未进行实车验收。历史证据如下，不能视为本轮重新执行。

## 当前公开副本验证

本轮只读核对现场源码，包含重启后的 arm_process_guard 哈希；整理公开副本，没有向设备发送运动或启停命令。本轮记录与测试输出保留在本地审查目录，不提交真实日志、轨迹、地图、身份或授权。最新修改的现场运动验收不由这些离线测试代替。

### 当前本地验证

共172项通过，等价于 tests/run_local.sh 当前列出的21个测试模块。先运行原有测试入口的16个更新后模块，再分别运行5个新增模块；不把重复执行计入总数。

| 范围 | 数量 |
| --- | --- |
| 基础状态机/仲裁、原始来源闭环、速度和反馈边界、既有策略、驱动契约、单击、故障确认、时钟、Qt、launch/公开配置/隔离检查 | 112 |
| 按轮策略与独立工具/RViz目标选择 | 18 |
| 后台控制存活、普通停止不锁定、最新快照录制 | 42 |
| 合计 | 172 |

测试使用合成轨迹、临时目录、假发布器/假进程表和offscreen Qt。没有创建ROS节点、打开相机/串口、发真实信号给设备进程或调用机械臂test/home。配置保持disabled、commissioned=false，现场声明均false。模型资格测试已按新行为替换，未把旧的模型拒绝或GUI失焦停车预期算作仍然有效。

本次同时检查所有Python语法、shell语法、源码去敏、变更清单与哈希。新增公开适配仅包括外部命令路径、RViz配置路径和一处过时工具提示，现场安装文件不因公开适配被替换。

### 历史测试与限制

旧版 d8b9bd1 曾记录41项ROS沙箱集成和C++看门狗CTest 1/1（内部59断言）通过。这是历史结果，本轮未重新运行，不能当作当前全部修改的集成验收。tests/run_isolated.sh 及其ROS用例保留供后续在真正隔离的网络/PID/设备环境中复核；不能直接在实车ROS域运行。

当前GUI卡住但后台健康时自主运动可继续，GUI按钮可能暂不可用。现场需独立急停；记录无法检测新障碍，无后向自动避障。原始里程是速度积分，不能证明真实地面位移、识别所有轮滑或验证停距。整机/驱动崩溃时的MCU独立停车能力不由本轮假反馈测试证明。

发布默认不会启用实车；维护者设备上的既有成功不能直接移植为其他硬件、无人运行、机械臂插接或电气充电验收。
