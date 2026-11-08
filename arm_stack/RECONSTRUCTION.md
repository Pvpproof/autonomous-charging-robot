# 缺件重建 / Missing-module reconstruction

2026-11-08 新写的两个模块，不是找回的现场原版，也未实机验证。

| 模块 | 已证实调用 | 本次能力 |
| --- | --- | --- |
| insertion_runtime.py | insert_execute.py 仅导入 atomic(path, data) | 临时文件、JSON 非有限值拒绝、fsync 与原子替换；不提供插入执行 |
| workflow_progress.py | charge_robot.sh 调用 begin camera/gun、fail、wait、missing；plan_flow_guard 导入 missing() | 保存/读取规划尝试、失败提示、有限等待和调用已有计划资格校验器 |

工作流重建不会创造 workflow_cycle、执行轨迹、启停服务或自动恢复。开始时必须已有 cycle_id；等待时必须是本次开始之后产生的计划、匹配阶段/周期且通过已有 plan_flow_guard.validate。旧文件、替换的尝试、校验失败、时间异常或等待超时都不会报告成功。它不能证明实际机械臂停止或插入完成。

显式要求 CHARGING_WORKFLOW_ROOT 指向已有绝对运行目录。默认等待 60 秒（可 --timeout 调整到不超过 600 秒），这是重建设计，不声称是旧版参数。当前历史布局尚未整合：默认校验器需可导入、使用同一运行目录，否则明确报错。工具提供 missing() 字符串接口；CLI missing 返回 2，供调用方表示没有可执行计划。

Atomic JSON writing and bounded file-only progress are reconstructed from visible callers. No motion, lifecycle control or unknown insertion APIs are implemented. Old outputs cannot satisfy a new attempt; existing eligibility validation is mandatory. Runtime directory, cycle and matching plan identity are required. The 60-second default timeout and new attempt schema are explicitly new choices, not recovered behavior. The historical layout is not yet integrated; unavailable/mismatched validators fail rather than granting success.

11 项针对性离线测试通过：写入失败保留旧文件、旧计划、周期/阶段/时间不符、被替换的任务、校验失败或期间变更、超时、未知接口。该测试不导入 ROS 或运行控制器。

尚未补齐：模型启动包装、完整执行适配器和仿真配置。历史样本的 80/75 mm 差异已按用户明确要求在公开副本统一为 75 mm；旧 80 mm 计划仍拒绝，现场版本未改。

The historical 80/75 mm mismatch has been resolved in the public copy at the owner’s explicit request: 75 mm throughout forward planning/execution. Old 80 mm plans remain rejected; field files are unchanged.

## 模型与纯规划启动补充 / Model and planning wrappers

另外补写 nova2_real_model.launch.py、start_nova2_real_model.sh 和 start_nova2_moveit_plan_only.sh。历史模型文件已找回，但含现场手眼变换，未原样发布；新入口只读取显式指定的已解析 URDF，发布 robot_state_publisher，不生成假关节或相机/枪头静态变换。需要本地 CHARGING_ARM_SETUP、CHARGING_ARM_URDF；纯规划包装固定传 allow_trajectory_execution:=false。

Three additional reconstructed entries display an explicitly supplied model or invoke the external MoveIt plan-only launch. They do not supply site transforms or fake joint feedback. The historical model file was found but its embedded private calibration was excluded.

6 项新增测试检查有效/缺失/模板模型、无私有 TF、包装参数和明确的 setup 要求；使用假的 ros2 记录参数，没有启动 ROS。仍需对应完整上游包、网格、现场反馈来源和标定 TF，不能据此称整套仿真已就绪。
