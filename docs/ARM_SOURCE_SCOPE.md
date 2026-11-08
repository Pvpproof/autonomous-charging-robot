# 机械臂源码补全 / Arm source import

用户确认 charge_robot 工具链、ICP、V3、机械臂仿真和路径规划应用均由 Pvpproof 编写。之前将“未收入仓库”与“第三方代码”混为一谈的说明已更正。用户自有代码拟按项目 MIT 发布，第三方 SDK、MoveIt 及厂家模型保持各自许可。

The author confirms ownership of the arm workflow, ICP, V3, simulation and planning application code. “Not bundled” did not imply third-party ownership. Project-owned applications are intended for MIT publication; SDKs, MoveIt and vendor models retain their own terms.

## 初次盘点（历史记录） / Initial inventory history

机器人连接不可达，无法核对现在的安装内容。已在仓库外准备 25 个历史候选文件，17 个哈希与安装收据相符；另外 8 个仅能确认来自本地缓存。未将它们冒充最新完整源码加入运行目录。

The robot is unreachable. There are 25 historical candidate files outside the public repository; 17 match installation receipts, while 8 are cached-only. They have not been represented as the latest complete implementation.

候选范围包括 charge_robot.sh、执行/插入/退回/home 包装、启动与就绪检查、真实关节反馈与 RViz launch、实时 ICP 和 V3 的七个 Python 模块。候选代码通过 Python AST 或 bash -n 语法检查；这不证明依赖齐全或可实机运行。

Candidates cover the workflow shell, execution/insertion/return wrappers, startup/readiness, joint feedback/RViz launch, realtime ICP and seven V3 Python modules. AST/bash syntax checks do not establish a runnable or hardware-validated package.

## 初次盘点缺件（历史列表） / Initial missing-source list

- freeze_pnp_v3_target.py
- icp_cloud_bridge.py
- plan_camera_pre_align_nearest_ik.py
- plan_charging_gun_prepose.py
- plan_flow_guard.py
- plan_insertion_cartesian.py
- pnp_v3_pose_bridge.py
- start_nova2_real_model.sh
- workflow_progress.py

该列表来自现有入口的静态引用，不是完整依赖锁定。仿真包、MoveIt 配置、模型文件、ICP 模板与初始化元数据也需现场盘点；不能假设缺失文件都属于厂家。待恢复连接后核对源码、哈希和来源，抽离设备地址与专用标定，再构建可运行入口和合成测试。

This list comes from static entry-point references, not a complete dependency lock. Simulation packages, MoveIt configuration, models, ICP templates and initialization metadata still need inventory. Missing files are not presumed vendor-owned. After reconnection, verify sources/hashes, separate device settings/calibration, and validate runnable entries with synthetic tests.

未启动设备、未改变现场文件、未提交或推送本次补全。No devices started, field files changed, or source-import changes committed/pushed.

## 后续恢复 / Subsequent recovery

公开副本已加入 [8 个历史恢复文件](../arm_stack/recovered/README.md)，原始哈希均找到历史记录匹配，证据日期为 2026-10-28 至 2026-11-06。它们包括初次清单中的 7 项，另加撤回入口 retract_inserted.py；所以不是“9 个缺件只剩 1 个”。workflow_progress.py 和 start_nova2_real_model.sh 仍缺。

Eight historically verified files are now included, covering seven initial missing entries plus retract_inserted.py. Two initial entries remain missing. Current field identity is still unverified.

本次新增 7 项无硬件测试通过，覆盖哈希、语法、显式路径、无默认标定和纯旋转数学函数。之前的 25 文件候选及 23 项针对性测试保留为独立证据，不合并宣称为完整工具链验收。依赖模块、模型和配置仍需继续核对。

Seven new publication tests passed. The prior 25-file inventory and 23 targeted tests are separate evidence, not full-stack acceptance. No device service was started; this supplement is not committed or pushed.

## 当前整理状态 / Current import status

此前 25 个候选已全部作为历史材料加入公开副本，保留 17 个安装回执匹配与 8 个缓存来源的区别；另加入 7 个缓存几何/绑定依赖。连同已恢复的 8 个文件，共 40 个源码/显示文件，2 个未启用算法配置示例。详见 [目录、缺件与运行边界](../arm_stack/README.md)。

The previous 25 candidates are now included as historical sources, plus seven cached geometry/binding dependencies and eight recovered files: 40 source/display files and two inactive examples. Evidence strength is preserved; this is not a complete runtime or current-field claim.

复制后验证：11 项真实姿态就绪逻辑、12 项 V3 合成时序、7 项恢复源码检查、6 项全档案检查，共 36 项针对性测试通过。未运行设备、未提交或推送本次补充。

Validation against imported copies: 11 alignment, 12 synthetic V3 timing, 7 recovered-source and 6 archive tests (36 total). No devices started; this supplement is not committed or pushed.

## 重建与上游核对 / Reconstruction and upstream audit

workflow_progress.py 与 insertion_runtime.py 现有标记清晰的有限重建，分别只负责文件规划状态/等待和原子 JSON 写入；11 项新增测试通过，详见 [重建边界](../arm_stack/RECONSTRUCTION.md)。加上既有测试与更新后的 7 项档案检查，本轮相关测试共 48 项通过。没有补写未知运动行为，80/75 mm 差异随后按用户要求在公开副本统一为 75 mm，未改现场。

DOBOT 官方 main 的 6 个 Nova2 模型/MoveIt 文件与历史现场清单一致，但完整现场提交未确定；[许可与依赖说明](../arm_stack/DOBOT_DEPENDENCY.md)已更新。未运行设备或推送。

## 最终本地整理汇总 / Local preparation summary

已收录 40 个历史源码/显示文件、5 个重建文件、2 个算法示例；另有配置模板、路径辅助和文档。模型启动链可明确补齐的部分已完成，54 项针对性测试通过。未启动 ROS/设备、未推送。当前不能称为可直接安装的完整实机或仿真系统，具体缺项见 arm_stack/README.md。
