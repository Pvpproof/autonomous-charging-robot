# 历史恢复源码 / Historically recovered sources

这 8 个自有文件来自可追溯的 2026-10-28 至 2026-11-06 本地快照，原始哈希与保留的历史记录匹配。现场当前离线，**不是当前安装版本的完整镜像，也不是可直接部署的工具链**。原始/整理后哈希与证据日期见 [source-manifest.json](source-manifest.json)。代码按项目 MIT 提供；不包含 SDK、厂家模型或标定数据。

Eight project-owned files were recovered from traceable historical snapshots. Original hashes match retained records; current field identity is unverified. This is a **partial source archive, not a deployable toolchain**. See the manifest for dates and before/after hashes. Project MIT applies to these authored files, not external dependencies.

| 文件 / File | 用途 / Purpose |
| --- | --- |
| plan_camera_pre_align_nearest_ik.py | 相机预对齐 IK/MoveIt 规划 / Camera pre-alignment planning |
| plan_charging_gun_prepose.py | 枪头预备位规划 / Gun prepose planning |
| plan_flow_guard.py | 文件状态与计划绑定 / Plan request binding |
| freeze_pnp_v3_target.py | 冻结 V3 目标 / Freeze V3 target |
| plan_insertion_cartesian.py | 笛卡尔插入规划与细化 / Cartesian insertion planning |
| retract_inserted.py | 撤回入口，会调用执行器 / Withdrawal entry invoking execution |
| icp_cloud_bridge.py | ICP 位姿和点云桥接 / ICP bridge |
| pnp_v3_pose_bridge.py | V3 位姿桥接 / V3 bridge |

公开改动仅替换个人目录和标定文件路径；未修改规划算法、历史深度/工具几何常量或异常处理。原历史规划为 80 mm；现已按用户要求调整为 75 mm，并记录公开改动，不能当作当前实测结果。`retract_inserted.py` 明确具有执行动作能力，绝不能为了“测试导入”而运行其 main。

Publication changes replace account-specific paths only. Historical geometry/depth constants are preserved for provenance, not endorsed as current settings. The withdrawal main can execute motion; do not invoke it as an import check.

需要显式设置绝对路径 `CHARGING_SOCKET_VISION_ROOT`（已有本地视觉工作目录）和 `CHARGING_HAND_EYE_FILE`（已有、审核过的手眼标定文件）。辅助函数不会寻找现场目录，也不提供示例实机标定。这两个变量不能解决缺失模块。

Paths require explicit existing `CHARGING_SOCKET_VISION_ROOT` and `CHARGING_HAND_EYE_FILE`. No calibration values are supplied. Setting paths does not complete dependencies.

仍需核对/纳入 `gun_pose_validation`、`stable_pose_selection`、`insert_force`、`prepare_insertion_target`、`insertion_geometry`、`insert_back` 的一致版本，以及原 charge_robot、反馈/执行适配器、仿真和 MoveIt 配置。`workflow_progress.py` 与 `start_nova2_real_model.sh` 尚未找回。另需 ROS 2、MoveIt、Dobot 消息、NumPy、SciPy、Open3D 等外部依赖，其许可另行适用。

Consistent application dependencies, workflow entry points, feedback/execution adapters and simulation/MoveIt configuration remain pending. `workflow_progress.py` and `start_nova2_real_model.sh` are still missing. External ROS/MoveIt/Dobot messages and numerical/point-cloud libraries are not bundled.

仅无设备测试 / Hardware-free checks:

```bash
/usr/bin/python3 tests/test_recovered_arm_sources.py
```

测试解析源码、验证路径和哈希，并提取纯数学函数做合成检查；不导入 ROS 节点模块、不运行入口、不访问相机或设备。

最新状态：workflow_progress/insertion_runtime 已有 [有限重建版](../RECONSTRUCTION.md)，不是现场原版；[DOBOT 依赖核对](../DOBOT_DEPENDENCY.md)见附文。
