# 机械臂、视觉与仿真原文件 / Field-original arm, vision and simulation sources

2026-11-09 从机器人 Jetson 只读复制。原件完整落盘在 pvpproof 的隔离恢复目录，470 个文件逐一核对传输哈希；本公开副本使用 85 个不同来源的自有源码/配置/资源，另同步 8 个旧 recovered 路径，共 93 个路径。详见 [来源清单](field-source-manifest.json)。

Sources were read directly from the robot on 2026-11-09. The private local recovery snapshot retains 470 original files. This public copy derives from 85 distinct project-source files, with eight synchronized legacy locations. Raw and adapted hashes are recorded separately.

## 原版替换 / Original implementations

workflow_progress.py、insertion_runtime.py、模型/纯规划启动脚本已经用现场原件替换昨天的重建实现。新增的 insertion_limits.py、publication_paths.py 及不在当前现场链中的旧 insert_force.py 已移出。重建历史保留于 Git 历史，不作为当前实现。

原始 nova2_env.sh 含设备地址，公开文件仅将地址分离为显式环境变量。原始模型 launch 含现场手眼变换，完整原件经作者授权收入 `../robot_originals/`；本目录的派生文件省略该相机静态 TF，不编造替代数值。个人目录统一替换为 /opt/charging-robot-local 占位布局。原始功能没有重新设计，但这些路径仍需部署时适配。

Original workflow/runtime and startup wrappers supersede yesterday's reconstructions. This reading view uses local device configuration and omits the camera transform without inventing a replacement; the authorized exact originals are published separately in `../robot_originals/`. Account-specific roots become a documented placeholder layout, not an automatic installation.

## 已恢复 / Recovered

- tools：原版 test/home 调度、workflow/runtime、执行/撤回、75 mm 插入规划、反馈与就绪、模型/MoveIt 启动及策略配置。
- vision/icp 与 vision：实时 ICP、桥接、模板采集与离线配准工具、预览辅助、算法配置、被源码 SHA-256 认可的 socket_frame_and_rim_2mm_m.ply。
- vision/v3：当前 V3 源码、实际 pnp_near.json 和 live_settings.json、环境检查与原版测试。
- simulation/charging_dock_sim：完整自有 Python ROS 包，含 setup、resource、launch、场景、规划、自动演示和示例场景参数。保留原 package.xml 的 Apache-2.0 声明，不改成 MIT。

## 依赖和未纳入内容 / Dependencies and exclusions

DOBOT 驱动、消息、MoveIt、Nova2 模型/网格等 509 个原文件（含后补的 126 个 ROS 接口）已收入 `../robot_originals/nova2_ros2_ws/src/dobot_ros2_v4/`，保留原 LICENSE 和包声明；不在本阅读视图中重复存放。现场目录没有可用的 Git remote/HEAD，不能宣称已确定整套上游提交及修改差异。执行适配器 action_move_server.py 属于这批依赖，文件已找回，但不能把其混合来源实现改署 Pvpproof MIT。

本车手眼标定、初始机械臂姿态、枪头横向补偿、相机参数和原现场配置经作者授权收入 `../robot_originals/`。密码、密钥和无关运行数据不公开。Python/ROS 与相机驱动仍需安装；其他机器人需要自己的标定与配置，源码恢复不等于运行验收。

The vendor dependency snapshot and authorized robot-specific calibration, saved home joints, compensation and original configuration are published in `../robot_originals/`, with original notices. Missing Git metadata prevents identifying its checkout revision. Credentials and unrelated runtime data are excluded. System dependencies and runtime acceptance remain separate.

## 验证 / Validation

本轮 33 项针对性无硬件检查通过：10 项来源/语法/模板/包结构、11 项真实姿态就绪逻辑、12 项合成 V3 时序。未运行 ROS 节点、相机、MoveIt、仿真或机械臂。原件的插入目标为 75 mm，规划标称速度为 10 mm/s；仿真示例 insert_depth 为 35 mm，属于独立演示参数，没有擅自统一。

```bash
/usr/bin/python3 tests/test_field_arm_sources.py
/usr/bin/python3 tests/test_arm_source_alignment.py
/usr/bin/python3 tests/test_arm_v3_timing.py
```

No device or simulation execution occurred. The actual field insertion planner uses 75 mm at a nominal 10 mm/s; the standalone simulation example uses a distinct 35 mm insertion setting. Neither was silently changed during recovery.

## 用户新增要求：同车原版恢复

现用 DOBOT 依赖、原始脚本和本车手眼/回位/补偿/相机配置已收入 [robot_originals](../robot_originals/README.md)，不再属于“尚未找回”。本目录保持去敏的源码阅读视图；同车恢复应以原版包为准。原版包保留原版权，不含系统/ROS 安装和虚拟环境。
