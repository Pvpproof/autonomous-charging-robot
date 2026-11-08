# 自有机械臂与视觉源码 / Project-owned arm and vision sources

此目录整理 Pvpproof 自写的 charge_robot 工具链、ICP、V3、机械臂反馈/显示和规划代码。**这是历史源码集合，当前机器人离线，未确认与现场最新版一致，且依赖尚未全部配齐。**不是可直接部署的软件包。没有自动安装或接入面板启动入口。

This directory contains Pvpproof-authored workflow, ICP, V3, feedback/display and planning code. It is a historical source collection, not a deployable package or a claim of current field identity. No automatic installer or panel integration is supplied.

## 范围 / Scope

| 目录 / Directory | 内容 / Content |
| --- | --- |
| tools | test/home 工作流、执行与退回、反馈和就绪、RViz launch/config，以及恢复的几何/绑定依赖 |
| vision/icp | 实时点云模板匹配 / Realtime point-cloud template matching |
| vision/v3 | 七个 RGB PnP、追踪、恢复和稳定测量模块 / Seven RGB PnP/tracking/recovery modules |
| recovered | 8 个带历史哈希证据的规划、桥接及撤回文件 / Eight traceable recovered files |
| examples | 未启用的历史算法设置示例 / Inactive historical algorithm examples |

最初 25 个候选中 17 个与安装记录哈希匹配，8 个仅有缓存证据；recovered 的 8 个均有历史哈希匹配；另外 7 个几何/绑定依赖来自历史审查或修复快照。合计 40 个源码/显示文件，另有路径适配器、2 个示例和文档。[主清单](source-manifest.json)与[恢复清单](recovered/source-manifest.json)记录原始/公开哈希及版本边界。这里的“匹配”不表示今天现场没有变化。

Of the initial 25 candidates, 17 match installation evidence and 8 are cached-only. Eight recovered files have historical hash matches; seven further dependencies come from historical review/fix snapshots. Manifests separate original/public hashes and evidence strength. Historical identity does not prove current installation identity.

## 配置边界 / Configuration boundary

个人账号目录已替换为 `/opt/charging-robot-local`，这是**不自动创建、不自动安装的布局占位符**，不能直接复制启动命令运行。历史脚本仍依赖其原有相对布局和缺失模块；移植时需逐项核对。`nova2_env.sh` 的控制器地址改为必须显式提供 `CHARGING_ARM_ADDRESS`，没有默认现场地址。

Account-specific paths are replaced by a non-live layout placeholder, not an installed tree. Historical scripts still need consistent layout and missing dependencies. The controller address has no default and must be explicitly supplied. Do not run these entry points as smoke tests: test/home/execution scripts can move hardware.

`recovered` 中的路径适配器另要求显式视觉根目录和手眼标定路径。两组历史入口尚未统一成一个安装布局，这是待完成集成项。examples 的特征点是历史目标几何，非相机内参或手眼标定；其数值不证明适用于其他插口，也不能当现场验收值。示例命名不会被默认读取。

Recovered modules use explicit vision/calibration path helpers; these and the archived workflow layout are not yet unified. Feature points in examples describe historical target geometry, not camera/hand-eye calibration. Examples are inactive and not universal hardware settings.

未包含：相机内参、手眼标定、初始机械臂关节位姿、枪头补偿配置、实拍数据、现场轨迹/状态、SDK、厂家模型网格、固件。自有代码 MIT；MoveIt、ROS、Dobot 消息、Open3D、OpenCV、RealSense 等仍为外部依赖。

## 未完成 / Missing or unverified

- `workflow_progress.py`、`insertion_runtime.py` 的原版仍未找到；已按明确调用接口提供有限重建版，见下方重建说明。
- 模型显示和纯规划启动已提供明确标记的重建入口；历史模型内的现场变换已排除，完整运行布局及 TF 仍待配置。
- 完整仿真/MoveIt 包、URDF/SRDF/网格的来源与许可、执行适配器一致版本仍需盘点。
- ICP PLY 模板、初始化元数据及项目特定配置需审核；不以空模板或伪造标定补齐。
- 历史版本可能混有不同阶段接口，尚未验证全链互通；现场恢复连接后须再核对。

Original workflow/insertion runtime files remain unavailable; limited, explicitly marked file-only reconstructions are provided. Model/MoveIt launch chain and complete simulation assets remain missing or unverified. No motion behavior or calibration is fabricated. This mixed historical collection has not passed end-to-end integration.

## 无设备验证 / Hardware-free validation

从仓库根目录运行 / From repository root:

```bash
/usr/bin/python3 tests/test_arm_source_alignment.py
/usr/bin/python3 tests/test_arm_v3_timing.py
/usr/bin/python3 tests/test_recovered_arm_sources.py
/usr/bin/python3 tests/test_arm_archive.py
```

仅使用合成时间/几何、AST、文件哈希和 bash 语法检查；不运行控制入口、ROS 节点或相机。原有面板测试结果不代表这些新加入历史源码已整体实机验证。

## 最新补充 / Latest supplement

两个缺失文件已提供显式标记的[有限重建版](RECONSTRUCTION.md)，不再算作完全没有实现，但原版行为及现场兼容仍未验证。[DOBOT 上游核对](DOBOT_DEPENDENCY.md)已确认 6 个模型/配置历史哈希匹配；未引入第三方资产。

最新汇总：40 个历史文件 + 5 个重建文件，2 个算法示例；针对性无设备测试 61 项通过。模型显示/纯规划包装已补，Gazebo/完整仿真、现场 TF/执行适配器和一致版本尚未验收。
