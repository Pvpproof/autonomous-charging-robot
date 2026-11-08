# 文档导航 / Documentation

[中文首页](../README.md) · [English home](../README.en.md)

| 阅读目标 / Purpose | 中文 | English |
| --- | --- | --- |
| 理解模块与兼容入口 / Modules and compatibility | [架构](ARCHITECTURE.md) | [Architecture](ARCHITECTURE.en.md) |
| 了解 test/home 逐步流程 / Inspect the staged arm workflow | [机械臂流程](ARM_WORKFLOWS.md) | [Arm workflows](ARM_WORKFLOWS.en.md) |
| 离线预览、外部依赖与现场边界 / Preview and integration | [部署](DEPLOYMENT.md) | [Deployment](DEPLOYMENT.en.md) |
| 选择合适的无硬件测试 / Select hardware-free checks | [测试](TESTING.md) | [Testing](TESTING.en.md) |

- [验证记录 / Evidence](VALIDATION.md)：当前软件检查与历史现场证据分开记录。
- [返程实现 / Recorded-route return](RAW_ODOM_RETURN.md) · [早期候选说明 / Historical candidate](RETURN_CANDIDATE.md)。
- [机械臂源码阶段记录 / Arm inventory history](ARM_SOURCE_SCOPE.md)；当前缺件以 [arm_stack](../arm_stack/README.md) 为准。
- [依赖核查 / Dependency audit](DEPENDENCIES.md) · [第三方许可 / Notices](../THIRD_PARTY_NOTICES.md) · [Git 使用](GIT_GUIDE.md)。
- [来源清单 / Provenance](source-manifest.json) · [迁移映射 / Source mapping](source-layout.json)。

`assets/robot-overview.svg` 与 `assets/robot-overview.en.svg` 是可编辑的原创 SVG，分别服务中英文首页，图例来自首页及验证记录。修改模块、证据或状态时同步修改两个版本；图中颜色不代表实时设备状态。

The two original SVG files share the same layout and evidence legend. Keep both languages aligned when capabilities or evidence change; these are documentation assets, not live telemetry.
