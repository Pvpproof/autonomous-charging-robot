# 更新日志 / Changelog

## Unreleased — 2026-11-08

- 精简双语首页，增加原创机器人模块 SVG 与明确证据图例；保留 test/home 流程图到折叠区，逐步流程归入文档。 / Refresh bilingual home pages with original evidence-labeled SVGs and preserved collapsible test/home diagrams; move detailed steps to guides.
- 汇总文档导航，将依赖/Git 指南归入 docs，补配置与测试目录说明及无硬件机械臂测试入口。 / Consolidate documentation and add configuration/test guides plus a hardware-free arm test runner.

- 自有源码迁至 `src/charging_robot` 六个职责目录；保留原启动入口与配置位置。 / Organize owned sources into six packages; preserve launch entry points and configuration locations.
- 增加中英文项目说明、流程/架构图、部署、测试和贡献指南。 / Add bilingual project, architecture, deployment, testing and contribution documentation.
- Pvpproof 自有代码选用 MIT，明确第三方许可边界。 / Apply MIT to Pvpproof-owned code with explicit third-party exclusions.
- 本轮为公开副本整理，未部署至设备。 / Public-copy reorganization only; not deployed to hardware.

## 41eca1b — 已发布基线 / Published baseline

独立机械臂工具、按轮实际轨迹返程、移除未经标定曲率分类、普通停止取消任务、后台自主租约和里程新鲜度修复。历史现场报告与本地验证范围见来源清单和验证记录。

Independent arm tools, per-route return, removal of uncalibrated curvature classification, non-latching ordinary cancellation, backend leases and feedback freshness fixes. See provenance and validation records for evidence boundaries.
