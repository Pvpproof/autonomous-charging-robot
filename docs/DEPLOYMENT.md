# 部署与运行边界

`charge_robot`、ICP/V3 与机械臂规划应用是作者自有源码，并非因第三方授权而排除。目前待补齐现场最新文件；在完整依赖闭包核对前，仍按独立安装工具配置入口，不能将本仓库当作完整机械臂部署包。

[English](DEPLOYMENT.en.md)

## 技术环境

工作台使用 ROS 2 Humble、PyQt5、NumPy、Pillow、PyYAML，独立工具终端还需要 gnome-terminal，并依赖特定底盘和雷达驱动、导航参数及外部对接工具。具体依赖和再分发边界见 [DEPENDENCIES.md](../DEPENDENCIES.md)。

## 离线预览与测试

使用已具备 PyQt5、NumPy、Pillow、PyYAML 的系统 Python：

```bash
bash start.sh --offline
```

离线界面不导入 ROS Bridge，不连接机器人、启动驱动或发送运动指令。地图、日志和状态保存在当前副本目录并由 Git 忽略。仓库不提供真实地图，测试使用合成数据。

在已安装 ROS 2 Humble 及以上 Python 依赖的环境中执行：

```bash
bash tests/run_local.sh
```

本次目录整理共176项本地测试通过（原有172项与新增4项兼容测试），覆盖记录与闭环、模型移除、后台租约、停止与取消、机械臂工具隔离、RViz目标选择及公开配置。使用合成数据、假发布器与离线Qt；本轮没有运行ROS节点或控制设备。历史ROS沙箱结果及当前验证边界见 [验证记录](VALIDATION.md)。

## 硬件环境配置

真实运行依赖厂家驱动、地图、已验收的 Nav2 参数和独立对接工具。机械臂工具仍需要单独安装的完整 charge_robot 环境；本仓库 arm_stack 已收录部分历史自有实现，但不包含专用标定、厂家 SDK 或完整仿真资产。本仓库不安装或复制这些依赖，也不会自动部署到运行工程。

| 本地配置 | 用途 |
| --- | --- |
| `CHARGING_ROBOT_SETUP` | 已审核机器人 ROS 工作空间的 `setup.bash` 路径；非离线启动时必须指定 |
| `CHARGING_GATED_DRIVER_PREFIX` | 已审核且实现仲裁门禁契约的底盘驱动安装前缀 |
| `CHARGING_NAV2_PARAMS` | 已获使用许可并完成现场核验的 Nav2 YAML 参数文件；导航启动时必须指定 |
| `CHARGING_DOCKING_COMMAND` | 外部人工对接工具可执行文件路径；独立 test/home 会附加对应子命令，未指定时使用 `~/bin/charge_robot` |
| `CHARGING_NAV_RVIZ_CONFIGS` | 可选，冒号分隔的绝对 RViz 配置路径；仅用于识别可独立关闭的导航显示 |

`examples/nav2.placeholder.yaml` 仅解释配置边界，启动器明确拒绝将其用于导航。`config/return_commissioning.json` 保持 `profile:disabled` 和 `commissioned:false`，仲裁对所有实车模式输出零。修改单一布尔值不能替代明确的设备/现场授权。仓库不提供一键实车部署或启用脚本。原始里程relay需从自有C++源码本地构建；底盘驱动必须实现v2看门狗及复位契约，自有核心与独立PTY测试见 [native/driver-watchdog](../native/driver-watchdog/README.md)，厂家驱动仍需外部提供。

完成依赖及现场审查后，`bash start.sh` 打开 ROS 工作台界面；界面按钮可能启动设备和运动流程，不能把该入口作为离线预览。不要与正在运行的工程或其他控制源并发使用。现场还必须确认驱动订阅重映射、独占输出、失联停车行为和短距离倒车表现。

## 数据与设备边界

真实场地图、充电点坐标、运动轨迹、运行日志、设备地址、设备锁定状态、标定、凭据及历史备份不随公开仓库分发。第三方驱动和原始厂家配置也不包含在当前版本中。发布文件来源和脱敏改动见 [来源清单](source-manifest.json)。

## 运行限制

软件停车依赖主机、ROS、通信和控制器正常工作，不能替代硬件急停。人工对接命令返回成功不代表插合、锁止或电气充电已确认。离线测试不能证明实车运行安全或导航精度。
