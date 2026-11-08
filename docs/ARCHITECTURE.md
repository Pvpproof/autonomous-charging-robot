# 架构与模块边界

[English](ARCHITECTURE.en.md)

README 的流程图描述用户选择，不代表强制流水线。机械臂 test/home 不清除完整去程记录，不以成功结果放行返程。外部工具仍逐段确认，面板不自动执行机械臂。

`ui` 负责 Qt 展示；`control/ros_bridge.py` 管理 ROS 回调和后台控制，使用独立仲裁与租约。`returning` 维护原始里程来源、会话、完整记录和倒序位姿跟踪；数据是驱动积分反馈，不是独立地面真值。去程取消、旧任务回调和会话变化不能复活已取消运动。

`arm` 只包装外部工具。启动独立 test 时，仅关闭可以明确归属的独立导航 RViz；面板地图是 Qt 画布，机械臂 RViz 保留。这条独立工具路径不自动停止 Nav2。保留的 Nav2 生命周期辅助模块不代表默认自动暂停行为。

## 路径兼容

实现位于 `src/charging_robot`。根目录 `app.py`、`control_arbiter_node.py`、`arm_tool_terminal.py`、`docking_console.py` 与 `navigation.launch.py` 是兼容入口。入口加入 `src` 搜索路径，再调用包实现。内部导入使用完整包名；开发脚本设置 `PYTHONPATH="$PWD/src"`。

`charging_robot.paths.PROJECT_ROOT` 固定配置、地图、日志和原生工具的位置，避免重构后写进源码目录。外部配置环境变量和原启动命令不变。普通 Python 模块导入应迁移到新包名；本次保证启动入口兼容，不承诺所有旧内部短模块名可导入。

源文件映射见 [source-layout.json](source-layout.json)。[来源清单](source-manifest.json)保留现场基线哈希，并另记公开整理后的哈希；新目录版本不声称与现场逐字相同。

## 控制与验证边界

普通停止取消任务，不额外锁住后续显式操作；真实仲裁故障仍需处理。返程目标 0.15 m/s，反馈界限 0.4 m/s，按轮记录不使用旧一次性 4 m/90 s 授权；保留路线相关看门狗和输出限幅。未经标定的曲率分类移除不等于取消所有运动限制。

GUI 心跳与后台自主控制分离：界面冻结不必然停车。硬件急停独立于 Qt/ROS。软件测试不证明实体停止距离、障碍检测或充电安全。
