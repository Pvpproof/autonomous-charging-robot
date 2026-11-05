# 同会话原始里程闭环返程

当前公开版本对应维护者报告已完成实车返回测试的“持续按住运行”版本。它记录去程实际驱动里程与车身朝向，返程以当前位置闭环追踪，不重放去程速度/舵机时间序列。地图用于去程导航，地图位置差不是返程主要门槛。

## 数据和控制约束

驱动 `/odom` 的真实 frame 是 `odom_combined`、child 是 `base_footprint`；这是驱动积分的原始数据，与 EKF/TF 的同名坐标不能混用。native relay 以每条消息实际 DDS GID、relay实例、序号封装反馈；面板和仲裁分别验证来源、时序、连续性、有限值、跳变。记录与候选绑定driver/relay/面板/仲裁会话，重启和重置不能套用旧轨迹。

累计位移与展开航向检查路线几何，避免低速瞬时角速度比值不稳定导致错误地把单帧当作整条路线不可返程。拒绝时保存首个样本及具体原因，不修改或平滑实际记录。

现场验收使用目标0.15m/s、反馈停车阈值0.4m/s、记录路线最大4m，另保留90秒及路线长度相关失控保护。反馈阈值允许较大的原始速度偏差，不代表地面速度已标定或目标速度能准确跟随。公开配置仍为disabled，不能将现场授权或验收结论直接移植到另一台设备。

完整试验策略为supervised_trial，需要明确现场授权；第一次非零发布之前原子持久化消耗一次许可，复位、重启、恢复通信和重复准备不能使旧许可重新可用。正式supervised_route的验收回执要求独立保留。默认不附带任何现场许可。

持续按住和GUI/授权心跳是本版运行条件；松手、失焦、取消、红色软件停车、失联、原始来源异常、冻结/跳变、偏差、无进展、竞争控制或到点均停止，不自动恢复。软件停车依赖软件/通信；物理急停直接断电是维护者的现场确认，不是本仓库的自动化验收结论。下一步“单击启动”尚未包含在这个提交中。

原始里程是速度积分，没有独立地面位置或轮胎转角验证；路径记录无法发现返程时新出现的障碍，没有后向自动避障。日志中的里程不能直接称作实际地面位移或物理停距。

## 原始里程relay

C++ relay只订阅Odometry、发布包含来源身份的JSON消息；不发布速度、不打开串口。需要ROS 2 Humble的ament_cmake、rclcpp、nav_msgs、std_msgs以及CMake/C++17：

```bash
source /opt/ros/humble/setup.bash
cmake -S native -B build/raw-relay -DCMAKE_INSTALL_PREFIX="$PWD/raw-odom-tools"
cmake --build build/raw-relay
cmake --install build/raw-relay
```

二进制和构建产物由Git忽略，不随源码发布。

## 外部驱动契约

不分发厂家驱动源码或设备配置。真实接入必须提供经审核的gated driver前缀：`CHARGING_GATED_DRIVER_PREFIX`；启动器验证包解析来自该前缀，仲裁核验只读参数`charging_return_driver_gate_version=1`。驱动只能订阅`/charging_control/chassis`，不能保留`/red_vel`、`/robot_recharge_flag`、`/set_charge`等旁路。

`CHARGING_ROBOT_SETUP`指定现有ROS工作空间，`CHARGING_NAV2_PARAMS`指定经许可且已现场核验的导航参数。公开仓库不提供/启用物理驱动，不复制现场校准和导航YAML。缺少以上依赖时应拒绝硬件启动。

## 诊断与测试

诊断包含按钮时序、请求、仲裁实际软件发布v/w、接收时刻、原始反馈、来源/序号及停止原因。单调时钟适用于同主机同启动会话的进程比较；ROS时钟用于源年龄和跳变检查。软件发布不等于下位机执行确认，角速度命令不等于轮胎转角实测。

`tests/run_local.sh`执行纯逻辑、Qt和配置描述测试，不运行ROS控制节点。`tests/run_isolated.sh`只可在操作者已经建立的私有网络/PID/设备命名空间中运行；普通ROS_DOMAIN_ID不能替代隔离。仿真测试使用合成地图和轨迹。原始现场地图、轨迹、GID、许可、日志、备份和环境凭据不随仓库分发。
