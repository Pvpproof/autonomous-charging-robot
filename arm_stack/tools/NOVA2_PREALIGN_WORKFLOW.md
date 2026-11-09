# Nova2 充电口相机预对准工作流

## 总原则

- 机器人模型：官方 Nova2 URDF：`dobot_rviz/urdf/nova2_robot.urdf`
- MoveIt：官方 `nova2_moveit`，规划组 `nova2_group`
- 真实机械臂：只启动一套 `dobot_bringup_v4`，当前阶段只读 GetPose / joint_states
- 不启动 FakeSystem、ros2_control_node、joint_state_broadcaster
- ICP 独立运行；桥接节点把真实 GetPose、手眼标定和 ICP 位姿合成为 RViz 目标
- 规划必须 `plan_only`，人工检查后才考虑真实执行

## 推荐终端

### 终端 1：真实连接（只读）

```bash
source /opt/charging-robot-local/nova2_tools/nova2_env.sh
ros2 launch dobot_bringup_v4 dobot_bringup_ros2.launch.py
```

只提供：

- `/joint_states`
- `/dobot_bringup_ros2/srv/GetPose`
- `/dobot_msgs_v4/msg/RobotStatus`

禁止在该阶段调用 `EnableRobot`、`MovJ`、`MovL`。

### 终端 2：Nova2 官方 URDF + TF

```bash
/opt/charging-robot-local/nova2_tools/start_nova2_real_model.sh
```

只启动官方 Nova2 URDF 的 `robot_state_publisher`，不启动 FakeSystem。

### 终端 3：实时 ICP

```bash
python3 /opt/charging-robot-local/charging_socket_vision/realtime_socket_pose.py \
  --fps 15 --no-display
```

输出：

- `latest_pose_readonly.json`
- `latest_target_camera.ply`
- `latest_rgbd_camera.ply`

### 终端 4：在线位姿/点云桥接

```bash
source /opt/charging-robot-local/nova2_tools/nova2_env.sh
python3 /opt/charging-robot-local/charging_socket_vision/icp_cloud_bridge.py
```

输出：

- `/icp/rgbd_pointcloud_base`
- `/icp/pointcloud_base`
- `/charging_port_target`
- `/icp/markers`

### 终端 5：Nova2 官方 MoveIt（纯规划）

```bash
source /opt/charging-robot-local/nova2_tools/nova2_env.sh
ros2 launch nova2_moveit move_group.launch.py \
  allow_trajectory_execution:=false
```

这里使用官方 Nova2 MoveIt，不启动 FakeSystem。只提供规划服务，不执行轨迹。

### 终端 6：RViz（必须在上位机本地桌面终端）

```bash
source /opt/charging-robot-local/nova2_tools/nova2_env.sh
rviz2 -d /opt/charging-robot-local/nova2_tools/nova2_icp.rviz
```

RViz 配置：

- Fixed Frame：`base_link`
- Nova2 RobotModel：官方 URDF
- 当前机器人姿态：真实 `/joint_states`
- RGB-D：`/icp/rgbd_pointcloud_base`
- ICP 点云：`/icp/pointcloud_base`
- 目标姿态：`/charging_port_target`
- 规划轨迹：`/display_planned_path`

## 一键纯规划

```bash
source /opt/charging-robot-local/nova2_tools/nova2_env.sh
python3 /opt/charging-robot-local/nova2_tools/plan_camera_pre_align_nearest_ik.py
```

规划内容：

1. 从实时 ICP 获取充电口 6D 位姿；
2. 使用真实 `/joint_states` 作为规划起点；
3. 目标为相机在充电口外侧 150 mm，光轴正对充电口；
4. 调用官方 `/compute_ik`；
5. 选择距离当前关节角最近的 IK 解；
6. 使用官方 `nova2_group` 做关节空间纯规划；
7. 发布 `/display_planned_path`；
8. 不执行轨迹。

## 启动前清理

```bash
/opt/charging-robot-local/nova2_tools/clean_nova2_sim.sh
```

注意：该脚本只清理仿真、RViz、ICP 和桥接；如果已有真实 bringup，请在其终端使用 `Ctrl+C` 停止，不要同时启动第二套。

## 启动前检查

```bash
ps -eo pid,args | grep -E '[d]obot_bringup|[r]obot_state_publisher|[m]ove_group|[r]os2_control_node|[r]viz2|[i]cp_cloud_bridge|[r]ealtime_socket_pose'
```

允许：

- 1 个 `dobot_bringup`
- 1 个官方 Nova2 `robot_state_publisher`
- 1 个 `move_group`
- 1 个 ICP
- 1 个桥接
- 1 个 RViz

禁止：

- `ros2_control_node`
- `joint_state_broadcaster`
- 第二个 `dobot_bringup`
- 第二个 `robot_state_publisher`
- 第二个 ICP
- 第二个 RViz

## 验收命令

```bash
ros2 topic info /joint_states -v
ros2 topic info /robot_description -v
ros2 topic hz /charging_port_target
ros2 topic hz /icp/rgbd_pointcloud_base
ros2 topic echo /charging_port_target --once
ros2 topic echo /display_planned_path --once
```

## 安全边界

当前阶段禁止：

```bash
/opt/charging-robot-local/nova2_tools/enable_robot.sh
```

禁止：

- `ExecuteTrajectory`
- `FollowJointTrajectory`
- `MovJ`
- `MovL`
- 真实插入
- 工具/充电枪动作

只有在 RViz 中确认路径和相机姿态后，才进入单独的真实低速执行阶段。
