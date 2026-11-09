# Nova2 MoveIt quick operation notes

Workspace:

```bash
/home/nvidia/nova2_ros2_ws
```

Environment used for every terminal/script:

```bash
source /opt/ros/humble/setup.bash
source /home/nvidia/nova2_ros2_ws/install/setup.bash
export DOBOT_TYPE=nova2
export IP_address=192.168.5.1
```

Verified milestone on 2026-10-12:

- Official DOBOT ROS2 V4 stack compiled successfully.
- Real Nova2 bringup connected.
- `GetAngle` read real joint angles.
- Read-only bridge publishes real joints to `/joint_states_robot`.
- `/joint_states` reflects real Nova2 joints.
- RViz model posture matches real arm.
- MoveIt plan succeeded.
- First real MoveIt Execute succeeded.
- Post-execute check: `RobotMode={5}`, `GetErrorID={[]}`.

Manual terminal stack that worked:

1. Dobot bringup:

```bash
ros2 launch dobot_bringup_v4 dobot_bringup_ros2.launch.py
```

2. Real joint bridge:

```bash
python3 /home/nvidia/nova2_ros2_ws/src/dobot_ros2_v4/dobot_moveit/scripts/get_angle_joint_state_publisher.py
```

3. MoveIt + RViz:

```bash
ros2 launch dobot_moveit dobot_moveit.launch.py
```

Useful checks:

```bash
ros2 topic echo /dobot_msgs_v4/msg/RobotStatus --once
ros2 service call /dobot_bringup_ros2/srv/RobotMode dobot_msgs_v4/srv/RobotMode "{}"
ros2 service call /dobot_bringup_ros2/srv/GetErrorID dobot_msgs_v4/srv/GetErrorID "{}"
ros2 service call /dobot_bringup_ros2/srv/GetPose dobot_msgs_v4/srv/GetPose "{}"
ros2 topic echo /joint_states --once
```

Safety:

- Do not auto-enable on startup.
- Do not auto-execute motion.
- First check status, then enable only when operator is ready.
- Physical emergency stop must be reachable before Execute.
- Use small MoveIt targets first.
