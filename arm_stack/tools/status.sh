#!/usr/bin/env bash
set +e
source /opt/charging-robot-local/nova2_tools/nova2_env.sh
printf '\n=== Nodes ===\n'
ros2 node list | grep -E 'dobot|move_group|get_angle|rviz|robot_state' || true
printf '\n=== Actions ===\n'
ros2 action list -t | grep -E 'trajectory|move' || true
printf '\n=== RobotStatus ===\n'
ros2 topic echo /dobot_msgs_v4/msg/RobotStatus --once
printf '\n=== RobotMode ===\n'
ros2 service call /dobot_bringup_ros2/srv/RobotMode dobot_msgs_v4/srv/RobotMode "{}"
printf '\n=== GetErrorID ===\n'
ros2 service call /dobot_bringup_ros2/srv/GetErrorID dobot_msgs_v4/srv/GetErrorID "{}"
printf '\n=== GetPose ===\n'
ros2 service call /dobot_bringup_ros2/srv/GetPose dobot_msgs_v4/srv/GetPose "{}"
printf '\n=== JointState ===\n'
ros2 topic echo /joint_states --once
