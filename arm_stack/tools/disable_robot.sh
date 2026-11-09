#!/usr/bin/env bash
set -e
source /opt/charging-robot-local/nova2_tools/nova2_env.sh
ros2 service call /dobot_bringup_ros2/srv/DisableRobot dobot_msgs_v4/srv/DisableRobot "{}"
printf '\nRobotStatus after disable:\n'
ros2 topic echo /dobot_msgs_v4/msg/RobotStatus --once
