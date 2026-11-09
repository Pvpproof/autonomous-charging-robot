#!/usr/bin/env bash
set -e
source /opt/charging-robot-local/nova2_tools/nova2_env.sh
printf 'About to call EnableRobot(). Ensure e-stop is reachable and arm area is clear.\n'
printf 'Press Enter to continue, or Ctrl+C to cancel.\n'
read -r _
ros2 service call /dobot_bringup_ros2/srv/EnableRobot dobot_msgs_v4/srv/EnableRobot "{}"
printf '\nRobotStatus after enable:\n'
ros2 topic echo /dobot_msgs_v4/msg/RobotStatus --once
printf '\nRobotMode after enable:\n'
ros2 service call /dobot_bringup_ros2/srv/RobotMode dobot_msgs_v4/srv/RobotMode "{}"
