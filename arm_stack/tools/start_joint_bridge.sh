#!/usr/bin/env bash
set -e
source /opt/charging-robot-local/nova2_tools/nova2_env.sh
exec python3 /opt/charging-robot-local/nova2_ros2_ws/src/dobot_ros2_v4/dobot_moveit/scripts/get_angle_joint_state_publisher.py
