#!/usr/bin/env bash
set -e
source /home/nvidia/nova2_tools/nova2_env.sh
exec python3 /home/nvidia/nova2_ros2_ws/src/dobot_ros2_v4/dobot_moveit/scripts/get_angle_joint_state_publisher.py
