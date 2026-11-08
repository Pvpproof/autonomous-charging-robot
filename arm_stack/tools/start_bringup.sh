#!/usr/bin/env bash
set -e
source /opt/charging-robot-local/nova2_tools/nova2_env.sh
exec ros2 launch dobot_bringup_v4 dobot_bringup_ros2.launch.py
