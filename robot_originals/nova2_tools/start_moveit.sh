#!/usr/bin/env bash
set -e
source /home/nvidia/nova2_tools/nova2_env.sh
exec ros2 launch dobot_moveit dobot_moveit.launch.py
