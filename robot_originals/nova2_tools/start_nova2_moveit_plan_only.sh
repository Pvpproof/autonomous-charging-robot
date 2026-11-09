#!/usr/bin/env bash
set -e
source /home/nvidia/nova2_tools/nova2_env.sh
exec ros2 launch nova2_moveit move_group.launch.py allow_trajectory_execution:=false
