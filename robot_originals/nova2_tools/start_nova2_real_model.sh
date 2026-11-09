#!/usr/bin/env bash
set -e
source /home/nvidia/nova2_tools/nova2_env.sh
exec ros2 launch /home/nvidia/nova2_tools/nova2_real_model.launch.py
