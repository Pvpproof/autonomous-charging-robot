#!/usr/bin/env bash
set -e
source /opt/charging-robot-local/nova2_tools/nova2_env.sh
exec ros2 launch /opt/charging-robot-local/nova2_tools/nova2_real_model.launch.py
