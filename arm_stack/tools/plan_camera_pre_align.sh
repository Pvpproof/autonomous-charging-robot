#!/usr/bin/env bash
set -e
source /opt/charging-robot-local/nova2_tools/nova2_env.sh
exec python3 /opt/charging-robot-local/nova2_tools/plan_camera_pre_align_nearest_ik.py
