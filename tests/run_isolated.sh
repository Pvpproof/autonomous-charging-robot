#!/usr/bin/env bash
# The caller must first establish private network/PID/IPC/user namespaces,
# a read-only host root and a private /dev without robot serial devices.
# test_ros_raw.py checks isolation before creating any ROS nodes.
set -eo pipefail
cd "$(dirname "$0")/.."
source /opt/ros/humble/setup.bash
if [[ -n "${CHARGING_ROBOT_SETUP:-}" ]]; then
    source "$CHARGING_ROBOT_SETUP"
fi
export ROS_DOMAIN_ID=187
export ROS_LOG_DIR=/tmp/return-work/ros_logs
export ROS_HOME=/tmp/return-work/ros_home
export QT_QPA_PLATFORM=offscreen
export XDG_RUNTIME_DIR=/tmp/runtime-return
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$PWD/src:$PWD:$PWD/tests:${PYTHONPATH:-}"
for suite in test_ros_raw.py test_ros_oneclick.py test_ros_driver_reset.py; do
    /usr/bin/python3 "tests/$suite" "$@"
done
