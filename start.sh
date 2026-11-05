#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"

# Offline preview never sources the robot workspace or imports the ROS bridge.
for argument in "$@"; do
    if [[ "$argument" == "--offline" ]]; then
        exec /usr/bin/python3 app.py "$@"
    fi
done

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
    echo "ROS 2 Humble is unavailable. Use --offline for UI preview." >&2
    exit 1
fi
if [[ -z "${CHARGING_ROBOT_SETUP:-}" || ! -f "$CHARGING_ROBOT_SETUP" ]]; then
    echo "Set CHARGING_ROBOT_SETUP to your reviewed robot workspace setup.bash." >&2
    exit 1
fi
set +u
source /opt/ros/humble/setup.bash
source "$CHARGING_ROBOT_SETUP"
set -u
exec /usr/bin/python3 app.py "$@"
