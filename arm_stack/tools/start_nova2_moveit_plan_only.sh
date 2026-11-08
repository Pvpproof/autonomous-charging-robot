#!/usr/bin/env bash
# Reconstructed wrapper; requires the separately installed reviewed upstream stack.
set -eo pipefail
: "${CHARGING_ARM_SETUP:?Set the reviewed ROS workspace setup script}"
[[ "$CHARGING_ARM_SETUP" = /* && -f "$CHARGING_ARM_SETUP" ]] || { echo 'Invalid absolute setup path' >&2; exit 2; }
source "$CHARGING_ARM_SETUP"
exec ros2 launch nova2_moveit move_group.launch.py allow_trajectory_execution:=false
