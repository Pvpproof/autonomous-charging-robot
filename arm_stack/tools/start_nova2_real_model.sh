#!/usr/bin/env bash
# Reconstructed model-only wrapper; does not start hardware or fake joint feedback.
set -eo pipefail
: "${CHARGING_ARM_SETUP:?Set the reviewed ROS workspace setup script}"
: "${CHARGING_ARM_URDF:?Set the reviewed resolved URDF path}"
[[ "$CHARGING_ARM_SETUP" = /* && -f "$CHARGING_ARM_SETUP" ]] || { echo 'Invalid absolute setup path' >&2; exit 2; }
[[ "$CHARGING_ARM_URDF" = /* && -f "$CHARGING_ARM_URDF" ]] || { echo 'Invalid absolute URDF path' >&2; exit 2; }
source "$CHARGING_ARM_SETUP"
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec ros2 launch "$script_dir/nova2_real_model.launch.py"
