#!/usr/bin/env bash
set -euo pipefail

# One-click Nova2 camera pre-alignment planning.
# Plans only; never executes trajectory or sends robot motion commands.

source /home/nvidia/nova2_tools/nova2_env.sh
exec python3 /home/nvidia/nova2_tools/plan_camera_pre_align_nearest_ik.py
