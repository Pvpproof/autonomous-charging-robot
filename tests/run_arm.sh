#!/usr/bin/env bash
# Static and synthetic checks only; no live workflow entry points or ROS nodes.
set -euo pipefail
cd "$(dirname "$0")/.."
export PYTHONDONTWRITEBYTECODE=1
for suite in test_field_arm_sources.py test_arm_source_alignment.py test_arm_v3_timing.py test_robot_original_bundle.py; do
    /usr/bin/python3 "tests/$suite"
done
