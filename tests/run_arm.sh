#!/usr/bin/env bash
# Static and synthetic checks only; no live workflow entry points or ROS nodes.
set -euo pipefail
cd "$(dirname "$0")/.."
export PYTHONDONTWRITEBYTECODE=1
for suite in test_arm_source_alignment.py test_arm_v3_timing.py test_recovered_arm_sources.py test_arm_archive.py test_reconstructed_workflow.py test_arm_model_entries.py test_insertion_depth75.py; do
    /usr/bin/python3 "tests/$suite"
done
