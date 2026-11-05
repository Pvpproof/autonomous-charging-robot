#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$0")/.."
source /opt/ros/humble/setup.bash
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$PWD:$PWD/tests:${PYTHONPATH:-}"
export QT_QPA_PLATFORM=offscreen
test_runtime="$(mktemp -d /tmp/charging-unit-tests.XXXXXX)"
export ROS_LOG_DIR="$test_runtime/ros_logs"
for test_file in test_return.py test_raw_mode.py test_curvature_fix.py test_speed015.py test_full_trial.py test_feedback025.py test_feedback040.py test_release_policy.py test_panel.py test_launch.py test_publication.py test_isolation_guard.py; do
    /usr/bin/python3 "tests/$test_file"
done
