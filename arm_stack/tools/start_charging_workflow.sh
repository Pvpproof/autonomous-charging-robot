#!/usr/bin/env bash
set -eo pipefail

# One-click Nova2 charging workflow launcher.
# Real robot connection is read-only; MoveIt execution is disabled.

export DISPLAY="${DISPLAY:-:0}"
source /opt/charging-robot-local/nova2_tools/nova2_env.sh

run_terminal() {
  local title="$1"; shift
  gnome-terminal --title="$title" -- bash -lc "$*; exec bash" >/dev/null 2>&1 &
}

has_cmdline() {
  local needle="$1"
  ps -eo args= | grep -F "$needle" | grep -v grep >/dev/null 2>&1
}

# 1. Real Dobot bringup: start only if absent.
if ! has_cmdline "/opt/ros/humble/bin/ros2 launch dobot_bringup_v4 dobot_bringup_ros2.launch.py"; then
  run_terminal "Nova2 REAL READ-ONLY" "source /opt/charging-robot-local/nova2_tools/nova2_env.sh; ros2 launch dobot_bringup_v4 dobot_bringup_ros2.launch.py"
fi

sleep 3

# 2. Official Nova2 URDF + real joint_states -> TF.
if ! has_cmdline "/opt/charging-robot-local/nova2_tools/nova2_real_model.launch.py"; then
  run_terminal "Nova2 URDF TF" "/opt/charging-robot-local/nova2_tools/start_nova2_real_model.sh"
fi

# 3. Official Nova2 MoveIt planning-only server.
if ! has_cmdline "ros2 launch nova2_moveit move_group.launch.py allow_trajectory_execution:=false"; then
  run_terminal "Nova2 MoveIt PLAN ONLY" "/opt/charging-robot-local/nova2_tools/start_nova2_moveit_plan_only.sh"
fi

# 4. RealSense + ICP.
if ! has_cmdline "/opt/charging-robot-local/charging_socket_vision/realtime_socket_pose.py"; then
  run_terminal "RealSense ICP" "python3 /opt/charging-robot-local/charging_socket_vision/realtime_socket_pose.py --fps 15 --no-display"
fi

# 5. Online GetPose + point cloud bridge.
if ! has_cmdline "/opt/charging-robot-local/charging_socket_vision/icp_cloud_bridge.py"; then
  run_terminal "ICP BRIDGE" "source /opt/charging-robot-local/nova2_tools/nova2_env.sh; python3 /opt/charging-robot-local/charging_socket_vision/icp_cloud_bridge.py"
fi

# 6. RViz must be launched from the desktop display session.
if ! has_cmdline "rviz2 -d /opt/charging-robot-local/nova2_tools/nova2_icp.rviz"; then
  run_terminal "Nova2 RViz" "source /opt/charging-robot-local/nova2_tools/nova2_env.sh; rviz2 -d /opt/charging-robot-local/nova2_tools/nova2_icp.rviz"
fi

echo "Nova2 charging workflow started. Use plan_charging_workflow.sh for plan-only planning."
