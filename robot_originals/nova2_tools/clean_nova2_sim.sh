#!/usr/bin/env bash
set -u

# Safe cleanup for the Nova2 simulation / RViz / ICP visualization stack.
# Does NOT touch the real Dobot bringup, system services, workspaces, or configs.

SELF_PID=$$
DRY_RUN=0
if [[ "${1:-}" == "--dry-run" ]]; then
  DRY_RUN=1
fi

patterns=(
  'ros2 launch charging_dock_sim'
  'ros2 launch nova2_moveit'
  '/opt/ros/humble/bin/ros2 launch charging_dock_sim'
  '/opt/ros/humble/bin/ros2 launch nova2_moveit'
  '/opt/ros/humble/lib/moveit_ros_move_group/move_group'
  '/opt/ros/humble/lib/controller_manager/ros2_control_node'
  '/opt/ros/humble/lib/robot_state_publisher/robot_state_publisher'
  '/home/nvidia/charging_socket_vision/icp_cloud_bridge.py'
  '/home/nvidia/charging_socket_vision/realtime_socket_pose.py'
  '/home/nvidia/nova2_ros2_ws/install/charging_dock_sim/lib/charging_dock_sim/scene_publisher'
  '/home/nvidia/nova2_ros2_ws/install/charging_dock_sim/lib/charging_dock_sim/auto_dock'
  '/home/nvidia/nova2_ros2_ws/install/charging_dock_sim/lib/charging_dock_sim/plan_docking'
)

find_pids() {
  local pattern pid cmd
  for pattern in "${patterns[@]}"; do
    while read -r pid cmd; do
      [[ -z "$pid" || "$pid" == "$SELF_PID" ]] && continue
      case "$cmd" in
        *"$pattern"*) echo "$pid";;
      esac
    done < <(ps -eo pid=,args=)
  done | sort -n -u
}

mapfile -t PIDS < <(find_pids)
if ((${#PIDS[@]} == 0)); then
  echo "Nova2 simulation cleanup: nothing to clean."
  exit 0
fi

echo "Nova2 simulation processes found: ${PIDS[*]}"
ps -o pid,ppid,stat,etime,args -p "${PIDS[@]}" 2>/dev/null || true

if ((DRY_RUN)); then
  echo "Dry run only; no process was stopped."
  exit 0
fi

echo "Stopping simulation/visualization processes..."
kill -TERM "${PIDS[@]}" 2>/dev/null || true
sleep 3

mapfile -t LEFT < <(find_pids)
if ((${#LEFT[@]} > 0)); then
  echo "Force-stopping remaining matching processes: ${LEFT[*]}"
  kill -KILL "${LEFT[@]}" 2>/dev/null || true
fi

# Clear only the ROS 2 discovery daemon cache/state. This does not change packages.
if command -v ros2 >/dev/null 2>&1; then
  ros2 daemon stop >/dev/null 2>&1 || true
  ros2 daemon start >/dev/null 2>&1 || true
fi

echo "Nova2 simulation cleanup complete."
echo "Real Dobot bringup was not targeted."
