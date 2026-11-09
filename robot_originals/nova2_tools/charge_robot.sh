#!/usr/bin/env bash
# Human-friendly Nova2 charging workflow controller.
# start  : begin a fresh cycle; start/reuse connection + visualization + planning
# status : show component status
# enable : enable the real Nova2 robot after explicit confirmation
# disable: disable the real Nova2 robot after explicit confirmation
# replan : use the latest ICP result and publish a new plan-only trajectory
# stop   : stop visualization/planning stack; optionally stop real bringup with --real
set -eo pipefail

BASE=/home/nvidia
TOOLS=$BASE/nova2_tools
VISION=$BASE/charging_socket_vision
PNP_V3=$VISION/pnp_live_test_20261023_M07DN5/pnp_live_rgb_v3
PNP_V3_BRIDGE=$VISION/pnp_v3_pose_bridge.py
PIDDIR=/tmp/nova2_charge_workflow
mkdir -p "$PIDDIR"

source_env() { source "$TOOLS/nova2_env.sh"; }

response_res() {
  python3 - "$1" <<'PYRES'
import re, sys
values = re.findall(r'\bres\s*[:=]\s*(-?\d+)\b', sys.argv[1])
if len(values) != 1:
    sys.exit(1)
print(int(values[0]))
PYRES
}

robot_service() {
  local service="$1"
  local type="$2"
  local response

  if ! has_exact '/opt/ros/humble/bin/ros2 launch dobot_bringup_v4 dobot_bringup_ros2.launch.py'; then
    echo "错误：真实 Nova2 bringup 未运行，请先执行 charge_robot start 或单独启动 bringup。" >&2
    return 2
  fi

  response=$(timeout 12 ros2 service call "$service" "$type" '{}' 2>&1) || {
    echo "服务调用失败：$service" >&2
    echo "$response" >&2
    return 3
  }
  echo "$response"
  local code
  if ! code=$(response_res "$response"); then
    echo "错误：无法确认服务返回码，不继续后续检查。" >&2
    return 4
  fi
  if [[ "$code" != 0 ]]; then
    if [[ "$code" == -4 ]]; then
      echo "错误：控制器返回 res=-4，机器人处于下电状态；请现场确认上电后再手动使能。未自动上电或重试。" >&2
    else
      echo "错误：控制器返回 res=$code。" >&2
    fi
    return 4
  fi
  return 0
}

confirm_robot_action() {
  local action="$1"
  local prompt="${2:-$1}"
  echo
  echo "[安全确认] $prompt"
  echo "确认急停可触及、机械臂工作区无人且无碰撞风险。"
  [[ -t 0 ]] || { echo "需要交互终端确认。"; return 2; }
  read -r -p "按 Enter 继续（也兼容输入 ${action}），其他内容取消：" answer || return 2
  [[ -z "$answer" || "$answer" == "$action" ]]
}

enable_robot() {
  source_env
  if ! confirm_robot_action ENABLE; then
    echo '已取消，未发送使能命令。'
    return 2
  fi
  echo '正在调用 EnableRobot...'
  robot_service '/dobot_bringup_ros2/srv/EnableRobot' 'dobot_msgs_v4/srv/EnableRobot' || return $?
  local status
  status=$(timeout 8 ros2 topic echo /dobot_msgs_v4/msg/RobotStatus --once 2>/dev/null || true)
  if grep -q 'is_enable: true' <<<"$status"; then
    echo '[成功] Nova2 已使能。'
  else
    echo '[警告] 服务已返回，但 RobotStatus 尚未显示 is_enable: true。'
    return 5
  fi
}

disable_robot() {
  source_env
  if ! confirm_robot_action DISABLE; then
    echo '已取消，未发送取消使能命令。'
    return 2
  fi
  echo '正在调用 DisableRobot...'
  robot_service '/dobot_bringup_ros2/srv/DisableRobot' 'dobot_msgs_v4/srv/DisableRobot' || return $?
  local status
  status=$(timeout 8 ros2 topic echo /dobot_msgs_v4/msg/RobotStatus --once 2>/dev/null || true)
  if grep -q 'is_enable: false' <<<"$status"; then
    echo '[成功] Nova2 已取消使能。'
  else
    echo '[警告] 服务已返回，但 RobotStatus 尚未显示 is_enable: false。'
    return 5
  fi
}

has_exact() {
  python3 "$TOOLS/charge_workflow_state.py" has "$1"
}

pid_for() {
  python3 "$TOOLS/charge_workflow_state.py" pids "$1" | head -n 1
}

stop_pattern() {
  python3 "$TOOLS/charge_workflow_state.py" stop "$1"
}

stop_planners() {
  stop_pattern "$TOOLS/plan_camera_pre_align_nearest_ik.py" || return 2
  stop_pattern "$TOOLS/plan_charging_gun_prepose.py" || return 2
}

retract_insert_if_needed() {
  local guard_rc=0
  if python3 - "$VISION/results/realtime_runs/latest_insertion_execution_feedback.json" <<'PYGUARD'
import json,sys,pathlib
p=pathlib.Path(sys.argv[1])
if p.exists():
 d=json.loads(p.read_text())
 if d.get('executor') in ('contact_steps','cartesian_once') and d.get('status')!='succeeded' and not d.get('retracted',False):
  sys.exit(3)
PYGUARD
  then guard_rc=0; else guard_rc=$?; fi
  if [[ "$guard_rc" == 3 ]]; then
    echo '[回位选择] 插入未完成；请先insert-back，或确认已完全拔枪后手动home。'
    echo '只有确认枪头已完全退出充电口、直接回位不会牵拉或碰撞时，才能跳过自动回退。'
    [[ -t 0 ]] || { echo '[取消] 手动home需要交互终端。'; return 2; }
    local manual_answer
    read -r -p '输入 HOME 直接执行正常回位（不会先拔枪）；其他输入取消：' manual_answer || return 2
    [[ "$manual_answer" == HOME ]] || { echo '[取消] 未发送回位命令。'; return 2; }
    MANUAL_HOME_OVERRIDE=1
    echo '[手动home] 已确认跳过自动回退，继续正常回位。'
    return 0
  elif [[ "$guard_rc" != 0 ]]; then
    echo '[拒绝] 无法读取插入状态，未执行回位。'; return 2
  fi
  local report="$VISION/results/realtime_runs/latest_insertion_execution_feedback.json"
  if [[ ! -s "$report" ]]; then return 0; fi
  if ! python3 - "$report" <<'PY'
import json,sys
try:
 d=json.load(open(sys.argv[1]))
 print('yes' if d.get('status')=='succeeded' and not d.get('retracted',False) else 'no')
except Exception:
 print('no')
PY
  then return 2; fi
  if [[ "$(python3 - "$report" <<'PY'
import json,sys
try:
 d=json.load(open(sys.argv[1])); print('yes' if d.get('status')=='succeeded' and not d.get('retracted',False) else 'no')
except Exception: print('no')
PY
)" == yes ]]; then
    echo '[回位前置] 检测到上一次 insert-go 已成功，先反向退回插入轨迹。'
    python3 "$TOOLS/retract_inserted.py" || { echo '[回位中止] 插入轨迹退回失败，未执行正常home。'; return 2; }
  fi
}

ensure_insert_action() {
  source_env
  if timeout 5 ros2 action list 2>/dev/null | grep -q '^/nova2_group_controller/follow_joint_trajectory$'; then
    return 0
  fi
  echo '[回位前置] 插入回退所需的轨迹服务未运行，启动执行适配器。'
  CHARGE_ROBOT_HEADLESS=1 start_component executor '3b 轨迹执行适配器'     'ros2 launch dobot_moveit dobot_joint.launch.py'     'source /home/nvidia/nova2_tools/nova2_env.sh; ros2 launch dobot_moveit dobot_joint.launch.py' || return 2
  local i
  for i in $(seq 1 20); do
    if timeout 3 ros2 action list 2>/dev/null | grep -q '^/nova2_group_controller/follow_joint_trajectory$'; then return 0; fi
    sleep 1
  done
  echo '[回位中止] 轨迹执行服务启动后仍不可用。' >&2
  return 2
}

home_robot() {
  MANUAL_HOME_OVERRIDE=0
  source_env
  confirm_robot_action HOME "机械臂将回初始姿态" || { echo "[已取消] 未发送回位命令。"; return 2; }
  if [[ -s "$VISION/results/realtime_runs/latest_insertion_execution_feedback.json" ]]; then
    ensure_insert_action || return 2
  fi
  retract_insert_if_needed || return 2
  python3 "$TOOLS/charge_workflow_state.py" assert-idle || return 2
  stop_camera_stage || return 2
  stop_v3_stage || return 2
  stop_planners || return 2
  python3 "$TOOLS/charge_workflow_state.py" transition homing || return 2
  if python3 "$TOOLS/return_to_initial_pose.py"; then
    if [[ "${MANUAL_HOME_OVERRIDE:-0}" == 1 ]]; then
      python3 - "$VISION/results/realtime_runs/latest_insertion_execution_feedback.json" <<'PYMANUAL'
import json,sys,time,pathlib
p=pathlib.Path(sys.argv[1]);d=json.loads(p.read_text())
d.update(retracted=True,retraction_method='operator_confirmed_clear_then_verified_home',manual_home_at=time.time())
t=p.with_suffix('.tmp');t.write_text(json.dumps(d,ensure_ascii=False,indent=2));t.replace(p)
PYMANUAL
    fi
    python3 "$TOOLS/charge_workflow_state.py" transition home_ready || return 2
    echo '[回位完成] 已确认实际到位；执行 charge_robot start 开始新一轮。'
  else
    echo '[回位未完成] 请检查机械臂状态，未标记回位成功。'
    return 2
  fi
}

set_home_robot() {
  source_env
  if ! has_exact '/opt/ros/humble/bin/ros2 launch dobot_bringup_v4 dobot_bringup_ros2.launch.py'; then
    echo '错误：真实 Nova2 bringup 未运行，请先执行 charge_robot start。'
    return 2
  fi
  if ! confirm_robot_action SET_HOME "将当前机械臂关节姿态保存为新的初始姿态（不发送运动命令）"; then
    echo '已取消，未修改初始姿态。'
    return 2
  fi
  local tmp="/tmp/charge_robot_getangle.$$"
  trap 'rm -f "$tmp"' RETURN
  if ! timeout 10 ros2 service call /dobot_bringup_ros2/srv/GetAngle dobot_msgs_v4/srv/GetAngle '{}' >"$tmp" 2>&1; then
    echo '读取当前关节姿态失败，未修改初始姿态。' >&2
    cat "$tmp" >&2
    return 2
  fi
  python3 - "$tmp" /home/nvidia/nova2_tools/initial_arm_pose.yaml <<'PY2'
import math,re,shutil,sys
from datetime import datetime
from pathlib import Path
src=Path(sys.argv[1]); dst=Path(sys.argv[2]); text=src.read_text(errors='replace')
match=re.search(r"robot_return\s*=\s*'\{([^}]*)\}'",text)
if not match:
    raise SystemExit('GetAngle响应中没有robot_return')
try:
    deg=[float(x) for x in match.group(1).split(',')]
except ValueError as exc:
    raise SystemExit(f'GetAngle数值解析失败: {exc}')
if len(deg)!=6 or not all(math.isfinite(x) for x in deg):
    raise SystemExit(f'GetAngle返回不是6个有限关节角: {deg}')
backup=dst.with_name(dst.name+'.bak_'+datetime.now().strftime('%Y%m%d_%H%M%S'))
shutil.copy2(dst,backup)
rad=[math.radians(x) for x in deg]
out='''# Nova2 charging robot fixed startup pose
# Captured from live GetAngle by charge_robot set-home
# Units: radians
initial_arm_pose:
'''+''.join(f'  joint{i}: {v:.15f}\n' for i,v in enumerate(rad,1))
dst.write_text(out)
print('当前姿态已保存为新的初始姿态。')
print('关节角(度): '+', '.join(f'{x:.4f}' for x in deg))
print('备份: '+str(backup))
print('文件: '+str(dst))
PY2
}

finish_gun_stage() {
  stop_v3_stage || return 2
  stop_planners || return 2
  python3 "$TOOLS/charge_workflow_state.py" transition completed || return 2
  echo '[本轮完成] 已关闭V3、V3桥接及规划节点，保留真实连接、模型、MoveIt和RViz。'
  echo '下一轮：charge_robot home → charge_robot run；也可继续分步操作。'
}

open_terminal() {
  local title="$1"; shift
  local cmd="$*"
  if [[ "${CHARGE_ROBOT_HEADLESS:-0}" == 1 ]]; then
    local logdir="$VISION/results/realtime_runs/workflow_logs"
    mkdir -p "$logdir"
    local safe
    safe=$(printf '%s' "$title" | tr -cs '[:alnum:]_.-' '_')
    nohup bash -lc "exec 9>&-; $cmd" >"$logdir/${safe}.log" 2>&1 </dev/null &
  else
    gnome-terminal 9>&- --title="$title" -- bash -lc "$cmd; echo; echo '[窗口保持打开，Ctrl+C 可停止本模块]'; exec bash" >/dev/null 2>&1 &
  fi
}

start_component() {
  local key="$1" title="$2" needle="$3" cmd="$4"
  if has_exact "$needle"; then
    echo "[已运行] $title"
  else
    echo "[启动] $title"
    open_terminal "$title" "$cmd"
  fi
}

start_bringup() {
  start_component real '1 真实Nova2连接' \
    '/opt/ros/humble/bin/ros2 launch dobot_bringup_v4 dobot_bringup_ros2.launch.py' \
    '/home/nvidia/nova2_tools/start_bringup.sh'
}

start_all() {
  echo '=== 启动 Nova2 充电视觉工作流 ==='
  source_env
  python3 "$TOOLS/charge_workflow_state.py" assert-idle || return 2
  # A new start is an explicit new cycle. Release either camera owner first.
  stop_camera_stage || return 2
  stop_v3_stage || return 2
  stop_planners || return 2
  python3 "$TOOLS/charge_workflow_state.py" transition started || return 2

  start_bringup || return 2

  echo "[提示] start仅启动环境；回初始姿态请单独执行 charge_robot home。"

  # Official Nova2 URDF + real joint states -> TF.
  start_component model '2 Nova2官方模型与TF' \
    '/home/nvidia/nova2_tools/nova2_real_model.launch.py' \
    '/home/nvidia/nova2_tools/start_nova2_real_model.sh' || return 2

  # Official Nova2 MoveIt planning-only server.
  start_component moveit '3 Nova2 MoveIt纯规划' \
    'ros2 launch nova2_moveit move_group.launch.py allow_trajectory_execution:=false' \
    'source /home/nvidia/nova2_tools/nova2_env.sh; ros2 launch nova2_moveit move_group.launch.py allow_trajectory_execution:=false' || return 2

  start_component executor '3b 轨迹执行适配器' \
    'ros2 launch dobot_moveit dobot_joint.launch.py' \
    'source /home/nvidia/nova2_tools/nova2_env.sh; ros2 launch dobot_moveit dobot_joint.launch.py' || return 2

  # ICP camera process.
  start_component icp '4 实时ICP相机' \
    '/home/nvidia/charging_socket_vision/realtime_socket_pose.py' \
    'python3 /home/nvidia/charging_socket_vision/realtime_socket_pose.py --no-display' || return 2

  # Online GetPose + point cloud bridge.
  start_component bridge '5 在线位姿与点云桥接' \
    '/home/nvidia/charging_socket_vision/icp_cloud_bridge.py' \
    'source /home/nvidia/nova2_tools/nova2_env.sh; python3 /home/nvidia/charging_socket_vision/icp_cloud_bridge.py' || return 2

  # RViz must run in the desktop display session.
  start_component rviz '6 RViz可视化' 'rviz2 -d /home/nvidia/nova2_tools/nova2_icp.rviz' \
    'source /home/nvidia/nova2_tools/nova2_env.sh; rviz2 -d /home/nvidia/nova2_tools/nova2_icp.rviz' || return 2

  echo
  echo '启动请求已提交；各组件正在并行初始化。执行 cam-plan 时检查所需条件：'
  echo '  charge_robot cam-plan'
}

clear_pending() {
  stop_planners || return 2
  rm -f "$VISION/results/realtime_runs/pending_trajectory.json" \
    "$VISION/results/realtime_runs/pending_camera_trajectory.json" \
    "$VISION/results/realtime_runs/pending_gun_trajectory.json"
}

insert_plan() {
  clear_pending || return 2
  rm -f "$VISION/results/realtime_runs/pending_insertion_trajectory.json"
  echo '=== 75mm轴向笛卡尔直线规划（不执行） ==='
  source_env
  python3 "$TOOLS/plan_insertion_cartesian.py" || return 2
  echo '[完成] 已保存直线插入轨迹；下一步执行 charge_robot insert-go。'
}

insert_go() {
  echo '=== 一次性轴向插入执行（带反馈取消） ==='
  source_env
  if ! has_exact '/opt/ros/humble/bin/ros2 launch dobot_bringup_v4 dobot_bringup_ros2.launch.py'; then
    echo '[拒绝] 真实Nova2 bringup未运行。'; return 2
  fi
  if [[ ! -s "$VISION/results/realtime_runs/pending_insertion_trajectory.json" ]]; then
    echo '[拒绝] 没有插入轨迹，请先执行 charge_robot insert-plan。'; return 2
  fi
  python3 "$TOOLS/insert_execute.py"
}

gun_prepose_plan() {
  clear_pending || return 2
  python3 "$TOOLS/workflow_progress.py" begin gun || return 2
  echo '=== 充电枪预备位姿纯规划（不执行） ==='
  source_env
  if ! python3 "$TOOLS/charge_startup_checks.py" stage gun --timeout 30; then
    python3 "$TOOLS/workflow_progress.py" fail '枪头阶段所需组件未就绪，请检查V3、桥接和MoveIt'
    return 2
  fi
  echo '等待V3多帧稳定后冻结代表测量；相机继续测量。'
  if ! python3 /home/nvidia/nova2_tools/freeze_pnp_v3_target.py; then
    python3 "$TOOLS/workflow_progress.py" fail 'V3稳定取值失败；保持静止，确认V3有效后重新 gun-plan'
    echo '[未生成轨迹] 请重新 gun-plan，本次不能 execute。'
    return 2
  fi
  # Planner startup grace must begin after component startup and sampling.
  python3 "$TOOLS/workflow_progress.py" begin gun || return 2
  local old
  old=$(pid_for '/home/nvidia/nova2_tools/plan_charging_gun_prepose.py' || true)
  if [[ -n "$old" ]]; then kill -TERM "$old" 2>/dev/null || true; sleep 2; fi
  echo '使用冻结的最新RGB PnP V3目标，按50mm预备距离规划枪头...'
  open_terminal '充电枪预备位姿纯规划' 'source /home/nvidia/nova2_tools/nova2_env.sh; python3 /home/nvidia/nova2_tools/plan_charging_gun_prepose.py'
  echo '[等待] 正在规划，成功保存轨迹后本命令才会返回。'
  python3 "$TOOLS/workflow_progress.py" wait gun || return 2
}

stop_v3_stage() {
  local patterns=("$PNP_V3/run_live.py" "$PNP_V3_BRIDGE")
  for pat in "${patterns[@]}"; do
    stop_pattern "$pat" || return 2
  done
  sleep 2
}

start_v3_stage() {
  echo '=== 切换到RGB PnP V3纯图像位姿阶段 ==='
  local stale="$PNP_V3/live_results/latest_pose_readonly.json"
  python3 - "$stale" <<'PY'
import json, pathlib, sys, time
p=pathlib.Path(sys.argv[1]); p.parent.mkdir(parents=True,exist_ok=True)
p.write_text(json.dumps({'pose_valid':False,'planning_ready':False,'T_camera_from_socket':None,
                         'reason':'awaiting_rgb_pnp_v3_restart','write_time_host':time.time()},indent=2))
PY
  start_component pnp_v3 '7 RGB PnP V3纯图像位姿（无显示）' "$PNP_V3/run_live.py --no-display" \
    "python3 -u $PNP_V3/run_live.py --no-display" || return 2
  start_component pnp_v3_bridge '8 RGB PnP V3位姿桥接' "$PNP_V3_BRIDGE" \
    "source $TOOLS/nova2_env.sh; python3 $PNP_V3_BRIDGE" || return 2
  echo '等待V3产生新鲜有效测量后，执行 charge_robot gun-plan。'
}

status() {
  echo '=== Nova2充电工作流状态 ==='
  source_env
  printf '%-16s %s\n' '真实连接' "$(has_exact '/opt/ros/humble/bin/ros2 launch dobot_bringup_v4 dobot_bringup_ros2.launch.py' && echo RUNNING || echo STOPPED)"
  printf '%-16s %s\n' 'Nova2模型TF' "$(has_exact '/home/nvidia/nova2_tools/nova2_real_model.launch.py' && echo RUNNING || echo STOPPED)"
  printf '%-16s %s\n' 'MoveIt纯规划' "$(has_exact 'ros2 launch nova2_moveit move_group.launch.py allow_trajectory_execution:=false' && echo RUNNING || echo STOPPED)"
  printf '%-16s %s\n' '实时ICP' "$(has_exact '/home/nvidia/charging_socket_vision/realtime_socket_pose.py' && echo RUNNING || echo STOPPED)"
  printf '%-16s %s\n' 'RGB PnP V3' "$(has_exact "$PNP_V3/run_live.py --no-display" && echo RUNNING || echo STOPPED)"
  printf '%-16s %s\n' '点云桥接' "$(has_exact '/home/nvidia/charging_socket_vision/icp_cloud_bridge.py' && echo RUNNING || echo STOPPED)"
  printf '%-16s %s\n' 'V3位姿桥接' "$(has_exact "$PNP_V3_BRIDGE" && echo RUNNING || echo STOPPED)"
  printf '%-16s %s\n' 'RViz' "$(has_exact 'rviz2 -d /home/nvidia/nova2_tools/nova2_icp.rviz' && echo RUNNING || echo STOPPED)"
  printf '%-16s %s\n' '一键规划节点' "$(has_exact '/home/nvidia/nova2_tools/plan_camera_pre_align_nearest_ik.py' && echo RUNNING || echo STOPPED)"

  local robot_status
  robot_status=$(timeout 8 ros2 topic echo /dobot_msgs_v4/msg/RobotStatus --once 2>/dev/null || true)
  if grep -q 'is_enable: true' <<<"$robot_status"; then
    echo '机械臂使能      ENABLED'
  elif grep -q 'is_enable: false' <<<"$robot_status"; then
    echo '机械臂使能      DISABLED'
  else
    echo '机械臂使能      UNKNOWN（未连接或无状态消息）'
  fi

  echo
  echo '最近视觉结果：'
  python3 - "$PNP_V3/live_results/latest_pose_readonly.json" <<'PY'
import json,sys,time
try:
 d=json.load(open(sys.argv[1])); stamp=d.get('measurement_time_host')
 age=None if stamp is None else time.time()-float(stamp)
 print(f"  source=RGB-PnP-V3 valid={d.get('pose_valid')} age_s={None if age is None else round(age,3)} reason={d.get('reason')}")
except Exception as e: print('  RGB PnP V3暂无结果:',e)
PY
  if command -v ros2 >/dev/null 2>&1; then
    echo
    echo '目标话题：'
    timeout 4 ros2 topic echo /charging_port_target --once 2>/dev/null | sed -n '1,12p' || echo '  暂无目标消息'
  fi
}

  replan() {
  clear_pending || return 2
  python3 "$TOOLS/workflow_progress.py" begin camera || return 2
  echo '=== cam-plan前置就绪等待（不执行运动） ==='
  source_env
  if ! python3 "$TOOLS/charge_startup_checks.py" stage camera --timeout "${CHARGE_ROBOT_CAMERA_READY_TIMEOUT_S:-60}"; then
    python3 "$TOOLS/workflow_progress.py" fail 'cam-plan前置条件超时：请查看上方尚未就绪的数据或接口；未启动规划'
    return 2
  fi
  echo '[开始cam-plan] 前置条件通过，继续ICP多帧稳定取值及纯规划。'
  # Do not count parallel startup time as planner startup time.
  python3 "$TOOLS/workflow_progress.py" begin camera || return 2
  local old
  old=$(pid_for '/home/nvidia/nova2_tools/plan_camera_pre_align_nearest_ik.py' || true)
  if [[ -n "$old" ]]; then kill -TERM "$old" 2>/dev/null || true; sleep 2; fi
  local request
  request=$(python3 /home/nvidia/nova2_tools/plan_flow_guard.py begin) || return 2
  echo "本次 cam-plan 请求: $request；规划完成前禁止使用旧轨迹。"
  open_terminal '摄像头预备位姿纯规划' "source /home/nvidia/nova2_tools/nova2_env.sh; CHARGE_PLAN_REQUEST='$request' python3 /home/nvidia/nova2_tools/plan_camera_pre_align_nearest_ik.py"
  echo '[等待] 正在规划并检查执行资格...'
  python3 "$TOOLS/workflow_progress.py" wait camera || return 2
}

finish_cam_plan() {
  local request_file="$VISION/results/realtime_runs/latest_camera_plan_request.json"
  python3 - "$request_file" <<'PY'
import json,sys
d=json.load(open(sys.argv[1]))
if d.get('status')!='complete' or d.get('stage')!='camera' or not d.get('plan_id'):
    raise SystemExit('最近cam-plan请求未成功完成；不切换到RGB PnP V3')
print(d.get('plan_id',''))
PY
}

stop_camera_stage() {
  local patterns=(
    '/home/nvidia/nova2_tools/plan_camera_pre_align_nearest_ik.py'
    '/home/nvidia/charging_socket_vision/icp_cloud_bridge.py'
    '/home/nvidia/charging_socket_vision/realtime_socket_pose.py'
  )
  for pat in "${patterns[@]}"; do
    stop_pattern "$pat" || return 2
  done
  sleep 2
}

preflight_execute() {
  if [[ ! -s "$VISION/results/realtime_runs/pending_trajectory.json" ]]; then
    python3 "$TOOLS/workflow_progress.py" missing
    return 2
  fi
  if [[ "${CHARGE_ROBOT_FAST_TEST:-0}" == 1 ]]; then
    source_env
    echo '[快速test] 按Enter后统一检查机器人状态、轨迹和真实起点。'
  else
  python3 /home/nvidia/nova2_tools/plan_flow_guard.py status || return 2
  echo '=== 真实预对准执行预检（当前不会发送运动命令） ==='
  source_env
  local errors=0
  if ! has_exact '/opt/ros/humble/bin/ros2 launch dobot_bringup_v4 dobot_bringup_ros2.launch.py'; then echo '[失败] 真实bringup未运行'; errors=$((errors+1)); else echo '[通过] 真实bringup运行中'; fi
  local status
  status=$(timeout 8 ros2 topic echo /dobot_msgs_v4/msg/RobotStatus --once 2>/dev/null || true)
  if grep -q 'is_connected: true' <<<"$status"; then echo '[通过] 机械臂已连接'; else echo '[失败] 机械臂未连接'; errors=$((errors+1)); fi
  if grep -q 'is_enable: true' <<<"$status"; then echo '[通过] 机械臂已使能'; else echo '[提示] 机械臂当前未使能'; fi
  local mode error target
  mode=$(timeout 8 ros2 service call /dobot_bringup_ros2/srv/RobotMode dobot_msgs_v4/srv/RobotMode '{}' 2>/dev/null || true)
  error=$(timeout 8 ros2 service call /dobot_bringup_ros2/srv/GetErrorID dobot_msgs_v4/srv/GetErrorID '{}' 2>/dev/null || true)
  echo "RobotMode: $(grep -E 'robot_return|res:' <<<"$mode" | tr '\\n' ' ')"
  echo "GetErrorID: $(grep -E 'robot_return|res:' <<<"$error" | tr '\\n' ' ')"
  if grep -q "robot_return='{\[\]}'" <<<"$error"; then echo '[通过] 无报警'; else echo '[失败] 存在报警或无法读取报警'; errors=$((errors+1)); fi
  echo '[通过] 轨迹及其保存的目标已校验；执行器还会核对实际起点。'
  echo '[提示] 执行依据是已保存且校验通过的轨迹；规划预览节点无需保持运行。'
  echo
  echo '[执行链路] 使用已存在的 FollowJointTrajectory -> Dobot ServoJ 适配器。'
  echo '[执行前确认] 只有在交互终端按 Enter 确认后才会发送当前待执行轨迹。'
  echo "预检错误数: $errors"
  if [[ "$errors" -ne 0 ]]; then
    return 2
  fi
  echo
  echo '[安全确认] 预检通过，下面将执行最近一次 cam-plan 或 gun-plan 保存的轨迹。'
  echo '[安全确认] 执行器会显示阶段；按 Enter 执行，输入其他内容取消。'
  fi
  local pending_stage pending_plan_id pending_request_id
  read -r pending_stage pending_plan_id pending_request_id < <(python3 - "$VISION/results/realtime_runs/pending_trajectory.json" <<'PY'
import json,sys
data=json.load(open(sys.argv[1])); print(data.get('stage',''),data.get('plan_id',''),data.get('request_id',''))
PY
  ) || return 2
  if [[ "$pending_stage" == camera ]]; then
    local latest_plan_id
    latest_plan_id=$(finish_cam_plan) || { echo '[拒绝] 最近cam-plan未完成；不执行相机轨迹。'; return 2; }
    if [[ "$pending_plan_id" != "$latest_plan_id" ]]; then
      echo '[拒绝] 待执行轨迹不属于最近cam-plan；不发送运动命令。'
      return 2
    fi
  fi
  if ! python3 /home/nvidia/nova2_tools/execute_pending_trajectory.py; then
    echo '[停止] 轨迹未成功执行；不切换视觉阶段。'
    return 2
  fi
  if [[ "$pending_stage" == camera ]]; then
    local cam_plan_id
    cam_plan_id=$(finish_cam_plan) || { echo '[停止] cam-plan请求状态不匹配；不启动V3。'; return 2; }
    if [[ "$pending_request_id" != "$(python3 -c "import json; print(json.load(open('$VISION/results/realtime_runs/latest_camera_plan_request.json')).get('request_id',''))")" ]]; then
      echo '[停止] 执行轨迹请求ID与当前cam-plan不匹配；不切换V3。'; return 2
    fi
    echo "cam-plan轨迹已成功执行，plan_id=$cam_plan_id；关闭ICP/规划栈并切换纯图像测量。"
    stop_camera_stage || return 2
    start_v3_stage || return 2
  elif [[ "$pending_stage" == gun ]]; then
    finish_gun_stage || return 2
  fi
}

stop_stack() {
  clear_pending || return 2
  echo '=== 停止视觉/规划/RViz栈 ==='
  local patterns=(
    '/home/nvidia/nova2_tools/plan_camera_pre_align_nearest_ik.py'
    '/home/nvidia/charging_socket_vision/icp_cloud_bridge.py'
    '/home/nvidia/charging_socket_vision/realtime_socket_pose.py'
    "$PNP_V3/run_live.py"
    "$PNP_V3_BRIDGE"
    'rviz2 -d /home/nvidia/nova2_tools/nova2_icp.rviz'
    '/home/nvidia/nova2_tools/nova2_real_model.launch.py'
    'ros2 launch nova2_moveit move_group.launch.py allow_trajectory_execution:=false'
  )
  for pat in "${patterns[@]}"; do
    stop_pattern "$pat" || return 2
  done
  sleep 3
  echo '已停止视觉、规划、模型TF和RViz。真实bringup默认保留。'
  if [[ "${1:-}" == '--real' ]]; then
    echo '停止真实bringup...'
    while read -r p; do [[ -n "$p" ]] && kill -TERM "$p" 2>/dev/null || true; done < <(ps -eo pid=,args= | awk 'index($0,"/opt/ros/humble/bin/ros2 launch dobot_bringup_v4")>0 {print $1}')
    while read -r p; do [[ -n "$p" ]] && kill -TERM "$p" 2>/dev/null || true; done < <(ps -eo pid=,args= | awk 'index($0,"/home/nvidia/nova2_ros2_ws/install/dobot_bringup_v4/lib")>0 {print $1}')
    echo '真实bringup已请求停止。'
  fi
}


run_cycle() {
  echo '=== 一轮充电预对接与插入：自动规划与切换，每段运动单独按 Enter 确认 ==='
  start_all || return 2
  replan || return 2
  preflight_execute || return 2
  gun_prepose_plan || return 2
  preflight_execute || return 2
  insert_plan || return 2
  insert_go || return 2
}

# Serialize workflow commands; planners still run in their own terminals.
exec 9>"$PIDDIR/command.lock"
flock -n 9 || { echo '另一个charge_robot命令正在运行'; exit 2; }
# Do not let spawned terminal processes retain the workflow lock.
case "${1:-help}" in
  insert-back) source_env; ensure_insert_action && python3 "$TOOLS/insert_back.py" ;;
  insert-back-check) source_env; python3 "$TOOLS/insert_back.py" --check ;;
  run) run_cycle ;;
  test) CHARGE_ROBOT_HEADLESS=1 CHARGE_ROBOT_FAST_TEST=1 run_cycle ;;
  bringup) start_bringup ;;
  start) start_all ;;
  home) home_robot ;;
  set-home|set_home) set_home_robot ;;
  status) status ;;
  enable) enable_robot ;;
  disable|disable-robot) disable_robot ;;
  cam-plan|replan|plan) replan ;;
  gun-plan|gun-prepose) gun_prepose_plan ;;
  insert-plan) insert_plan ;;
  insert-go) insert_go ;;
  v3-start) start_v3_stage ;;
  execute) preflight_execute ;;
  stop) stop_stack ;;
  stop-all) stop_stack --real ;;
  *)
    echo '用法：'
    echo '  charge_robot insert-back  沿原插入轴线退回50mm预备位（Enter确认）'
    echo '  charge_robot run        启动并串联两阶段规划/执行，每段运动按 Enter 确认（不自动回位）'
    echo '  charge_robot test       单终端连续测试；后台模块日志写入 realtime_runs/workflow_logs（每段运动仍需 Enter）'
    echo '  charge_robot bringup    仅启动Nova2连接层（调用start_bringup.sh）'
    echo '  charge_robot start      启动环境（不回初始姿态）'
    echo '  charge_robot home       确认后回初始姿态'
    echo '  charge_robot set-home  将当前姿态保存为新的初始姿态（不运动）'
    echo '  charge_robot status     查看状态和ICP质量'
    echo '  charge_robot enable     确认后使能真实 Nova2 机械臂'
    echo '  charge_robot disable    确认后取消使能真实 Nova2 机械臂'
    echo '  charge_robot cam-plan   规划摄像头预备位姿（纯规划）'
    echo '  charge_robot gun-plan   规划充电枪预备位姿（50mm，+Z对准充电口，+Y朝上）'
    echo '  charge_robot insert-plan 生成轴向插入候选深度（只规划）'
    echo '  charge_robot insert-go   执行一次性75mm总轴向插入（带反馈取消）'
    echo '  charge_robot execute    执行最近一次 cam-plan 或 gun-plan 轨迹（需确认）'
    echo '  charge_robot stop       停止视觉/规划/RViz，保留真实连接'
    echo '  charge_robot stop-all   停止全部，包括真实连接'
    ;;
esac
