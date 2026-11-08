"""Read-only arm process identity and conflict checks; never signals processes."""
from pathlib import Path
DOMAIN='67'
LOCALHOST='1'
def require(condition,message):
    if not condition:raise ValueError(message)

def arm_process_kind(args):
    if not args:return None
    exe=Path(args[0]).name
    if exe in ('bash','sh','zsh','grep','awk','ssh'):return None
    if any('/nova2_ros2_ws/install/' in token for token in args):return 'arm-package'
    for package,launch,kind in [('dobot_bringup_v4','dobot_bringup_ros2.launch.py','bringup'),('nova2_moveit','move_group.launch.py','moveit'),('dobot_moveit','dobot_joint.launch.py','adapter')]:
        if 'launch' in args and package in args and launch in args:return kind
    scripts={'nova2_real_model.launch.py':'model','realtime_socket_pose.py':'ICP','icp_cloud_bridge.py':'ICP-bridge','pnp_v3_pose_bridge.py':'V3-bridge'}
    for token in args[1:]:
        if Path(token).name in scripts and ('/nova2_tools/' in token or '/charging_socket_vision/' in token):return scripts[Path(token).name]
        if Path(token).name=='run_live.py' and '/pnp_live_rgb_v3/' in token:return 'V3'
    if exe=='rviz2' and any(token.endswith('/nova2_tools/nova2_icp.rviz') for token in args):return 'arm-RViz'
    return None


def scan_arm_process_domains(proc_root='/proc'):
    """Read only two ROS variables for arm startup roots and descendants."""
    table={}
    for path in Path(proc_root).iterdir():
        if not path.name.isdigit():continue
        try:
            args=[p.decode(errors='replace') for p in (path/'cmdline').read_bytes().split(b'\0') if p]
            stat=(path/'stat').read_text().rsplit(')',1)[1].split()
            if stat[0]!='Z':table[int(path.name)]=(path,int(stat[1]),args,stat[19])
        except (FileNotFoundError,ProcessLookupError):continue
    selected={pid:arm_process_kind(row[2]) for pid,row in table.items() if arm_process_kind(row[2])}
    while True:
        children={pid:'arm-child' for pid,row in table.items() if row[1] in selected and pid not in selected}
        if not children:break
        selected.update(children)
    evidence=[]
    for pid,kind in selected.items():
        path,_,args,start=table[pid]
        try:env=dict(p.decode(errors='replace').split('=',1) for p in (path/'environ').read_bytes().split(b'\0') if b'=' in p)
        except OSError as error:raise RuntimeError('无法核对机械臂进程ROS域 pid='+str(pid)) from error
        row=dict(pid=pid,component=kind,executable=args[0],start_ticks=start,ROS_DOMAIN_ID=env.get('ROS_DOMAIN_ID','0'),ROS_LOCALHOST_ONLY=env.get('ROS_LOCALHOST_ONLY','0'))
        evidence.append(row)
        require(row['ROS_DOMAIN_ID']==DOMAIN and row['ROS_LOCALHOST_ONLY']==LOCALHOST,'机械臂已有进程域不符，拒绝复用且不停止服务: pid={} domain={} localhost={}'.format(pid,row['ROS_DOMAIN_ID'],row['ROS_LOCALHOST_ONLY']))
    return evidence


MOTION_SCRIPTS={'execute_pending_trajectory.py','insert_execute.py','insert_back.py',
                'return_to_initial_pose.py','demo_sequence.py','legacy_demo_sequence.py'}
WORKFLOW_ACTIONS={'test','home','run','execute','insert-execute','insert-back','auto-demo'}

def active_arm_tools(proc_root='/proc', *, exclude_pid=None):
    found=[]
    for p in Path(proc_root).iterdir():
        if not p.name.isdigit() or int(p.name)==exclude_pid:continue
        try:
            state=(p/'stat').read_text().rsplit(')',1)[1].split()
            if state[0]=='Z':continue
            args=[v.decode(errors='replace') for v in (p/'cmdline').read_bytes().split(b'\0') if v]
            # Match the actual executable/script position, never arbitrary shell text.
            if not args:continue
            exe=Path(args[0]).name
            index=1 if exe.startswith('python') or exe in ('bash','sh','zsh') else 0
            if len(args)<=index:continue
            script=Path(args[index]);rest=args[index+1:]
            is_runner=script.name=='arm_tool_terminal.py'
            is_motion='/nova2_tools/' in str(script) and script.name in MOTION_SCRIPTS
            is_workflow=(str(script).endswith('/nova2_tools/charge_robot.sh') or str(script).endswith('/bin/charge_robot')) and bool(rest) and rest[0] in WORKFLOW_ACTIONS
            if is_runner or is_motion or is_workflow:
                found.append(dict(pid=int(p.name),start_ticks=state[19],script=str(script)))
        except (FileNotFoundError,ProcessLookupError):continue
    return found
