"""Select only navigation/mapping processes; never match arbitrary command text."""
import os
from pathlib import Path
import signal

NAV_EXECUTABLES = {
    'nav2_amcl': {'amcl'}, 'nav2_map_server': {'map_server'},
    'nav2_controller': {'controller_server'}, 'nav2_planner': {'planner_server'},
    'nav2_bt_navigator': {'bt_navigator'}, 'nav2_behaviors': {'behavior_server'},
    'nav2_recoveries': {'recoveries_server'}, 'nav2_waypoint_follower': {'waypoint_follower'},
    'nav2_velocity_smoother': {'velocity_smoother'}, 'nav2_lifecycle_manager': {'lifecycle_manager'},
    'nav2_smoother': {'smoother_server'}, 'slam_gmapping': {'slam_gmapping'},
    'slam_toolbox': {'async_slam_toolbox_node','sync_slam_toolbox_node','localization_slam_toolbox_node'},
}


def service_kind(argv, root):
    if not argv:return None
    executable=Path(argv[0])
    args=argv[1:]
    if executable.name.startswith('python') and args:
        executable=Path(args[0]);args=args[1:]
    if executable.name=='ros2' and len(args)>=3 and args[0]=='launch':
        if args[1]==str(Path(root)/'navigation.launch.py'):
            return next((mode for mode in ('navigation','mapping') if 'mode:='+mode in args[2:]),None)
        if args[1]=='nav2_bringup' and args[2] in ('bringup_launch.py','localization_launch.py','navigation_launch.py'):
            return 'navigation'
        return None
    if executable.name in NAV_EXECUTABLES.get(executable.parent.name,set()) and executable.parent.parent.name=='lib':
        return 'mapping' if executable.parent.name.startswith('slam_') else 'navigation'
    if executable.name in ('component_container','component_container_isolated') and '__node:=nav2_container' in args:
        return 'navigation'
    return None


def read_process(directory):
    try:
        if directory.stat().st_uid!=os.getuid():return None
        fields=(directory/'stat').read_text().rsplit(')',1)[1].split()
        if fields[0]=='Z':return None
        argv=[a.decode(errors='replace') for a in (directory/'cmdline').read_bytes().split(b'\0') if a]
        return dict(pid=int(directory.name),ppid=int(fields[1]),pgid=int(fields[2]),start=fields[19],argv=argv)
    except (FileNotFoundError,ProcessLookupError,PermissionError):
        return None


def find_services(root, device_group=None):
    selected=[]
    for directory in Path('/proc').glob('[0-9]*'):
        process=read_process(directory)
        if not process:continue
        kind=service_kind(process['argv'],root)
        if device_group and process['pgid']==device_group:kind='owned_devices'
        if kind:
            process['kind']=kind;selected.append(process)
    return sorted(selected,key=lambda p:p['pid'])


def remaining(processes):
    result=[]
    for process in processes:
        current=read_process(Path('/proc')/str(process['pid']))
        if current and current['start']==process['start']:result.append(process)
    return result


def send_signal(processes, sig=signal.SIGINT):
    result=[]
    for process in remaining(processes):
        try:
            os.kill(process['pid'],sig)
            result.append(dict(pid=process['pid'],signal=int(sig),result='submitted'))
        except ProcessLookupError:pass
        except PermissionError as error:
            result.append(dict(pid=process['pid'],signal=int(sig),error=str(error)))
    return result


def shutdown_targets(processes):
    """Let ros2 launch terminate its children; signal orphans directly."""
    pids={p['pid'] for p in processes}
    return [p for p in processes if p['ppid'] not in pids]
