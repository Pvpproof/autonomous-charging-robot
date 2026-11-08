"""Close only an identified, independently launched navigation RViz.

No ROS imports, process-group signals, driver commands or automatic restarts.
"""
import os,signal,time
from pathlib import Path

# Explicit absolute display configs are safest for nonstandard ROS workspaces.
# Defaults cover the standard ROS install and the current user's workspace.
NAV_ROOTS=(Path.home()/'ychd_ros2_ws/install/nav2_bringup/share/nav2_bringup/rviz',
           Path.home()/'ychd_ros2_ws/src/navigation2-humble/nav2_bringup/rviz',
           Path('/opt/ros/humble/share/nav2_bringup/rviz'))
NAV_CONFIGS={str((p/n).resolve()) for p in NAV_ROOTS for n in ('nav2_default_view.rviz','nav2_namespaced_view.rviz')}
NAV_CONFIGS.update(str(Path(value).expanduser().resolve())
                   for value in os.environ.get('CHARGING_NAV_RVIZ_CONFIGS','').split(os.pathsep)
                   if value and Path(value).expanduser().is_absolute())


def read_process(pid,root=Path('/proc')):
    p=Path(root)/str(pid)
    try:
        state=(p/'stat').read_text().rsplit(')',1)[1].split()
        if state[0]=='Z':return None
        args=[a.decode(errors='replace') for a in (p/'cmdline').read_bytes().split(b'\0') if a]
        if not args:return None
        return dict(pid=int(pid),ppid=int(state[1]),start=state[19],args=args)
    except (OSError,ValueError,IndexError):return None

def navigation_rviz(row):
    a=row['args']
    if Path(a[0]).name!='rviz2':return False
    configs=[]
    for i,arg in enumerate(a):
        if arg in ('-d','--display-config') and i+1<len(a):configs.append(a[i+1])
        elif arg.startswith('--display-config='):configs.append(arg.split('=',1)[1])
    return len(configs)==1 and Path(configs[0]).is_absolute() and str(Path(configs[0]).resolve()) in NAV_CONFIGS

def independently_launched(row,table):
    visited=set();pid=row['ppid']
    harmless={'bash','sh','zsh','fish','gnome-terminal-server','gnome-terminal','sshd','systemd','init','login','tmux: server','tmux','screen'}
    while pid>1:
        if pid in visited:return False
        visited.add(pid);parent=table.get(pid)
        if parent is None:return False
        args=parent['args'];exe=Path(args[0]).name
        ros_args=args[1:] if exe.startswith('python') else args
        if ros_args and Path(ros_args[0]).name=='ros2' and len(ros_args)>=4 and ros_args[1]=='launch':
            # A dedicated visualization-only launch is safe. A bringup launch
            # may shut its entire group down when RViz exits, so never touch it.
            return ros_args[2:4]==['nav2_bringup','rviz_launch.py']
        if exe not in harmless:return False
        pid=parent['ppid']
    return True

def close_navigation_rviz(*,root=Path('/proc'),read=read_process,send=os.kill,clock=time.monotonic,sleep=time.sleep):
    table={}
    for p in Path(root).iterdir():
        if p.name.isdigit():
            row=read(int(p.name),root)
            if row is not None:table[row['pid']]=row
    report=dict(closed=[],skipped=[],errors=[])
    targets=[]
    for row in table.values():
        if not navigation_rviz(row):continue
        if not independently_launched(row,table):
            report['skipped'].append(dict(pid=row['pid'],reason='无法确认独立显示启动；保留以免连带停止导航'));continue
        current=read(row['pid'],root)
        if current!=row:continue
        try:send(row['pid'],signal.SIGINT);targets.append(row)
        except ProcessLookupError:pass
        except OSError as error:report['errors'].append(dict(pid=row['pid'],reason=str(error)))
    deadline=clock()+3.
    while targets and clock()<deadline:
        pending=[]
        for row in targets:
            current=read(row['pid'],root)
            if current is None or current['start']!=row['start']:report['closed'].append(row['pid'])
            else:pending.append(row)
        targets=pending
        if targets:sleep(.05)
    for row in targets:report['skipped'].append(dict(pid=row['pid'],reason='正常关闭未完成；未强杀、未停止导航'))
    return report
