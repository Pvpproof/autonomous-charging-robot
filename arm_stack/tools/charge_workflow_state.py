#!/usr/bin/env python3
"""Workflow process/file lifecycle only. Never sends robot commands."""
import os,sys,json,time,signal,shlex,uuid
from pathlib import Path
ROOT=Path('/opt/charging-robot-local/charging_socket_vision/results/realtime_runs')
VISION=Path('/opt/charging-robot-local/charging_socket_vision')
LIVE=VISION/'pnp_live_test_20261023_M07DN5/pnp_live_rgb_v3/live_results/latest_pose_readonly.json'
ICP=VISION/'results/v3_diagnostics/latest_pose_readonly.json'

def atomic(path,data):
 path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
 tmp=path.with_name(path.name+'.'+uuid.uuid4().hex+'.tmp');tmp.write_text(json.dumps(data,indent=2));tmp.replace(path)

def matches(args,needle):
 tokens=shlex.split(needle)
 if not tokens:return False
 # Never match shell command strings, grep/awk, or this helper's own arguments.
 executable=Path(args[0]).name if args else ''
 if not (executable in ('python3','python','ros2','rviz2','ros2_control_node','robot_state_publisher') or executable.startswith('python3.')):
  return False
 for i in range(len(args)-len(tokens)+1):
  first=(args[i]==tokens[0] or ('/' not in tokens[0] and Path(args[i]).name==tokens[0]))
  if first and args[i+1:i+len(tokens)]==tokens[1:]:return True
 return False

def processes():
 out={}
 for p in Path('/proc').iterdir():
  if not p.name.isdigit():continue
  try:
   args=[x.decode(errors='replace') for x in (p/'cmdline').read_bytes().split(b'\0') if x]
   stat=(p/'stat').read_text().rsplit(')',1)[1].split()
   if stat[0]!='Z':out[int(p.name)]=(int(stat[1]),args)
  except (OSError,ValueError,IndexError):pass
 return out

def pids(needle):return [pid for pid,(_,args) in processes().items() if pid!=os.getpid() and matches(args,needle)]

def stop(needle):
 table=processes();parents={pid for pid,(_,args) in table.items() if pid!=os.getpid() and matches(args,needle)}
 if not parents:return
 allp=set(parents)
 while True:
  more={pid for pid,(ppid,_) in table.items() if ppid in allp}-allp
  if not more:break
  allp.update(more)
 # Remember process identities so PID reuse cannot terminate unrelated processes.
 identities={}
 for pid in allp:
  try:identities[pid]=Path(f'/proc/{pid}/stat').read_text().rsplit(')',1)[1].split()[19]
  except (OSError,IndexError):pass
 def alive():
  current=processes();result=[]
  for pid,start in identities.items():
   try:
    if pid in current and Path(f'/proc/{pid}/stat').read_text().rsplit(')',1)[1].split()[19]==start:result.append(pid)
   except OSError:pass
  return result
 def send(ids,sig):
  for pid in ids:
   if pid in alive():
    try:os.kill(pid,sig)
    except ProcessLookupError:pass
 send(parents,signal.SIGINT)
 for duration,sig in [(4.0,signal.SIGTERM),(2.0,signal.SIGKILL),(1.0,None)]:
  deadline=time.monotonic()+duration
  while alive() and time.monotonic()<deadline:time.sleep(.1)
  if not alive():return
  if sig is not None:send(alive(),sig)
 if alive():raise RuntimeError('程序未退出: '+needle)

def assert_idle():
 if pids('/opt/charging-robot-local/nova2_tools/execute_pending_trajectory.py'):
  raise RuntimeError('轨迹执行器仍运行，禁止重置流程或回位')

def invalidate(reason):
 ROOT.mkdir(parents=True,exist_ok=True)
 for path in [LIVE,ICP]:
  atomic(path,dict(pose_valid=False,pose_fresh=False,planning_ready=False,
      T_camera_from_socket=None,T_camera_from_socket_raw=None,reason=reason,write_time_host=time.time()))
 atomic(ROOT/'latest_camera_plan_request.json',dict(status='invalid',reason=reason,updated_at=time.time()))
 atomic(ROOT/'latest_camera_socket_target.json',dict(pose_valid=False,T_base_from_socket=None,reason=reason))

FILES=['pending_insertion_trajectory.json','latest_planning_attempt.json','pending_trajectory.json','pending_camera_trajectory.json','pending_gun_trajectory.json',
 'frozen_pnp_v3_request.json','frozen_pnp_v3_socket_target.json',
 'latest_camera_plan_request.json','latest_camera_socket_target.json','camera_target_pose_cache.json',
 'latest_camera_pose_selection.json','latest_gun_pose_selection.json','latest_gun_prepose_plan_only.json',
 'last_dispatched_trajectory.json','latest_gun_execution_feedback.json','workflow_cycle.json','latest_camera_execution_feedback.json']

# The insertion planner needs the target and the verified gun execution from
# this cycle after the visualization stage has been closed. These are current
# state, not a historical cycle archive; the next start/home clears them.
INSERTION_STATE={'frozen_pnp_v3_request.json','frozen_pnp_v3_socket_target.json',
 'latest_camera_pose_selection.json','latest_gun_prepose_plan_only.json',
 'last_dispatched_trajectory.json','latest_gun_execution_feedback.json'}

def transition(event):
 assert_idle();ROOT.mkdir(parents=True,exist_ok=True)
 for name in FILES:
  if name!='workflow_cycle.json' and not (event=='completed' and name in INSERTION_STATE):
   (ROOT/name).unlink(missing_ok=True)
 invalidate(event)
 try:previous=json.loads((ROOT/'workflow_cycle.json').read_text())
 except (OSError,ValueError):previous={}
 cycle_id=uuid.uuid4().hex if event=='started' else previous.get('cycle_id',uuid.uuid4().hex)
 atomic(ROOT/'workflow_cycle.json',dict(cycle_id=cycle_id,status=event,updated_at=time.time()))
 print('流程状态:',event)

def main():
 action=sys.argv[1];arg=sys.argv[2] if len(sys.argv)>2 else ''
 if action=='pids':print('\n'.join(map(str,pids(arg))))
 elif action=='has':return 0 if pids(arg) else 1
 elif action=='stop':stop(arg)
 elif action=='assert-idle':assert_idle()
 elif action=='transition':transition(arg)
 else:raise ValueError('unknown lifecycle action')
 return 0
if __name__=='__main__':
 try:sys.exit(main())
 except Exception as e:print('[流程拒绝]',str(e),file=sys.stderr);sys.exit(2)
