"""Fake process table only: no real signals, camera or robot commands."""
import copy,importlib,json,os,signal,tempfile,types,unittest
from pathlib import Path
from unittest.mock import patch
import charging_robot.navigation.navigation_display as displays
import charging_robot.arm.arm_tool_terminal as terminal
class Clock:
 def __init__(self):self.t=10.
 def __call__(self):return self.t
 def sleep(self,dt):self.t+=dt
NAV='/opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz'
def row(pid,args,ppid=1):return dict(pid=pid,ppid=ppid,start=str(pid*10),args=args)
class DisplayTests(unittest.TestCase):
 def test_only_exact_navigation_config_matches_not_arm_unknown_or_shell(self):
  self.assertTrue(displays.navigation_rviz(row(1,['rviz2','-d',NAV])))
  for args in (['rviz2','-d','/synthetic/arm/nova2_icp.rviz'],['rviz2','-d','/tmp/nav2_default_view.rviz'],['bash','-c','rviz2 -d '+NAV],['rviz2','-d',NAV,'-d','/tmp/other.rviz']):self.assertFalse(displays.navigation_rviz(row(1,args)))
 def test_dedicated_launch_allowed_shared_bringup_and_unknown_parent_preserved(self):
  rv=row(10,['rviz2','-d',NAV],20)
  for args,allowed in ((['python3','/opt/ros/humble/bin/ros2','launch','nav2_bringup','rviz_launch.py'],True),(['ros2','launch','nav2_bringup','bringup_launch.py'],False),(['python3','/tmp/other.py'],False)):
   self.assertEqual(displays.independently_launched(rv,{20:row(20,args)}),allowed)
  self.assertFalse(displays.independently_launched(rv,{}))
 def fixture(self,rows):
  tmp=tempfile.TemporaryDirectory();root=Path(tmp.name);table={r['pid']:copy.deepcopy(r) for r in rows}
  for pid in table:(root/str(pid)).mkdir()
  return tmp,root,table
 def test_only_navigation_pid_receives_sigint_and_no_restarts(self):
  rows=[row(10,['rviz2','-d',NAV]),row(11,['rviz2','-d','/synthetic/arm/nova2_icp.rviz']),row(12,['driver'])];tmp,root,table=self.fixture(rows);sent=[];clock=Clock()
  try:
   def send(pid,sig):sent.append((pid,sig));table.pop(pid)
   result=displays.close_navigation_rviz(root=root,read=lambda pid,root:table.get(pid),send=send,clock=clock,sleep=clock.sleep)
   self.assertEqual(sent,[(10,signal.SIGINT)]);self.assertEqual(result['closed'],[10]);self.assertEqual(set(table),{11,12})
   displays.close_navigation_rviz(root=root,read=lambda pid,root:table.get(pid),send=send,clock=clock,sleep=clock.sleep);self.assertEqual(len(sent),1)
  finally:tmp.cleanup()
 def test_identity_change_prevents_signal(self):
  tmp,root,table=self.fixture([row(10,['rviz2','-d',NAV])]);calls=[];reads=[]
  def read(pid,root):
   reads.append(1);r=copy.deepcopy(table[pid]);r['start']='reused' if len(reads)>1 else r['start'];return r
  try:displays.close_navigation_rviz(root=root,read=read,send=lambda *a:calls.append(a));self.assertEqual(calls,[])
  finally:tmp.cleanup()
 def test_timeout_never_escalates_to_term_kill_or_group_signals(self):
  tmp,root,table=self.fixture([row(10,['rviz2','-d',NAV])]);sent=[];c=Clock()
  try:
   result=displays.close_navigation_rviz(root=root,read=lambda pid,root:table.get(pid),send=lambda *a:sent.append(a),clock=c,sleep=c.sleep)
   self.assertEqual(sent,[(10,signal.SIGINT)]);self.assertEqual(len(result['skipped']),1)
  finally:tmp.cleanup()

class ToolConfigurationTests(unittest.TestCase):
 def test_command_uses_explicit_external_executable_as_one_argument(self):
  with patch.dict(os.environ,{'CHARGING_DOCKING_COMMAND':'/synthetic/tool with spaces'}):self.assertEqual(terminal.command('test'),['/synthetic/tool with spaces','test'])
 def test_home_and_test_remain_separate_commands(self):
  with patch.dict(os.environ,{'CHARGING_DOCKING_COMMAND':'/synthetic/tool'}):self.assertEqual(terminal.command('home'),['/synthetic/tool','home'])
  with self.assertRaises(ValueError):terminal.command('run')
 def test_child_environment_does_not_inherit_auto_execution(self):
  e=terminal.child_env({'CHARGE_ROBOT_DEMO_AUTO':'1','CHARGE_ROBOT_HOME_REQUEST_ID':'old','ROS_DOMAIN_ID':'31'});self.assertNotIn('CHARGE_ROBOT_DEMO_AUTO',e);self.assertNotIn('CHARGE_ROBOT_HOME_REQUEST_ID',e);self.assertEqual(e['ROS_DOMAIN_ID'],'67');self.assertEqual(e['ROS_LOCALHOST_ONLY'],'1')
 def test_tool_conflict_prevents_display_close_and_command(self):
  with tempfile.TemporaryDirectory() as tmp:
   called=[];code=terminal.run('test',Path(tmp)/'state.json','synthetic',run_command=lambda *a,**k:called.append('run'),scanner=lambda:None,active_scanner=lambda **k:[{'pid':123}],prepare_display=lambda:called.append('display'));self.assertEqual(code,2);self.assertEqual(called,[])
 def test_fake_run_has_no_implicit_home_or_return(self):
  with tempfile.TemporaryDirectory() as tmp:
   called=[]
   def run(args,**kw):called.append(args);return types.SimpleNamespace(returncode=0)
   code=terminal.run('test',Path(tmp)/'state.json','synthetic',run_command=run,scanner=lambda:None,active_scanner=lambda **k:[]);self.assertEqual(code,0);self.assertEqual(len(called),1);self.assertEqual(called[0][-1],'test')
 def test_explicit_navigation_rviz_config_is_allowed_without_device_path(self):
  previous=set(displays.NAV_CONFIGS)
  try:
   with patch.dict(os.environ,{'CHARGING_NAV_RVIZ_CONFIGS':'/synthetic/custom.rviz'}):importlib.reload(displays);self.assertTrue(displays.navigation_rviz(row(1,['rviz2','-d','/synthetic/custom.rviz'])))
  finally:displays.NAV_CONFIGS=previous

if __name__=='__main__':unittest.main(verbosity=2)
