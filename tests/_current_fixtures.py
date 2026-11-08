"""Fake feedback, clocks, publishers and offscreen windows. No ROS nodes."""
import copy,json,queue,sys,tempfile,threading,time,types,unittest
from pathlib import Path
from unittest.mock import patch
if 'std_msgs.msg' not in sys.modules:
 msgs=types.ModuleType('std_msgs.msg');msgs.String=type('String',(),{});package=types.ModuleType('std_msgs');package.msg=msgs;sys.modules['std_msgs']=package;sys.modules['std_msgs.msg']=msgs
from charging_robot.returning.return_bridge import ReturnBridge
from charging_robot.control.stop_recovery import StopRecoveryBridge
from charging_robot.arm.arm_tools_bridge import ArmToolsBridge
from test_raw_mode import snap
from PyQt5 import QtCore,QtWidgets
import charging_robot.ui.app as panel
import charging_robot.ui.arm_tools_ui as tools
QT=QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

def stopped():
 return dict(busy=False,return_active=False,manual=False,manual_stop_pending=False,stop_reset_pending=False,software_stop=False,docking_hold=False,odom_age=.01,arbiter_age=.01,control_owner='panel',velocity=(0.,0.),stationary_s=3.,stationary_samples=30,hold_feedback_confirmed=True,arbiter=dict(owner='panel',mode='stop',graph_ready=True,driver_gate_verified=True,fault=None,output_v_m_s=0.,output_w_rad_s=0.))

class Base(StopRecoveryBridge,ArmToolsBridge,ReturnBridge):
 def __init__(self,t,now):
  self.lock=threading.Lock();self.s=snap(t,now);self.s.update(stopped());self.arbiter_state=dict(snap(t,now)['arbiter'],**stopped()['arbiter'],commissioned=True,release_profile='short_only',return_speed_m_s=.05,return_start_mode='click',feedback_speed_limit_m_s=.4);self.arbiter_at=now;self.control_owner='panel';self.gui_at=now;self.return_runtime=None;self.used_revalidations=set();self.docking_hold=False;self.software_stop=False;self.manual=False;self.manual_stop_pending=False;self.cancel_requested=False;self.busy=False;self.server_active=False;self.motion_recovery_required=False;self.commands=queue.Queue();self.stop_reset_pending=None;self.reset_arbiter=False;self.control_sequence=0;self.control_mode='stop';self.return_state={'phase':'idle'};self.return_held=False;self.return_trial_id=None;self.return_short_test=False;self.return_arbiter_instance=None;self.raw_tracker=t;self.manual_target=(0.,0.);self.known_arbiter_instance=None;self.known_raw_arbiter_epoch=None;self.last_diagnostic_mode=None;self.stop_cause=None;self.stop_observation=types.SimpleNamespace(first=1.,duration=lambda _:3.);self.events=[];self.packets=[];self.authority_pub=types.SimpleNamespace(publish=lambda m:self.packets.append(json.loads(m.data)))
 def _snapshot(self,now):return dict(copy.deepcopy(self.s),arbiter=copy.deepcopy(self.arbiter_state),docking_hold=self.docking_hold,cancel_requested=self.cancel_requested,software_stop=self.software_stop,return_active=self.return_active(),motion_recovery_required=self.motion_recovery_required)
 def event(self,name,**kw):self.events.append((name,kw))
 def get_clock(self):return types.SimpleNamespace(now=lambda:types.SimpleNamespace(nanoseconds=int(1e9*self.s['ros_now'])))

class UiBase:
 def __init__(self):
  self.s=stopped();self.calls=[];self.docking_hold=False;self.software_stop=False;self.motion_recovery_required=False;self.cancel_requested=False;self.manual=False;self.closed=False
 def snapshot(self):return dict(copy.deepcopy(self.s),software_stop=self.software_stop,docking_hold=self.docking_hold)
 def arm_hold_snapshot(self):return self.snapshot()
 def emergency_stop(self,**kwargs):self.software_stop=True;self.calls.append(('stop',kwargs))
 def close(self):self.closed=True
 def reset_view(self):pass

class WindowTests(unittest.TestCase):
 def setUp(self):
  self.tmp=tempfile.TemporaryDirectory();self.previous=panel.ROOT;panel.ROOT=Path(self.tmp.name);self.rows=[];self.launches=[]
  self.scan=patch.object(tools,'active_arm_tools',side_effect=lambda:self.rows);self.scan.start()
  def launch(cmd,args,cwd):self.launches.append((cmd,args));return True,123
  self.spawn=patch.object(tools.QtCore.QProcess,'startDetached',side_effect=launch);self.spawn.start()
  self.w=panel.Window(offline=True);self.w.timer.stop();self.w.manual_timer.stop();self.b=UiBase();self.w.bridge=self.b;self.w.offline=False;self.w.latest=self.b.snapshot();self.errors=[];self.w.error=self.errors.append;self.w.confirm=lambda _:True;self.w.show();QT.processEvents()
 def tearDown(self):
  self.w.bridge=None;self.w.offline=True;self.w.allow_close=True;self.w.arm_external_active=False;self.w.arm_process=None;self.w.close();QT.removeEventFilter(self.w.return_hold);self.w.deleteLater();QtCore.QCoreApplication.sendPostedEvents(None,QtCore.QEvent.DeferredDelete);QT.processEvents();self.spawn.stop();self.scan.stop();panel.ROOT=self.previous;self.tmp.cleanup()
ui=types.SimpleNamespace(WindowTests=WindowTests,QT=QT)
