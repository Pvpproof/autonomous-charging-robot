"""Real source methods with fake ROS interfaces and offscreen UI. No devices."""
from charging_robot.paths import source_path
import ast,copy,json,tempfile,types,unittest,queue,time
from pathlib import Path
from unittest.mock import patch
import _current_fixtures as fixture
from test_raw_mode import make_plan,snap,update
from _recording_helpers import session_fixture
from charging_robot.control.control_causes import recording_block_reason
import charging_robot.ui.app as app
ROOT=Path(__file__).resolve().parents[1]

def methods(*names):
 tree=ast.parse((source_path('ros_bridge.py')).read_text());c=next(n for n in tree.body if getattr(n,'name',None)=='Bridge');space=dict(time=time,Twist=lambda:object())
 for name in names:
  node=next(n for n in c.body if getattr(n,'name',None)==name);exec(compile(ast.Module(body=[node],type_ignores=[]),'<installed-'+name+'>','exec'),space)
 return space

class StopTests(unittest.TestCase):
 def setUp(self):
  self.plan,self.t,self.now=make_plan();self.b=fixture.Base(self.t,self.now);self.b.stop_cause=None;self.b.cancel_requested=False;self.b.navigation_id=None;self.b.odom_time=self.now;self.b.nav2_pause_gate=types.SimpleNamespace(phase='idle');self.b.manual_pub=types.SimpleNamespace(publish=lambda m:None)
  for name,method in methods('emergency_stop','enable_manual','request_goal','send_goal','accepted').items():
   if name in ('emergency_stop','enable_manual','request_goal','send_goal','accepted'):setattr(self.b,name,types.MethodType(method,self.b))
 def stop(self):
  with patch('charging_robot.returning.return_bridge.time.monotonic',return_value=self.now):self.b.emergency_stop()
 def test_stop_cancels_intent_without_creating_lock(self):
  self.b.busy=True;self.b.navigation_id='old';self.stop()
  self.assertFalse(self.b.software_stop);self.assertFalse(self.b.motion_recovery_required);self.assertTrue(self.b.cancel_requested);self.assertEqual(self.b.packets[-1]['mode'],'stop');self.assertEqual(self.b.commands.get_nowait(),('cancel',()))
 def test_cancellation_remains_zero_even_if_gui_and_nav_recover(self):
  self.b.busy=True;self.b.navigation_id='old';self.stop()
  for _ in range(5):
   self.now+=.05;self.b.gui_at=self.now
   with patch('charging_robot.returning.return_bridge.time.monotonic',return_value=self.now):self.b.control_tick()
   self.assertEqual(self.b.packets[-1]['mode'],'stop')
 def test_explicit_new_manual_start_needs_no_unlock(self):
  self.stop();self.b.commands.get_nowait()
  with patch('charging_robot.returning.return_bridge.time.monotonic',return_value=self.now):self.b.enable_manual()
  self.assertTrue(self.b.manual);self.assertFalse(self.b.cancel_requested);self.assertIsNone(self.b.stop_cause)
 def test_explicit_new_goal_needs_no_unlock_after_cancel_finished(self):
  self.stop();self.b.commands.get_nowait();self.b.request_goal((1.,0.,0.),'new')
  self.assertEqual(self.b.commands.get_nowait(),('goal',(1.,0.,0.,'new')));self.assertFalse(self.b.cancel_requested);self.assertFalse(self.b.software_stop)
 def test_completed_route_can_be_prepared_after_ordinary_stop_without_unlock(self):
  self.stop();confirm={n:True for n in ('operator_present','route_cleared','physical_stop_ready','arm_stowed','exclusive_control_confirmed','no_rear_detection_acknowledged')}
  with tempfile.TemporaryDirectory() as tmp,patch('charging_robot.returning.return_bridge.time.monotonic',return_value=self.now):self.b.prepare_return(self.plan,Path(tmp)/'start.json','/not-map',confirm,short_test=True)
  self.assertFalse(self.b.cancel_requested);self.assertIsNone(self.b.stop_cause);self.assertEqual(self.b.return_runtime.phase,'ready');self.assertIsNone(self.b.return_runtime.run_session)
 def test_queued_old_goal_cannot_dispatch_after_stop(self):
  self.b.busy=True;self.b.navigation_id='old';self.stop();self.b.send_goal(1,0,0,'old');self.assertFalse(self.b.busy);self.assertTrue(self.b.cancel_requested)
 def test_late_goal_acceptance_is_cancelled_instead_of_resumed(self):
  self.b.busy=True;self.b.navigation_id='old';self.stop();calls=[];self.b.cancel=lambda:calls.append('cancel')
  goal=types.SimpleNamespace(accepted=True,get_result_async=lambda:types.SimpleNamespace(add_done_callback=lambda cb:None));self.b.accepted(types.SimpleNamespace(result=lambda:goal),'old');self.assertEqual(calls,['cancel'])
 def test_genuine_arbiter_fault_not_cleared_by_ordinary_stop(self):
  self.b.latch_software_stop('arbiter_fault','真实控制故障',self.now);self.b.motion_recovery_required=True;self.stop()
  self.assertTrue(self.b.software_stop);self.assertTrue(self.b.motion_recovery_required);self.assertEqual(self.b.stop_cause['code'],'arbiter_fault')
 def test_arm_tool_release_does_not_require_extra_software_unlock(self):
  with patch('charging_robot.arm.arm_tools_bridge.time.monotonic',return_value=self.now):self.b.begin_independent_arm_hold()
  self.assertTrue(self.b.docking_hold);self.assertFalse(self.b.software_stop)
  with patch('charging_robot.arm.arm_tools_bridge.time.monotonic',return_value=self.now):self.b.release_docking()
  self.assertFalse(self.b.docking_hold);self.assertFalse(self.b.software_stop);self.assertFalse(self.b.motion_recovery_required);self.assertTrue(self.b.cancel_requested)
 def test_recording_still_invalidates_when_cancel_requested_without_lock(self):
  with tempfile.TemporaryDirectory() as tmp:
   t,now,s=session_fixture(tmp);v=snap(t,now);v['cancel_requested']=True;s.update(v,'meta',now)
   self.assertEqual(s.phase,'blocked');self.assertIn('停止或取消',s.block_reason);self.assertTrue(s.rejection['control']['cancel_requested'])
 def test_repeated_stop_preserves_first_cause_without_latching(self):
  self.stop();cause=copy.deepcopy(self.b.stop_cause);self.b.latch_software_stop('recording_failed','次生记录失败',self.now+.1);self.assertEqual(self.b.stop_cause,cause);self.assertFalse(self.b.software_stop)

class UiTests(unittest.TestCase):
 setUp=fixture.ui.WindowTests.setUp
 tearDown=fixture.ui.WindowTests.tearDown
 def test_old_stop_marker_is_not_restored_as_software_lock(self):
  self.w.motion_recovery_marker.write_text('{"required":true,"automatic_resume":false}')
  with patch.object(app.time,'strftime',return_value='reopen_nonlatch'):w=app.Window(offline=True)
  w.timer.stop();w.manual_timer.stop();self.assertFalse(w.saved_motion_recovery);self.assertFalse(w.latest.get('software_stop',False));w.allow_close=True;fixture.ui.QT.removeEventFilter(w.return_hold);w.close();w.deleteLater();fixture.ui.QT.processEvents()
 def test_actual_ui_stop_does_not_invent_or_persist_a_normal_lock(self):
  self.w.offline=False;self.b.emergency_stop=lambda:None;self.b.software_stop=False;self.b.s['motion_recovery_required']=False
  self.w.emergency_stop();self.assertFalse(self.w.latest['software_stop']);self.assertFalse(self.w.saved_motion_recovery);self.assertFalse(json.loads(self.w.motion_recovery_marker.read_text())['required'])
 def test_normal_state_hides_unlock_button_and_keeps_stop_button(self):
  self.w.latest['software_stop']=False;self.w.latest['motion_recovery_required']=False;self.w.update_navigation_ui();self.assertTrue(self.w.release_stop_button.isHidden());self.assertIn('停止底盘',self.w.emergency_button.text())
 def test_real_fault_exposes_specific_recovery_not_ordinary_stop_lock(self):
  self.w.latest['software_stop']=True;self.w.update_navigation_ui();self.assertFalse(self.w.release_stop_button.isHidden());self.assertEqual(self.w.release_stop_button.text(),'处理控制故障')

if __name__=='__main__':unittest.main(verbosity=2)
