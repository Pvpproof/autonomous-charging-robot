"""Actual fault-reset handshake only; ordinary stops are tested separately."""
from charging_robot.paths import source_path
import ast,copy,json,math,queue,tempfile,time,types,unittest
from pathlib import Path
from unittest.mock import patch
from _current_fixtures import Base
from test_raw_mode import make_plan
class RecoveryTests(unittest.TestCase):
 def setUp(self):self.tmp=tempfile.TemporaryDirectory();self.plan,self.t,self.now=make_plan();self.b=Base(self.t,self.now);self.b.cancel_requested=True
 def tearDown(self):self.tmp.cleanup()
 def call(self):
  with patch('charging_robot.control.stop_recovery.time.monotonic',return_value=self.now):return self.b.release_stop()
 def stale(self):self.b.s.update(odom_age=math.inf,arbiter_age=math.inf);self.b.software_stop=True
 def feedback(self,**kw):
  data=dict(self.b.arbiter_state,authority_sequence=self.b.control_sequence,output_v_m_s=0.,output_w_rad_s=0.,fault=None);data.update(kw)
  with patch('charging_robot.returning.return_bridge.time.monotonic',return_value=self.now+.1):self.b.on_arbiter(types.SimpleNamespace(data=json.dumps(data)))
 def test_no_feedback_clears_only_ui_lock_not_actual_motion(self):
  self.stale();result=self.call();self.assertEqual(result['kind'],'panel_only');self.assertFalse(self.b.software_stop);self.assertTrue(self.b.motion_recovery_required);self.assertFalse(self.b.reset_arbiter);self.assertIsNone(self.b.stop_reset_pending);self.assertEqual(self.b.manual_target,(0.,0.));self.assertEqual(self.b.commands.get_nowait(),('cancel',()))
 def test_fresh_feedback_returning_never_automatically_arms_motion(self):
  self.stale();self.call();self.b.s.update(odom_age=.01,arbiter_age=.01);self.feedback();self.assertTrue(self.b.motion_recovery_required);self.assertIsNone(self.b.stop_reset_pending)
 def test_occupancy_recovery_requires_motion_gate_before_clearing_hold(self):
  self.b.docking_hold=True
  with self.assertRaises(ValueError):self.b.clear_recovered_tool_hold()
  self.stale();self.call();self.b.clear_recovered_tool_hold();self.assertFalse(self.b.docking_hold);self.assertTrue(self.b.motion_recovery_required)
 def test_original_quiet_and_graph_conditions_apply_to_downstream_confirmation(self):
  self.stale();self.call();self.b.s.update(odom_age=.01,arbiter_age=.01);result=self.call();self.assertEqual(result['kind'],'downstream_reset');self.assertTrue(self.b.motion_recovery_required);self.assertTrue(self.b.software_stop);self.assertTrue(self.b.reset_arbiter)
 def test_current_acknowledged_zero_reset_enables_without_resuming_tasks(self):
  self.call();self.b.stop_reset_pending['sequence']=1;self.b.control_sequence=1;self.feedback();self.assertFalse(self.b.motion_recovery_required);self.assertFalse(self.b.software_stop);self.assertFalse(self.b.manual);self.assertIsNone(self.b.return_runtime);self.assertTrue(self.b.cancel_requested)
 def test_feedback_loss_during_reset_cannot_be_hidden_by_zero_ack(self):
  self.call();self.b.stop_reset_pending['sequence']=1;self.b.control_sequence=1;self.b.s['odom_age']=math.inf;self.feedback()
  self.assertTrue(self.b.motion_recovery_required);self.assertTrue(self.b.software_stop);self.assertIsNone(self.b.stop_reset_pending)
 def test_old_ack_wrong_instance_fault_and_nonzero_do_not_arm(self):
  for fields in ({'authority_sequence':0},{'instance_id':'other'},{'fault':'driver watchdog'},{'output_v_m_s':.01}):
   self.b=Base(self.t,self.now);self.b.cancel_requested=True;self.call();self.b.stop_reset_pending['sequence']=1;self.b.control_sequence=1;self.feedback(**fields);self.assertTrue(self.b.motion_recovery_required,fields)
 def test_missing_motion_or_invalid_feedback_never_requests_downstream_reset(self):
  for kw in (dict(busy=True),dict(velocity=(.1,0.)),dict(odom_age=float('nan')),dict(arbiter_age=float('inf'))):
   self.b=Base(self.t,self.now);self.b.cancel_requested=True;self.b.s.update(kw);result=self.call();self.assertEqual(result['kind'],'panel_only',kw);self.assertFalse(self.b.reset_arbiter)
 def test_authority_output_stays_zero_even_if_old_navigation_remains_busy(self):
  self.stale();self.b.busy=True;self.b.s['busy']=True;self.call()
  with patch('charging_robot.returning.return_bridge.time.monotonic',return_value=self.now):self.b.control_tick()
  self.assertEqual((self.b.packets[-1]['mode'],self.b.packets[-1]['v'],self.b.packets[-1]['w']),('stop',0.,0.));self.assertFalse(self.b.packets[-1]['reset'])
 def test_return_prepare_and_start_block_until_motion_confirmation(self):
  self.stale();self.call();confirm={n:True for n in ('operator_present','route_cleared','physical_stop_ready','arm_stowed','exclusive_control_confirmed','no_rear_detection_acknowledged')}
  with patch('charging_robot.returning.return_bridge.time.monotonic',return_value=self.now),self.assertRaises(ValueError):self.b.prepare_return(self.plan,Path(self.tmp.name)/'s.json','/not-map',confirm,short_test=True)
  with self.assertRaisesRegex(ValueError,'底盘就绪'):self.b.start_return()
 def test_timed_out_actual_reset_and_late_feedback_do_not_arm(self):
  self.call()
  with patch('charging_robot.returning.return_bridge.time.monotonic',return_value=self.now+2.1):self.b.control_tick()
  self.assertIsNone(self.b.stop_reset_pending);self.assertTrue(self.b.motion_recovery_required)
  self.feedback();self.assertTrue(self.b.motion_recovery_required)
 def test_raw_runtime_direct_use_also_respects_unconfirmed_motion_gate(self):
  from charging_robot.returning.raw_odom_mode import RawReturnRuntime
  from test_raw_mode import conditions
  r=RawReturnRuntime(self.plan,conditions(self.plan),Path(self.tmp.name)/'s.json')
  s=self.b._snapshot(self.now);s['motion_recovery_required']=True
  with self.assertRaisesRegex(ValueError,'控制权'):r.check_coordinates(s,self.now)
 def test_real_manual_and_queued_navigation_entrypoints_block(self):
  tree=ast.parse((source_path('ros_bridge.py')).read_text());klass=next(x for x in tree.body if getattr(x,'name',None)=='Bridge')
  space={'time':time}
  for name in ('enable_manual','request_goal','send_goal'):
   node=copy.deepcopy(next(x for x in klass.body if getattr(x,'name',None)==name));module=ast.Module(body=[node],type_ignores=[]);exec(compile(module,'<isolated-motion-entry>','exec'),space)
  self.b.motion_recovery_required=True;self.b.docking_hold=False;self.b.software_stop=False;self.b.nav2_pause_gate=types.SimpleNamespace(phase='idle');self.b.navigation_id='old'
  with self.assertRaises(ValueError):space['enable_manual'](self.b)
  with self.assertRaises(ValueError):space['request_goal'](self.b,(1,0,0))
  space['send_goal'](self.b,1,0,0,'old');self.assertFalse(self.b.busy)

class RecordingRecoveryTests(unittest.TestCase):
 def test_in_progress_record_is_not_certified_when_panel_recovers(self):
  from _recording_helpers import session_fixture
  from test_raw_mode import snap
  with tempfile.TemporaryDirectory() as tmp:
   t,now,record=session_fixture(tmp);s=snap(t,now);s['motion_recovery_required']=True;record.update(s,'map',now)
   self.assertEqual(record.phase,'blocked');self.assertFalse(record.return_eligible)
 def test_already_completed_record_is_preserved_but_execution_remains_gated(self):
  from _recording_helpers import session_fixture,move,finish
  from test_raw_mode import snap
  with tempfile.TemporaryDirectory() as tmp:
   t,now,record=session_fixture(tmp);now=move(t,now,record,.1,0,50);now=finish(t,now,record);saved=copy.deepcopy(record.samples)
   self.assertTrue(record.return_eligible);s=snap(t,now);s['motion_recovery_required']=True;record.update(s,'map',now)
   self.assertEqual(record.samples,saved);self.assertEqual(record.phase,'outbound_stopped');self.assertTrue(record.return_eligible)

if __name__=="__main__":unittest.main(verbosity=2)
