"""Synthetic clocks/feedback and offscreen Qt only: never a ROS node or device."""
import ast,copy,json,tempfile,types,unittest
from pathlib import Path
from unittest.mock import patch
import _current_fixtures as fixture
from test_raw_mode import make_plan,snap,update,conditions
from _recording_helpers import session_fixture
from raw_odom_mode import RawReturnRuntime
from control_arbiter import Arbiter
from control_causes import recording_block_reason,control_evidence,fault_detail
from return_hold_pad import ReturnStartPad,HoldPad
from PyQt5 import QtCore
ROOT=Path(__file__).resolve().parents[1]

class LivenessTests(unittest.TestCase):
 def setUp(self):
  self.tmp=tempfile.TemporaryDirectory();self.plan,self.tracker,self.now=make_plan();self.b=fixture.Base(self.tracker,self.now)
  self.b.cancel_requested=False;self.b.navigation_id=None;self.b.stop_cause=None;self.b.manual_stamp=self.now;self.b.gui_at=self.now
 def tearDown(self):self.tmp.cleanup()
 def tick(self,now=None):
  with patch('return_bridge.time.monotonic',return_value=self.now if now is None else now):self.b.control_tick()
  return self.b.packets[-1]
 def feedback(self):
  self.b.s.update(snap(self.tracker,self.now));self.b.arbiter_at=self.now
 def return_runtime(self,start=True):
  r=RawReturnRuntime(self.plan,conditions(self.plan),Path(self.tmp.name)/'start.json',activation_mode='click');self.b.return_runtime=r;self.b.return_prepared_at=self.now;self.b.return_arbiter_instance='arb';self.b.cancel_requested=False
  self.b.arbiter_state.update(owner='panel',mode='return',fault=None)
  if start:r.start_run(self.now,'a'*32)
  return r
 def test_own_navigation_keeps_running_through_gui_delay(self):
  self.b.busy=True;self.b.navigation_id='own-goal';a=Arbiter()
  for i in range(30):
   self.now+=.05;self.b.s['ros_now']=self.now;p=self.tick();a.odom(.1,0,self.now,self.now);a.receive(p,self.now,self.now);a.navigation(.1,0,self.now)
   self.assertEqual(p['mode'],'navigation');self.assertEqual(a.tick(self.now),(.1,0.));self.assertFalse(self.b.software_stop)
  events=[n for n,_ in self.b.events];self.assertEqual(events.count('ui_update_delayed'),1)
  self.b.gui_at=self.now;self.tick();self.assertEqual([n for n,_ in self.b.events].count('ui_update_recovered'),1)
 def test_successful_reset_clears_old_stop_cause(self):
  self.b.latch_software_stop('operator_stop','用户请求停车',self.now)
  with patch('stop_recovery.time.monotonic',return_value=self.now):self.b.release_stop()
  self.b.stop_reset_pending['sequence']=1;self.b.control_sequence=1
  data=dict(self.b.arbiter_state,authority_sequence=1,output_v_m_s=0.,output_w_rad_s=0.,fault=None)
  with patch('return_bridge.time.monotonic',return_value=self.now+.1):self.b.on_arbiter(types.SimpleNamespace(data=json.dumps(data)))
  self.assertFalse(self.b.software_stop);self.assertIsNone(self.b.stop_cause)
 def test_foreign_server_goal_not_adopted(self):
  self.b.server_active=True;self.assertEqual(self.tick()['mode'],'stop')
 def test_manual_stale_input_stops_and_records_actual_source(self):
  self.b.manual=True;self.b.manual_target=(.1,0);self.b.gui_at=self.now-.5
  self.assertEqual(self.tick()['mode'],'stop');self.assertEqual(self.b.stop_cause['code'],'manual_input_expired')
 def test_manual_fresh_gui_but_stale_key_input_stops(self):
  self.b.manual=True;self.b.manual_stamp=self.now-.5;self.assertEqual(self.tick()['mode'],'stop');self.assertTrue(self.b.cancel_requested);self.assertFalse(self.b.software_stop)
 def test_ready_click_does_not_start_automatically(self):
  r=self.return_runtime(False);self.b.gui_at=self.now-1;self.tick();self.assertIsNone(r.run_session);self.assertEqual(r.phase,'ready')
 def test_click_return_backend_renews_while_gui_stale(self):
  r=self.return_runtime();self.b.gui_at=self.now-1;p=list(self.plan['poses'][0]);state={'v':0.,'w':0.}
  for _ in range(15):
   self.now+=.05;v,w=state['v'],state['w'];p[0]+=v*.05;update(self.tracker,self.now,p,(v,0,w));self.feedback();packet=self.tick();state=self.b.return_state
   self.assertEqual(packet['mode'],'return',state);self.assertFalse(self.b.software_stop,state)
  self.assertLess(p[0],self.plan['poses'][0][0]);self.assertGreater(r.run_sequence,10);self.assertEqual(packet['run_lease_source'],'backend_controller')
 def test_gui_keepalive_cannot_mask_backend_gap_or_resume(self):
  r=self.return_runtime();seq=r.run_sequence
  self.assertTrue(self.b.return_keepalive('a'*32));self.assertEqual(r.run_sequence,seq)
  self.now+=.30;self.b.gui_at=self.now;update(self.tracker,self.now-.1,self.plan['poses'][0]);update(self.tracker,self.now,self.plan['poses'][0]);self.feedback();self.tick()
  self.assertEqual(r.phase,'stopped');self.assertIn('控制循环过期',r.reason);self.assertFalse(self.b.return_keepalive('a'*32));self.assertFalse(r.run_heartbeat(self.now+.01,'a'*32))
 def test_duplicate_start_idempotent_wrong_session_cannot_continue(self):
  r=self.return_runtime(False)
  with patch('return_bridge.time.monotonic',return_value=self.now):
   token=self.b.start_return();self.assertEqual(self.b.start_return(),token)
  self.assertFalse(r.run_heartbeat(self.now,'b'*32));self.assertEqual(r.phase,'stopped')
 def test_first_cause_preserved_after_secondary_recording_stop(self):
  self.b.latch_software_stop('operator_stop','用户请求底盘停车',self.now);cause=copy.deepcopy(self.b.stop_cause)
  self.b.latch_software_stop('recording_failed','记录因停车失败',self.now+.1);self.assertEqual(self.b.stop_cause,cause)
  self.assertEqual(sum(n=='motion_stop_requested' for n,_ in self.b.events),1)
 def test_actual_emergency_stop_method_cancels_and_zeroes(self):
  tree=ast.parse((ROOT/'ros_bridge.py').read_text());klass=next(n for n in tree.body if getattr(n,'name',None)=='Bridge');node=next(n for n in klass.body if getattr(n,'name',None)=='emergency_stop');space={'Twist':lambda:object()};exec(compile(ast.Module(body=[node],type_ignores=[]),'<actual-stop>','exec'),space)
  self.b.manual_pub=types.SimpleNamespace(publish=lambda _:None);self.b.busy=True;self.b.navigation_id='own'
  with patch('return_bridge.time.monotonic',return_value=self.now):space['emergency_stop'](self.b)
  self.assertEqual(self.b.packets[-1]['mode'],'stop');self.assertEqual(self.b.stop_cause['code'],'operator_stop');self.assertEqual(self.b.commands.get_nowait(),('cancel',()))
 def test_control_loss_is_independent_of_gui(self):
  a=Arbiter();self.b.busy=True;self.b.navigation_id='own';p=self.tick();a.receive(p,self.now,self.now);a.odom(0,0,self.now,self.now);a.navigation(.1,0,self.now)
  self.now+=.3;self.b.gui_at=self.now;a.odom(0,0,self.now,self.now);a.navigation(.1,0,self.now)
  self.assertEqual(a.tick(self.now),(0.,0.));self.assertEqual(a.fault,'panel authority heartbeat expired')
 def test_nav_or_odom_loss_stops_with_healthy_backend(self):
  for lost in ('nav','odom'):
   a=Arbiter();start=self.now;self.b.busy=True;self.b.navigation_id='own'
   for i in range(8):
    self.now=start+i*.05;self.b.s['ros_now']=self.now;p=self.tick();a.receive(p,self.now,self.now)
    if i==0 or lost!='odom':a.odom(0,0,self.now,self.now)
    if i==0 or lost!='nav':a.navigation(.1,0,self.now)
    a.tick(self.now)
   self.assertEqual(a.fault,{'nav':'navigation command expired','odom':'odometry feedback expired'}[lost])
 def test_fault_diagnostic_not_mislabeled_as_human_takeover(self):
  self.assertEqual(fault_detail('panel authority heartbeat expired'),'后台底盘控制指令中断')
  self.assertNotIn('接管',recording_block_reason({'software_stop':True}))
 def test_record_rejection_persists_flags_and_first_cause(self):
  with tempfile.TemporaryDirectory() as tmp:
   t,now,record=session_fixture(tmp);s=snap(t,now);s.update(software_stop=True,stop_cause={'code':'operator_stop','detail':'用户请求底盘停车'},gui_age_s=.55,control_mode='navigation');record.update(s,'meta',now)
   evidence=json.loads((record.directory/'first-rejection.json').read_text())['control'];self.assertEqual(evidence['stop_cause']['code'],'operator_stop');self.assertEqual(evidence['gui_age_s'],.55);self.assertFalse(evidence['manual']);self.assertIn('用户请求',record.block_reason)
 def test_record_reasons_are_distinct(self):
  messages=[recording_block_reason({flag:True}) for flag in ('software_stop','motion_recovery_required','manual','manual_stop_pending','docking_hold')];self.assertEqual(len(set(messages)),5)

class PadTests(unittest.TestCase):
 def test_click_focus_hide_do_not_cancel_but_close_does(self):
  p=ReturnStartPad();calls=[];p.cancel_requested.connect(lambda:calls.append(1));p.set_running(True)
  for kind in (QtCore.QEvent.ApplicationDeactivate,QtCore.QEvent.Hide,QtCore.QEvent.WindowDeactivate,QtCore.QEvent.FocusOut):p.eventFilter(p.window(),QtCore.QEvent(kind))
  self.assertEqual(calls,[]);p.eventFilter(p.window(),QtCore.QEvent(QtCore.QEvent.Close));self.assertEqual(calls,[1]);fixture.ui.QT.removeEventFilter(p);p.deleteLater()
 def test_hold_focus_loss_still_releases(self):
  p=HoldPad();p.set_held(True);p.eventFilter(p.window(),QtCore.QEvent(QtCore.QEvent.ApplicationDeactivate));self.assertFalse(p.held);fixture.ui.QT.removeEventFilter(p);p.deleteLater()

if __name__=='__main__':unittest.main(verbosity=2)
