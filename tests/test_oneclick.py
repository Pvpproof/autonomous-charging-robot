import unittest,tempfile,copy
from pathlib import Path
from charging_robot.control.run_lease import RunLeaseGate
from charging_robot.returning.raw_odom_mode import RawReturnRuntime
from charging_robot.returning.release_policy import ReleasePolicy, TrialLimiter
from test_raw_mode import make_plan,conditions,snap,update
from test_full_trial import full_config
from test_release_policy import packet

TOKEN='1'*32

def clickpacket(**kw):
 p=packet(short_test=False,route_length_m=1.,activation_mode='click',run_state='running',run_session=TOKEN,run_sequence=1,held=False)
 p.update(kw);return p

class ClickLeaseTests(unittest.TestCase):
 def test_explicit_start_and_progress_required(self):
  for field,value in [('run_session',None),('run_sequence',True),('run_sequence',0),('run_state','ready')]:
   with self.assertRaises(ValueError):RunLeaseGate().check(clickpacket(**{field:value}),10.)
  g=RunLeaseGate();g.check(clickpacket(),10.);g.check(clickpacket(),10.24)
  with self.assertRaisesRegex(ValueError,'expired'):g.check(clickpacket(),10.251)
 def test_advanced_sequence_renews_but_replay_or_session_change_fails(self):
  for kw in [dict(run_sequence=0),dict(run_sequence=True),dict(run_session='2'*32),dict(owner='other')]:
   g=RunLeaseGate();g.check(clickpacket(),10.)
   with self.assertRaises(ValueError):g.check(clickpacket(**kw),10.1)
  g=RunLeaseGate();g.check(clickpacket(),10.);g.check(clickpacket(run_sequence=2),10.2);g.check(clickpacket(run_sequence=2),10.44)
  with self.assertRaises(ValueError):g.check(clickpacket(run_sequence=1),10.45)
 def test_late_new_sequence_cannot_revive_expired_lease(self):
  g=RunLeaseGate();g.check(clickpacket(),10.)
  with self.assertRaisesRegex(ValueError,'expired'):g.check(clickpacket(run_sequence=2),10.251)
 def test_closed_session_cannot_resume(self):
  g=RunLeaseGate();g.check(clickpacket(),10.);g.close()
  with self.assertRaisesRegex(ValueError,'cannot resume'):g.check(clickpacket(run_sequence=2),10.1)
 def test_controller_sequence_freeze_stops_even_authority_is_fresh(self):
  c=full_config();c['return_start_mode']='click';c['feedback_speed_limit_m_s']=.4
  g=TrialLimiter(ReleasePolicy(c));g.claim_trial=lambda *a:True
  for i in range(7):
   now=10+i*.05;out=g.filter('return',clickpacket(),(-.15,0.),now,odom_pose=(-i*.0075,0),odom_stamp=now,odom_age=0.,measured_velocity=(-.15,0.))
  self.assertEqual(out,(0.,0.));self.assertIn('controller run lease expired',g.fault)
 def test_hold_packet_rejected_by_click_policy_and_reverse(self):
  c=full_config();c['return_start_mode']='click';g=TrialLimiter(ReleasePolicy(c));g.claim_trial=lambda *a:True
  for mode in ['hold','click']:
   c['return_start_mode']=mode;g=TrialLimiter(ReleasePolicy(c));p=clickpacket(activation_mode='hold' if mode=='click' else 'click',held=True)
   self.assertEqual(g.filter('return',p,(-.15,0.),10.,odom_pose=(0,0),odom_stamp=10.,odom_age=0.,measured_velocity=(0,0)),(0.,0.));self.assertTrue(g.fault)
 def test_invalid_policy_mode_disabled(self):
  c=full_config();c['return_start_mode']='typo';self.assertFalse(ReleasePolicy(c).enabled)

class ClickRuntimeTests(unittest.TestCase):
 def test_start_once_cancel_and_late_heartbeat_never_resume(self):
  plan,t,now=make_plan()
  with tempfile.TemporaryDirectory() as tmp:
   r=RawReturnRuntime(plan,conditions(plan),Path(tmp)/'s.json',activation_mode='click')
   self.assertTrue(r.start_run(now,TOKEN));self.assertFalse(r.start_run(now,'2'*32));self.assertEqual(r.run_session,TOKEN)
   self.assertEqual(r.tick(snap(t,now),now,True)['phase'],'running')
   r.stop('cancel');self.assertFalse(r.run_heartbeat(now+.05,TOKEN));self.assertFalse(r.start_run(now+.05,TOKEN))
   self.assertEqual(r.tick(snap(t,now),now,True)['phase'],'stopped')
 def test_controller_stall_cannot_be_revived_by_heartbeat(self):
  plan,t,now=make_plan()
  with tempfile.TemporaryDirectory() as tmp:
   r=RawReturnRuntime(plan,conditions(plan),Path(tmp)/'s.json',activation_mode='click');r.start_run(now,TOKEN)
   self.assertFalse(r.run_heartbeat(now+.251,TOKEN));self.assertFalse(r.run_heartbeat(now+.252,TOKEN));self.assertEqual(r.phase,'stopped')
 def test_wrong_lease_stops_and_plain_hold_does_not_start_click(self):
  plan,t,now=make_plan()
  with tempfile.TemporaryDirectory() as tmp:
   r=RawReturnRuntime(plan,conditions(plan),Path(tmp)/'s.json',activation_mode='click');r.heartbeat(now,True)
   self.assertEqual(r.tick(snap(t,now),now,True)['phase'],'stopped')
  with tempfile.TemporaryDirectory() as tmp:
   r=RawReturnRuntime(plan,conditions(plan),Path(tmp)/'s.json',activation_mode='click');r.start_run(now,TOKEN);self.assertFalse(r.run_heartbeat(now,'2'*32));self.assertEqual(r.phase,'stopped')

if __name__=='__main__':unittest.main(verbosity=2)
