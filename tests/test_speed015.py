import unittest,tempfile,math,copy
from pathlib import Path
from dataclasses import replace
from charging_robot.returning.release_policy import ReleasePolicy, TrialLimiter
from charging_robot.control.control_arbiter import Arbiter
from charging_robot.returning.raw_odom_mode import RawReturnRuntime, distance, wrap
from test_release_policy import config,packet
from test_raw_mode import make_plan,conditions,snap,update

class Speed015Tests(unittest.TestCase):
 def test_explicit_speed_selection_and_no_short_or_unaccepted_full_bypass(self):
  c=config('supervised_route');c['return_speed_m_s']=.15
  p=ReleasePolicy(c);self.assertFalse(p.enabled);self.assertTrue(p.errors)
  self.assertTrue(ReleasePolicy(c,isolated_test=True).enabled)
  c=config();c['return_speed_m_s']=.15;self.assertFalse(ReleasePolicy(c).enabled)
  for bad in [.3,0.,True,float('nan')]:
   c=config('supervised_route');c['return_speed_m_s']=bad;self.assertFalse(ReleasePolicy(c,isolated_test=True).enabled)
 def test_independent_speed_envelope_scales_and_still_rejects_overspeed(self):
  c=config('supervised_route');c['return_speed_m_s']=.15
  for command,measured,allowed in [((-.15,0),(-.15,0),True),((-.151,0),(-.15,0),False),((-.15,.09),(-.15,0),False),((-.15,0),(-.181,0),False),((.05,0),(-.15,0),False)]:
   g=TrialLimiter(ReleasePolicy(c,isolated_test=True))
   out=g.filter('return',packet(short_test=False),command,10.,odom_pose=(0.,0.),odom_stamp=10.,odom_age=0.,measured_velocity=measured)
   self.assertEqual(out==command and g.fault is None,allowed)
 def test_speed015_actual_closed_loop_straight_arcs_and_hold_stop(self):
  for curve in [0,1/3,-1/3]:
   plan,t,now=make_plan(curve=curve)
   with tempfile.TemporaryDirectory() as tmp:
    c=replace(conditions(plan),speed_limit_m_s=.15);rt=RawReturnRuntime(plan,c,Path(tmp)/'map.json');p=list(plan['poses'][0]);peak=0.
    for i in range(600):
     rt.heartbeat(now,True);state=rt.tick(snap(t,now),now,True);peak=max(peak,abs(state['v']))
     if state['phase'] in ['stopped','complete']:break
     v,w=state['v'],state['w'];now+=.05;p[0]+=v*math.cos(p[2])*.05;p[1]+=v*math.sin(p[2])*.05;p[2]=wrap(p[2]+w*.05);update(t,now,p,(v,0,w))
    self.assertEqual(state['phase'],'complete',state);self.assertAlmostEqual(peak,.15);self.assertLess(distance(p,plan['poses'][-1]),.04)
 def test_speed015_loss_hold_and_no_progress_still_stop(self):
  for cause in ['hold','stale','no_progress','measured_speed']:
   plan,t,now=make_plan()
   with tempfile.TemporaryDirectory() as tmp:
    rt=RawReturnRuntime(plan,replace(conditions(plan),speed_limit_m_s=.15),Path(tmp)/'map.json');state={}
    for i in range(60):
     now+=.05;update(t,now,plan['poses'][0]);rt.heartbeat(now,True);s=snap(t,now)
     if cause=='hold':rt.heartbeat(now,False)
     if cause=='stale':s['raw_odom']['received_at']-=1
     if cause=='measured_speed':s['raw_odom']['velocity'][0]=-.181
     state=rt.tick(s,now,True)
     if state['phase']=='stopped':break
    self.assertEqual(state['phase'],'stopped',state);self.assertEqual(state['v'],0.)
if __name__=='__main__':unittest.main(verbosity=2)
