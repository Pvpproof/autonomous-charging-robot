import copy,json,math,tempfile,unittest
from pathlib import Path
from test_raw_mode import identity,snap,update,make_plan,conditions
from raw_odom_mode import *

class CurvatureTests(unittest.TestCase):
 def tracker(self):
  t=RawTracker();t.bind(identity());return t
 def session(self,tmp):
  t=self.tracker();now=10.
  for i in range(45):now+=.05;update(t,now)
  return t,now,RawChassisSession('metadata',Path(tmp)/'map-reference.json',tmp,snap(t,now),now,t.since)
 def test_quantized_low_speed_pulses_record_and_certify_without_changing_return_limits(self):
  # Synthetic stress beyond the old single-frame gate, not a claim that the
  # unavailable first rejected field sample had this exact angular velocity.
  with tempfile.TemporaryDirectory() as tmp:
   t,now,session=self.session(tmp);p=[0.,0.,0.]
   for i in range(120):
    v=.042;w=.09 if i%2==0 else -.09;now+=.05
    p[0]+=v*math.cos(p[2])*.05;p[1]+=v*math.sin(p[2])*.05;p[2]+=w*.05
    self.assertGreater(abs(w),abs(v)/1.78+.05)
    update(t,now,p,(v,0,w));session.update(snap(t,now),'same',now)
    self.assertEqual(session.phase,'outbound')
   end=now
   for i in range(46):
    now+=.05;update(t,now,p);s=snap(t,now);s.update(goal_status=4,navigation_id=session.goal_id,goal_finished_at=end);session.update(s,'same',now)
   self.assertEqual(session.phase,'outbound_stopped');plan=session.return_candidate();check_raw_plan(plan)
   rt=RawReturnRuntime(plan,conditions(plan,True),Path(tmp)/'map-reference.json');self.assertEqual(rt.c.speed_limit_m_s,.05)
 def test_all_intervals_cone_matches_bruteforce_with_wrap_and_switching(self):
  poses=[];p=[0.,0.,3.13]
  for i in range(200):
   v=.05;w=.02*math.sin(i/10);p=[p[0]+v*.05*math.cos(p[2]),p[1]+v*.05*math.sin(p[2]),wrap(p[2]+w*.05)];poses.append(p)
  cone=HeadingCone();s=[0.];h=[0.]
  for i,p in enumerate(poses):
   if i:s.append(s[-1]+distance(p,poses[i-1]));h.append(h[-1]+wrap(p[2]-poses[i-1][2]))
   bad=any(abs(h[i]-h[j])>(s[i]-s[j])/1.78+.015+1e-9 for j in range(i))
   self.assertEqual(cone.update(p) is not None,bad)
 def test_tight_curve_persistent_spin_and_heading_noise_cannot_spend_budget_repeatedly(self):
  for v,w in [(0.,.09),(.042,.09),(.1,.1)]:
   cone=HeadingCone();p=[0.,0.,0.];cone.update(p);failure=None
   for i in range(500):
    p=[p[0]+v*.05*math.cos(p[2]),p[1]+v*.05*math.sin(p[2]),p[2]+w*.05];failure=cone.update(p)
    if failure:break
   self.assertIsNotNone(failure,(v,w));self.assertGreater(failure['excess_rad'],.015)
 def test_exact_minimum_radius_and_stationary_quantization_are_accepted(self):
  for radius in [1.78,3.,-1.78]:
   cone=HeadingCone()
   for p in __import__('offline_reverse').forward_path(lambda s:1/radius,total=2.,ds=.002):self.assertIsNone(cone.update(p))
  cone=HeadingCone()
  for i in range(1000):self.assertIsNone(cone.update((0,0,.00015*(i%2))))
 def test_route_gate_rejects_tight_arc_even_after_valid_hash(self):
  plan,_,_=make_plan(curve=1.,length=.3)
  with self.assertRaisesRegex(ValueError,'累计航向'):check_raw_plan(plan)
 def test_geometry_rejection_is_saved_and_first_reason_survives_new_request(self):
  with tempfile.TemporaryDirectory() as tmp:
   t,now,session=self.session(tmp);p=[0.,0.,0.]
   for i in range(30):
    now+=.05;p[0]+=.042*.05;p[2]+=.09*.05;update(t,now,p,(.042,0,.09));session.update(snap(t,now),'same',now)
    if session.phase=='blocked':break
   self.assertEqual(session.phase,'blocked');why=session.block_reason
   receipt=json.loads((session.directory/'first-rejection.json').read_text());self.assertEqual(receipt['geometry']['code'],'path_heading_cone');self.assertEqual(receipt['input']['velocity'],[.042,0,.09])
   self.assertEqual(receipt['input'],json.loads((session.directory/'actual-raw-trace.jsonl').read_text().splitlines()[-1]))
   session.stop('new request',now);session.stop_or_take_over('another request');session.save(now)
   self.assertEqual(json.loads((session.directory/'cycle.json').read_text())['block_reason'],why)
 def test_raw_rejection_retains_first_full_sample_and_is_valid_json_for_nan(self):
  for pose,v,reason in [((0,0,0),(.601,0,0),'纵向'),((0,0,0),(0,.011,0),'侧向'),((0,0,0),(0,0,.4),'绝对角'),((float('nan'),0,0),(0,0,0),'无效')]:
   t=self.tracker();update(t,1.)
   with self.assertRaisesRegex(ValueError,reason):update(t,1.05,pose,v)
   first=copy.deepcopy(t.rejection);json.dumps(first,allow_nan=False)
   self.assertEqual(first['input']['velocity'],list(v));self.assertEqual(first['last_valid']['source_stamp'],1.)
   with self.assertRaises(ValueError):update(t,1.1,(1,0,0),(0,0,0))
   self.assertEqual(t.rejection,first);t.reset();self.assertIsNone(t.rejection)
 def test_tracker_rejection_is_persisted_in_session_even_when_current_sample_missing(self):
  with tempfile.TemporaryDirectory() as tmp:
   t,now,session=self.session(tmp)
   with self.assertRaises(ValueError):update(t,now+.05,(2,0,0),(0,0,0))
   session.update({'raw_odom':None,'raw_odom_error':t.error,'raw_odom_rejection':t.rejection},'same',now+.05)
   evidence=json.loads((session.directory/'first-rejection.json').read_text());self.assertEqual(evidence['input']['pose'],[2,0,0]);self.assertIn('跳变',evidence['reason'])
 def test_return_sustained_uncommanded_yaw_still_stops(self):
  plan,t,now=make_plan()
  with tempfile.TemporaryDirectory() as tmp:
   rt=RawReturnRuntime(plan,conditions(plan),Path(tmp)/'reference.json');rt.heartbeat(now,True);rt.tick(snap(t,now),now,True);p=list(plan['poses'][0])
   for i in range(10):
    now+=.05;p[2]+=.005;update(t,now,p,(0,0,.1));rt.heartbeat(now,True);state=rt.tick(snap(t,now),now,True)
    if state['phase']=='stopped':break
   self.assertEqual(state['phase'],'stopped');self.assertIn('累计航向',state['reason'])
if __name__=='__main__':unittest.main(verbosity=2)
