import copy,json,math,tempfile,unittest
from pathlib import Path
from raw_odom_mode import *
from supervised_draft import TestConditions
from offline_reverse import forward_path

def identity(gid='g'):return dict(boot_id='b',driver=dict(pid=1,start_ticks='1'),publisher_gid=gid,topic='/odom',source_kind=SOURCE)
def snap(tracker,now,epoch='arb-epoch'):
 return dict(raw_odom=tracker.current(now,now),raw_odom_error=None,ros_now=now,control_owner='panel',arbiter=dict(raw_odom_ready=True,raw_source_key=tracker.source_key,raw_epoch=epoch,instance_id='arb'),raw_stationary_s=now-tracker.still_since if tracker.still_since is not None else 0.,raw_stationary_since=tracker.still_since,raw_stationary_samples=tracker.still_samples,scan_age=0.,busy=False,software_stop=False,manual=False,cancel_requested=False,docking_hold=False)
def update(t,now,p=(0.,0.,0.),v=(0.,0.,0.),**kw):return t.update(p,v,now,now,now,kw.get('frame','odom_combined'),'base_footprint',kw.get('gid','g'))
def make_plan(curve=0.,length=1.):
 t=RawTracker();t.bind(identity());samples=[];now=10.;path=forward_path(lambda s:curve,total=length,ds=.005)
 for i,p in enumerate(path):
  now+=.05;samples.append(update(t,now,p,(.1,0.,curve*.1)))
 for i in range(45):now+=.05;samples.append(update(t,now,path[-1]))
 ss=snap(t,now);scope_id=scope(ss);poses=[]
 for p in reversed(samples):
  if not poses or p['pose']!=poses[-1]:poses.append(p['pose'])
 return dict(source=SOURCE,cycle_id='c',goal_id='r',scope=scope_id,raw_samples=samples,poses=poses,reverse_only=True,executable=False,samples_sha256=key(samples)),t,now

def conditions(plan,short=False):
 return TestConditions(plan['cycle_id'],plan['goal_id'],True,True,True,True,True,True,.05,1.78,3. if short else 60.,.15 if short else 3.)
class RawTests(unittest.TestCase):
 def test_source_frame_scope_not_filtered_tf(self):
  t=RawTracker();t.bind(identity());update(t,1.)
  self.assertEqual(t.latest['published_frame'],'odom_combined');self.assertEqual(t.latest['source'],SOURCE)
  with self.assertRaisesRegex(ValueError,'发布端'):update(t,1.05,gid='other')
 def test_source_rebind_latches_and_explicit_reset_changes_epoch(self):
  t=RawTracker();t.bind(identity());update(t,1.);old=t.epoch;t.bind(identity('new'))
  with self.assertRaises(ValueError):t.current(1.,1.)
  t.reset();t.update((0,0,0),(0,0,0),2,2,2,'odom_combined','base_footprint','new');self.assertNotEqual(old,t.epoch)
 def test_timestamp_gap_repeat_and_clock_future_latch(self):
  for dt in [0.,-.01,.251]:
   t=RawTracker();t.bind(identity());update(t,1.)
   with self.assertRaises(ValueError):update(t,1+dt)
   with self.assertRaises(ValueError):t.current(2,2)
  t=RawTracker();t.bind(identity())
  with self.assertRaises(ValueError):t.update((0,0,0),(0,0,0),2,1,1,'odom_combined','base_footprint','g')
 def test_pose_reset_yaw_reset_and_frame_invalid(self):
  for pose,frame in [((2,0,0),'odom_combined'),((0,0,.05),'odom_combined'),((0,0,0),'odom')]:
   t=RawTracker();t.bind(identity());update(t,1.)
   with self.assertRaises(ValueError):update(t,1.05,pose,frame=frame)
 def test_numeric_and_velocity_fail_safe(self):
  for pose,v in [((float('nan'),0,0),(0,0,0)),((0,0,0),(.8,0,0)),((0,0,0),(0,.02,0)),((0,0,0),(0,0,.4))]:
   t=RawTracker();t.bind(identity())
   with self.assertRaises(ValueError):update(t,1.,pose,v)
 def test_map_jump_missing_map_and_localizer_restart_do_not_gate_raw(self):
  plan,t,now=make_plan();s=snap(t,now);s.update(pose=(100,100,2),map_sha256='changed',localization_epoch='new-map',pose_age=999,alignment_age=999)
  with tempfile.TemporaryDirectory() as tmp:
   rt=RawReturnRuntime(plan,conditions(plan),Path(tmp)/'map-start.json');rt.check_coordinates(s,now,True)
 def test_raw_stop_difference_and_epoch_remain_hard_gates(self):
  plan,t,now=make_plan()
  with tempfile.TemporaryDirectory() as tmp:
   rt=RawReturnRuntime(plan,conditions(plan),Path(tmp)/'map-start.json');s=snap(t,now);s['raw_odom']['pose'][0]+=.11
   with self.assertRaisesRegex(ValueError,'停点'):rt.check_coordinates(s,now,True)
   s=snap(t,now,epoch='restarted')
   with self.assertRaisesRegex(ValueError,'会话'):rt.check_coordinates(s,now,True)
 def test_plan_tampering_and_missing_raw_samples_reject(self):
  plan,t,now=make_plan();check_raw_plan(plan)
  for modify in [lambda p:p['poses'][0].__setitem__(1,5),lambda p:p['raw_samples'].pop(2),lambda p:p.update(source='actual_map_pose_measurements')]:
   p=copy.deepcopy(plan);modify(p)
   with self.assertRaises(ValueError):check_raw_plan(p)
 def test_full_closed_loop_arcs_and_save_raw_only(self):
  for curve in [0.,1/3,-1/3]:
   plan,t,now=make_plan(curve)
   with tempfile.TemporaryDirectory() as tmp:
    original=Path(tmp)/'map-start.json';original.write_text('do not overwrite')
    rt=RawReturnRuntime(plan,conditions(plan),original);p=list(plan['poses'][0]);rt.heartbeat(now,True);state=rt.tick(snap(t,now),now,True)
    for i in range(1400):
     v,w=state['v'],state['w'];now+=.05;old=p[:];p[0]+=v*math.cos(p[2])*.05;p[1]+=v*math.sin(p[2])*.05;p[2]=wrap(p[2]+w*.05)
     update(t,now,p,(v,0,w));rt.heartbeat(now,True);state=rt.tick(snap(t,now),now,True)
     if state['phase'] in ('complete','stopped'):break
    self.assertEqual(state['phase'],'complete',state);self.assertLess(distance(p,plan['poses'][-1]),.04);self.assertEqual(original.read_text(),'do not overwrite');self.assertTrue(rt.receipt.exists())
 def test_short_limit_hold_and_loss_stop_without_receipt(self):
  for case in ('short','hold','stale','direction','no-progress'):
   plan,t,now=make_plan()
   with tempfile.TemporaryDirectory() as tmp:
    rt=RawReturnRuntime(plan,conditions(plan,case=='short'),Path(tmp)/'start.json');rt.heartbeat(now,True);s=snap(t,now);state=rt.tick(s,now,True)
    for i in range(100):
     now+=.05;update(t,now,plan['poses'][0]);rt.heartbeat(now,True);s=snap(t,now)
     if case=='hold':rt.heartbeat(now,False)
     if case=='stale':s['raw_odom']['received_at']-=1.
     if case=='direction':s['raw_odom']['velocity'][0]=.02
     if case=='short':rt.c=conditions(plan,True);rt.last_progress=now # Isolate time bound from no-progress guard.
     state=rt.tick(s,now,True)
     if state['phase']=='stopped':break
    self.assertEqual(state['phase'],'stopped',state);self.assertFalse(rt.receipt.exists())
 def test_record_all_raw_samples_no_map_reference_dependency(self):
  with tempfile.TemporaryDirectory() as tmp:
   t=RawTracker();t.bind(identity());now=10.
   for i in range(45):now+=.05;update(t,now)
   session=RawChassisSession('map-only-metadata',Path(tmp)/'nonexistent-map-reference.json',Path(tmp),snap(t,now),now,t.since)
   for i in range(60):
    now+=.05;update(t,now,(.005*(i+1),0,0),(.1,0,0))
    if i%4==3:session.update(snap(t,now),'arbitrary-map-change',now)
   finished=now
   for i in range(50):
    now+=.05;update(t,now,(.3,0,0));s=snap(t,now);s.update(goal_status=4,navigation_id=session.goal_id,goal_finished_at=finished);session.update(s,'changed',now)
    if session.phase=='outbound_stopped':break
   self.assertEqual(session.phase,'outbound_stopped',session.status());p=session.return_candidate();check_raw_plan(p);self.assertGreater(len(p['raw_samples']),90)
   trace=[json.loads(x) for x in (session.directory/'actual-raw-trace.jsonl').read_text().splitlines()];self.assertEqual(trace,p['raw_samples']);self.assertFalse((Path(tmp)/'nonexistent-map-reference.json').exists())
if __name__=='__main__':unittest.main(verbosity=2)
