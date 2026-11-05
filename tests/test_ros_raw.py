import unittest,tempfile,time,math,json,copy,subprocess,signal
from pathlib import Path
from dataclasses import replace
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from model import MapDocument
from ros_bridge import Bridge
from control_arbiter_node import ArbiterNode
from raw_odom_mode import SOURCE,scope,key,RawChassisSession
from release_policy import ReleasePolicy,TrialLimiter
from test_full_trial import full_config
import uuid
from legacy_ros_harness import Simulation,Sink,isolation,NS

class RawIntegration(unittest.TestCase):
 def setUp(self):
  isolation();self.tmp=tempfile.TemporaryDirectory(dir='/tmp/return-work');self.root=Path(self.tmp.name)
  (self.root/'map.pgm').write_bytes(b'P5\n3 3\n255\n'+bytes([254]*9));self.map_path=self.root/'map.yaml';self.map_path.write_text('image: map.pgm\nresolution: 1.0\norigin: [0,0,0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')
  args=['--ros-args']
  for t in ['odom','scan','tf','tf_static','map']:args+=['-r','/'+t+':='+NS+'/'+t]
  for t in ['authority','status','legacy_zero','raw_odometry']:args+=['-r','/charging_control/'+t+':='+NS+'/'+t]
  args+=['-r','/navigate_to_pose/_action/status:='+NS+'/nav_status','-r','/navigate_to_pose:='+NS+'/navigate_to_pose','-r','/initialpose:='+NS+'/initialpose']
  self.bridge=Bridge(ros_args=args);trial_config=None
  if self._testMethodName in ('test_one_full_trial_claim_is_persistent_and_cannot_resume','test_feedback025_keeps_target015_and_rejects_excess','test_feedback040_keeps_target015_and_rejects_excess'):
   trial_config=full_config();trial_config['full_trial_authorization']['authorization_id']=uuid.uuid4().hex
  if self._testMethodName=='test_feedback025_keeps_target015_and_rejects_excess':trial_config['feedback_speed_limit_m_s']=.25
  if self._testMethodName=='test_feedback040_keeps_target015_and_rejects_excess':trial_config['feedback_speed_limit_m_s']=.4
  self.test_trial_config=trial_config
  self.arbiter=ArbiterNode(isolated=True,test_return_speed=.15 if self._testMethodName.startswith("test_full015") else .05,test_config=trial_config)
  self.relay_log=(self.root/'relay.log').open('w')
  self.relay=subprocess.Popen(['/tmp/return-candidate/raw-odom-tools/bin/raw_odom_relay','--ros-args','-r','/odom:='+NS+'/odom','-r','/charging_control/raw_odometry:='+NS+'/raw_odometry'],stdout=self.relay_log,stderr=subprocess.STDOUT)
  self.sim=Simulation(self.bridge);self.sim.map_sha=MapDocument.load(self.map_path).fingerprint;self.sink=Sink(self.sim)
  self.behavior=Node('behavior_server',enable_rosout=False);self.pubs=[self.behavior.create_publisher(Twist,NS+'/navigation',1) for _ in range(4)]
  self.ex=SingleThreadedExecutor()
  for n in [self.arbiter,self.sim,self.sink,self.behavior]:self.ex.add_node(n)
  self.spin(3.2);self.bridge.release_stop();self.spin(.5)
  if self.bridge.software_stop:self.bridge.release_stop();self.spin(.4)
  self.assertIsNone(self.arbiter.core.fault)
  self.until(lambda:self.bridge.snapshot().get('raw_stationary_s',0)>=2.1,4.)
  self.assertTrue(self.bridge.snapshot()['arbiter']['raw_odom_ready'])
  state=self.bridge.snapshot();self.plan=self.make_plan(state);self.store=self.root/'map-reference.json';self.store.write_text('retain me')
 def make_plan(self,state):
  last=state['raw_odom'];sc=scope(state);pts=[]
  for i in range(201):
   p=copy.deepcopy(last);p.update(sequence=last['sequence']-240+i,source_stamp=last['source_stamp']-(240-i)*.05,received_at=last['received_at']-(240-i)*.05,pose=[i*.005,0,0],velocity=[.1,0,0]);pts.append(p)
  for i in range(40):
   p=copy.deepcopy(last);p.update(sequence=last['sequence']-39+i,source_stamp=last['source_stamp']-(39-i)*.05,received_at=last['received_at']-(39-i)*.05,pose=[1.,0,0],velocity=[0,0,0]);pts.append(p)
  poses=[]
  for p in reversed(pts):
   if not poses or p['pose']!=poses[-1]:poses.append(p['pose'])
  return dict(source=SOURCE,cycle_id='raw-case',goal_id='return-case',scope=sc,raw_samples=pts,poses=poses,reverse_only=True,executable=False,samples_sha256=key(pts))
 def spin(self,seconds):
  end=time.monotonic()+seconds
  while time.monotonic()<end:self.ex.spin_once(timeout_sec=.003)
 def until(self,p,timeout):
  end=time.monotonic()+timeout
  while time.monotonic()<end:
   self.ex.spin_once(timeout_sec=.003)
   if p():return
  self.fail('timeout '+str(self.bridge.snapshot().get('return_state'))+' '+str(self.arbiter.core.fault)+' '+str(self.bridge.snapshot().get('raw_odom_error')))
 def start(self,short=True):
  confirms={k:True for k in ('operator_present','route_cleared','physical_stop_ready','arm_stowed','exclusive_control_confirmed','no_rear_detection_acknowledged')}
  self.bridge.prepare_return(self.plan,self.store,self.map_path,confirms,short_test=short)
  self.sim.held=True;self.until(lambda:any(v<0 for _,v,w in self.sink.commands),2.)
 def tearDown(self):
  self.sim.gui=False
  for n in [self.behavior,self.sink,self.sim,self.arbiter]:self.ex.remove_node(n);n.destroy_node()
  self.ex.shutdown();self.bridge.close();self.relay.send_signal(signal.SIGINT);self.relay.wait(timeout=4);self.relay_log.close();self.tmp.cleanup()
 def test_feedback040_keeps_target015_and_rejects_excess(self):
  self.assertEqual(self.bridge.snapshot()['arbiter']['feedback_speed_limit_m_s'],.4)
  self.start(short=False);self.until(lambda:any(v<-.14 for _,v,w in self.sink.commands),2.)
  self.sim.feedback_override=lambda:(-.35,0.)
  self.spin(.5);self.assertTrue(self.bridge.return_active());self.assertIsNone(self.arbiter.core.fault)
  self.assertTrue(all(abs(v)<=.15 for _,v,w in self.sink.commands))
  frozen=copy.deepcopy(self.bridge.snapshot()['raw_odom']);self.bridge.return_runtime.check_coordinates=lambda *a,**k:copy.deepcopy(frozen)
  self.sim.feedback_override=lambda:(-.401,0.)
  self.spin(.3);self.assertEqual(self.sink.commands[-1][1:],(0.,0.));self.assertIn('measured return speed exceeded',self.arbiter.core.fault)
 def test_feedback025_keeps_target015_and_rejects_excess(self):
  self.assertEqual(self.bridge.snapshot()['arbiter']['feedback_speed_limit_m_s'],.25)
  self.start(short=False);self.until(lambda:any(v<-.14 for _,v,w in self.sink.commands),2.)
  self.sim.feedback_override=lambda:(-.20,0.)
  self.spin(.5);self.assertTrue(self.bridge.return_active());self.assertIsNone(self.arbiter.core.fault)
  self.assertTrue(all(abs(v)<=.15 for _,v,w in self.sink.commands));self.assertAlmostEqual(self.bridge.snapshot()['raw_odom']['velocity'][0],-.20)
  frozen=copy.deepcopy(self.bridge.snapshot()['raw_odom'])
  self.bridge.return_runtime.check_coordinates=lambda *a,**k:copy.deepcopy(frozen)
  self.sim.feedback_override=lambda:(-.251,0.)
  self.spin(.3);self.assertEqual(self.sink.commands[-1][1:],(0.,0.));self.assertIn('measured return speed exceeded',self.arbiter.core.fault)
 def test_one_full_trial_claim_is_persistent_and_cannot_resume(self):
  self.assertEqual(self.bridge.snapshot()['arbiter']['release_profile'],'supervised_trial')
  self.start(short=False);self.until(lambda:self.arbiter.limiter.full_trial_consumed,1.)
  self.spin(.3);self.sim.held=False;self.bridge.return_heartbeat(False);self.spin(.3)
  authid=self.test_trial_config['full_trial_authorization']['authorization_id']
  ledger=Path('/tmp/return-work/trial-ledger')/(authid+'.json');self.assertTrue(ledger.exists())
  claim=json.loads(ledger.read_text());self.assertEqual(claim['packet']['route_length_m'],1.)
  self.assertFalse(self.bridge.return_active());self.assertEqual(self.sink.commands[-1][1:],(0.,0.))
  self.spin(2.2);self.bridge.release_stop();self.spin(.2)
  with self.assertRaisesRegex(ValueError,'已使用'):self.start(short=False)
  self.ex.remove_node(self.arbiter);self.arbiter.destroy_node()
  self.arbiter=ArbiterNode(isolated=True,test_config=self.test_trial_config);self.ex.add_node(self.arbiter)
  self.assertTrue(self.arbiter.limiter.full_trial_consumed)
  self.spin(.2);self.assertEqual(self.sink.commands[-1][1:],(0.,0.))
 def test_full015_software_stop_remains_effective(self):
  self.start(short=False);self.until(lambda:any(v<-.14 for _,v,w in self.sink.commands),2.)
  self.bridge.emergency_stop();self.spin(.35)
  self.assertEqual(self.sink.commands[-1][1:],(0.,0.));self.assertFalse(self.bridge.return_active());self.assertTrue(self.bridge.software_stop)
 def test_full015_raw_feedback_loss_stops(self):
  self.start(short=False);self.until(lambda:any(v<-.14 for _,v,w in self.sink.commands),2.)
  frozen=copy.deepcopy(self.bridge.snapshot()['raw_odom'])
  self.bridge.return_runtime.check_coordinates=lambda *a,**k:copy.deepcopy(frozen) # Producer fault injection: independent arbiter must stop by itself.
  self.sim.emit_odom=False;self.spin(.65)
  self.assertEqual(self.sink.commands[-1][1:],(0.,0.));self.assertFalse(self.bridge.return_active());self.assertIsNotNone(self.arbiter.core.fault)
 def test_full015_outputs_and_timestamped_diagnostics(self):
  self.assertEqual(self.bridge.snapshot()['arbiter']['return_speed_m_s'],.15)
  self.start(short=False);self.until(lambda:any(v<-.14 for _,v,w in self.sink.commands),2.)
  self.spin(.4);self.sim.held=False;self.bridge.return_heartbeat(False);self.spin(.3)
  self.assertFalse(self.bridge.return_active());self.assertEqual(self.sink.commands[-1][1:],(0.,0.))
  events=[]
  while not self.bridge.events.empty():events.append(self.bridge.events.get())
  kinds={e['event'] for e in events}
  self.assertTrue({'return_hold_changed','return_authority_published','return_arbiter_output','return_raw_feedback','return_finished'}<=kinds,kinds)
  outputs=[e for e in events if e['event']=='return_arbiter_output']
  self.assertTrue(any(e['status']['output_v_m_s']<-.14 for e in outputs))
  seq=[e['status']['output_sequence'] for e in outputs];self.assertEqual(seq,sorted(set(seq)))
  self.assertTrue(all(0<=e['received_monotonic_s']-e['status']['output_monotonic_s']<.25 for e in outputs))
  self.assertTrue(any(e.get('held') is False for e in events if e['event']=='return_hold_changed'))
  self.assertTrue(any(e.get('software_stop') is True for e in events if e['event']=='return_finished'))
 def test_low_speed_quantized_pulses_cross_old_gate_without_raw_fault(self):
  counter=[0]
  def feedback():
   counter[0]+=1
   return .042,(.09 if counter[0]%2 else -.09)
  self.sim.feedback_override=feedback;self.spin(.8)
  self.assertIsNone(self.bridge.snapshot()['raw_odom_error'])
  self.assertTrue(self.bridge.snapshot()['arbiter']['raw_odom_ready'])
  self.assertIsNone(self.arbiter.core.fault)
  self.assertGreater(counter[0],8)
  self.assertIsNone(self.bridge.raw_tracker.rejection)
 def test_first_rejected_sample_visible_in_both_ros_consumers(self):
  self.sim.feedback_override=lambda:(.042,.4)
  self.until(lambda:self.bridge.snapshot().get('raw_odom_error'),1.)
  state=self.bridge.snapshot();self.assertIn('绝对角速度',state['raw_odom_error'])
  rejection=state['raw_odom_rejection'];self.assertEqual(rejection['input']['velocity'],[.042,0.,.4])
  self.until(lambda:self.arbiter.raw_tracker.rejection is not None,1.)
  self.assertEqual(self.arbiter.raw_tracker.rejection['input']['velocity'],[.042,0.,.4])
  self.assertIsNotNone(self.arbiter.core.fault)
  events=[]
  while not self.bridge.events.empty():events.append(self.bridge.events.get())
  self.assertTrue(any(e['event']=='raw_odometry_rejected' and e['rejection']['input']['velocity']==[.042,0.,.4] for e in events))
 def test_map_jump_and_tf_loss_do_not_replace_raw_feedback(self):
  self.start();self.sim.shift=100.;self.sim.emit_tf=False;self.spin(.5)
  self.assertTrue(any(v<0 for _,v,w in self.sink.commands[-5:]));self.assertIsNone(self.arbiter.core.fault)
  self.sim.held=False;self.bridge.return_heartbeat(False);self.spin(.2);self.assertEqual(self.sink.commands[-1][1:],(0.,0.))
 def test_hold_release_latches(self):
  self.start();self.sim.held=False;self.bridge.return_heartbeat(False);self.spin(.2);self.sim.held=True;self.spin(.3);self.assertEqual(self.sink.commands[-1][1:],(0.,0.));self.assertTrue(self.bridge.software_stop)
 def test_software_stop_remains_effective(self):
  self.start();self.bridge.emergency_stop();self.spin(.3);self.assertEqual(self.sink.commands[-1][1:],(0.,0.));self.assertTrue(self.bridge.software_stop)
 def test_raw_loss_stops_independently(self):
  self.start();frozen=copy.deepcopy(self.bridge.snapshot()['raw_odom'])
  self.bridge.return_runtime.check_coordinates=lambda *a,**k:copy.deepcopy(frozen) # Fault injection: producer ignores stale input.
  self.sim.emit_odom=False;self.spin(.5);self.assertEqual(self.sink.commands[-1][1:],(0.,0.));self.assertIsNotNone(self.arbiter.core.fault)
 def test_raw_reset_jump_stops(self):
  self.start();self.sim.x=0.;self.spin(.3);self.assertEqual(self.sink.commands[-1][1:],(0.,0.));self.assertIsNotNone(self.arbiter.core.fault)
 def test_gui_loss_stops(self):
  self.start();self.sim.gui=False;self.spin(.5);self.assertEqual(self.sink.commands[-1][1:],(0.,0.))
 def test_navigation_velocity_reacquisition_stops(self):
  self.start();m=Twist();m.linear.x=.05;self.sim.nav.publish(m);self.spin(.25);self.assertEqual(self.sink.commands[-1][1:],(0.,0.));self.assertIsNotNone(self.arbiter.core.fault)
 def test_second_raw_publisher_rejected(self):
  self.start();from nav_msgs.msg import Odometry
  bad=Node('other_odom',enable_rosout=False);pub=bad.create_publisher(Odometry,NS+'/odom',1)
  try:self.spin(.4);self.assertEqual(self.sink.commands[-1][1:],(0.,0.));self.assertIsNotNone(self.arbiter.core.fault)
  finally:bad.destroy_node()
 def test_arbiter_restart_invalidates_scope_and_never_resumes(self):
  self.start();self.ex.remove_node(self.arbiter);self.arbiter.destroy_node();self.arbiter=ArbiterNode(isolated=True);self.ex.add_node(self.arbiter);self.spin(.6);self.assertTrue(self.bridge.software_stop);self.assertEqual(self.sink.commands[-1][1:],(0.,0.))
 def test_independent_short_time_limit_survives_modified_producer(self):
  self.arbiter.policy=ReleasePolicy({'profile':'short_only'},isolated_test=True);self.arbiter.limiter=TrialLimiter(self.arbiter.policy);self.spin(.15);self.start()
  self.bridge.return_runtime.c=replace(self.bridge.return_runtime.c,max_time_s=999,max_distance_m=999)
  self.until(lambda:self.arbiter.limiter.fault is not None,4.);self.assertIn('independent short trial limit',self.arbiter.limiter.fault);self.spin(.1);self.assertEqual(self.sink.commands[-1][1:],(0.,0.))
 def test_independent_short_distance_bound(self):
  self.arbiter.policy=ReleasePolicy({'profile':'short_only'},isolated_test=True);self.arbiter.limiter=TrialLimiter(self.arbiter.policy);self.spin(.15);self.start();self.sim.travel_multiplier=2.
  self.bridge.return_runtime.c=replace(self.bridge.return_runtime.c,max_time_s=999,max_distance_m=999)
  self.until(lambda:self.arbiter.limiter.fault is not None,3.);self.assertIn('independent short trial limit',self.arbiter.limiter.fault)
 def test_full_synthetic_return_writes_scoped_receipt_only(self):
  self.start(False);self.until(lambda:self.bridge.snapshot()['return_state'].get('phase')=='complete',30.);self.assertEqual(self.store.read_text(),'retain me');self.assertTrue((self.root/'raw-return-raw-case.json').exists())
 def test_new_recording_uses_all_raw_samples(self):
  s=self.bridge.snapshot();session=RawChassisSession('nav-map',self.store,self.root,s,time.monotonic(),self.bridge.raw_samples_since)
  with self.bridge.lock:self.bridge.busy=True;self.bridge.navigation_id=session.goal_id
  cmd=Twist();cmd.linear.x=.1;timer=self.sim.create_timer(.05,lambda:self.sim.nav.publish(cmd))
  for i in range(15):self.spin(.1);session.update(self.bridge.snapshot(),'map',time.monotonic())
  self.sim.destroy_timer(timer)
  with self.bridge.lock:self.bridge.busy=False
  done=time.monotonic()
  for i in range(30):
   self.spin(.1);s=self.bridge.snapshot();s.update(navigation_id=session.goal_id,goal_status=4,goal_finished_at=done);session.update(s,'map',time.monotonic())
  self.assertEqual(session.phase,'outbound_stopped',session.status());self.assertGreater(len(session.samples),50)
  self.assertGreater(math.dist(session.samples[0]['pose'][:2],session.samples[-1]['pose'][:2]),.1)
 def test_native_relay_restart_latches_without_resume(self):
  self.start();self.relay.send_signal(signal.SIGINT);self.relay.wait(timeout=4)
  self.relay=subprocess.Popen(['/tmp/return-candidate/raw-odom-tools/bin/raw_odom_relay','--ros-args','-r','/odom:='+NS+'/odom','-r','/charging_control/raw_odometry:='+NS+'/raw_odometry'],stdout=self.relay_log,stderr=subprocess.STDOUT)
  self.spin(.7);self.assertEqual(self.sink.commands[-1][1:],(0.,0.));self.assertIsNotNone(self.arbiter.core.fault);self.assertTrue(self.bridge.software_stop)
if __name__=='__main__':unittest.main(verbosity=2)
