"""Real ROS/Bridge/arbiter + synthetic chassis in mandatory isolated namespaces."""
import unittest,time,copy,uuid,json,signal
from geometry_msgs.msg import Twist
from rclpy.node import Node
from test_ros_raw import RawIntegration
from test_full_trial import full_config
from legacy_ros_harness import NS

class OneClickIntegration(RawIntegration):
 def make_test_config(self):
  c=full_config();c['return_start_mode']='click';c['feedback_speed_limit_m_s']=.4
  c['full_trial_authorization'].update(authorization_id=uuid.uuid4().hex,max_route_m=4.)
  return c
 def start(self,short=False):
  confirms={k:True for k in ('operator_present','route_cleared','physical_stop_ready','arm_stowed','exclusive_control_confirmed','no_rear_detection_acknowledged')}
  self.bridge.prepare_return(self.plan,self.store,self.map_path,confirms,short_test=False)
  self.spin(.2);self.assertTrue(all(v==0 and w==0 for _,v,w in self.sink.commands[-3:]))
  self.sim.run_session=self.bridge.start_return()
  self.until(lambda:any(v<-.14 for _,v,w in self.sink.commands),2.)
 def stopped(self):
  self.spin(.4);self.assertEqual(self.sink.commands[-1][1:],(0.,0.));self.assertFalse(self.bridge.return_active());self.assertTrue(self.bridge.software_stop)
 def test_click_duplicate_cancel_does_not_restart(self):
  self.start();token=self.sim.run_session;self.assertEqual(self.bridge.start_return(),token)
  self.spin(.3);self.assertTrue(self.bridge.return_active());self.assertFalse(self.bridge.return_held)
  self.bridge.stop_return('operator cancel');self.stopped();self.spin(.4);self.assertEqual(self.sink.commands[-1][1:],(0.,0.))
  with self.assertRaises(ValueError):self.bridge.start_return()
 def test_click_gui_lease_freeze_independently_stops(self):
  self.start();runtime=self.bridge.return_runtime
  with self.bridge.lock:
   runtime.run_heartbeat=lambda *a:True
   runtime.tick_original=runtime.tick
   def bypass_lease(*a,**k):
    runtime.heartbeat_at=a[1];return runtime.tick_original(*a,**k)
   runtime.tick=bypass_lease
   self.sim.keepalive=False
  self.stopped();self.assertIn('GUI run lease expired',self.arbiter.core.fault or 'NO ARBITER FAULT '+str(self.bridge.snapshot()['return_state']))
  self.sim.keepalive=True;self.spin(.3);self.assertEqual(self.sink.commands[-1][1:],(0.,0.))
 def test_click_gui_loss_and_recovery_never_resume(self):
  self.start();self.sim.gui=False;self.stopped();self.sim.gui=True;self.spin(.4);self.assertEqual(self.sink.commands[-1][1:],(0.,0.))
 def test_click_red_software_stop(self):
  self.start();self.bridge.emergency_stop();self.stopped()
 def test_click_raw_loss_independent_and_recovery(self):
  self.start();frozen=copy.deepcopy(self.bridge.snapshot()['raw_odom']);self.bridge.return_runtime.check_coordinates=lambda *a,**k:copy.deepcopy(frozen)
  self.sim.emit_odom=False;self.stopped();self.assertIsNotNone(self.arbiter.core.fault)
  self.sim.emit_odom=True;self.spin(.4);self.assertEqual(self.sink.commands[-1][1:],(0.,0.))
 def test_click_raw_jump(self):
  self.start();self.sim.x-=1.;self.stopped();self.assertIsNotNone(self.arbiter.core.fault)
 def test_click_no_progress(self):
  self.start();self.sim.travel_multiplier=0.
  self.until(lambda:self.bridge.snapshot()['return_state'].get('phase')=='stopped',3.)
  self.assertIn('无原始里程进展',self.bridge.snapshot()['return_state']['reason']);self.stopped()
 def test_click_nav_competition(self):
  self.start();m=Twist();m.linear.x=.05;self.sim.nav.publish(m);self.stopped();self.assertIn('navigation',self.arbiter.core.fault)
 def test_click_graph_competition(self):
  self.start();bad=Node('rogue_owner',enable_rosout=False);pub=bad.create_publisher(Twist,NS+'/chassis',1)
  try:self.stopped();self.assertIn('graph',self.arbiter.core.fault)
  finally:bad.destroy_node()
 def test_click_arbiter_restart_never_resume(self):
  self.start();self.ex.remove_node(self.arbiter);self.arbiter.destroy_node()
  from charging_robot.control.control_arbiter_node import ArbiterNode
  self.arbiter=ArbiterNode(isolated=True,test_config=self.test_trial_config);self.ex.add_node(self.arbiter)
  self.stopped();self.assertTrue(self.arbiter.limiter.full_trial_consumed)
 def test_click_relay_restart_never_resume(self):
  self.start();self.relay.send_signal(signal.SIGINT);self.relay.wait(timeout=4);self.stopped()
 def test_click_full_return_completes_and_never_restarts(self):
  self.start();self.until(lambda:self.bridge.snapshot()['return_state'].get('phase')=='complete',15.)
  self.assertEqual(self.sink.commands[-1][1:],(0.,0.));self.assertTrue((self.root/'raw-return-raw-case.json').exists());self.assertEqual(self.store.read_text(),'retain me')
  self.spin(.4);self.assertEqual(self.sink.commands[-1][1:],(0.,0.))
  with self.assertRaises(ValueError):self.bridge.start_return()

def load_tests(loader,tests,pattern):
 return unittest.TestSuite(OneClickIntegration(name) for name in loader.getTestCaseNames(OneClickIntegration) if name.startswith('test_click_'))
if __name__=='__main__':unittest.main(verbosity=2)
