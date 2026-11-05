"""ROS service/state reset orchestration with explicit synthetic driver contract."""
import json,time,uuid,unittest
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger
from test_ros_oneclick import OneClickIntegration
from legacy_ros_harness import NS

class SyntheticDriverWatchdog(Node):
 def __init__(self):
  super().__init__('charging_watchdog_test',enable_rosout=False)
  self.session=uuid.uuid4().hex;self.seq=0;self.latched=True;self.reason='startup_reset_required';self.reject=False;self.stale=False;self.ack_wrong_session=False;self.emit=True;self.resets=0
  self.pub=self.create_publisher(String,NS+'/driver/command_watchdog_state',1)
  self.service=self.create_service(Trigger,NS+'/driver/reset_command_watchdog',self.reset)
  self.create_timer(.04,self.tick)
 def data(self,event='state'):
  self.seq+=1
  return dict(version=2,session=self.session,state_seq=self.seq,latched=self.latched,reason=self.reason,event=event,monotonic_ns=time.monotonic_ns()-(int(1e9) if self.stale else 0),command_age_s=.01,feedback_age_s=.01)
 def tick(self):
  if self.emit:
   m=String();m.data=json.dumps(self.data());self.pub.publish(m)
 def reset(self,request,response):
  self.resets+=1
  if not self.reject:self.latched=False;self.reason='ready'
  response.success=not self.reject;data=self.data('reset_ack' if response.success else 'reset_rejected')
  if self.ack_wrong_session:data['session']='f'*32
  response.message=json.dumps(data);return response

class DriverResetIntegration(OneClickIntegration):
 def setUp(self):
  super().setUp()
  self.arbiter.init_driver_watchdog(True)
  self.driver=SyntheticDriverWatchdog();self.ex.add_node(self.driver);self.spin(2.4)
  self.assertTrue(self.arbiter.graph_ready);self.assertFalse(self.arbiter.driver_watchdog.motion_ready(time.monotonic()))
 def tearDown(self):
  self.ex.remove_node(self.driver);self.driver.destroy_node();super().tearDown()
 def test_driver_reset_complete_chain_and_click_cancel(self):
  self.assertTrue(self.bridge.software_stop);self.bridge.release_stop()
  self.assertTrue(self.bridge.software_stop)
  self.until(lambda:not self.bridge.software_stop,2.)
  self.assertEqual(self.driver.resets,1);self.assertIsNone(self.arbiter.core.fault)
  self.assertTrue(self.arbiter.driver_watchdog.motion_ready(time.monotonic()))
  self.start();self.bridge.stop_return('cancel');self.stopped()
 def test_driver_reset_rejection_keeps_lock(self):
  self.driver.reject=True;self.bridge.release_stop();self.spin(.5)
  self.assertTrue(self.bridge.software_stop);self.assertIsNotNone(self.arbiter.core.fault);self.assertEqual(self.sink.commands[-1][1:],(0.,0.));self.assertIsNone(self.bridge.stop_reset_pending)
 def test_driver_reset_ack_old_session_never_unlocks(self):
  self.driver.ack_wrong_session=True;self.bridge.release_stop();self.spin(.5)
  self.assertTrue(self.bridge.software_stop);self.assertIn('ACK invalid',self.arbiter.core.fault);self.assertEqual(self.sink.commands[-1][1:],(0.,0.))
 def test_driver_source_state_expiry_blocks_motion_and_no_recovery(self):
  self.bridge.release_stop();self.until(lambda:not self.bridge.software_stop,2.);self.start()
  self.driver.stale=True;self.stopped();self.driver.stale=False;self.spin(.5)
  self.assertTrue(self.bridge.software_stop);self.assertEqual(self.sink.commands[-1][1:],(0.,0.))
 def test_driver_reset_ack_without_new_state_never_unlocks(self):
  self.driver.emit=False;self.bridge.release_stop();self.spin(.15)
  self.assertTrue(self.bridge.software_stop);self.assertIsNotNone(self.bridge.stop_reset_pending)
  self.spin(.45);self.assertTrue(self.bridge.software_stop);self.assertEqual(self.sink.commands[-1][1:],(0.,0.))
 def test_driver_watchdog_second_status_publisher_rejected(self):
  bad=Node('rogue_watchdog',enable_rosout=False);pub=bad.create_publisher(String,NS+'/driver/command_watchdog_state',1)
  try:
   self.spin(.3);self.assertFalse(self.arbiter.graph_ready);self.assertTrue(self.bridge.software_stop)
   self.assertEqual(self.sink.commands[-1][1:],(0.,0.))
  finally:bad.destroy_node()
 def test_driver_reset_cancelled_by_emergency_stays_locked(self):
  self.bridge.release_stop();self.bridge.emergency_stop();self.spin(.5)
  self.assertTrue(self.bridge.software_stop);self.assertEqual(self.sink.commands[-1][1:],(0.,0.));self.assertIsNone(self.bridge.stop_reset_pending)

def load_tests(loader,tests,pattern):
 return unittest.TestSuite(DriverResetIntegration(name) for name in loader.getTestCaseNames(DriverResetIntegration) if name.startswith('test_driver_'))
if __name__=='__main__':unittest.main(verbosity=2)
