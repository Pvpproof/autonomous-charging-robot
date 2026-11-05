"""Real Bridge/TF/ROS/arbiter, synthetic vehicle, private network/device namespace."""
import json
import math
import os
from pathlib import Path
import socket
import tempfile
import time
import unittest
import glob
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage
from action_msgs.msg import GoalStatusArray
from control_arbiter_node import ArbiterNode
from ros_bridge import Bridge
from model import MapDocument
from return_map import saved_grid_digest
from test_return import fixture

NS='/charging_return_isolated'

def isolation():
    try:
        host_namespace = Path('/tmp/return-work/host_netns.txt').read_text().strip()
    except OSError as error:
        raise RuntimeError('Private test namespace setup is required before ROS integration tests') from error
    if not host_namespace or os.readlink('/proc/self/ns/net') == host_namespace:
        raise RuntimeError('Integration tests require a network namespace separate from the host')
    if {name for _,name in socket.if_nameindex()} != {'lo'}:
        raise RuntimeError('Integration tests require loopback-only networking')
    if any(glob.glob(p) for p in ('/dev/ychd*','/dev/ttyUSB*','/dev/ttyACM*')):
        raise RuntimeError('Robot serial devices must be absent from the private test namespace')


class Simulation(Node):
    def __init__(self,bridge):
        super().__init__('controller_server',enable_rosout=False)
        self.bridge=bridge;self.x=1.;self.v=self.w=0.;self.y=self.yaw=0.
        self.held=False;self.gui=True;self.emit_odom=True;self.emit_tf=True
        self.shift=0.;self.active=False;self.last=time.monotonic();self.commands=[]
        self.odom=self.create_publisher(Odometry,NS+'/odom',qos_profile_sensor_data)
        self.scan=self.create_publisher(LaserScan,NS+'/scan',qos_profile_sensor_data)
        self.tf=self.create_publisher(TFMessage,NS+'/tf',10)
        self.map=self.create_publisher(OccupancyGrid,NS+'/map',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.status=self.create_publisher(GoalStatusArray,NS+'/nav_status',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.nav=self.create_publisher(Twist,NS+'/navigation',1)
        self.create_timer(.05,self.tick)
    def tick(self):
        now=time.monotonic();dt=now-self.last;self.last=now
        self.x+=self.v*math.cos(self.yaw)*dt;self.y+=self.v*math.sin(self.yaw)*dt;self.yaw+=self.w*dt
        if self.gui:
            self.bridge.gui_heartbeat(self.map_sha)
            if self.held:self.bridge.return_heartbeat(True)
        stamp=self.get_clock().now().to_msg()
        if self.emit_odom:
            m=Odometry();m.header.stamp=stamp;m.header.frame_id='odom_combined';m.child_frame_id='base_footprint'
            m.twist.twist.linear.x=self.v;m.twist.twist.angular.z=self.w;self.odom.publish(m)
        scan=LaserScan();scan.header.stamp=stamp;scan.header.frame_id='laser';self.scan.publish(scan)
        if self.emit_tf:
            transforms=[]
            for parent,child,x,y,yaw in [('map','odom_combined',self.shift,0.,0.),
                    ('odom_combined','base_footprint',self.x,self.y,self.yaw)]:
                tf=TransformStamped();tf.header.stamp=stamp;tf.header.frame_id=parent;tf.child_frame_id=child
                tf.transform.translation.x=x;tf.transform.translation.y=y
                tf.transform.rotation.z=math.sin(yaw/2);tf.transform.rotation.w=math.cos(yaw/2);transforms.append(tf)
            self.tf.publish(TFMessage(transforms=transforms))
        grid=OccupancyGrid();grid.header.frame_id='map';grid.header.stamp=stamp
        grid.info.width=grid.info.height=3;grid.info.resolution=1.;grid.info.origin.orientation.w=1.;grid.data=[0]*9
        self.map.publish(grid)
        status=GoalStatusArray()
        if self.active:
            from action_msgs.msg import GoalStatus
            item=GoalStatus();item.status=2;status.status_list=[item]
        self.status.publish(status)


class Sink(Node):
    def __init__(self,simulation):
        super().__init__('charging_return_test_sink',enable_rosout=False)
        self.sim=simulation;self.commands=[]
        self.create_subscription(Twist,NS+'/chassis',self.command,1)
    def command(self,msg):
        self.sim.v,self.sim.w=msg.linear.x,msg.angular.z
        self.commands.append((time.monotonic(),msg.linear.x,msg.angular.z))


class Integration(unittest.TestCase):
    def setUp(self):
        isolation()
        self.tmp=tempfile.TemporaryDirectory(dir='/tmp/return-work')
        self.root=Path(self.tmp.name)
        (self.root/'map.pgm').write_bytes(b'P5\n3 3\n255\n'+bytes([254]*9))
        self.map_path=self.root/'map.yaml';self.map_path.write_text('image: map.pgm\nresolution: 1.0\norigin: [0, 0, 0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\nmode: trinary\n')
        topics=['odom','scan','tf','tf_static','map']
        args=['--ros-args']
        for topic in topics:args+=['-r','/'+topic+':='+NS+'/'+topic]
        for topic in ['authority','status','legacy_zero']:args+=['-r','/charging_control/'+topic+':='+NS+'/'+topic]
        args+=['-r','/navigate_to_pose/_action/status:='+NS+'/nav_status',
               '-r','/navigate_to_pose:='+NS+'/navigate_to_pose','-r','/initialpose:='+NS+'/initialpose']
        self.bridge=Bridge(ros_args=args)
        self.arbiter=ArbiterNode(isolated=True)
        self.sim=Simulation(self.bridge);self.sim.map_sha=MapDocument.load(self.map_path).fingerprint
        self.sink=Sink(self.sim)
        self.executor=SingleThreadedExecutor()
        for node in [self.arbiter,self.sim,self.sink]:self.executor.add_node(node)
        self.spin(2.8)
        self.assertTrue(self.arbiter.graph_ok())
        self.bridge.release_stop();self.spin(.35)
        # A delayed pre-reset status is allowed to keep the panel stopped; acknowledge once more.
        if self.bridge.software_stop:self.bridge.release_stop();self.spin(.2)
        self.assertIsNone(self.arbiter.core.fault)
        with self.bridge.lock:
            self.bridge.goal_finished_at=time.monotonic()-3.
            self.bridge.goal_status=4
        self.plan=fixture();self.plan['map_sha256']=self.sim.map_sha
        self.plan['localization_epoch']=self.bridge.localization_epoch
        map_grid_sha=saved_grid_digest(self.map_path)
        for point in self.plan['coordinate_samples']:point['map_grid_sha']=map_grid_sha
        self.spin(.1)
        self.store=self.root/'start.json'
    def tearDown(self):
        self.sim.gui=False
        for node in [self.sink,self.sim,self.arbiter]:
            self.executor.remove_node(node);node.destroy_node()
        self.executor.shutdown();self.bridge.close();self.tmp.cleanup()
    def spin(self,seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:self.executor.spin_once(timeout_sec=.005)
    def until(self,predicate,timeout):
        end=time.monotonic()+timeout
        while time.monotonic()<end:
            self.executor.spin_once(timeout_sec=.005)
            if predicate():return
        self.fail('timeout: '+str(self.bridge.snapshot().get('return_state'))+' '+str(self.arbiter.core.fault))
    def start(self,short=True):
        confirms={k:True for k in ['operator_present','route_cleared','physical_stop_ready','arm_stowed',
            'exclusive_control_confirmed','no_rear_detection_acknowledged']}
        self.bridge.prepare_return(self.plan,self.store,self.map_path,confirms,short_test=short)
        self.assertIsNone(self.bridge.goal_status)
        self.assertIsNone(self.bridge.goal_target)
        self.sim.held=True
        self.until(lambda:any(v<0 for _,v,w in self.sink.commands),2.)
        self.assertFalse(self.bridge.get_publishers_info_by_topic('/cmd_vel'))
    def test_short_bound_stops_and_does_not_save(self):
        self.start();self.until(lambda:self.bridge.snapshot()['return_state'].get('phase')=='stopped',4.)
        self.spin(.15);self.assertEqual(self.sink.commands[-1][1:],(0.,0.));self.assertFalse(self.store.exists())
    def test_full_return_and_actual_stop_saved(self):
        self.start(False);self.until(lambda:self.bridge.snapshot()['return_state'].get('phase')=='complete',28.)
        self.assertTrue(self.store.exists());self.assertLess(abs(self.sim.x),.05)
    def test_gui_heartbeat_loss(self):
        self.start();at=time.monotonic();self.sim.gui=False
        self.until(lambda:self.bridge.snapshot()['return_state'].get('phase')=='stopped',1.)
        self.spin(.1);self.assertEqual(self.sink.commands[-1][1:],(0.,0.));self.assertLess(time.monotonic()-at,.65)
    def test_hold_release_latches(self):
        self.start();self.sim.held=False;self.bridge.return_heartbeat(False)
        self.spin(.25);self.assertEqual(self.sink.commands[-1][1:],(0.,0.))
        self.sim.held=True;self.spin(.3);self.assertEqual(self.sink.commands[-1][1:],(0.,0.))
    def test_localization_shift(self):
        self.start();self.sim.shift=.69
        self.until(lambda:self.bridge.snapshot()['return_state'].get('phase')=='stopped',1.)
        self.assertFalse(self.store.exists())
    def test_odometry_loss(self):
        self.start();self.sim.emit_odom=False
        self.until(lambda:self.arbiter.core.fault is not None or self.bridge.snapshot()['return_state'].get('phase')=='stopped',1.)
        self.spin(.1);self.assertEqual(self.sink.commands[-1][1:],(0.,0.))
    def test_tf_loss(self):
        self.start();self.sim.emit_tf=False
        self.until(lambda:self.bridge.snapshot()['return_state'].get('phase')=='stopped',1.)
    def test_navigation_reacquires_during_return(self):
        self.start();self.sim.active=True
        self.until(lambda:self.arbiter.core.fault is not None or self.bridge.snapshot()['return_state'].get('phase')=='stopped',1.)
        self.spin(.1);self.assertEqual(self.sink.commands[-1][1:],(0.,0.))
    def test_competing_authority_publisher(self):
        self.start();bad=Node('unexpected_panel',enable_rosout=False)
        from std_msgs.msg import String
        pub=bad.create_publisher(String,NS+'/authority',1)
        try:
            self.until(lambda:self.arbiter.core.fault is not None or self.bridge.snapshot()['return_state'].get('phase')=='stopped',1.)
            self.spin(.1);self.assertEqual(self.sink.commands[-1][1:],(0.,0.))
        finally:bad.destroy_node()
    def test_software_stop_requests_zero_and_latches(self):
        self.start();self.bridge.emergency_stop()
        self.until(lambda:self.bridge.snapshot()['return_state'].get('phase')=='stopped',1.)
        self.spin(.15);self.assertEqual(self.sink.commands[-1][1:],(0.,0.))
        self.assertTrue(self.bridge.software_stop);self.assertFalse(self.store.exists())
    def test_uncommissioned_node_never_drives(self):
        node=ArbiterNode(isolated=False)
        try:
            self.assertFalse(node.commissioned)
            node.core.mode='return';node.tick()
        finally:node.destroy_node()

    def test_map_file_change(self):
        self.start();self.map_path.write_text(self.map_path.read_text()+'\n# changed\n')
        self.until(lambda:self.bridge.snapshot()['return_state'].get('phase')=='stopped',1.)
        self.assertFalse(self.store.exists())


if __name__=='__main__':unittest.main(verbosity=2)
