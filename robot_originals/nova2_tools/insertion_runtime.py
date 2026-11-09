#!/usr/bin/env python3
"""Shared ordinary insertion/retreat motion and stop support."""
import json, math, time, sys, statistics
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from dobot_msgs_v4.srv import RobotMode, GetAngle, Stop
from std_msgs.msg import String
from insertion_safety import FeedbackGuard, wait_future, stop_and_confirm
from insertion_geometry import check_tcp, segment_samples
from moveit_msgs.srv import GetStateValidity
from scipy.spatial.transform import Rotation
from gun_pose_validation import tcp_from_link6
import re
ROOT=Path('/home/nvidia/charging_socket_vision/results/realtime_runs')
def atomic(p,d):
 t=p.with_suffix('.tmp');t.write_text(json.dumps(d,ensure_ascii=False,indent=2));t.replace(p)
def safe_report(p,d):
 try:atomic(p,d)
 except BaseException as e:
  try:print('REPORT_WRITE_FAILED '+repr(e)+' '+json.dumps(d,ensure_ascii=False),file=sys.stderr,flush=True)
  except BaseException:pass
def angles(r):
 if r is None or r.res!=0:raise RuntimeError('反馈服务失败')
 a=[float(x) for x in re.findall(r'[-+]?(?:\d+(?:\.\d*)?|\.\d+)',r.robot_return)]
 if len(a)!=6 or not np.isfinite(a).all():raise RuntimeError('关节反馈无效')
 return np.radians(a)
def segment_positions(start,end,max_mm=2):
 return max(1,int(math.ceil(np.linalg.norm(np.asarray(end)-start)*1000/max_mm)))

class Runner(Node):
 def __init__(self):
  super().__init__('ordinary_insertion');self.active=None;self.pending_goal=None;self.phase='idle';self.step=0;self.monitoring=False
  self.guard=FeedbackGuard(self)
  self.action=ActionClient(self,FollowJointTrajectory,'/nova2_group_controller/follow_joint_trajectory')
  self.robot_clients={name:self.create_client(typ,'/dobot_bringup_ros2/srv/'+name) for name,typ in [('GetAngle',GetAngle),('RobotMode',RobotMode),('Stop',Stop)]}
  self.fk=self.create_client(GetPositionFK,'/compute_fk');self.ik=self.create_client(GetPositionIK,'/compute_ik')
 @property
 def feed(self):return self.guard.feed
 @property
 def stamp(self):return self.guard.stamp
 @property
 def history(self):return self.guard.history
 @history.setter
 def history(self,v):self.guard.history=v
 @property
 def events(self):return self.guard.samples
 @events.setter
 def events(self,v):self.guard.samples=v
 def checked_wait(self,future,timeout):
  return wait_future(self,future,timeout,self.guard.check if self.monitoring else None)
 def call(self,c,r):
  deadline=time.monotonic()+15
  while not c.service_is_ready():
   if self.monitoring:self.guard.check()
   if time.monotonic()>deadline:raise RuntimeError('服务不可用 '+c.srv_name)
   rclpy.spin_once(self,timeout_sec=.02)
  f=c.call_async(r)
  return self.checked_wait(f,15 if c.srv_name.endswith('/compute_cartesian_path') else 5)

 def current(self):return angles(self.call(self.robot_clients['GetAngle'],GetAngle.Request()))
 def mode(self):
  r=self.call(self.robot_clients['RobotMode'],RobotMode.Request())
  if r.res!=0 or r.robot_return.strip()!='{5}':raise RuntimeError('需要RobotMode=5，当前 '+r.robot_return)
 def pose(self,q,names,raw=False):
  req=GetPositionFK.Request();req.header.frame_id='base_link';req.fk_link_names=['Link6'];req.robot_state.joint_state.name=names;req.robot_state.joint_state.position=list(q)
  r=self.call(self.fk,req)
  if r.error_code.val!=1 or len(r.pose_stamped)!=1:raise RuntimeError('FK失败')
  p=r.pose_stamped[0].pose;T=np.eye(4);T[:3,3]=[p.position.x,p.position.y,p.position.z];T[:3,:3]=Rotation.from_quat([p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w]).as_matrix();return T if raw else tcp_from_link6(T)
 def check(self,baseline=None,limits=None):self.guard.check()
 def stop(self):
  self.monitoring=False;self.phase='stopping'
  evidence=stop_and_confirm(self,self.guard,self.active,self.pending_goal)
  self.active=None;self.pending_goal=None
  return evidence
 def check_feedback(self):self.guard.check(False)

 def wait_endpoint(self,target,baseline,limits,report):
  # A single absolute deadline includes RPC time. Keep monitoring while RPC is pending.
  self.phase='endpoint_wait';deadline=time.monotonic()+3;consecutive=0;err=None;future=None;previous=None;last_source=None
  client=self.robot_clients['GetAngle']
  try:
   while time.monotonic()<deadline:
    self.guard.check()
    if baseline is not None:self.check(baseline,limits)
    if future is None:future=client.call_async(GetAngle.Request())
    rclpy.spin_once(self,timeout_sec=min(.02,max(0,deadline-time.monotonic())))
    self.guard.check()
    if baseline is not None:self.check(baseline,limits)
    if time.monotonic()>=deadline:break
    if not future.done():continue
    actual=angles(future.result());future=None
    token=self.guard.source.token
    if token==last_source:continue
    last_source=token;err=np.degrees(actual-target)
    report['endpoint_check']={'step':self.step,'target_deg':np.degrees(target).tolist(),'actual_deg':np.degrees(actual).tolist(),'error_deg':err.tolist(),'max_error_deg':float(max(abs(err)))}
    stable=previous is not None and max(abs(actual-previous))<=math.radians(.02)
    consecutive=consecutive+1 if max(abs(err))<=.1 and stable else 0
    previous=actual
    if consecutive>=3:return actual
   maximum=float(max(abs(err))) if err is not None else None
   detail='无有效关节响应' if maximum is None else '最大误差%.3f度'%maximum
   raise RuntimeError('实际关节到位等待超时：步骤%d，%s（要求0.1度且连续稳定，等待上限3秒）'%(self.step,detail))
  finally:
   if future is not None and not future.done():future.cancel()

 def save_path(self,data,visited):
  p=dict(data);p['points']=[]
  for i,q in enumerate(visited):p['points'].append({'positions':list(q),'velocities':[0.]*6,'accelerations':[0.]*6,'time_from_start':{'sec':i,'nanosec':0}})
  atomic(ROOT/'last_inserted_trajectory.json',p)
