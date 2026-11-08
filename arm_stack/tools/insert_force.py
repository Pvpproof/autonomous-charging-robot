#!/usr/bin/env python3
"""Experimental contact-monitored insertion. No automatic lateral correction."""
from insertion_limits import checked_depth_mm
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
from insertion_safety import FeedbackGuard, TorqueDetector, ContactDetected, wait_future, stop_and_confirm
from insertion_geometry import check_tcp, segment_samples
from moveit_msgs.srv import GetStateValidity
from scipy.spatial.transform import Rotation
from gun_pose_validation import tcp_from_link6
import re
ROOT=Path('/opt/charging-robot-local/charging_socket_vision/results/realtime_runs')
def atomic(p,d):
 t=p.with_suffix('.tmp');t.write_text(json.dumps(d,ensure_ascii=False,indent=2));t.replace(p)
def angles(r):
 if r is None or r.res!=0:raise RuntimeError('反馈服务失败')
 a=[float(x) for x in re.findall(r'[-+]?(?:\d+(?:\.\d*)?|\.\d+)',r.robot_return)]
 if len(a)!=6 or not np.isfinite(a).all():raise RuntimeError('关节反馈无效')
 return np.radians(a)
def segment_positions(start,end,max_mm=2):
 return max(1,int(math.ceil(np.linalg.norm(np.asarray(end)-start)*1000/max_mm)))

class Runner(Node):
 def __init__(self):
  super().__init__('contact_insertion');self.active=None;self.pending_goal=None;self.phase='idle';self.step=0;self.monitoring=False
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
  self.phase='endpoint_wait';deadline=time.monotonic()+3;consecutive=0;err=None;future=None;previous=None
  client=self.robot_clients['GetAngle']
  try:
   while time.monotonic()<deadline:
    self.check_feedback()
    if baseline is not None:self.check(baseline,limits)
    if future is None:future=client.call_async(GetAngle.Request())
    rclpy.spin_once(self,timeout_sec=min(.02,max(0,deadline-time.monotonic())))
    self.check_feedback()
    if baseline is not None:self.check(baseline,limits)
    if time.monotonic()>=deadline:break
    if not future.done():continue
    actual=angles(future.result());future=None;err=np.degrees(actual-target)
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
 def run(self):
  data=json.loads((ROOT/'pending_insertion_trajectory.json').read_text());names=data['joint_names'];pts=data['points']
  from prepare_insertion_target import prepare
  prepare()
  source=json.loads((ROOT/'last_dispatched_trajectory.json').read_text())
  if data.get('source_gun_plan_id')!=source.get('plan_id') or data.get('cycle_id')!=source.get('cycle_id'):raise RuntimeError('插入轨迹不属于本轮已验收gun预备位')
  if data.get('path_validation',{}).get('method')!='cartesian_full_path_joint_interpolation_samples':raise RuntimeError('请重新insert-plan生成直线路径')
  self.mode();q=self.current()
  if max(abs(q-np.array(pts[0]['positions'])))>math.radians(.2):raise RuntimeError('当前关节不在插入起点，请先重新到gun预备位并insert-plan')
  T0=self.pose(q,names);axis=T0[:3,2];origin=T0[:3,3]
  link=self.pose(q,names,raw=True)
  depth=checked_depth_mm(data['insertion_depth_mm'])
  selected=[q];n=int(math.ceil(depth/2));quat=Rotation.from_matrix(link[:3,:3]).as_quat()
  for i in range(1,n+1):
   req=GetPositionIK.Request();r=req.ik_request;r.group_name='nova2_group';r.ik_link_name='Link6';r.avoid_collisions=True;r.timeout.sec=1
   r.robot_state.joint_state.name=names;r.robot_state.joint_state.position=list(selected[-1])
   r.pose_stamped.header.frame_id='base_link';p=r.pose_stamped.pose
   xyz=link[:3,3]+axis*(depth*i/n/1000)
   p.position.x,p.position.y,p.position.z=map(float,xyz)
   p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w=map(float,quat)
   res=self.call(self.ik,req)
   if res.error_code.val!=1:raise RuntimeError('第%d步IK/碰撞检查失败'%i)
   lookup=dict(zip(res.solution.joint_state.name,res.solution.joint_state.position));qi=np.array([lookup[k] for k in names])
   if qi.shape!=(6,) or not np.isfinite(qi).all():raise RuntimeError('IK返回无效关节值')
   if max(abs(qi-selected[-1]))>math.radians(3):raise RuntimeError('IK关节跳变')
   T=self.pose(qi,names);target=origin+axis*(depth*i/n/1000)
   if np.linalg.norm(T[:3,3]-target)>.001:raise RuntimeError('直线目标FK误差超1mm')
   if Rotation.from_matrix(T0[:3,:3].T@T[:3,:3]).magnitude()>math.radians(.5):raise RuntimeError('姿态FK误差超0.5度')
   validity=self.create_client(GetStateValidity,'/check_state_validity')
   try:
    for sample in segment_samples(selected[-1],qi):
     check_tcp(self.pose(sample,names),origin,axis,T0[:3,:3],depth/1000)
     vr=GetStateValidity.Request();vr.group_name='nova2_group';vr.robot_state.joint_state.name=names;vr.robot_state.joint_state.position=list(sample)
     if not self.call(validity,vr).valid:raise RuntimeError('小步插补采样碰撞/约束不满足')
   finally:self.destroy_client(validity)
   selected.append(qi)
  progress=depth/1000
  print('计划轴向推进 %.1fmm，共%d小步；接触触发后停止，不自动横向修正。'%(progress*1000,len(selected)-1),flush=True)
  if '--execute' not in sys.argv:print('只读检查完成。执行需 charge_robot insert-force-go');return
  if not sys.stdin.isatty() or input('确认插入区无人、插口未锁扣且急停可触及；按Enter执行，其他取消：').strip():return
  # Require the updated adapter and a complete fresh baseline before dispatch.
  ready=time.monotonic()+1.0
  while time.monotonic()<ready:rclpy.spin_once(self,timeout_sec=.02)
  self.guard.idle();self.mode();self.events=[];self.phase='baseline'
  baseline,limits=self.guard.baseline()
  self.monitoring=True
  print('原始扭矩阈值（非牛顿）',limits.round(3).tolist(),flush=True)
  if not self.action.server_is_ready():raise RuntimeError('轨迹action不可用')
  if max(abs(self.current()-q))>math.radians(.1):raise RuntimeError('确认/基线期间位置改变')
  visited=[selected[0].tolist()];self.save_path(data,visited)
  report={'stage':'insertion','executor':'contact_steps','status':'partial','retracted':False,'baseline':baseline.tolist(),'limits':limits.tolist(),'motion_commands_sent':False,'insertion_axis':data.get('insertion_axis')}
  atomic(ROOT/'latest_insertion_execution_feedback.json',report)
  try:
   for i,target in enumerate(selected[1:],1):
    self.step=i;self.phase="precheck"
    self.check(baseline,limits);self.mode();actual=self.current()
    if max(abs(actual-np.array(visited[-1])))>math.radians(.2):raise RuntimeError('实际位置与上一步不一致')
    traj=JointTrajectory();traj.joint_names=names
    for j,positions in enumerate([actual,target]):
     p=JointTrajectoryPoint();p.positions=list(positions);p.time_from_start.nanosec=j*400000000;traj.points.append(p)
    g=FollowJointTrajectory.Goal();g.trajectory=traj
    report.update(status='uncertain',motion_commands_sent=True,commanded_step=i,commanded_target_rad=target.tolist(),commanded_start_rad=actual.tolist());atomic(ROOT/'latest_insertion_execution_feedback.json',report)
    self.phase='moving'
    self.pending_goal=self.action.send_goal_async(g)
    h=self.checked_wait(self.pending_goal,5)
    if not h.accepted:raise RuntimeError('动作未接受')
    self.active=h;self.pending_goal=None;rf=self.active.get_result_async();deadline=time.monotonic()+8
    while not rf.done():
     rclpy.spin_once(self,timeout_sec=.02);self.check(baseline,limits)
     if time.monotonic()>deadline:raise RuntimeError('小步执行超时')
    out=rf.result();self.active=None
    if out.status!=4 or out.result.error_code!=0:raise RuntimeError('小步执行失败')
    actual=self.wait_endpoint(target,baseline,limits,report)
    visited.append(actual.tolist());self.save_path(data,visited)
    self.phase='settling';settle_until=time.monotonic()+.5
    while time.monotonic()<settle_until:
     rclpy.spin_once(self,timeout_sec=.02);self.check(baseline,limits)
    report.update(status='partial',completed_steps=i);atomic(ROOT/'latest_insertion_execution_feedback.json',report)
    print('推进 %d/%d'%(i,len(selected)-1),flush=True)
   report['status']='succeeded'
  except BaseException as e:
   report.update(status='uncertain',reason=str(e),stop_method='cancel_and_stop_confirmed')
   try:
    evidence=self.stop();actual=np.asarray(evidence['joint_rad']);report['stop_evidence']=evidence
    # Save actual measured stop even if it is between two commanded waypoints.
    if max(abs(actual-np.array(visited[-1])))>1e-8:visited.append(actual.tolist())
    self.save_path(data,visited)
    report.update(status='contact_stopped' if isinstance(e,ContactDetected) else 'stopped',
                  final_joint_rad=actual.tolist(),robot_mode=evidence['robot_mode'])
    print('[已请求停止并确认静止] 未继续推进；实际停止位置已保存。',flush=True)
   except BaseException as failure:report['stop_failure']=str(failure)
   if isinstance(e,ContactDetected) and report['status']=='contact_stopped':return 3
   raise
  finally:
   report['samples']=self.events;report['detector']={'window_s':.15,'persistence_s':.35,'settle_s':.5,'baseline_policy':'fixed_no_rezero','timer_policy':'per_joint','force_calibrated':False};report['finished_at']=time.time();atomic(ROOT/'latest_insertion_execution_feedback.json',report)
  print('分步轨迹完成；这不代表电气连接或锁止已确认。')
def main():
 rclpy.init();n=Runner()
 try:return n.run() or 0
 except (Exception,KeyboardInterrupt) as e:print('INSERT_FORCE_STOPPED:',e);return 2
 finally:n.destroy_node();rclpy.shutdown()
if __name__=='__main__':sys.exit(main())
