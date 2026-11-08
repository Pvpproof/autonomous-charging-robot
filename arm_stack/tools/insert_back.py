#!/usr/bin/env python3
"""Position-only Cartesian retreat; no torque/source-frame estimator."""
import sys,math,time,json
import numpy as np
import rclpy
from scipy.spatial.transform import Rotation
from moveit_msgs.srv import GetPositionFK,GetPositionIK,GetStateValidity
from insertion_geometry import check_tcp,segment_samples
from gun_pose_validation import tcp_from_link6
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory,JointTrajectoryPoint
from dobot_msgs_v4.srv import RobotMode
from insert_execute import E,ROOT,atomic

class Retreat(E):
 def __init__(self):
  super().__init__()
  self.fk=self.create_client(GetPositionFK,'/compute_fk')
  self.ik=self.create_client(GetPositionIK,'/compute_ik')
  self.validity=self.create_client(GetStateValidity,'/check_state_validity')
 def call(self,client,request):
  if not client.wait_for_service(timeout_sec=15):raise RuntimeError('服务不可用 '+client.srv_name)
  for attempt in range(2):
   future=client.call_async(request)
   try:return self.wait(future,5)
   except RuntimeError as error:
    future.cancel()
    if attempt==0:
     print('[重试回退规划查询] '+client.srv_name,flush=True)
     continue
    raise RuntimeError('回退规划查询超时 '+client.srv_name+'；未发送回退运动') from error
 def current(self):return np.asarray(self.joints())
 def pose(self,q,names,raw=False):
  req=GetPositionFK.Request();req.header.frame_id='base_link';req.fk_link_names=['Link6']
  req.robot_state.joint_state.name=names;req.robot_state.joint_state.position=list(q)
  out=self.call(self.fk,req)
  if out.error_code.val!=1 or len(out.pose_stamped)!=1:raise RuntimeError('回退FK失败')
  p=out.pose_stamped[0].pose;T=np.eye(4);T[:3,3]=[p.position.x,p.position.y,p.position.z]
  T[:3,:3]=Rotation.from_quat([p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w]).as_matrix()
  return T if raw else tcp_from_link6(T)

def plan(n,d,current):
 names=d['joint_names'];home=np.array(d['points'][0]['positions'],float)
 start=n.pose(home,names);now=n.pose(current,names);axis=start[:3,2]
 offset=now[:3,3]-start[:3,3];depth=float(offset@axis)
 lateral=np.linalg.norm(offset-axis*depth)
 angle=Rotation.from_matrix(start[:3,:3].T@now[:3,:3]).magnitude()
 if lateral>.0015 or angle>math.radians(.5) or depth<-.001 or depth>.085:
  raise RuntimeError('当前位置不在原插入轴线上，拒绝自动回退：轴向%.2fmm 横向%.2fmm 姿态%.2f度'%(depth*1000,lateral*1000,math.degrees(angle)))
 count=max(1,int(math.ceil(max(depth,0)/.002)));path=[current]
 link=n.pose(current,names,raw=True);endlink=n.pose(home,names,raw=True);quat=Rotation.from_matrix(endlink[:3,:3]).as_quat()
 for i in range(1,count+1):
  f=i/count;xyz=link[:3,3]*(1-f)+endlink[:3,3]*f
  req=GetPositionIK.Request();r=req.ik_request;r.group_name='nova2_group';r.ik_link_name='Link6';r.avoid_collisions=True;r.timeout.sec=1
  r.robot_state.joint_state.name=names;r.robot_state.joint_state.position=list(path[-1]);r.pose_stamped.header.frame_id='base_link';p=r.pose_stamped.pose
  p.position.x,p.position.y,p.position.z=map(float,xyz);p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w=map(float,quat)
  out=n.call(n.ik,req)
  if out.error_code.val!=1:raise RuntimeError('回退IK/碰撞检查失败')
  vals=dict(zip(out.solution.joint_state.name,out.solution.joint_state.position));q=np.array([vals[k] for k in names])
  if q.shape!=(6,) or not np.isfinite(q).all():raise RuntimeError('回退IK返回无效关节值')
  if max(abs(q-path[-1]))>math.radians(3):raise RuntimeError('回退IK关节跳变')
  T=n.pose(q,names);desired=now[:3,3]*(1-f)+start[:3,3]*f
  if np.linalg.norm(T[:3,3]-desired)>.001 or Rotation.from_matrix(start[:3,:3].T@T[:3,:3]).magnitude()>math.radians(.5):raise RuntimeError('回退FK检查失败')
  vclient=n.validity
  for qi in segment_samples(path[-1],q):
   check_tcp(n.pose(qi,names),start[:3,3],axis,start[:3,:3],max(depth,0))
   vr=GetStateValidity.Request();vr.group_name='nova2_group';vr.robot_state.joint_state.name=names;vr.robot_state.joint_state.position=list(qi)
   if not n.call(vclient,vr).valid:raise RuntimeError('回退插补采样状态无效')
  path.append(q)
 return path,depth

def read_mode(n):return int(n.read(n.mode,RobotMode.Request()).strip().strip('{}'))

def trajectory(n,d,path):
 traj=JointTrajectory();traj.joint_names=d['joint_names'];elapsed=0.
 previous=None;previous_pose=None
 for q in path:
  pose=n.pose(q,d['joint_names'])
  if previous is not None:
   elapsed+=max(.05,float(np.linalg.norm(pose[:3,3]-previous_pose[:3,3]))/.005,
                float(max(abs(q-previous)))/math.radians(10))
  p=JointTrajectoryPoint();p.positions=list(q)
  p.time_from_start.sec,p.time_from_start.nanosec=divmod(round(elapsed*1e9),1000000000)
  traj.points.append(p);previous=q;previous_pose=pose
 return traj

def run(n,confirmed=False):
 print('[回退准备] 读取RobotMode/GetAngle并检查原轴线退离路径（此阶段不运动）',flush=True)
 src=ROOT/'last_inserted_trajectory.json'
 saved=src.read_text();d=json.loads(saved)
 if read_mode(n)!=5:raise RuntimeError('回退需要静止使能状态5；不会自动停止、清报警或恢复暂停')
 current=n.current();path,depth=plan(n,d,current)
 print('普通位置回退 %.2f mm，到插入前预备位；规划速度不超过5mm/s，无力控。'%(depth*1000),flush=True)
 if '--check' in sys.argv:return
 if not confirmed and (not sys.stdin.isatty() or input('确认插口锁扣已解除、退回路径安全；按Enter回退，其他取消：').strip()):return
 if read_mode(n)!=5 or max(abs(n.current()-current))>math.radians(.1) or src.read_text()!=saved:
  raise RuntimeError('确认期间位置或插入记录已改变；请重新回退规划')
 traj=trajectory(n,d,path)
 if not n.action.wait_for_server(timeout_sec=10):raise RuntimeError('FollowJointTrajectory action不可用')
 report=ROOT/'latest_insertion_execution_feedback.json'
 data=json.loads(report.read_text()) if report.exists() else {}
 data.update(retracted=False,retraction_status='uncertain',retraction_method='position_cartesian',retreat_contact_monitoring=False)
 atomic(report,data)
 try:
  if depth>.0001:
   g=FollowJointTrajectory.Goal();g.trajectory=traj;n.submitted=True

   try:n.active=n.wait(n.action.send_goal_async(g),10)
   except RuntimeError as error:raise RuntimeError('回退Action接受响应超时；可能已接收，禁止重复提交') from error
   if not n.active.accepted:
    n.active=None;n.submitted=False;raise RuntimeError('回退轨迹被拒绝；未发送回退运动')
   try:out=n.wait(n.active.get_result_async(),180)
   except RuntimeError as error:raise RuntimeError('回退Action执行结果超时；已进入取消/停止处理') from error
   if out.status!=4 or out.result.error_code!=0:raise RuntimeError('回退执行失败 '+out.result.error_string)
  deadline=time.monotonic()+5;stable=0;home=np.array(d['points'][0]['positions'])
  while time.monotonic()<deadline:
   actual=n.current();error=max(abs(np.degrees(actual-home)))
   stable=stable+1 if error<=.1 and read_mode(n)==5 else 0
   if stable>=3:break
   rclpy.spin_once(n,timeout_sec=.1)
  else:raise RuntimeError('回退执行结束，但未确认实际到位')
  target=n.pose(home,d['joint_names']);actual_pose=n.pose(actual,d['joint_names'])
  if np.linalg.norm(target[:3,3]-actual_pose[:3,3])>.001:raise RuntimeError('回退预备位最终位置未达标')
  n.active=None;n.submitted=False
  data.update(retracted=True,retraction_status='succeeded',retracted_at=time.time(),retreat_final_joint_rad=actual.tolist())
  print('已退回插入前预备位；实际关节到位检查通过。')
 except BaseException as error:
  n.cancel();n.submitted=False;data['retraction_error']=str(error)
  raise
 finally:atomic(report,data)

def main():
 rclpy.init();n=Retreat()
 try:run(n);return 0
 except (Exception,KeyboardInterrupt) as error:print('INSERT_BACK_REFUSED:',error);return 2
 finally:n.destroy_node();rclpy.shutdown()
if __name__=='__main__':sys.exit(main())
