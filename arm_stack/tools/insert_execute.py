#!/usr/bin/env python3
"""Submit the saved insertion trajectory; no torque or source-frame estimator."""
import json,math,os,sys,time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory,JointTrajectoryPoint
from dobot_msgs_v4.srv import GetAngle,RobotMode
from std_srvs.srv import Trigger
from insertion_runtime import atomic
from insertion_limits import checked_depth_mm
ROOT=Path('/opt/charging-robot-local/charging_socket_vision/results/realtime_runs')
P=ROOT/'pending_insertion_trajectory.json';REPORT=ROOT/'latest_insertion_execution_feedback.json'

class E(Node):
 def __init__(self):
  super().__init__('charge_robot_position_insertion')
  self.action=ActionClient(self,FollowJointTrajectory,'/nova2_group_controller/follow_joint_trajectory')
  self.angle=self.create_client(GetAngle,'/dobot_bringup_ros2/srv/GetAngle')
  self.mode=self.create_client(RobotMode,'/dobot_bringup_ros2/srv/RobotMode')
  self.stop_client=self.create_client(Trigger,'/charge_robot/trajectory_adapter_stop')
  self.active=None;self.submitted=False
 def wait(self,future,timeout):
  rclpy.spin_until_future_complete(self,future,timeout_sec=timeout)
  if not future.done() or future.result() is None:raise RuntimeError('服务或轨迹响应超时；执行状态未确认')
  return future.result()
 def read(self,client,request):
  if not client.wait_for_service(timeout_sec=5):raise RuntimeError('服务不可用 '+client.srv_name)
  for attempt in range(2):
   future=client.call_async(request)
   try:result=self.wait(future,5)
   except RuntimeError as error:
    future.cancel()
    if attempt==0:
     print('[重试只读查询] '+client.srv_name,flush=True)
     continue
    raise RuntimeError('只读查询超时 '+client.srv_name+'；未取得状态响应') from error
   if result.res!=0:raise RuntimeError('读取失败 '+client.srv_name+' res='+str(result.res))
   return result.robot_return
 def joints(self):
  values=[float(x) for x in self.read(self.angle,GetAngle.Request()).strip().strip('{}').split(',')]
  if len(values)!=6 or not all(math.isfinite(x) for x in values):raise RuntimeError('GetAngle关节数据无效')
  return [math.radians(x) for x in values]
 def cancel(self):
  if not self.submitted:return
  # Cancellation failure must not prevent the independent adapter stop request.
  if self.active is not None:
   try:self.active.cancel_goal_async()
   except Exception as e:print('取消提交失败:',e)
  if self.stop_client.service_is_ready():
   try:self.wait(self.stop_client.call_async(Trigger.Request()),2)
   except Exception as e:print('适配器停止请求未确认:',e)
  print('已请求取消/停止；未确认实际停止，请核实机械臂状态。')
 def run(self):
  saved=P.read_text();d=json.loads(saved)
  from prepare_insertion_target import prepare
  prepare()
  source=json.loads((ROOT/'last_dispatched_trajectory.json').read_text())
  if d.get('source_gun_plan_id')!=source.get('plan_id') or d.get('cycle_id')!=source.get('cycle_id'):raise RuntimeError('插入轨迹不属于本轮gun预备位')
  if d.get('planning_ready') is not True:raise RuntimeError('插入轨迹未标记规划成功')
  checked_depth_mm(d.get('insertion_depth_mm',0))
  if not sys.stdin.isatty() or input('普通位置插入（无力控/扭矩接触判断）。确认路径安全、急停可触及；按Enter插入，其他取消：').strip():return 2
  if self.read(self.mode,RobotMode.Request()).strip()!='{5}':raise RuntimeError('机械臂未处于静止使能状态')
  q=self.joints()
  if max(abs(a-b) for a,b in zip(q,d['points'][0]['positions']))>math.radians(.1):raise RuntimeError('实际起点与插入计划不一致，请重新insert-plan')
  if P.read_text()!=saved:raise RuntimeError('确认期间轨迹已改变，请重新确认')
  if not self.action.wait_for_server(timeout_sec=10):raise RuntimeError('FollowJointTrajectory action不可用')
  scale=max(1.,float(os.environ.get('INSERTION_TIME_SCALE','1.')))
  if not math.isfinite(scale):raise RuntimeError('插入时间倍率无效')
  traj=JointTrajectory();traj.joint_names=d['joint_names']
  for x in d['points']:
   p=JointTrajectoryPoint();p.positions=x['positions'];t=x['time_from_start'];ns=round((t['sec']+t['nanosec']/1e9)*scale*1e9)
   p.time_from_start.sec,p.time_from_start.nanosec=divmod(ns,1000000000);traj.points.append(p)
  # Keep the complete original path, including its start, for insert-back/home.
  atomic(ROOT/'last_inserted_trajectory.json',d)
  report=dict(stage='insertion',executor='cartesian_once',status='uncertain',plan_id=d['plan_id'],insertion_depth_mm=d['insertion_depth_mm'],motion_commands_sent=False,contact_monitoring=False,insertion_axis=d['insertion_axis'])
  try:
   g=FollowJointTrajectory.Goal();g.trajectory=traj
   report['motion_commands_sent']=True;atomic(REPORT,report);self.submitted=True
   self.active=self.wait(self.action.send_goal_async(g),10)
   if not self.active.accepted:
    self.active=None;self.submitted=False;report.update(status='rejected',motion_commands_sent=False)
    raise RuntimeError('插入轨迹被适配器拒绝；未开始插入')
   out=self.wait(self.active.get_result_async(),180)
   report.update(action_status=out.status,error_code=out.result.error_code,error_string=out.result.error_string)
   if out.status!=4 or out.result.error_code!=0:raise RuntimeError('插入执行失败 '+out.result.error_string)
   deadline=time.monotonic()+5;stable=0
   while time.monotonic()<deadline:
    actual=self.joints();mode=self.read(self.mode,RobotMode.Request()).strip()
    error=max(abs(math.degrees(a-b)) for a,b in zip(actual,d['points'][-1]['positions']))
    stable=stable+1 if error<=.1 and mode=='{5}' else 0
    if stable>=3:break
    rclpy.spin_once(self,timeout_sec=.1)
   else:raise RuntimeError('执行结束但未确认实际关节到位')
   self.submitted=False;self.active=None
   report.update(status='succeeded',final_joint_rad=actual,endpoint_max_error_deg=error)
   print('普通插入执行完成，关节到位检查通过；未验证锁止或电气连接。')
   return 0
  except BaseException as error:
   report['reason']=str(error);self.cancel();self.submitted=False
   raise
  finally:
   report['finished_at']=time.time();atomic(REPORT,report)

def main():
 rclpy.init();n=E()
 try:return n.run()
 except (Exception,KeyboardInterrupt) as e:print('INSERT_GO_REFUSED:',e);return 2
 finally:n.destroy_node();rclpy.shutdown()
if __name__=='__main__':sys.exit(main())
