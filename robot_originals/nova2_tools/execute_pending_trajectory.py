#!/usr/bin/env python3
import json, sys, fcntl, math, re, time, os
import numpy as np
from scipy.spatial.transform import Rotation
from plan_flow_guard import validate, validate_start_positions
from dobot_msgs_v4.srv import GetAngle, GetPose, RobotMode, GetErrorID
from moveit_msgs.srv import GetPositionFK
from gun_pose_validation import validate_saved, tcp_from_link6, evaluate, require, link6_from_controller
from stable_pose_selection import atomic_json
from pathlib import Path
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from action_msgs.msg import GoalStatus
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
PENDING=Path('/home/nvidia/charging_socket_vision/results/realtime_runs/pending_trajectory.json')
CAM=Path('/home/nvidia/charging_socket_vision/results/realtime_runs/pending_camera_trajectory.json')
GUN=Path('/home/nvidia/charging_socket_vision/results/realtime_runs/pending_gun_trajectory.json')

def confirm_execution():
    if not sys.stdin.isatty():
        raise RuntimeError('执行确认需要交互终端，禁止通过管道自动确认')
    import termios
    termios.tcflush(sys.stdin.fileno(),termios.TCIFLUSH)
    try:
        return input('确认工件未移动、工作区安全：按 Enter 执行，输入其他内容取消：').strip()==''
    except EOFError:
        return False


class Executor(Node):
    def __init__(self):
        super().__init__('charge_robot_pending_executor')
        self.dispatched_gun=None
        self.adapter_status=None
        self.create_subscription(String,'/charge_robot/trajectory_adapter_status',self.observe_adapter,10)
        self.client=ActionClient(self, FollowJointTrajectory, '/nova2_group_controller/follow_joint_trajectory')
    def observe_adapter(self,msg):
        self.adapter_status=(json.loads(msg.data),time.monotonic())

    def check_adapter_ready(self):
        self.adapter_status=None
        deadline=time.monotonic()+2.0
        while self.adapter_status is None and time.monotonic()<deadline:
            rclpy.spin_once(self,timeout_sec=0.1)
        if self.adapter_status is None:
            raise RuntimeError('执行适配器状态未收到；请检查执行适配器。计划尚未派发。')
        status,_=self.adapter_status
        if status.get('feed_error'):
            raise RuntimeError('执行适配器反馈失效：'+status['feed_error']+
                               '；若刚重启bringup，请确认机械臂静止后手动重启执行适配器。计划尚未派发。')
        if status.get('busy') or not status.get('quiescent') or status.get('stop_latched'):
            raise RuntimeError('执行适配器忙碌或停止状态未解除；请先核实机械臂状态。计划尚未派发。')

    def read_service(self,client,request):
        """Bounded retries for read-only services; never retries motion commands."""
        started=time.monotonic()
        if not client.wait_for_service(timeout_sec=1.0):
            print(f'[等待] 正在发现反馈服务 {client.srv_name}（最长15秒）...',flush=True)
            if not client.wait_for_service(timeout_sec=14.0):
                raise RuntimeError(f'15秒内未发现反馈服务 {client.srv_name}；请检查bringup服务及ROS网络')
        for attempt in range(2):
            future=client.call_async(request)
            rclpy.spin_until_future_complete(self,future,timeout_sec=5.0)
            if future.done():
                result=future.result()
                if result is not None:return result
            future.cancel()
            if attempt==0:
                print(f'[重试读取] {client.srv_name} 未响应，再读取一次（不发送运动）',flush=True)
        raise RuntimeError(f'反馈服务 {client.srv_name} 已发现但两次查询均未取得响应；耗时{time.monotonic()-started:.1f}秒')

    def check_robot_ready(self):
        for typ,name,expected in ((RobotMode,'RobotMode',[5]),(GetErrorID,'GetErrorID',[[]])):
            client=self.create_client(typ,'/dobot_bringup_ros2/srv/'+name)
            try:
                result=self.read_service(client,typ.Request())
                if result.res!=0:raise RuntimeError(name+'查询失败，未发送运动')
                try:value=json.loads('['+result.robot_return.strip().removeprefix('{').removesuffix('}')+']')
                except (ValueError,AttributeError):raise RuntimeError(name+'反馈格式无效，未发送运动')
                if value!=expected:raise RuntimeError(name+'状态不允许执行：'+result.robot_return)
            finally:self.destroy_client(client)
        print('[通过] 机器人静止使能且无报警',flush=True)

    def check_start(self, data):
        client = self.create_client(GetAngle, '/dobot_bringup_ros2/srv/GetAngle')
        try:
            result=self.read_service(client,GetAngle.Request())
            if result.res != 0:
                raise RuntimeError(f'GetAngle返回失败 res={result.res}；未发送运动')
            angles = [float(v) for v in re.findall(r'[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?', result.robot_return)]
            if len(angles) != 6 or not all(math.isfinite(v) for v in angles):
                raise RuntimeError('真实关节角格式异常；未发送运动')
            by_name = {f'joint{i+1}': math.radians(v) for i,v in enumerate(angles)}
            error = validate_start_positions(data, [by_name[n] for n in data['joint_names']])
            print(f'[通过] 真实机械臂与轨迹起点最大偏差 {error:.3f}度')
        finally:
            self.destroy_client(client)

    def query(self,client,request):
        return self.read_service(client,request)

    def gun_target(self,data):
        path=PENDING.with_name('frozen_pnp_v3_socket_target.json')
        target=json.loads(path.read_text())
        if target.get('plan_id')!=data.get('camera_plan_id') or target.get('request_id')!=data.get('camera_request_id'):
            raise RuntimeError('冻结枪头目标发生变化')
        if data.get('cycle_id')!=target.get('cycle_id'):raise RuntimeError('枪头目标周期变化')
        if data.get('fusion',{}).get('enabled') and data['fusion'].get('icp_reference')!=target.get('icp_reference'):raise RuntimeError('冻结ICP绑定变化')
        return validate_saved(data,target.get('T_base_from_socket'))

    def check_gun_model_endpoint(self,data):
        if data.get('stage')!='gun':return
        target=self.gun_target(data)
        client=self.create_client(GetPositionFK,'/compute_fk')
        try:
            request=GetPositionFK.Request();request.header.frame_id='base_link';request.fk_link_names=['Link6']
            request.robot_state.joint_state.name=data['joint_names']
            request.robot_state.joint_state.position=data['points'][-1]['positions']
            response=self.query(client,request)
            if response.error_code.val!=1 or len(response.pose_stamped)!=1 or response.pose_stamped[0].header.frame_id.lstrip('/')!='base_link':
                raise RuntimeError('执行前FK验收失败，未发送运动')
            pose=response.pose_stamped[0].pose;T=np.eye(4);T[:3,3]=[pose.position.x,pose.position.y,pose.position.z]
            T[:3,:3]=Rotation.from_quat([pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w]).as_matrix()
            require(tcp_from_link6(T),target)
        finally:self.destroy_client(client)

    def record_gun_finish(self,data):
        report_path=PENDING.with_name('latest_gun_execution_feedback.json')
        report={'stage':'gun','plan_id':data['plan_id'],'checked_at':time.time(),
                'passed':False,'physical_tcp_calibration_verified':False,'samples':[]}
        clients={}
        try:
            target=self.gun_target(data)
            for kind,typ in [('GetAngle',GetAngle),('GetPose',GetPose),('RobotMode',RobotMode)]:
                clients[kind]=self.create_client(typ,'/dobot_bringup_ros2/srv/'+kind)
            deadline=time.monotonic()+8;consecutive=0
            while time.monotonic()<deadline:
                angle=self.query(clients['GetAngle'],GetAngle.Request())
                pose=self.query(clients['GetPose'],GetPose.Request(user=0,tool=0))
                mode=self.query(clients['RobotMode'],RobotMode.Request())
                if any(r.res!=0 for r in (angle,pose,mode)):raise RuntimeError('实际反馈服务返回失败')
                pattern=r'[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?'
                angles=[float(v) for v in re.findall(pattern,angle.robot_return)]
                values=[float(v) for v in re.findall(pattern,pose.robot_return)]
                modes=[int(v) for v in re.findall(r'[-+]?\d+',mode.robot_return)]
                if len(angles)!=6 or not np.isfinite(angles).all() or len(modes)!=1:raise RuntimeError('实际反馈格式无效')
                actual=tcp_from_link6(link6_from_controller(values));result=evaluate(actual,target)
                by_name={f'joint{i+1}':math.radians(a) for i,a in enumerate(angles)}
                joint_error=max(abs(math.degrees(by_name[n]-v)) for n,v in zip(data['joint_names'],data['points'][-1]['positions']))
                passed=result['passed'] and joint_error<=0.1 and modes[0]==5
                consecutive=consecutive+1 if passed else 0
                report['samples'].append(dict(measured_at=time.time(),controller_pose_tool0_mm_deg=values,
                    controller_joint_deg=angles,robot_mode=modes[0],joint_endpoint_error_deg=joint_error,
                    endpoint=result,passed=passed))
                if consecutive>=3:
                    report.update(passed=True,status='verified',finished_at=time.time())
                    atomic_json(report_path,report)
                    print(f"[实际枪头终点验收通过] 名义TCP位置误差{result['position_error_mm']:.3f}mm，方向误差{result['rotation_error_deg']:.3f}度")
                    return
                rclpy.spin_once(self,timeout_sec=.2)
            raise RuntimeError('轨迹动作返回成功，但实际终点反馈未通过验收；请检查位置并重新规划，未标记本轮完成')
        except Exception as error:
            report.update(status='unverified',error=str(error),finished_at=time.time())
            atomic_json(report_path,report)
            raise
        finally:
            for client in clients.values():self.destroy_client(client)

    def record_camera_finish(self,data):
        report={'stage':'camera','plan_id':data['plan_id'],'passed':False,'status':'unverified'}
        try:
            client=self.create_client(GetAngle,'/dobot_bringup_ros2/srv/GetAngle')
            mode_client=self.create_client(RobotMode,'/dobot_bringup_ros2/srv/RobotMode')
            try:
                angle=self.read_service(client,GetAngle.Request())
                mode=self.read_service(mode_client,RobotMode.Request())
                values=[float(v) for v in re.findall(r'[-+]?(?:\d+(?:\.\d*)?|\.\d+)',angle.robot_return)]
                if angle.res!=0 or len(values)!=6 or not np.isfinite(values).all() or mode.res!=0 or mode.robot_return.strip()!='{5}':
                    raise RuntimeError('cam-plan执行后实际反馈无效')
                lookup={f'joint{i+1}':math.radians(v) for i,v in enumerate(values)}
                error=max(abs(math.degrees(lookup[n]-v)) for n,v in zip(data['joint_names'],data['points'][-1]['positions']))
                if error>.1:raise RuntimeError('cam-plan执行后实际关节未到位')
                report.update(passed=True,status='verified',actual_joint_deg=values,joint_endpoint_error_deg=error)
            finally:self.destroy_client(client);self.destroy_client(mode_client)
        finally:
            report['checked_at']=time.time()
            atomic_json(PENDING.with_name('latest_camera_execution_feedback.json'),report)

    def run(self):
        self.lock_file = open(str(PENDING) + '.execute.lock', 'w')
        fcntl.flock(self.lock_file, fcntl.LOCK_EX | fcntl.LOCK_NB)
        if not PENDING.exists(): raise RuntimeError('没有待执行轨迹，请先 cam-plan 或 gun-plan')
        data=json.loads(PENDING.read_text())
        print(f"找到计划 id={data.get('plan_id', 'legacy')} stage={data.get('stage')} selected_distance_mm={None if data.get('selected_distance_m') is None else data['selected_distance_m']*1000}",flush=True)
        fast_test=os.environ.get('CHARGE_ROBOT_FAST_TEST')=='1'
        validate(data)
        if not fast_test:
            self.check_start(data)
            self.check_gun_model_endpoint(data)
        traj=JointTrajectory(); traj.joint_names=data['joint_names']
        for item in data['points']:
            p=JointTrajectoryPoint(); p.positions=item['positions']
            p.velocities=item.get('velocities',[]); p.accelerations=item.get('accelerations',[])
            t=item['time_from_start']; p.time_from_start.sec=t['sec']; p.time_from_start.nanosec=t['nanosec']; traj.points.append(p)
        stage=data.get('stage','unknown')
        duration=traj.points[-1].time_from_start.sec+traj.points[-1].time_from_start.nanosec/1e9
        print(f'待执行阶段: {stage}; 轨迹点: {len(traj.points)}; 时长: {duration:.2f}s')
        if stage=='gun':
            c=data.get('lateral_compensation',{})
            print(f"本轨迹补偿：启用={c.get('enabled',False)}；向左 {c.get('left_mm',0):.2f} mm；向右 {c.get('right_yaw_deg',0):.2f} 度",flush=True)

        if not confirm_execution():
            print('已取消，未发送运动'); return 2
        latest=json.loads(PENDING.read_text())
        if latest != data: raise RuntimeError('确认期间轨迹已改变，拒绝执行；请重新检查计划')
        validate(latest)
        if not self.client.wait_for_server(timeout_sec=10): raise RuntimeError('FollowJointTrajectory action 不可用')
        if fast_test:self.check_robot_ready()
        self.check_start(latest)
        self.check_gun_model_endpoint(latest)
        if json.loads(PENDING.read_text()) != data:
            raise RuntimeError('检查期间轨迹已改变，请重新确认')
        self.check_adapter_ready()
        goal=FollowJointTrajectory.Goal(); goal.trajectory=traj
        # Consume before dispatch: uncertain/partial execution must never replay silently.
        if stage=='gun':
            self.dispatched_gun=data
            atomic_json(PENDING.with_name('latest_gun_execution_feedback.json'),
                        dict(stage='gun',plan_id=data['plan_id'],passed=False,status='dispatching',checked_at=time.time()))
        PENDING.replace(PENDING.with_name('last_dispatched_trajectory.json'))
        future=self.client.send_goal_async(goal); rclpy.spin_until_future_complete(self,future,timeout_sec=10); handle=future.result()
        if handle is None or not handle.accepted: raise RuntimeError('轨迹目标被拒绝')
        result_future=handle.get_result_async(); rclpy.spin_until_future_complete(self,result_future,timeout_sec=180)
        if result_future.result() is None:
            handle.cancel_goal_async()
            raise RuntimeError('执行结果超时，已请求取消；请确认机械臂实际状态，禁止直接重试')
        outcome=result_future.result(); result=outcome.result
        print('执行结果 status=',outcome.status,'error_code=',result.error_code)
        if outcome.status==GoalStatus.STATUS_SUCCEEDED and result.error_code==0:
            if stage=='gun':self.record_gun_finish(data)
            if stage=='camera':self.record_camera_finish(data)
            (CAM if stage=='camera' else GUN).unlink(missing_ok=True); return 0
        return 3
def main():
    rclpy.init(); node=Executor()
    try: code=node.run()
    except Exception as exc:
        if node.dispatched_gun is not None:
            path=PENDING.with_name('latest_gun_execution_feedback.json')
            try:report=json.loads(path.read_text())
            except (OSError,ValueError):report={}
            report.update(stage='gun',plan_id=node.dispatched_gun['plan_id'],passed=False,error=str(exc),checked_at=time.time())
            if report.get('status')!='unverified':report['status']='execution_unconfirmed'
            atomic_json(path,report)
            print('EXECUTION_UNCONFIRMED:',exc)
        else:print('EXECUTE_REFUSED:',exc)
        code=2
    node.destroy_node(); rclpy.shutdown(); return code
if __name__=='__main__': sys.exit(main())
