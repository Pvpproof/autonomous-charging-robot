#!/usr/bin/env python3
"""Bounded startup probes. Robot probe only subscribes to status."""
import argparse,json,math,time,sys,re,os,shlex
import numpy as np
from pathlib import Path

ICP_POSE = Path('/home/nvidia/charging_socket_vision/results/v3_diagnostics/latest_pose_readonly.json')

def wait_process(present,timeout,clock=time.monotonic,sleep=time.sleep,log=print):
    deadline=clock()+timeout;stable_since=None;next_log=clock()
    while clock()<deadline:
        now=clock()
        if present():
            if stable_since is None:stable_since=now
            if now-stable_since>=0.6:return True
        else:stable_since=None
        if now>=next_log:
            log('[等待启动] 正在等待组件进程稳定出现...',flush=True);next_log=now+5
        sleep(.2)
    return False

class ConnectionEvidence:
    def __init__(self):self.count=0;self.received=False
    def observe(self,message):
        self.received=True
        self.count=self.count+1 if message.is_connected else 0
    @property
    def ready(self):return self.count>=2

def wait_robot(timeout):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile,ReliabilityPolicy
    from dobot_msgs_v4.msg import RobotStatus
    rclpy.init();node=Node('charge_robot_connection_probe')
    evidence=ConnectionEvidence()
    sub=node.create_subscription(RobotStatus,'/dobot_msgs_v4/msg/RobotStatus',evidence.observe,
        QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT))
    deadline=time.monotonic()+timeout;next_log=0.
    try:
        while time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.2)
            if evidence.ready:
                print('[连接就绪] 已连续收到真实机械臂 is_connected=true 状态。',flush=True)
                return True
            now=time.monotonic()
            if now>=next_log:
                text='尚未收到状态' if not evidence.received else '已收到状态，正在等待连接完成'
                print('[等待连接] '+text+'...',flush=True);next_log=now+5
        print('[连接超时] 请查看真实Nova2连接窗口及机器人网络状态；未发送使能或运动命令。',file=sys.stderr)
        return False
    finally:node.destroy_node();rclpy.shutdown()


def wait_group(missing, timeout, step, clock=time.monotonic, log=print, confirm=None):
    """Wait for a stage's requirements concurrently; never require stable PIDs."""
    deadline = clock() + timeout
    next_log = 0.0
    while True:
        outstanding = missing()
        if not outstanding and confirm is not None:
            outstanding=confirm()
        if not outstanding:
            return True
        now = clock()
        if now >= deadline:
            log('[阶段未就绪] ' + '、'.join(outstanding), flush=True)
            return False
        if now >= next_log:
            log('[等待阶段条件] ' + '、'.join(outstanding), flush=True)
            next_log = now + 5.0
        step(min(.2, deadline - now))


def camera_data_missing(pose, joints, joints_age, pose_response, pose_response_age, now):
    result=[]
    if (joints is None or set(joints)!={f'joint{i}' for i in range(1,7)}
            or not all(math.isfinite(v) for v in joints.values()) or not 0<=joints_age<=1.0):
        result.append('新鲜完整六关节/joint_states')
    if pose_response is None or not 0<=pose_response_age<=1.0:
        result.append('GetPose有效实际响应')
    try:
        stamp=pose.get('measurement_time')
        valid_stamp=(not isinstance(stamp,bool) and isinstance(stamp,(int,float))
                     and math.isfinite(stamp) and 0<=now-stamp<=1.0)
        T=np.asarray(pose.get('T_camera_from_socket_raw'),float)
        geometry=(T.shape==(4,4) and np.isfinite(T).all() and np.allclose(T[3],[0,0,0,1])
                  and np.allclose(T[:3,:3].T@T[:3,:3],np.eye(3),atol=1e-3)
                  and abs(np.linalg.det(T[:3,:3])-1)<=1e-3)
        if (pose.get('pose_valid') is not True or pose.get('pose_fresh') is not True
                or pose.get('quality_level')!='full_geometry' or not valid_stamp or not geometry):
            result.append('ICP新鲜完整几何位姿')
    except (ValueError,TypeError,AttributeError):result.append('ICP新鲜完整几何位姿')
    return result


def parse_getpose(response):
    if response is None or response.res!=0:return None
    values=[float(x) for x in re.findall(r'[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?',response.robot_return)]
    return values if len(values)==6 and all(math.isfinite(x) for x in values) else None



class ProcessProbe:
    """Bound process scans; require a new scan before granting readiness."""
    def __init__(self,patterns,scan,clock=time.monotonic,interval=.5):
        self.patterns={k:shlex.split(v) for k,v in patterns.items()};self.scan=scan;self.clock=clock;self.interval=interval;self.checked=-math.inf;self.result=[];self.scan_s=0.;self.scans=0
    def missing(self,force=False):
        start=self.clock()
        if not force and start-self.checked<self.interval:return list(self.result)
        table=self.scan();matched=set()
        for _,args in table.values():
            executable=Path(args[0]).name if args else ''
            if not (executable in ('python3','python','ros2','rviz2','ros2_control_node','robot_state_publisher') or executable.startswith('python3.')):continue
            for label,tokens in self.patterns.items():
                if label in matched:continue
                for i in range(len(args)-len(tokens)+1):
                    first=args[i]==tokens[0] or ('/' not in tokens[0] and Path(args[i]).name==tokens[0])
                    if first and args[i+1:i+len(tokens)]==tokens[1:]:matched.add(label);break
        self.result=[k for k in self.patterns if k not in matched];self.checked=self.clock();self.scan_s=self.checked-start;self.scans+=1
        return list(self.result)

class PoseProbe:
    """Separate response-dispatch time from polling delay; never re-age late data."""
    def __init__(self,clock=time.monotonic,log=print):
        self.clock=clock;self.log=log;self.future=None;self.started=0.;self.last_sent=-1.;self.values=None;self.last_ok=0.;self.last_kind=None;self.last_log=-math.inf;self.attempt=0;self.done_at=None;self.last_metrics={}
    def diagnostic(self,kind,**fields):
        now=self.clock()
        if kind!=self.last_kind or now-self.last_log>=5:
            self.log('[GetPose诊断] '+json.dumps(dict(kind=kind,attempt=self.attempt,domain=os.environ.get('ROS_DOMAIN_ID','0'),**fields),ensure_ascii=False),flush=True)
            self.last_kind=kind;self.last_log=now
    def completed(self,future):
        if future is self.future:self.done_at=self.clock()
    def tick(self,client,request_factory):
        now=self.clock();future=self.future
        if future is not None and future.done():
            done=self.done_at if self.done_at is not None else now
            self.last_metrics=dict(observed_roundtrip_s=round(now-self.started,4),client_completion_s=round(done-self.started,4),poll_after_completion_s=round(now-done,4),completion_callback_observed=self.done_at is not None)
            try:
                response=future.result();values=parse_getpose(response)
                self.values=values
                if values is not None and now-self.started<=1.:
                    # A service reply has no independent source timestamp. Use
                    # request start conservatively, never the late poll instant.
                    self.last_ok=self.started;self.diagnostic('response_ok',res=response.res,parsed_six_finite=True,**self.last_metrics)
                elif values is not None:
                    self.values=None;self.diagnostic('late_response',res=response.res,fresh_limit_s=1.,**self.last_metrics)
                else:
                    code=getattr(response,'res',None)
                    self.diagnostic('response_missing' if response is None else ('response_error' if code!=0 else 'invalid_pose'),res=code,parsed_six_finite=False,**self.last_metrics)
            except Exception as error:
                self.values=None;self.diagnostic('future_exception',exception_type=type(error).__name__,error=str(error)[:256],**self.last_metrics)
            self.future=None
        elif future is not None and now-self.started>5.:
            future.cancel();self.future=None;self.values=None
            self.diagnostic('request_timeout',observed_roundtrip_s=round(now-self.started,4),timeout_s=5.)
        available=client.service_is_ready()
        if not available:self.values=None;self.diagnostic('service_missing',service='/dobot_bringup_ros2/srv/GetPose')
        elif self.future is None and now-self.last_sent>=.5:
            self.started=now;self.last_sent=now;self.attempt+=1;self.done_at=None
            try:
                self.future=client.call_async(request_factory())
                self.future.add_done_callback(self.completed)
            except Exception as error:self.values=None;self.diagnostic('submit_exception',exception_type=type(error).__name__,error=str(error)[:256])
        if available and self.values is not None and now-self.last_ok>1.:self.diagnostic('last_response_stale',last_valid_response_age_s=round(now-self.last_ok,4),fresh_limit_s=1.)
        return available
    def close(self):
        if self.future is not None and not self.future.done():self.future.cancel()

def pump_callbacks(spin_once, seconds, clock=time.monotonic):
    """Drain callbacks in a bounded budget; do not sleep between ready callbacks."""
    end=clock()+min(.05,max(0.,seconds));count=0
    while count<64 and clock()<end:
        spin_once(timeout_sec=min(.005,max(0.,end-clock())));count+=1
    return count

def wait_stage(stage, timeout):
    """Check only the next stage's prerequisites while all launches overlap."""
    from charge_workflow_state import processes
    patterns = {
        'camera': {
            '真实连接进程': '/opt/ros/humble/bin/ros2 launch dobot_bringup_v4 dobot_bringup_ros2.launch.py',
            'MoveIt进程': 'ros2 launch nova2_moveit move_group.launch.py allow_trajectory_execution:=false',
            'ICP进程': '/home/nvidia/charging_socket_vision/realtime_socket_pose.py',
        },
        'gun': {
            'MoveIt进程': 'ros2 launch nova2_moveit move_group.launch.py allow_trajectory_execution:=false',
            'V3进程': '/home/nvidia/charging_socket_vision/pnp_live_test_20261023_M07DN5/pnp_live_rgb_v3/run_live.py --no-display',
            'V3桥接进程': '/home/nvidia/charging_socket_vision/pnp_v3_pose_bridge.py',
        },
    }[stage]
    robot = {'connected': False, 'last': 0.0}
    node = None
    joints={'values':None,'last':0.0}
    process_probe=ProcessProbe(patterns,processes)
    pose_query=PoseProbe()
    if stage == 'camera':
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        from dobot_msgs_v4.msg import RobotStatus
        from dobot_msgs_v4.srv import GetPose
        from sensor_msgs.msg import JointState
        from moveit_msgs.srv import GetPositionIK,GetStateValidity
        from moveit_msgs.action import MoveGroup
        from rclpy.action import ActionClient
        rclpy.init()
        node = Node('charge_robot_stage_connection_probe')
        def observe(message):
            robot['connected'] = bool(message.is_connected)
            robot['last'] = time.monotonic()
        sub = node.create_subscription(RobotStatus, '/dobot_msgs_v4/msg/RobotStatus', observe,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        def observe_joints(message):
            try:
                if len(message.name)!=len(message.position):return
                lookup=dict(zip(message.name,message.position))
                values={f'joint{i}':float(lookup[f'joint{i}']) for i in range(1,7)}
                if all(math.isfinite(x) for x in values.values()):
                    joints['values']=values;joints['last']=time.monotonic()
            except (KeyError,ValueError,TypeError):pass
        joint_sub=node.create_subscription(JointState,'/joint_states',observe_joints,
            QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT))
        getpose=node.create_client(GetPose,'/dobot_bringup_ros2/srv/GetPose')
        ik=node.create_client(GetPositionIK,'/compute_ik')
        validity=node.create_client(GetStateValidity,'/check_state_validity')
        move=ActionClient(node,MoveGroup,'/move_action')

    def missing(force=False):
        result=process_probe.missing(force=force)
        if stage == 'camera':
            if not robot['connected'] or time.monotonic() - robot['last'] > 2.0:
                result.append('真实机械臂连接状态')
            for label,client in [('IK服务/compute_ik',ik),('状态检查服务/check_state_validity',validity)]:
                if not client.service_is_ready():result.append(label)
            if not move.server_is_ready():result.append('MoveIt规划Action/move_action')
            def request_pose():
                request=GetPose.Request();request.user=0;request.tool=0;return request
            pose_query.tick(getpose,request_pose)
            now=time.monotonic()
            if not getpose.service_is_ready():result.append('GetPose服务')
            try:pose=json.loads(ICP_POSE.read_text())
            except (OSError,ValueError):pose={}
            result.extend(camera_data_missing(pose,joints['values'],now-joints['last'],
                pose_query.values,now-pose_query.last_ok,time.time()))
        return result

    def step(seconds):
        if node is None:
            time.sleep(seconds)
        else:
            pump_callbacks(lambda **kw:rclpy.spin_once(node,**kw),seconds)
    try:
        ready=wait_group(missing, timeout, step,confirm=lambda:missing(force=True))
        if ready and stage=='camera':print('[cam-plan就绪] 真实连接、六关节反馈、GetPose、MoveIt规划接口和ICP几何数据均已到位。',flush=True)
        return ready
    finally:
        if node is not None:
            pose_query.close()
            move.destroy()
            node.destroy_node()
            rclpy.shutdown()


def main():
    parser=argparse.ArgumentParser();parser.add_argument('kind',choices=['process','robot','stage']);parser.add_argument('needle',nargs='?');parser.add_argument('--timeout',type=float,default=30.)
    args=parser.parse_args()
    if not 0<args.timeout<=120:parser.error('timeout must be between 0 and 120 seconds')
    if args.kind=='stage':
        if args.needle not in ('camera','gun'):parser.error('stage needs camera or gun')
        return 0 if wait_stage(args.needle,args.timeout) else 2
    if args.kind=='process':
        from charge_workflow_state import pids
        if not args.needle:parser.error('process needs a command pattern')
        return 0 if wait_process(lambda:bool(pids(args.needle)),args.timeout) else 2
    return 0 if wait_robot(args.timeout) else 2
if __name__=='__main__':sys.exit(main())
