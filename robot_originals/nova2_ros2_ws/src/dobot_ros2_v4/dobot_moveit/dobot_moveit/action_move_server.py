#!/usr/bin/env python3
"""Serialized trajectory streaming with cancellation barrier; never clears errors."""
import json, math, os, re, threading, time, fcntl
try:
    from .source_feedback import SourceProgress
except ImportError:
    from source_feedback import SourceProgress
import rclpy
from rclpy.action import ActionServer,CancelResponse,GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
from control_msgs.action import FollowJointTrajectory
from dobot_msgs_v4.srv import ServoJ,Stop,GetAngle
from std_srvs.srv import Trigger

class FollowJointTrajectoryServer(Node):
    def __init__(self):
        super().__init__('dobot_group_controller')
        self.declare_parameter('trajectory_execution_timeout',120.)
        self._timeout=float(self.get_parameter('trajectory_execution_timeout').value)
        self.feedback_group=MutuallyExclusiveCallbackGroup()
        self.group=ReentrantCallbackGroup();self.lock=threading.RLock();self.busy=False
        self.cancel=threading.Event();self.streaming=False;self.inflight=0
        self.last_stop_ok=None;self.quiescent=True;self.goal_id=None;self.sent=0
        self.source=SourceProgress();self.feed_error=None;self.timing_events=[];self.stop_evidence={}
        self.feed=None;self.stamp=0;self.stop_lock=threading.Lock()
        self.stop_epoch=0;self.stop_requested_monotonic=0.;self.goal_terminal=True;self.stop_latched=False
        self.servo=self.create_client(ServoJ,'/dobot_bringup_ros2/srv/ServoJ',callback_group=self.group)
        self.stop=self.create_client(Stop,'/dobot_bringup_ros2/srv/Stop',callback_group=self.group)
        self.angle=self.create_client(GetAngle,'/dobot_bringup_ros2/srv/GetAngle',callback_group=self.group)
        self.pub=self.create_publisher(String,'/charge_robot/trajectory_adapter_status',10)
        self.create_subscription(String,'/dobot_bringup_ros2/msg/FeedInfo',self.receive,10,callback_group=self.feedback_group)
        self.create_timer(.1,self.publish,callback_group=self.group)
        self.stop_service=self.create_service(Trigger,'/charge_robot/trajectory_adapter_stop',
            self.request_stop,callback_group=self.group)
        name=os.getenv('DOBOT_TYPE','cr5')
        self.server=ActionServer(self,FollowJointTrajectory,f'/{name}_group_controller/follow_joint_trajectory',
            execute_callback=self.execute,goal_callback=self.on_goal,cancel_callback=self.on_cancel,callback_group=self.group)
    def receive(self,msg):
        try:
            d=json.loads(msg.data)
            for k in ('q_actual','qd_actual'):
                if len(d[k])!=6 or not all(math.isfinite(float(x)) for x in d[k]):raise ValueError(k)
            if any(k not in d for k in ('ErrorStatus','CollisionStates','robot_mode')):raise ValueError('feedback flags')
            with self.lock:
                if not self.source.accept(d):return
                self.feed=d;self.stamp=time.monotonic();self.feed_error=None
        except Exception as e:
            with self.lock:self.feed_error=str(e)
    def publish(self):
        with self.lock:
            d=dict(protocol=3,source_token=self.source.token,source_count=self.source.count,feed_error=self.feed_error,busy=self.busy,streaming=self.streaming,inflight=self.inflight,
                   quiescent=self.quiescent,last_stop_ok=self.last_stop_ok,
                   timing_events=self.timing_events[-32:],stop_evidence=dict(self.stop_evidence),
                   goal_id=self.goal_id,dispatch_count=self.sent,cancelled=self.cancel.is_set())
            d.update(protocol=4,stop_epoch=self.stop_epoch,stop_requested_monotonic=self.stop_requested_monotonic,
                     stop_latched=self.stop_latched)
        m=String();m.data=json.dumps(d);self.pub.publish(m)
    @staticmethod
    def points(traj):
        if len(traj.joint_names)!=6 or set(traj.joint_names)!={f'joint{i}' for i in range(1,7)}:raise ValueError('invalid joint names')
        order=[traj.joint_names.index(f'joint{i}') for i in range(1,7)]
        pts=[]
        for p in traj.points:
            if len(p.positions)!=6 or not all(math.isfinite(float(v)) for v in p.positions):raise ValueError('invalid positions')
            t=p.time_from_start.sec+p.time_from_start.nanosec/1e9
            if not math.isfinite(t) or t<0 or (pts and t<=pts[-1][1]):raise ValueError('invalid trajectory time')
            pts.append(([math.degrees(p.positions[i]) for i in order],t))
        if len(pts)<2 or pts[-1][1]<=0:raise ValueError('empty trajectory')
        if pts[0][1]!=0 or any(pts[i][1]-pts[i-1][1]<.004 for i in range(1,len(pts))):
            raise ValueError('trajectory needs zero-time start and intervals >=4ms')
        return pts
    def on_goal(self,request):
        try:self.points(request.trajectory)
        except Exception:return GoalResponse.REJECT
        with self.lock:
            if self.busy or not self.quiescent or self.stop_latched:return GoalResponse.REJECT
            try:self.source.check()
            except Exception:return GoalResponse.REJECT
            if self.feed_error:return GoalResponse.REJECT
            if self.feed is None or time.monotonic()-self.stamp>.4:return GoalResponse.REJECT
            if self.feed['robot_mode']!=5 or self.feed['ErrorStatus'] or self.feed['CollisionStates']:return GoalResponse.REJECT
            self.busy=True;self.goal_terminal=False;self.quiescent=False;self.cancel.clear();self.last_stop_ok=None
        return GoalResponse.ACCEPT
    def on_cancel(self,handle):
        with self.lock:
            if not handle.is_active:return CancelResponse.REJECT
            self.cancel.set();self.streaming=False;self.stop_latched=True
        self.publish()
        # Execute callback owns stop processing; callback responds promptly.
        return CancelResponse.ACCEPT
    def request_stop(self,request,response):
        # Separate from Action cancellation: cancellation submission itself may fail.
        # The acknowledgement means DISPATCH LATCHED, not robot physically stopped.
        with self.lock:
            self.cancel.set();self.streaming=False;self.quiescent=False;self.stop_latched=True
            self.stop_epoch+=1;self.stop_requested_monotonic=time.monotonic()
            execute_owns_stop=self.busy and not self.goal_terminal
            response.success=True;response.message=json.dumps(dict(
                state='dispatch_latched',stop_epoch=self.stop_epoch))
        self.publish()
        if not execute_owns_stop:self.do_stop()
        return response
    def response(self,client,request,timeout,monitor=False):
        if not client.service_is_ready():raise RuntimeError('service unavailable '+client.srv_name)
        future=client.call_async(request);return self.wait(future,timeout,monitor)
    def wait(self,future,timeout,monitor=False):
        end=time.monotonic()+timeout
        while not future.done():
            if monitor:self.guard()
            if time.monotonic()>=end:raise RuntimeError('service timeout')
            # Executor runs callbacks on other threads. Never nest spin() on this node.
            self.cancel.wait(.005) if monitor else time.sleep(.005)
        if monitor:self.guard()
        r=future.result()
        if r is None or r.res!=0:raise RuntimeError('controller request failed')
        return r
    def guard(self):
        if self.cancel.is_set():raise RuntimeError('cancelled')
        with self.lock:
            d=self.feed;stamp=self.stamp
            self.source.check()
            if self.feed_error:raise RuntimeError(self.feed_error)
        if d is None or time.monotonic()-stamp>.4:raise RuntimeError('feedback timeout')
        if d['ErrorStatus'] or d['CollisionStates'] or d['robot_mode'] not in (5,7):raise RuntimeError('controller fault/mode')
    def do_stop(self):
        with self.stop_lock:
            with self.lock:
                self.cancel.set();self.streaming=False;self.stop_latched=True
                stop_deadline=time.monotonic()+5.
                had_inflight=self.inflight>0
                self.stop_evidence=dict(requested_monotonic=time.monotonic(),confirmed=False,stop_epoch=self.stop_epoch,
                    inflight_at_request=self.inflight)
            # No ClearError. Stop is submitted before waiting for an in-flight ServoJ reply.
            try:
                res=self.response(self.stop,Stop.Request(),2.)
                self.stop_evidence.update(controller_res=res.res,response_monotonic=time.monotonic())
                # A previously submitted ServoJ may be processed AFTER this first Stop.
                # Drain its REAL response, then place a final Stop behind it. No replay/retry of ServoJ.
                if had_inflight:
                    while True:
                        with self.lock:pending=self.inflight
                        if not pending:break
                        if time.monotonic()>=stop_deadline:raise RuntimeError('inflight command unresolved; final Stop ordering unknown')
                        time.sleep(.005)
                    self.stop_evidence['drained_monotonic']=time.monotonic()
                    remaining=stop_deadline-time.monotonic()
                    if remaining<=0:raise RuntimeError('stop deadline before final Stop')
                    final=self.response(self.stop,Stop.Request(),min(2.,remaining))
                    self.stop_evidence.update(final_controller_res=final.res,final_response_monotonic=time.monotonic())
                deadline=stop_deadline;previous=None;stable=0;last_stamp=None;stop_started=time.monotonic()
                while time.monotonic()<deadline:
                    with self.lock:
                        d=self.feed;stamp=self.stamp;inflight=self.inflight
                        self.source.check()
                        if self.feed_error:raise RuntimeError(self.feed_error)
                    if d and stamp!=last_stamp:
                        q=d['q_actual'];stationary=(stamp>stop_started and time.monotonic()-stamp<=.4 and not inflight
                            and d['robot_mode'] in (5,9,10,11) and max(abs(x) for x in d['qd_actual'])<=.1)
                        stable=stable+1 if stationary and previous is not None and max(abs(a-b) for a,b in zip(q,previous))<=.02 else 0
                        previous=q;last_stamp=stamp
                        if stable>=3:
                            with self.lock:
                                self.last_stop_ok=True;self.quiescent=True
                                self.stop_evidence.update(confirmed=True,source_token=self.source.token,confirmed_monotonic=time.monotonic())
                            return
                    time.sleep(.01)
                raise RuntimeError('physical stop unconfirmed')
            except Exception as e:
                with self.lock:
                    self.last_stop_ok=False;self.quiescent=False
                    self.stop_evidence.update(error=repr(e),confirmed=False)
                self.get_logger().error('STOP UNCONFIRMED: '+str(e))
    def send_servo(self,deg,duration):
        self.guard()
        req=ServoJ.Request();req.a,req.b,req.c,req.d,req.e,req.f=map(float,deg)
        req.param_value=[f't={max(.004,duration)}']
        with self.lock:
            # Same lock as cancellation: no dispatch can cross this barrier.
            self.guard()
            if self.cancel.is_set():raise RuntimeError('cancelled before dispatch')
            self.inflight+=1;self.sent+=1
            try:
                dispatched=time.monotonic()
                self.timing_events=self.timing_events[-19999:]
                self.timing_events.append(dict(event='dispatch',t=dispatched,duration=duration,target_deg=list(deg)))
                f=self.servo.call_async(req)
            except BaseException:self.inflight-=1;raise
        # Only the real response drains inflight; a timeout does not prove completion.
        def drained(_):
            with self.lock:
                self.inflight-=1
                self.timing_events.append(dict(event="response_done",t=time.monotonic()))
        f.add_done_callback(drained)
        self.wait(f,3.,True)
    def endpoint(self,target):
        deadline=time.monotonic()+3.;stable=0;last_stamp=None;previous=None
        while time.monotonic()<deadline:
            self.guard()
            with self.lock:d=self.feed;stamp=self.stamp
            if stamp!=last_stamp:
                q=d['q_actual'];good=(d['robot_mode']==5 and max(abs(a-b) for a,b in zip(q,target))<=.1
                    and max(abs(v) for v in d['qd_actual'])<=.1
                    and previous is not None and max(abs(a-b) for a,b in zip(q,previous))<=.02)
                stable=stable+1 if good else 0;previous=q;last_stamp=stamp
                if stable>=3:return
            self.cancel.wait(.01)
        raise RuntimeError('actual joint endpoint not reached')
    def execute(self,handle):
        result=FollowJointTrajectory.Result();failed=None
        with self.lock:self.goal_id=bytes(handle.goal_id.uuid).hex();self.streaming=True
        self.publish()
        try:
            pts=self.points(handle.request.trajectory);t0=time.monotonic()
            with self.lock:start=list(self.feed['q_actual'])
            if max(abs(a-b) for a,b in zip(start,pts[0][0]))>.1:
                raise RuntimeError('actual start differs from zero-time point')
            # Restore continuous streaming on the original trajectory timeline.
            # Intermediate waypoints are not stop-and-settle targets.
            for i,(deg,target_time) in enumerate(pts):
                while time.monotonic()-t0<target_time:
                    self.guard()
                    if time.monotonic()-t0>self._timeout:raise RuntimeError('trajectory timeout')
                    self.cancel.wait(.005)
                self.guard()
                if time.monotonic()-t0>self._timeout:raise RuntimeError('trajectory timeout')
                duration=.1 if i==0 else target_time-pts[i-1][1]
                self.send_servo(deg,max(.05,duration))
            self.endpoint(pts[-1][0])
            # Final guard and terminal success share the cancellation barrier.
            with self.lock:
                self.guard()
                if handle.is_cancel_requested:raise RuntimeError('cancelled at completion')
                handle.succeed();self.goal_terminal=True;self.streaming=False;self.quiescent=True
        except BaseException as e:
            failed=str(e);self.get_logger().error('trajectory stopped: '+failed);self.do_stop()
        finally:
            with self.lock:self.streaming=False;self.busy=False
            self.publish()
        result.error_string=failed or ''
        if handle.is_cancel_requested:
            handle.canceled();result.error_code=FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED
        elif failed:
            handle.abort();result.error_code=FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED
        else:
            result.error_code=FollowJointTrajectory.Result.SUCCESSFUL
        return result

def main(args=None):
    # Hold for the process lifetime; one owner for the hardware trajectory Action.
    instance=open('/tmp/nova2_trajectory_adapter.instance.lock','a')
    try:fcntl.flock(instance,fcntl.LOCK_EX|fcntl.LOCK_NB)
    except BlockingIOError:
        print('执行适配器已运行；拒绝启动第二个实例。',flush=True)
        return
    rclpy.init(args=args);node=FollowJointTrajectoryServer();executor=MultiThreadedExecutor(num_threads=4);executor.add_node(node)
    try:executor.spin()
    except KeyboardInterrupt:pass
    finally:
        if node.busy:node.do_stop()
        executor.shutdown();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()
