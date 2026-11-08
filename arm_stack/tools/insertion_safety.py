"""Insertion feedback guard; raw torque is a contact proxy, not force in newtons."""
import json, math, time
import numpy as np
from std_msgs.msg import String
from dobot_msgs_v4.srv import Stop

ADAPTER_PROTOCOL = 2

class ContactDetected(RuntimeError):
    pass

class TorqueDetector:
    def __init__(self, baseline, limits):
        self.baseline=np.asarray(baseline,float);self.limits=np.asarray(limits,float)
        self.since=[None]*6
        if (self.baseline.shape!=(6,) or self.limits.shape!=(6,) or not np.isfinite(self.baseline).all()
                or not np.isfinite(self.limits).all() or np.any(self.limits<=0)):
            raise ValueError('无效扭矩基线/阈值')
    def update(self, history, now):
        recent=[v for t,v in history if now-.15<=t<=now]
        if len(recent)<3:return
        delta=np.abs(np.median(recent,axis=0)-self.baseline)
        # Every joint has its own clock; a changing argmax cannot reset another joint.
        for j in range(6):
            self.since[j]=(now if self.since[j] is None else self.since[j]) if delta[j]>self.limits[j] else None
        elapsed=[0 if s is None else now-s for s in self.since]
        if max(elapsed)>=.35:
            j=int(np.argmax(elapsed))
            raise ContactDetected('持续接触代理 J%d 超限%.2fs Δτ=%s'%(j+1,elapsed[j],delta.round(3).tolist()))

class FeedbackGuard:
    def __init__(self,node):
        self.node=node;self.feed=None;self.stamp=0;self.history=[];self.samples=[];self.error=None
        self.detector=None;self.adapter=None;self.adapter_stamp=0;self.expected_goal=None
        node.create_subscription(String,'/dobot_bringup_ros2/msg/FeedInfo',self.receive,10)
        node.create_subscription(String,'/charge_robot/trajectory_adapter_status',self.receive_adapter,10)
    def receive(self,msg):
        try:
            d=json.loads(msg.data)
            for key in ('m_actual','q_actual','qd_actual'):
                a=np.asarray(d[key],float)
                if a.shape!=(6,) or not np.isfinite(a).all():raise ValueError(key)
            for key in ('ErrorStatus','CollisionStates','robot_mode'):
                if key not in d:raise ValueError(key)
            now=time.monotonic();self.feed=d;self.stamp=now
            self.history.append((now,d['m_actual']));self.history=self.history[-300:]
            self.samples.append(dict(t=time.time(),**d));self.samples=self.samples[-20000:]
            self.error=None
        except Exception as e:self.error='反馈格式无效:'+str(e)
    def receive_adapter(self,msg):
        try:
            d=json.loads(msg.data)
            if d.get('protocol')!=ADAPTER_PROTOCOL:return
            self.adapter=d;self.adapter_stamp=time.monotonic()
        except Exception:pass
    def check(self,contact=True):
        now=time.monotonic()
        if self.error:raise RuntimeError(self.error)
        if self.feed is None or now-self.stamp>.4:raise RuntimeError('机器人反馈中断/超时')
        if self.feed['ErrorStatus'] or self.feed['CollisionStates']:raise RuntimeError('机器人碰撞或报警')
        if self.feed['robot_mode'] not in (5,7):raise RuntimeError('运动期间RobotMode异常:'+str(self.feed['robot_mode']))
        self.check_adapter()
        if contact and self.detector:self.detector.update(self.history,now)
    def check_adapter(self):
        d=self.adapter
        if d is None or time.monotonic()-self.adapter_stamp>.5:raise RuntimeError('新版轨迹适配器未就绪或心跳超时，请重新start')
    def idle(self):
        self.check(False);self.check_adapter()
        if self.feed['robot_mode']!=5 or self.adapter.get('busy') or self.adapter.get('streaming'):
            raise RuntimeError('机械臂/适配器尚未静止')
    def baseline(self,duration=1.5,torque=True):
        end=time.monotonic()+duration;self.history=[]
        while time.monotonic()<end:
            import rclpy
            rclpy.spin_once(self.node,timeout_sec=.02);self.idle()
            if max(abs(float(v)) for v in self.feed['qd_actual'])>.1:raise RuntimeError('采集基线时关节未静止')
        values=np.array([v for _,v in self.history])
        if len(values)<15:raise RuntimeError('基线样本不足')
        b=np.median(values,axis=0);noise=1.4826*np.median(abs(values-b),axis=0)
        limits=np.maximum(6*noise,[1.5,1.5,1.,.8,.8,.5])
        if torque:self.detector=TorqueDetector(b,limits)
        return b,limits
    def stopped(self):
        # An alarm may remain latched: stopped confirmation is distinct from readiness.
        if self.feed is None or time.monotonic()-self.stamp>.4:return False
        self.check_adapter()
        a=self.adapter
        return (not a.get('streaming') and not a.get('busy') and not a.get('inflight')
                and a.get('quiescent') is True and a.get('last_stop_ok') is True
                and self.feed['robot_mode'] in (5,9,10,11)
                and max(abs(float(x)) for x in self.feed['qd_actual'])<=.1)

def wait_future(node,future,timeout,guard=None):
    import rclpy
    deadline=time.monotonic()+timeout
    while True:
        if guard:guard()
        if future.done():
            result=future.result()
            if result is None:raise RuntimeError('异步请求返回空响应')
            return result
        if time.monotonic()>=deadline:raise RuntimeError('异步请求超时')
        rclpy.spin_once(node,timeout_sec=min(.02,max(0,deadline-time.monotonic())))

def stop_and_confirm(node,guard,handle=None,pending_goal=None,timeout=5.):
    """Request cancellation and controller Stop without waiting for cancel acknowledgement."""
    import rclpy
    futures=[];cancel_sent=False;started=time.monotonic();expected_goal=None
    if handle is not None:expected_goal=bytes(handle.goal_id.uuid).hex()
    if handle is not None:
        futures.append(handle.cancel_goal_async());cancel_sent=True
    client=node.create_client(Stop,'/dobot_bringup_ros2/srv/Stop')
    if not client.service_is_ready():
        # Do not spend seconds waiting for cancel before attempting Stop.
        client.wait_for_service(timeout_sec=.1)
    stop_future=client.call_async(Stop.Request()) if client.service_is_ready() else None
    previous=None;stable=0;last_stamp=None;deadline=time.monotonic()+timeout
    try:
        while time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.02)
            if not cancel_sent and pending_goal is not None and pending_goal.done():
                h=pending_goal.result()
                if h is not None and h.accepted:
                    expected_goal=bytes(h.goal_id.uuid).hex();futures.append(h.cancel_goal_async());cancel_sent=True
            try:
                guard.check_adapter();a=guard.adapter
                direct_ok=stop_future is not None and stop_future.done() and getattr(stop_future.result(),'res',-1)==0
                settled_adapter=(guard.adapter_stamp>started and not a.get('busy') and not a.get('streaming')
                    and not a.get('inflight') and a.get('quiescent') is True
                    and (expected_goal is None or a.get('goal_id')==expected_goal))
                pending_resolved=pending_goal is None or pending_goal.done()
                stopped=(pending_resolved and settled_adapter and (direct_ok or a.get('last_stop_ok') is True)
                    and guard.feed is not None and time.monotonic()-guard.stamp<=.4
                    and guard.feed['robot_mode'] in (4,5,9,10,11)
                    and max(abs(float(x)) for x in guard.feed['qd_actual'])<=.1)
            except Exception:stopped=False
            q=None if guard.feed is None else np.array(guard.feed['q_actual'],float)
            # Count distinct feedback messages only.
            if guard.stamp!=last_stamp:
                stable=stable+1 if stopped and previous is not None and max(abs(q-previous))<=.02 else 0
                previous=q;last_stamp=guard.stamp
            if stable>=3:
                return {'confirmed':True,'joint_rad':np.radians(q).tolist(),
                        'robot_mode':guard.feed['robot_mode'],'adapter':dict(guard.adapter),
                        'controller_stop_response':(getattr(stop_future.result(),'res',None)
                          if stop_future is not None and stop_future.done() else None)}
        raise RuntimeError('停止请求已发送，但5秒内未确认静止及ServoJ派发停止；需在控制端核实')
    finally:
        node.destroy_client(client)
