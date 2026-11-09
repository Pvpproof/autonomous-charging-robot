"""Ordinary motion feedback and confirmed-stop guards."""
import json, math, time
from source_feedback import SourceProgress
import numpy as np
from std_msgs.msg import String
from dobot_msgs_v4.srv import Stop
from std_srvs.srv import Trigger

ADAPTER_PROTOCOL = 4



class FeedbackGuard:
    def __init__(self,node):
        self.node=node;self.feed=None;self.stamp=0;self.history=[];self.samples=[];self.error=None
        self.source=SourceProgress()
        self.adapter=None;self.adapter_error=None;self.adapter_stamp=0;self.expected_goal=None
        node.create_subscription(String,'/dobot_bringup_ros2/msg/FeedInfo',self.receive,10)
        node.create_subscription(String,'/charge_robot/trajectory_adapter_status',self.receive_adapter,10)
    def receive(self,msg):
        try:
            d=json.loads(msg.data)
            for key in ('q_actual','qd_actual'):
                a=np.asarray(d[key],float)
                if a.shape!=(6,) or not np.isfinite(a).all():raise ValueError(key)
            for key in ('ErrorStatus','CollisionStates','robot_mode'):
                if key not in d:raise ValueError(key)
            if not self.source.accept(d):return
            now=time.monotonic();self.feed=d;self.stamp=now
            self.samples.append(dict(t=time.time(),**d));self.samples=self.samples[-20000:]
            self.error=None
        except Exception as e:self.error='反馈格式无效:'+str(e)
    def receive_adapter(self,msg):
        try:
            d=json.loads(msg.data)
            if d.get('protocol')!=ADAPTER_PROTOCOL:raise ValueError('adapter protocol')
            for key in ('busy','streaming','quiescent','stop_latched'):
                if type(d.get(key)) is not bool:raise ValueError('adapter '+key)
            if type(d.get('inflight')) is not int or d['inflight']<0:raise ValueError('adapter inflight')
            if type(d.get('stop_epoch')) is not int or d['stop_epoch']<0:raise ValueError('adapter stop epoch')
            t=d.get('stop_requested_monotonic')
            if type(t) not in (int,float) or not math.isfinite(t) or t<0:raise ValueError('adapter stop time')
            self.adapter=d;self.adapter_stamp=time.monotonic();self.adapter_error=None
        except Exception as e:self.adapter_error=str(e)
    def check(self,contact=True):
        now=time.monotonic()
        if self.error:raise RuntimeError(self.error)
        self.source.check()
        if self.feed is None or now-self.stamp>.4:raise RuntimeError('机器人反馈中断/超时')
        if self.feed['ErrorStatus'] or self.feed['CollisionStates']:raise RuntimeError('机器人碰撞或报警')
        if self.feed['robot_mode'] not in (5,7):raise RuntimeError('运动期间RobotMode异常:'+str(self.feed['robot_mode']))
        self.check_adapter()
    def check_adapter(self):
        d=self.adapter
        if self.adapter_error:raise RuntimeError('适配器心跳格式异常: '+self.adapter_error)
        if d is None or time.monotonic()-self.adapter_stamp>.5:raise RuntimeError('新版轨迹适配器未就绪或心跳超时，请重新start')
    def idle(self):
        self.check(False);self.check_adapter()
        if (self.feed['robot_mode']!=5 or self.adapter.get('busy') or self.adapter.get('streaming') or self.adapter.get('stop_latched')
                or self.adapter.get('inflight') or self.adapter.get('quiescent') is not True
                or max(abs(float(v)) for v in self.feed['qd_actual'])>.1):
            raise RuntimeError('机械臂/适配器尚未静止')
    def stopped(self):
        # An alarm may remain latched: stopped confirmation is distinct from readiness.
        if self.error:return False
        self.source.check()
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

class StopUnconfirmed(RuntimeError):
    def __init__(self,evidence):
        self.evidence=evidence
        super().__init__('停止未确认；需在控制端核实')

def stop_and_confirm(node,guard,handle=None,pending_goal=None,timeout=5.):
    """Independent best-effort cancellation/Stop; retain every outcome, never clear alarms."""
    import rclpy
    started=time.monotonic();deadline=started+timeout;expected_goal=None;clients=[]
    evidence=dict(confirmed=False,status='uncertain',started_monotonic=started,
                  cancellation=[],adapter_latch={'submitted':False},controller_stop={'submitted':False},actual_confirmation={})
    cancels=[];late_handled=pending_goal is None;stop_future=None
    def cancel(h,label):
        nonlocal expected_goal
        entry={'kind':label,'submitted':False,'response_state':'not_submitted'};evidence['cancellation'].append(entry)
        try:expected_goal=bytes(h.goal_id.uuid).hex()
        except BaseException as e:entry['goal_id_error']=repr(e)
        try:
            f=h.cancel_goal_async();entry['submitted']=True;entry['response_state']='pending';cancels.append((f,entry))
        except BaseException as e:entry['error']=repr(e)
    # Latch the sole ServoJ writer independently of the Action cancellation API.
    # Never wait for any of these requests before submitting the other requests.
    initial_epoch=(guard.adapter or {}).get('stop_epoch',-1)
    latch_future=None
    try:
        latch_client=node.create_client(Trigger,'/charge_robot/trajectory_adapter_stop');clients.append(latch_client)
        if not latch_client.service_is_ready():raise RuntimeError('adapter latch unavailable')
        latch_future=latch_client.call_async(Trigger.Request());evidence['adapter_latch']['submitted']=True
    except BaseException as e:evidence['adapter_latch']['error']=repr(e)
    if handle is not None:cancel(handle,'active')
    # No cancellation outcome can prevent the independent controller Stop attempt.
    try:
        client=node.create_client(Stop,'/dobot_bringup_ros2/srv/Stop');clients.append(client)
        if not client.service_is_ready():client.wait_for_service(timeout_sec=.1)
        if not client.service_is_ready():raise RuntimeError('Stop service unavailable')
        stop_future=client.call_async(Stop.Request());evidence['controller_stop']['submitted']=True
    except BaseException as e:evidence['controller_stop']['error']=repr(e)
    previous=None;stable=0;last_token=guard.source.token
    try:
        while time.monotonic()<deadline:
            try:rclpy.spin_once(node,timeout_sec=.02)
            except BaseException as e:evidence['spin_error']=repr(e)
            if not late_handled:
                try:
                    if pending_goal.done():
                        h=pending_goal.result();late_handled=True
                        if h is not None and h.accepted:cancel(h,'late')
                        else:evidence['cancellation'].append({'kind':'late','accepted':False})
                except BaseException as e:
                    evidence['pending_goal_error']=repr(e)  # Unresolved => cannot confirm stop.
            for f,entry in cancels:
                try:
                    if f.done() and 'result' not in entry and 'response_error' not in entry:
                        entry['result']=repr(f.result());entry['response_state']='returned'
                except BaseException as e:entry['response_error']=repr(e);entry['response_state']='failed'
            direct_ok=False
            try:
                if latch_future is not None and latch_future.done():
                    response=latch_future.result()
                    evidence['adapter_latch'].update(accepted=response.success is True,message=response.message)
            except BaseException as e:evidence['adapter_latch']['response_error']=repr(e)
            try:
                if stop_future is not None and stop_future.done():
                    res=getattr(stop_future.result(),'res',None)
                    evidence['controller_stop']['res']=res;direct_ok=res==0
            except BaseException as e:evidence['controller_stop']['response_error']=repr(e)
            try:
                if guard.error:raise RuntimeError(guard.error)
                guard.source.check();guard.check_adapter();a=guard.adapter
                settled=(guard.adapter_stamp>started and not a.get('busy') and not a.get('streaming')
                    and not a.get('inflight') and a.get('quiescent') is True
                    and a['stop_epoch']>initial_epoch and a['stop_requested_monotonic']>=started
                    and (expected_goal is None or a.get('goal_id')==expected_goal))
                stopped=(late_handled and settled and (direct_ok or a.get('last_stop_ok') is True)
                    and guard.stamp>started and time.monotonic()-guard.stamp<=.4
                    and guard.feed['robot_mode'] in (4,5,9,10,11)
                    and max(abs(float(x)) for x in guard.feed['qd_actual'])<=.1)
                q=np.asarray(guard.feed['q_actual'],float);token=guard.source.token
                if token!=last_token:
                    stable=stable+1 if stopped and previous is not None and max(abs(q-previous))<=.02 else 0
                    previous=q;last_token=token
                evidence['actual_confirmation']=dict(stable_source_frames=stable,source_token=token,
                    adapter=dict(a),checked_monotonic=time.monotonic())
                if stable>=3:
                    evidence.update(confirmed=True,status='stopped',joint_rad=np.radians(q).tolist(),
                        robot_mode=guard.feed['robot_mode'],finished_monotonic=time.monotonic())
                    return evidence
            except BaseException as e:
                stable=0;evidence['actual_confirmation']['error']=repr(e)
        evidence['finished_monotonic']=time.monotonic();raise StopUnconfirmed(evidence)
    finally:
        for client in clients:
            try:node.destroy_client(client)
            except BaseException as e:evidence['cleanup_error']=repr(e)

def stop_then_report(stop,report,original_error,write):
    """Stop first. Filesystem failures cannot suppress stopping or replace the original fault."""
    data={'status':'uncertain','retracted':False,'original_fault':repr(original_error)}
    try:
        ev=stop();data['stop_evidence']=ev
        if ev.get('confirmed') is True:data.update(status='stopped',final_joint_rad=ev['joint_rad'])
    except BaseException as e:
        data['stop_failure']=repr(e)
        if hasattr(e,'evidence'):data['stop_evidence']=e.evidence
    # Only after the stop attempt may even exists()/read_text()/JSON parsing run.
    try:
        previous=json.loads(report.read_text())
        if not isinstance(previous,dict):raise ValueError('report is not an object')
        previous.update(data);data=previous
    except BaseException as e:data['report_read_error']=repr(e)
    try:write(report,data)
    except BaseException as e:
        data['report_write_error']=repr(e)
        import sys
        try:print('STOP_REPORT_FALLBACK '+json.dumps(data,ensure_ascii=False),file=sys.stderr,flush=True)
        except BaseException:pass  # Even stderr failure cannot replace the original motion fault.
    return data
