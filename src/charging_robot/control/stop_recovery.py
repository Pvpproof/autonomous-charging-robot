"""Separate operator UI recovery from downstream motion authorization."""
import math,time

def finite(v):return type(v) in (float,int) and math.isfinite(v)

def motion_reset_reasons(s):
    reasons=[]
    if s.get('busy') or s.get('navigation_busy') or s.get('return_active'):reasons.append('底盘任务尚未结束')
    age=s.get('odom_age');velocity=s.get('velocity')
    if not finite(age) or not 0<=age<=1:reasons.append('里程计未就绪，停稳未确认')
    elif not (isinstance(velocity,(list,tuple)) and len(velocity)==2 and all(finite(v) for v in velocity) and abs(velocity[0])<=.01 and abs(velocity[1])<=.02):reasons.append('底盘仍运动或速度无效')
    if not finite(s.get('stationary_s')) or s['stationary_s']<2:reasons.append('尚未连续观察停稳两秒')
    age=s.get('arbiter_age');a=s.get('arbiter',{})
    if not finite(age) or not 0<=age<=.25:reasons.append('独立仲裁反馈未就绪')
    elif not a.get('instance_id') or a.get('graph_ready') is not True:reasons.append('底盘控制链未就绪')
    return reasons

class StopRecoveryBridge:
    def clear_recovered_tool_hold(self):
        # Caller separately checks live arm tools and obtains physical-clear
        # confirmation. Only an already gated panel recovery can use this path.
        with self.lock:
            if not self.motion_recovery_required:raise ValueError('请先恢复面板，保持底盘运动禁用')
            self.docking_hold=False
        self.event('stale_tool_hold_cleared_for_panel_recovery',motion_enabled=False,
                   operator_confirmed_clear=True,automatic_resume=False)

    def release_stop(self):
        with self.lock:
            now=time.monotonic();s=self._snapshot(now)
            # Reset standstill evidence is based on all recent feedback, rather
            # than time since a navigation goal that may never have existed.
            s['stationary_s']=self.stop_observation.duration(self.stop_observation.first)
            reasons=motion_reset_reasons(s)
            self.manual=False;self.manual_target=(0.,0.);self.manual_stop_pending=False
            self.motion_recovery_required=True
            if reasons:
                if self.return_runtime:self.return_state=self.return_runtime.stop('人工恢复面板；旧返程不续行')
                self.software_stop=False;self.stop_cause=None;self.reset_arbiter=False;self.stop_reset_pending=None
                self.cancel_requested=True
                result=dict(kind='panel_only',motion_enabled=False,reasons=reasons)
            else:
                if self.stop_reset_pending is not None:raise ValueError('正在等待底盘就绪确认')
                self.software_stop=True;self.return_runtime=None;self.reset_arbiter=True
                if not getattr(self,'stop_cause',None):self.stop_cause=dict(code='reset_confirmation_pending',detail='等待底盘就绪确认',source_monotonic_s=now)
                self.stop_reset_pending=dict(instance_id=self.arbiter_state['instance_id'],requested_at=now,sequence=None,request_id=self.control_owner+'-'+str(self.control_sequence+1))
                if self.raw_tracker.error:self.raw_tracker.reset()
                result=dict(kind='downstream_reset',motion_enabled=False,reasons=[])
        if result['kind']=='panel_only':
            self.commands.put(('cancel',()))
            self.event('panel_recovered_motion_disabled',reasons=reasons,standstill_verified=False,automatic_resume=False)
        else:self.event('software_stop_reset_requested',automatic_resume=False)
        return result
