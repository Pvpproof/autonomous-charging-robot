"""Offline supervised/no-rear-detection prototype. Never publishes to a robot.

Uses current measured pose to follow reversed geometry, not recorded commands.
No site confirmation in this module is a claim that real hardware was verified.
"""
from dataclasses import dataclass
import math
from charging_robot.returning.chassis_route import ReturnMonitor
from charging_robot.returning.offline_reverse import ReversePursuit
from charging_robot.returning.route_cycle import CycleError


@dataclass(frozen=True)
class TestConditions:
    cycle_id: str = ''
    goal_id: str = ''
    operator_present: bool = False
    route_cleared: bool = False
    physical_stop_ready: bool = False
    arm_stowed: bool = False
    exclusive_control_confirmed: bool = False
    no_rear_detection_acknowledged: bool = False
    speed_limit_m_s: float | None = None
    minimum_radius_m: float | None = None
    max_time_s: float | None = None
    max_distance_m: float | None = None
    heartbeat_timeout_s: float = .25
    controller_id: str = 'offline-supervised-pursuit'

    def require(self):
        if not (self.cycle_id and self.goal_id and self.operator_present and
            self.route_cleared and self.physical_stop_ready and self.arm_stowed and
            self.exclusive_control_confirmed and self.no_rear_detection_acknowledged):
            raise CycleError('缺少本轮现场监护/路线清空/实体停车/机构收拢/控制权确认')
        values=(self.speed_limit_m_s,self.minimum_radius_m,self.max_time_s,
                self.max_distance_m,self.heartbeat_timeout_s)
        if not all(v is not None and math.isfinite(v) and v>0 for v in values):
            raise CycleError('需要明确的速度、曲率、时间、距离和心跳边界')


class SupervisedDraft:
    def __init__(self,plan,conditions,store):
        conditions.require()
        if (conditions.cycle_id!=plan['cycle_id'] or conditions.goal_id!=plan['goal_id'] or
            plan.get('reverse_only') is not True):raise CycleError('确认信息未绑定本轮返程')
        self.plan=plan;self.conditions=conditions
        self.monitor=ReturnMonitor(plan,conditions,store)
        self.follower=ReversePursuit(plan['poses'],conditions.speed_limit_m_s,conditions.minimum_radius_m)
        self.phase='ready';self.reason=None;self.heartbeat_at=None;self.holding=False
        self.started_at=None;self.last_tick=None;self.last_observation=None;self.measured_distance=0.

    def output(self,intent,v=0.,w=0.):
        return dict(intent=intent,v=v,w=w,publish=False,execution_enabled=False,
            cycle_id=self.plan['cycle_id'],goal_id=self.plan['goal_id'],phase=self.phase,
            measured_distance_m=self.measured_distance,reason=self.reason,
            mode='supervised-no-rear-detection-offline',
            detects_new_obstacles=False,path_source=self.plan.get('source','unspecified'))

    def stop(self,reason):
        self.phase='stopped';self.reason=reason
        return self.output('STOP_REQUIRED')

    def heartbeat(self,cycle_id,goal_id,at,held):
        if self.phase=='stopped':return self.output('STOP_REQUIRED')
        if (cycle_id!=self.plan['cycle_id'] or goal_id!=self.plan['goal_id'] or
            not math.isfinite(at) or (self.heartbeat_at is not None and at<self.heartbeat_at)):
            return self.stop('心跳轮次不匹配或乱序')
        self.heartbeat_at=at;self.holding=bool(held)
        if not held:return self.stop('现场人员已松开保持运行')
        return self.output('HEARTBEAT_ONLY')

    def held_now(self,now):
        return (self.holding and self.heartbeat_at is not None and
                0<=now-self.heartbeat_at<=self.conditions.heartbeat_timeout_s)

    def start(self,obs,now):
        if self.phase!='ready' or not self.held_now(now):return self.stop('未按住运行或阶段不允许')
        if (obs.still_since is None or not math.isfinite(obs.still_since) or
            obs.still_samples<3 or obs.received_at-obs.still_since<2 or
            abs(obs.v_m_s)>.01 or abs(obs.w_rad_s)>.02):return self.stop('未取得实测停稳前提')
        try:self.monitor.update(obs,now)
        except CycleError as error:return self.stop(str(error))
        self.phase='running';self.started_at=now;self.last_tick=now;self.last_observation=obs
        return self.output('STARTED_DRAFT_ONLY')

    def step(self,obs,now,*,owns_control):
        if self.phase!='running':return self.output('STOP_REQUIRED')
        c=self.conditions
        if not math.isfinite(now) or not self.held_now(now):return self.stop('保持运行心跳丢失')
        if not owns_control:return self.stop('控制权丢失')
        if not 0<=now-self.last_tick<=c.heartbeat_timeout_s:return self.stop('控制循环停顿')
        self.last_tick=now
        if now-self.started_at>=c.max_time_s:return self.stop('达到本次短时测试边界')
        if any(not math.isfinite(age) or age<0 or age>c.heartbeat_timeout_s
               for age in (now-obs.received_at,obs.pose_age_s,obs.odom_age_s)):
            return self.stop('短程测试定位或里程计反馈过期')
        try:self.monitor.update(obs,now)
        except CycleError as error:return self.stop(str(error))
        if obs.source_stamp!=self.last_observation.source_stamp:
            self.measured_distance+=math.dist(obs.pose[:2],self.last_observation.pose[:2])
            self.last_observation=obs
        if self.measured_distance>=c.max_distance_m:return self.stop('达到本次实测行驶距离边界')
        try:v,w,done,error=self.follower.control(obs.pose)
        except ValueError as error:return self.stop(str(error))
        if done:
            try:self.monitor.action_result(self.plan['cycle_id'],self.plan['goal_id'],'succeeded',now,now)
            except CycleError as error:return self.stop(str(error))
            self.phase='goal_wait_still'
            return self.output('GOAL_REACHED_DRAFT_AWAIT_STOP_VALIDATION')
        return self.output('REVERSE_DRAFT_ONLY',v,w)

    def observe_stop(self,obs,now,*,owns_control):
        if self.phase!='goal_wait_still':return self.output('STOP_REQUIRED')
        if (not self.held_now(now) or not owns_control or
            not 0<=now-self.last_tick<=self.conditions.heartbeat_timeout_s):
            return self.stop('停点验收期间监护或控制权失效')
        self.last_tick=now
        if any(not math.isfinite(age) or age<0 or age>self.conditions.heartbeat_timeout_s
               for age in (now-obs.received_at,obs.pose_age_s,obs.odom_age_s)):
            return self.stop('停点验收反馈过期')
        try:saved=self.monitor.save_actual_stop(obs,now)
        except CycleError as error:
            if self.monitor.blocked:return self.stop(str(error))
            return self.output('AWAIT_ACTUAL_STOP')
        self.phase='complete'
        data=self.output('ACTUAL_REFERENCE_SAVED')
        data['saved_reference']=saved
        return data
