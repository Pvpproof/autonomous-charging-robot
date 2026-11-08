"""Passive route recording and offline return contracts. No ROS or motion output."""
import json
import math
from pathlib import Path
import uuid
from charging_robot.returning.route_cycle import CycleError, Observation, RouteCycle, StartStore, near


def atomic_json(path,data):
    temp=path.with_suffix('.tmp');temp.write_text(json.dumps(data,ensure_ascii=False,indent=2)+'\n')
    temp.replace(path)


def observe(s,map_sha):
    if s.get('pose') is None or s.get('pose_stamp') is None or not s.get('localization_epoch'):
        raise CycleError('尚无实际 TF 测量和定位会话')
    return Observation(map_sha,s['localization_epoch'],'map',s['pose'],s['pose_stamp'],
        s['pose_received_at'],s['pose_age'],s['odom_age'],s['scan_age'],
        *s['velocity'],s['pose'] is not None,s['stationary_since'],s['stationary_samples'],
        s.get('software_stop',False),s.get('manual',False) or s.get('manual_stop_pending',False))


class ChassisSession:
    def __init__(self,map_sha,start_path,logdir,snapshot,now):
        from charging_robot.returning.return_runtime import coordinate_sample
        coordinate_sample(snapshot,now)
        self.coordinate_samples=[]
        self.coordinate_snapshot=snapshot
        self.coordinate_error=None
        self.core=RouteCycle(map_sha,snapshot['localization_epoch'],StartStore(start_path))
        self.goal_id=uuid.uuid4().hex
        self.core.begin_outbound(observe(snapshot,map_sha),self.goal_id,now)
        self.directory=Path(logdir)/('chassis-'+self.core.cycle_id);self.directory.mkdir()
        self.written=0;self.last_phase=None;self.last_checkpoint=now;self.candidate_attempted=False
        self.save(now)

    def save(self,now):
        c=self.core
        if len(c.trace)>len(self.coordinate_samples) and not self.coordinate_error:
            try:
                from charging_robot.returning.return_runtime import coordinate_sample
                point=coordinate_sample(self.coordinate_snapshot,now)
                if point['source_stamp']!=c.trace[-1]['source_stamp']:
                    raise CycleError('坐标采样与轨迹时间不一致')
                self.coordinate_samples.append(point)
            except (ValueError,KeyError) as error:self.coordinate_error=str(error)
        with (self.directory/'actual-trace.jsonl').open('a') as stream:
            for point in c.trace[self.written:]:stream.write(json.dumps(point)+'\n')
        self.written=len(c.trace)
        if c.phase!=self.last_phase or now-self.last_checkpoint>=5:
            data=c.export_record()
            data.update(passive_recorder_integrated=True,arm_integration=False,
                        return_execution_enabled=False,coordinate_samples=self.coordinate_samples,coordinate_error=self.coordinate_error)
            atomic_json(self.directory/'cycle.json',data)
            self.last_phase=c.phase;self.last_checkpoint=now

    def update(self,snapshot,map_sha,now):
        self.coordinate_snapshot=snapshot
        c=self.core
        if c.phase not in ('outbound','arrival_check'):return
        try:
            if snapshot['navigation_id']!=self.goal_id or snapshot.get('cancel_requested'):
                raise CycleError('请求身份变化或人工取消；本轮记录不可用于返程')
            if c.phase=='outbound' and snapshot.get('goal_status') is not None:
                c.outbound_result(self.goal_id,'succeeded' if snapshot['goal_status']==4 else 'failed',
                                  snapshot['goal_finished_at'])
            obs=observe(snapshot,map_sha)
            c.record_outbound(obs,now)
            if c.phase=='arrival_check':
                try:c._still(obs,now,c.arrived_at)
                except CycleError:
                    if c.phase=='blocked':raise
                else:c.phase='outbound_stopped'
        except CycleError as error:c.stop_or_take_over(str(error))
        self.save(now)

    def stop(self,reason,now):
        self.core.stop_or_take_over(reason);self.save(now)

    def return_candidate(self):
        self.candidate_attempted=True
        c=self.core
        if c.phase!='outbound_stopped':raise CycleError('去程未成功结束并停稳')
        if any(p['observed_v_m_s']<-.01 for p in c.trace):
            raise CycleError('去程包含倒车，不能保证反向后仍全程倒车')
        poses=[]
        for point in reversed(c.trace):
            if not poses or point['pose']!=poses[-1]:poses.append(point['pose'])
        if len(poses)<2 or sum(math.dist(a[:2],b[:2]) for a,b in zip(poses,poses[1:]))<.05:
            raise CycleError('实际路径不足以构成返程')
        for a,b in zip(poses,poses[1:]):
            forward=(b[0]-a[0])*math.cos(a[2])+(b[1]-a[1])*math.sin(a[2])
            if forward>.03:raise CycleError('实测位移与纯倒车方向不一致，需核查定位或去程方向')
        plan=dict(cycle_id=c.cycle_id,goal_id=uuid.uuid4().hex,map_sha256=c.map_sha256,
                  localization_epoch=c.epoch,frame_id='map',poses=poses,
                  reverse_only=True,executable=False,speed_limit_m_s=None,
                  source='actual_map_pose_measurements',required_action='FollowPath',
                  blockers=['倒车专用控制器尚未验证','后方障碍物覆盖与停车链路尚未验证',
                            '底盘控制权仲裁与接管尚未验证','倒车速度尚未核定'])
        plan['coordinate_samples']=self.coordinate_samples if not self.coordinate_error else []
        plan['coordinate_error']=self.coordinate_error
        plan['required_action']='supervised-feedback-return'
        plan['blockers']=['需新鲜成对坐标与停点一致','需独立底盘仲裁及现场验收','需人工持续监护与按住运行']
        atomic_json(self.directory/'return-candidate.json',plan)
        return plan

    def status(self):
        c=self.core
        names={'outbound':'记录实际去程','arrival_check':'导航成功，继续记录至停稳',
               'outbound_stopped':'去程记录已完成；自动返程尚未启用',
               'blocked':'记录已失效：'+str(c.block_reason)}
        pair_note=('配对失效：'+self.coordinate_error) if self.coordinate_error else '坐标配对 {} 点'.format(len(self.coordinate_samples))
        return names.get(c.phase,c.phase)+'\n实测轨迹 {} 点；{}；不调用机械臂。'.format(len(c.trace),pair_note)


class ReturnMonitor:
    """Offline feedback monitor, NOT a path controller; never emits velocities.

    Limits are provisional test guards, not calibrated hardware safety limits.
    A future FollowPath adapter must continuously provide actual feedback.
    """
    def __init__(self,plan,capabilities,store,xy_tolerance=.10,yaw_tolerance=.10):
        capabilities.require()  # Defaults unavailable; no implicit reverse speed.
        self.plan=plan;self.store=store;self.poses=plan['poses']
        if len(self.poses)<2:raise CycleError('返程路径不足')
        self.lengths=[0.]
        for a,b in zip(self.poses,self.poses[1:]):
            self.lengths.append(self.lengths[-1]+math.dist(a[:2],b[:2]))
        self.progress=0.;self.last=None;self.blocked=False;self.finished_at=None;self.completed=False
        self.last_progress_at=None;self.progress_anchor=0.;self.speed_limit=capabilities.speed_limit_m_s
        self.xy=xy_tolerance;self.yaw=yaw_tolerance

    def fail(self,reason):
        self.blocked=True;raise CycleError(reason)

    def update(self,obs,now):
        if self.completed:raise CycleError('本轮已完成，不能继续改写实际停点')
        if self.blocked:raise CycleError('监测已中止，禁止自动恢复')
        p=self.plan
        if (obs.map_sha256!=p['map_sha256'] or obs.localization_epoch!=p['localization_epoch'] or
            obs.frame_id!='map' or not obs.localized or obs.manual_active or obs.software_stop or
            not all(math.isfinite(v) for v in (*obs.pose,obs.source_stamp,obs.v_m_s,obs.w_rad_s)) or
            not all(math.isfinite(t) and 0<=t<=1 for t in
                    (now-obs.received_at,obs.pose_age_s,obs.odom_age_s,obs.scan_age_s))):
            self.fail('地图/定位/环境反馈或控制权失效')
        if self.last:
            if obs.source_stamp==self.last.source_stamp:
                if obs.pose!=self.last.pose:self.fail('同时间测量改变')
                return self.progress
            if (obs.source_stamp<self.last.source_stamp or
                not 0<obs.received_at-self.last.received_at<=1):self.fail('反馈乱序或中断')
        if obs.v_m_s>.01:self.fail('返程反馈出现前进，停止接管流程')
        if abs(obs.v_m_s)>self.speed_limit+.01:self.fail('实际速度超过独立核定的倒车上限')
        if self.last is None and not near(obs.pose,self.poses[0],self.xy,self.yaw):
            self.fail('实际返程起点与记录终点不符')
        candidates=[]
        for i,(a,b) in enumerate(zip(self.poses,self.poses[1:])):
            dx,dy=b[0]-a[0],b[1]-a[1];length=math.hypot(dx,dy)
            if length<1e-6:continue
            t=max(0.,min(1.,((obs.pose[0]-a[0])*dx+(obs.pose[1]-a[1])*dy)/(length*length)))
            progress=self.lengths[i]+t*length
            if not self.progress-.03<=progress<=self.progress+.50:continue
            x,y=a[0]+t*dx,a[1]+t*dy
            yaw=a[2]+t*math.atan2(math.sin(b[2]-a[2]),math.cos(b[2]-a[2]))
            distance=math.hypot(obs.pose[0]-x,obs.pose[1]-y)
            heading=abs(math.atan2(math.sin(obs.pose[2]-yaw),math.cos(obs.pose[2]-yaw)))
            candidates.append((distance,progress,heading))
        if not candidates:self.fail('没有连续可达的路径投影，禁止跳到交叉路段')
        distance,progress,heading=min(candidates)
        if distance>self.xy or heading>self.yaw:self.fail('实际横向或车身朝向误差超限')
        if self.last_progress_at is None or progress>self.progress_anchor+.005:
            self.last_progress_at=now;self.progress_anchor=progress
        if now-self.last_progress_at>10:self.fail('返程持续无实际进展')
        self.progress=max(self.progress,progress);self.last=obs
        return self.progress

    def action_result(self,cycle_id,goal_id,status,finished_at,now):
        if (self.blocked or self.last is None or cycle_id!=self.plan['cycle_id'] or
            goal_id!=self.plan['goal_id'] or status!='succeeded' or
            not all(math.isfinite(t) for t in (finished_at,now)) or
            not self.last.received_at<=finished_at<=now or now-finished_at>1 or now-self.last.received_at>1 or
            self.lengths[-1]-self.progress>self.xy):self.fail('返程实际进度或本轮结果未通过验收')
        self.finished_at=finished_at

    def save_actual_stop(self,obs,now):
        self.update(obs,now)
        if (self.finished_at is None or obs.still_since is None or obs.still_samples<3 or
            obs.received_at-max(obs.still_since,self.finished_at)<2 or
            abs(obs.v_m_s)>.01 or abs(obs.w_rad_s)>.02 or
            not near(obs.pose,self.poses[-1],self.xy,self.yaw)):
            raise CycleError('尚未取得返程结果之后的新鲜实际停点验收')
        saved=self.store.write(self.plan['map_sha256'],obs.pose,'observed_return_stop',self.plan['cycle_id'])
        self.completed=True
        return saved
