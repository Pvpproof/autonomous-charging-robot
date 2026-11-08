"""Raw driver odometry is a scoped integrator, never the identically named EKF TF.
No ROS, device, network or automatic recovery. Sensor-consistency is not slip detection.
"""
import copy,hashlib,json,math,uuid
from collections import deque
from release_policy import feedback_speed_exceeded
from pathlib import Path
from control_causes import recording_block_reason,control_evidence
from route_cycle import CycleError,near
from pose_aware_reverse import PoseAwareReversePursuit
from offline_reverse import wrap

SOURCE='raw_driver_odom_measurements_v1'
def key(value):return hashlib.sha256(json.dumps(value,sort_keys=True,separators=(',',':'),allow_nan=False).encode()).hexdigest()
def distance(a,b):return math.dist(a[:2],b[:2])
def finite(v):return isinstance(v,(int,float)) and not isinstance(v,bool) and math.isfinite(v)

# Preserve the original absolute feedback speed envelope; this does not
# evaluate path curvature or divide angular speed by low linear speed.
MIN_RETURN_RADIUS=1.78
RAW_MAX_LINEAR=.6
RAW_MAX_ANGULAR=RAW_MAX_LINEAR/MIN_RETURN_RADIUS+.05
def diagnostic(value):
    # Rejected non-finite or malformed input must still be valid JSON evidence.
    if isinstance(value,float) and not math.isfinite(value):return repr(value)
    if isinstance(value,dict):return {str(k):diagnostic(v) for k,v in value.items()}
    if isinstance(value,(list,tuple)):return [diagnostic(v) for v in value]
    if value is None or isinstance(value,(str,int,float,bool)):return value
    return repr(value)

class RawTracker:
    def __init__(self):
        self.identity=None;self.source_key=None;self.epoch=uuid.uuid4().hex;self.error=None;self.latest=None;self.sequence=0;self.history=deque(maxlen=6000);self.still_since=None;self.still_samples=0;self.rejection=None;self.pending=None
    def fail(self,text):
        self.error=text;self.still_since=None;self.still_samples=0
        if self.rejection is None:self.rejection=dict(reason=text,raw_epoch=self.epoch,source_key=self.source_key,input=diagnostic(self.pending),last_valid=diagnostic(copy.deepcopy(self.latest)))
        raise CycleError(text)
    def bind(self,identity):
        if not identity:return
        digest=key(identity)
        if self.source_key and digest!=self.source_key:
            self.error='原始里程发布端/驱动身份改变；需停车显式复位并重新录制'
        self.identity=copy.deepcopy(identity);self.source_key=digest
    def reset(self):
        self.epoch=uuid.uuid4().hex;self.error=None;self.latest=None;self.history.clear();self.still_since=None;self.still_samples=0;self.rejection=None;self.pending=None
    def update(self,pose,velocity,stamp,received,ros_now,frame,child,publisher_gid):
        if self.error:raise CycleError(self.error)
        if not self.identity:return None
        self.pending=dict(pose=pose,velocity=velocity,source_stamp=stamp,received_at=received,ros_now=ros_now,frame=frame,child=child,publisher_gid=publisher_gid)
        if publisher_gid!=self.identity['publisher_gid']:return self.fail('原始里程消息发布端不符')
        if frame!='odom_combined' or child!='base_footprint':return self.fail('原始里程帧不符')
        if len(pose)!=3 or len(velocity)!=3 or not all(finite(x) for x in (*pose,*velocity,stamp,received,ros_now)):return self.fail('原始里程含无效数值')
        if not 0<=ros_now-stamp<=.25:return self.fail('原始里程源时间过期或倒退')
        if abs(velocity[0])>RAW_MAX_LINEAR:return self.fail('原始里程纵向速度异常')
        if abs(velocity[1])>.01:return self.fail('原始里程侧向速度异常')
        if abs(velocity[2])>RAW_MAX_ANGULAR:return self.fail('原始里程绝对角速度异常')
        old=self.latest
        if old:
            dt=stamp-old['source_stamp'];dr=received-old['received_at']
            if not 0<dt<=.25 or not 0<dr<=.25:return self.fail('原始里程重复、倒序或失联；旧轨迹失效')
            expected=abs(velocity[0])*dt
            if distance(pose,old['pose'])>.6*dt+.01:return self.fail('原始里程位移跳变/重置')
            dx=velocity[0]*math.cos(old['pose'][2])*dt;dy=velocity[0]*math.sin(old['pose'][2])*dt
            if math.hypot(pose[0]-old['pose'][0]-dx,pose[1]-old['pose'][1]-dy)>.02+expected*.15:return self.fail('原始位移与速度积分不一致')
            if abs(wrap(pose[2]-old['pose'][2]-velocity[2]*dt))>.015+abs(velocity[2]*dt)*.2:return self.fail('原始航向跳变/重置')
        quiet=abs(velocity[0])<=.01 and abs(velocity[2])<=.02
        if quiet:
            if self.still_since is None:self.still_since=received
            self.still_samples+=1
        else:self.still_since=None;self.still_samples=0
        self.sequence+=1
        sample=dict(sequence=self.sequence,source=SOURCE,source_key=self.source_key,raw_epoch=self.epoch,source_stamp=stamp,received_at=received,pose=list(pose),velocity=list(velocity),published_frame=frame,child_frame=child,publisher_gid=publisher_gid)
        self.latest=sample;self.history.append(sample);self.pending=None;return sample
    def current(self,now,ros_now):
        if self.error:raise CycleError(self.error)
        if not self.latest:raise CycleError('尚未取得经发布端验证的原始里程')
        if not 0<=now-self.latest['received_at']<=.25 or not 0<=ros_now-self.latest['source_stamp']<=.25:raise CycleError('原始里程反馈过期')
        return copy.deepcopy(self.latest)
    def since(self,sequence):
        if self.error:raise CycleError(self.error)
        if self.history and sequence<self.history[0]['sequence']-1:raise CycleError('原始里程记录缓存溢出')
        return copy.deepcopy([s for s in self.history if s['sequence']>sequence])

def scope(snapshot):
    v=snapshot.get('raw_odom');a=snapshot.get('arbiter',{})
    if not isinstance(v,dict) or snapshot.get('raw_odom_error'):raise CycleError(snapshot.get('raw_odom_error') or '原始里程未就绪')
    if (not a.get('raw_odom_ready') or a.get('raw_source_key')!=v['source_key'] or not a.get('raw_epoch') or not a.get('instance_id')):raise CycleError('独立仲裁未验证同一原始里程源')
    return dict(source_key=v['source_key'],panel_raw_epoch=v['raw_epoch'],arbiter_raw_epoch=a['raw_epoch'],arbiter_instance=a['instance_id'],panel_owner=snapshot['control_owner'])

def raw_timing(snapshot,now):
    """Evidence only; never freshen a sample or change an acceptance limit."""
    s=snapshot.get('raw_odom') or {};last=snapshot.get('raw_odom_timing') or {}
    received=s.get('received_at',last.get('received_at'))
    stamp=s.get('source_stamp',last.get('source_stamp'))
    taken=snapshot.get('snapshot_taken_at',last.get('checked_at'))
    ros_now=snapshot.get('ros_now')
    return diagnostic(dict(check_monotonic=now,snapshot_taken_at=taken,
        snapshot_completed_at=snapshot.get('snapshot_completed_at'),
        snapshot_age_s=now-taken if finite(now) and finite(taken) else None,
        sequence=s.get('sequence',last.get('sequence')),source_stamp=stamp,
        received_at=received,receive_age_s=now-received if finite(now) and finite(received) else None,
        ros_now_at_snapshot=ros_now,
        source_age_at_snapshot_s=ros_now-stamp if finite(ros_now) and finite(stamp) else None,
        tracker_error=snapshot.get('raw_odom_error'),limit_s=.25))

def sample_now(snapshot,now):
    s=snapshot.get('raw_odom')
    if snapshot.get('raw_odom_error') or not s:raise CycleError(snapshot.get('raw_odom_error') or '缺少原始里程')
    if not 0<=now-s['received_at']<=.25 or not 0<=snapshot['ros_now']-s['source_stamp']<=.25:raise CycleError('原始里程过期')
    return s

def check_raw_plan(plan):
    if plan.get('source')!=SOURCE or plan.get('reverse_only') is not True or plan.get('executable') is not False:raise CycleError('不是本模式原始里程候选')
    pts=plan.get('raw_samples',[])
    if len(pts)<3:raise CycleError('原始轨迹不足')
    reverse=[];last=None
    for s in pts:
        if s['source_key']!=plan['scope']['source_key'] or s['raw_epoch']!=plan['scope']['panel_raw_epoch']:raise CycleError('轨迹混入其他原始会话')
        if not all(finite(x) for x in (*s['pose'],*s['velocity'],s['source_stamp'],s['received_at'])):raise CycleError('轨迹数值无效')
        if s['velocity'][0]<-.01:raise CycleError('去程含倒车，不能纯倒车返回')
        if last and (s['sequence']!=last['sequence']+1 or not 0<s['source_stamp']-last['source_stamp']<=.25):raise CycleError('轨迹缺失、倒序或间断')
        if not reverse or s['pose']!=reverse[-1]:reverse.append(s['pose'])
        last=s
    reverse=list(reversed(reverse))
    if reverse!=plan['poses'] or key(pts)!=plan['samples_sha256']:raise CycleError('轨迹几何或样本被修改')
    if sum(distance(a,b) for a,b in zip(reverse,reverse[1:]))<.05:raise CycleError('轨迹长度不足')
    for a,b in zip(reverse,reverse[1:]):
        if (b[0]-a[0])*math.cos(a[2])+(b[1]-a[1])*math.sin(a[2])>.01:raise CycleError('原始去程方向不支持纯倒车')

class RawChassisSession:
    """Record trustworthy outbound feedback independently of return capability.

    Direction failures never impersonate sensor loss. Integrity/control failures
    still block recording; no uncalibrated curvature model qualifies the path.
    """
    def __init__(self,map_sha,start_path,logdir,snapshot,now,provider):
        self.core=self;self.phase='outbound';self.block_reason=None;self.cycle_id=uuid.uuid4().hex;self.goal_id=uuid.uuid4().hex;self.return_goal_id=uuid.uuid4().hex;self.candidate_attempted=False;self.map_sha=map_sha;self.provider=provider
        self.scope=scope(snapshot);s=sample_now(snapshot,now)
        self.rejection=None;self.rejection_written=False
        self.return_rejection=None;self.return_rejection_written=False
        self.return_qualification=dict(status='pending',reason='去程尚未完整到达并停稳')
        self.last_return_status=None
        if snapshot.get('software_stop') or snapshot.get('manual') or snapshot.get('busy') or snapshot.get('docking_hold'):raise CycleError('控制权尚未交接')
        if snapshot.get('raw_stationary_s',0)<2 or snapshot.get('raw_stationary_samples',0)<3:raise CycleError('原始反馈尚未连续停稳2秒')
        self.samples=[copy.deepcopy(s)];self.cursor=s['sequence'];self.arrived_at=None;self.written=0;self.last_save=now;self.last_phase=None
        self.directory=Path(logdir)/('raw-chassis-'+self.cycle_id);self.directory.mkdir();self.save(now)

    @property
    def return_eligible(self):
        return self.phase=='outbound_stopped' and self.return_qualification['status']=='eligible'

    def _block(self,reason):
        if self.phase!='blocked':self.phase='blocked';self.block_reason=reason
        self.return_qualification=dict(status='invalid_record',reason=self.block_reason)

    def stop(self,reason,now):
        self._block(reason);self.save(now)

    def stop_or_take_over(self,reason):
        self._block(reason)

    def _mark_return_issue(self,reason,code,sample,geometry=None):
        if self.return_rejection is not None:return
        self.return_rejection=dict(reason=reason,code=code,input=copy.deepcopy(sample))
        if geometry is not None:self.return_rejection['geometry']=copy.deepcopy(geometry)
        self.return_qualification=dict(status='ineligible',reason=reason,code=code)

    def _plan(self):
        poses=[]
        for s in reversed(self.samples):
            if not poses or s['pose']!=poses[-1]:poses.append(copy.deepcopy(s['pose']))
        return dict(schema_version=1,source=SOURCE,frame_id='session_scoped_raw_driver_integrator',published_frame='odom_combined',cycle_id=self.cycle_id,goal_id=self.return_goal_id,scope=copy.deepcopy(self.scope),raw_samples=copy.deepcopy(self.samples),poses=poses,reverse_only=True,executable=False,map_sha_for_navigation_only=self.map_sha,samples_sha256=key(self.samples))

    def assess_return(self):
        if self.phase=='blocked':
            self.return_qualification=dict(status='invalid_record',reason=self.block_reason)
        elif self.return_rejection is not None:
            self.return_qualification=dict(status='ineligible',reason=self.return_rejection['reason'],code=self.return_rejection['code'])
        elif self.phase!='outbound_stopped':
            self.return_qualification=dict(status='pending',reason='去程尚未完整到达并停稳')
        else:
            try:check_raw_plan(self._plan())
            except (ValueError,KeyError) as error:
                self.return_qualification=dict(status='ineligible',reason=str(error),code='return_plan_check')
            else:self.return_qualification=dict(status='eligible',reason='轨迹检查通过；准备返程时仍需实时重验')
        return copy.deepcopy(self.return_qualification)

    def update(self,snapshot,map_sha,now):
        if self.phase not in ('outbound','arrival_check'):return
        try:
            if scope(snapshot)!=self.scope:raise CycleError('原始里程/控制会话变化，重新录制')
            sample_now(snapshot,now)
            reason=recording_block_reason(snapshot)
            if reason:raise CycleError(reason)
            for s in self.provider(self.cursor):
                if s['sequence']!=self.cursor+1:raise CycleError('原始记录缺样')
                self.samples.append(copy.deepcopy(s));self.cursor=s['sequence']
                if s['velocity'][0]<-.01:
                    self._mark_return_issue('去程含倒车，不支持当前纯倒车返程','outbound_reverse',s)
            if snapshot.get('goal_status') in (4,5,6) and snapshot.get('navigation_id')==self.goal_id:
                if snapshot['goal_status']!=4:raise CycleError('去程导航未成功')
                self.arrived_at=snapshot.get('goal_finished_at');self.phase='arrival_check'
            if self.phase=='arrival_check' and self.arrived_at is not None:
                quiet_since=snapshot.get('raw_stationary_since')
                if quiet_since is not None and now-max(quiet_since,self.arrived_at)>=2 and snapshot.get('raw_stationary_samples',0)>=3:
                    tail=[p for p in self.samples if p['received_at']>=now-2.]
                    if tail and max(distance(p['pose'],tail[0]['pose']) for p in tail)>.01:raise CycleError('停稳尾段原始位姿不稳定')
                    self.phase='outbound_stopped';self.assess_return()
        except (ValueError,KeyError) as error:
            self._block(str(error))
            if self.rejection is None:
                self.rejection=copy.deepcopy(snapshot.get('raw_odom_rejection') or snapshot.get('arbiter',{}).get('raw_odom_rejection') or dict(reason=str(error)))
                self.rejection['timing']=raw_timing(snapshot,now)
                self.rejection['control']=control_evidence(snapshot)
        self.save(now)

    def save(self,now):
        if self.rejection is not None and not self.rejection_written:
            (self.directory/'first-rejection.json').write_text(json.dumps(diagnostic(self.rejection),ensure_ascii=False,indent=2,allow_nan=False)+'\n');self.rejection_written=True
        if self.return_rejection is not None and not self.return_rejection_written:
            (self.directory/'return-ineligible.json').write_text(json.dumps(diagnostic(self.return_rejection),ensure_ascii=False,indent=2,allow_nan=False)+'\n');self.return_rejection_written=True
        with (self.directory/'actual-raw-trace.jsonl').open('a') as f:
            for s in self.samples[self.written:]:f.write(json.dumps(s)+'\n')
        self.written=len(self.samples)
        status=self.return_qualification['status']
        if self.last_phase!=self.phase or self.last_return_status!=status or now-self.last_save>=5:
            data=dict(schema_version=2,source=SOURCE,cycle_id=self.cycle_id,goal_id=self.goal_id,phase=self.phase,block_reason=self.block_reason,rejection=self.rejection,return_qualification=self.return_qualification,return_rejection=self.return_rejection,scope=self.scope,samples=self.samples,map_sha_for_navigation_only=self.map_sha,return_execution_enabled=False)
            tmp=self.directory/'cycle.tmp';tmp.write_text(json.dumps(data,ensure_ascii=False,indent=2)+'\n');tmp.replace(self.directory/'cycle.json');self.last_phase=self.phase;self.last_return_status=status;self.last_save=now

    def return_candidate(self):
        self.candidate_attempted=True
        if self.phase!='outbound_stopped':raise CycleError('原始去程未完成并停稳')
        self.assess_return()
        if not self.return_eligible:raise CycleError('记录已完成，但暂不能返程：'+self.return_qualification['reason'])
        plan=self._plan()
        check_raw_plan(plan)  # Mandatory authoritative check, never a UI flag bypass.
        (self.directory/'return-candidate.json').write_text(json.dumps(plan,indent=2)+'\n')
        return copy.deepcopy(plan)

    def status(self):
        text={'outbound':'正在完整记录去程','arrival_check':'导航成功，等待原始反馈停稳','outbound_stopped':'去程记录完整，返程轨迹检查通过' if self.return_eligible else '去程记录完整，但暂不能返程','blocked':'记录失效：'+str(self.block_reason)}.get(self.phase,self.phase)
        if self.return_qualification['status']=='ineligible':text+=('：' if self.phase=='outbound_stopped' else '；暂不能返程：')+self.return_qualification['reason']
        return text+'\n原始样本 {} 点；地图仅用于去程导航。'.format(len(self.samples))

class RawReturnRuntime:
    def __init__(self,plan,conditions,store_path,*,feedback_speed_limit_m_s=None,activation_mode="hold"):
        conditions.require();check_raw_plan(plan);
        if conditions.speed_limit_m_s not in (.05,.10,.15) or conditions.minimum_radius_m<1.78:raise CycleError("返程速度/转弯半径配置不支持")
        self.plan=copy.deepcopy(plan);self.c=conditions;self.runner=self;self.phase='ready';self.reason=None;self.started=None;self.heartbeat_at=None;self.held=False;self.last=None;self.distance=0.;self.progress=0.;self.last_progress=None;self.progress_anchor=0.;self.goal_at=None;self.stop_anchor=None;self.receipt=Path(store_path).parent/('raw-return-'+plan['cycle_id']+'.json')
        if self.receipt.exists():raise CycleError('本轮原始返程已完成，不允许重复覆盖')
        self.feedback_speed_limit=conditions.speed_limit_m_s*1.2 if feedback_speed_limit_m_s is None else feedback_speed_limit_m_s
        if type(self.feedback_speed_limit) not in (int,float) or not math.isfinite(self.feedback_speed_limit) or not conditions.speed_limit_m_s<=self.feedback_speed_limit<=.40:raise CycleError('反馈速度停车边界配置无效')
        if activation_mode not in ('hold','click'):raise CycleError('返程启动模式无效')
        self.activation_mode=activation_mode;self.run_session=None;self.run_sequence=0
        self.follower=PoseAwareReversePursuit(self.plan['poses'],self.c.speed_limit_m_s,1.78)
    def result(self,v=0.,w=0.):return dict(phase=self.phase,reason=self.reason,v=v,w=w,cycle_id=self.plan['cycle_id'],goal_id=self.plan['goal_id'],coordinate_mode=SOURCE,measured_distance_m=self.distance,progress_m=self.progress)
    def stop(self,reason):self.phase='stopped';self.reason=reason;return self.result()
    def heartbeat(self,now,held):
        if self.activation_mode!='hold':return
        if self.phase in ('stopped','complete'):return
        self.held=bool(held);self.heartbeat_at=now
        if not held:self.stop('松手停车；不自动恢复')
    def start_run(self,now,session):
        if self.activation_mode!='click':raise CycleError('当前仲裁不支持单击启动')
        if self.phase!='ready' or self.run_session is not None:return False
        if not isinstance(session,str) or len(session)!=32:raise CycleError('启动会话无效')
        self.run_session=session;self.run_sequence=1;self.heartbeat_at=now
        return True
    def run_heartbeat(self,now,session):
        if self.activation_mode!='click' or self.phase in ('stopped','complete'):return False
        if not self.run_session or session!=self.run_session:
            self.stop('返程运行会话失效；不自动恢复');return False
        if self.heartbeat_at is None or not 0<=now-self.heartbeat_at<=.25:
            self.stop('返程控制循环过期；不自动恢复');return False
        self.heartbeat_at=now;self.run_sequence+=1;return True
    def check_coordinates(self,snapshot,now,starting=False):
        if scope(snapshot)!=self.plan['scope']:raise CycleError('原始里程/发布端/控制会话已变化；不得套旧轨迹')
        p=sample_now(snapshot,now)
        if snapshot.get('software_stop') or snapshot.get('motion_recovery_required') or snapshot.get('manual') or snapshot.get('manual_stop_pending') or snapshot.get('docking_hold') or snapshot.get('busy') or snapshot.get('cancel_requested'):raise CycleError('控制权/停车锁定异常')
        if not 0<=snapshot.get('scan_age',float('inf'))<=.25:raise CycleError('雷达反馈过期（不代表后向避障）')
        if starting and not near(p['pose'],self.plan['raw_samples'][-1]['pose'],.10,.10):raise CycleError('当前原始位姿与本会话实测停点不符')
        if p['velocity'][0]>.01 or feedback_speed_exceeded(p['velocity'][0],self.feedback_speed_limit):raise CycleError('原始返程实测速度异常')
        return p
    def tick(self,snapshot,now,owns_control):
        if self.phase in ('stopped','complete'):return self.result()
        try:
            p=self.check_coordinates(snapshot,now,starting=self.started is None)
            if self.heartbeat_at is None or not 0<=now-self.heartbeat_at<=.25:raise CycleError('返程运行租约丢失')
            if self.activation_mode=='hold' and not self.held:raise CycleError('保持运行心跳丢失')
            if self.activation_mode=='click' and not self.run_session:raise CycleError('尚未显式启动返程')
            if self.started is None:
                if snapshot.get('raw_stationary_s',0)<2:raise CycleError('尚未实测停稳')
                self.started=now;self.phase='running';self.last_progress=now
            if not owns_control:return self.result() # Zero-only acquisition handshake; independent arbiter requires standstill.
            if self.last:
                dt=p['source_stamp']-self.last['source_stamp']
                if dt<0 or now-self.last_tick>.25:raise CycleError('返程控制/测量时序异常')
                if dt>0:
                    step=distance(p['pose'],self.last['pose'])
                    if dt>.25 or step>self.c.speed_limit_m_s*dt+.025:raise CycleError('返程原始位姿跳变/失联')
                    self.distance+=step
            self.last=copy.deepcopy(p);self.last_tick=now
            if now-self.started>=self.c.max_time_s or self.distance>=self.c.max_distance_m:raise CycleError('达到本次短测边界')
            if self.phase=='goal_wait_still':
                if not near(p['pose'],self.plan['poses'][-1],.04,.05):raise CycleError('到点后原始位姿离开容差')
                if snapshot.get('raw_stationary_since') is not None and now-max(snapshot['raw_stationary_since'],self.goal_at)>=2:
                    self.phase='complete';self.receipt.write_text(json.dumps(dict(source=SOURCE,scope=self.plan['scope'],actual_raw_stop=p,physical_docking_verified=False,hardware_tested=False),indent=2)+'\n')
                return self.result()
            choices=[];lengths=self.follower.s
            for i,(a,b) in enumerate(zip(self.plan['poses'],self.plan['poses'][1:])):
                length=lengths[i+1]-lengths[i]
                if length<1e-8:continue
                t=max(0.,min(1.,((p['pose'][0]-a[0])*(b[0]-a[0])+(p['pose'][1]-a[1])*(b[1]-a[1]))/length**2));progress=lengths[i]+t*length
                if self.progress-.03<=progress<=self.progress+.15:
                    xy=[a[0]+t*(b[0]-a[0]),a[1]+t*(b[1]-a[1])];heading=a[2]+t*wrap(b[2]-a[2]);choices.append((distance(p['pose'],xy),progress,abs(wrap(p['pose'][2]-heading))))
            if not choices:raise CycleError('原始轨迹没有连续可达投影')
            cross,progress,yaw=min(choices)
            if cross>.10 or yaw>.10:raise CycleError('原始轨迹横向/朝向误差超限')
            self.progress=max(self.progress,progress)
            if self.progress>self.progress_anchor+.005:self.last_progress=now;self.progress_anchor=self.progress
            if now-self.last_progress>2:raise CycleError('倒车持续无原始里程进展，停车')
            v,w,done,_=self.follower.control(p['pose'])
            if done:self.phase='goal_wait_still';self.goal_at=now;return self.result()
            return self.result(v,w)
        except (ValueError,KeyError,TypeError) as e:return self.stop(str(e))
