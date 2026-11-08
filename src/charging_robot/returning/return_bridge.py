"""Panel bridge integration; all chassis commands pass through the arbiter."""
import json
import copy
import math
import time
import uuid
from pathlib import Path
from std_msgs.msg import String
from charging_robot.returning.return_runtime import ReturnRuntime
from charging_robot.returning.raw_odom_mode import RawTracker, RawReturnRuntime, SOURCE
from charging_robot.shared.model import MapDocument
from charging_robot.returning.return_map import saved_grid_digest
from charging_robot.returning.supervised_draft import TestConditions
from charging_robot.returning.session_revalidation import revalidate, runtime_identity
from charging_robot.returning.runtime_provenance import panel_identity
from charging_robot.control.stop_recovery import motion_reset_reasons
from charging_robot.control.control_causes import fault_detail


class ReturnBridge:
    def init_return(self):
        self.control_owner = uuid.uuid4().hex
        self.control_sequence = 0
        self.control_mode = 'stop'
        self.gui_at = None
        self.gui_map_sha = None
        self.return_runtime = None
        self.return_held = False
        self.return_ever_held = False
        self.return_prepared_at = None
        self.return_state = dict(phase='idle', reason=None)
        self.arbiter_state = {}
        self.arbiter_at = None
        self.known_arbiter_instance=None
        self.return_arbiter_instance=None;self.return_trial_id=None;self.return_short_test=True
        self.used_revalidations=set()
        self.raw_tracker=RawTracker();self.known_raw_arbiter_epoch=None
        self.reset_arbiter = False
        self.stop_reset_pending = None
        self.stop_cause={'code':'restored_stop','detail':'已恢复保存的停车状态'} if self.software_stop else None
        self.ui_late_reported=False
        self.odom_frame_valid = False
        self.odom_frame = None
        self.odom_source_stamp = None
        self.odom_source_age = math.inf
        self.paired_pose = None
        self.authority_pub = self.create_publisher(String, '/charging_control/authority', 1)
        self.create_subscription(String, '/charging_control/status', self.on_arbiter, 1)

    def on_arbiter(self, message):
        try:
            data = json.loads(message.data)
            with self.lock:
                instance=data.get('instance_id')
                if self.known_arbiter_instance is not None and instance!=self.known_arbiter_instance:
                    self.latch_software_stop('arbiter_restarted','独立仲裁实例改变，旧任务不恢复');self.motion_recovery_required=True;self.stop_reset_pending=None;self.reset_arbiter=False;self.manual=False;self.manual_target=(0.,0.)
                    self.commands.put(('cancel',()))
                    if self.return_runtime:self.return_state=self.return_runtime.stop('独立仲裁重启，禁止恢复旧任务')
                if data.get('raw_source_identity'):self.raw_tracker.bind(data['raw_source_identity'])
                raw_epoch=data.get('raw_epoch')
                if self.known_raw_arbiter_epoch is not None and raw_epoch!=self.known_raw_arbiter_epoch:
                    self.raw_tracker.reset()
                    if self.return_active():
                        self.latch_software_stop('raw_epoch_changed','原始里程仲裁会话重置');self.return_state=self.return_runtime.stop('原始里程仲裁会话重置')
                self.known_raw_arbiter_epoch=raw_epoch
                self.known_arbiter_instance=instance
                self.arbiter_state = data; self.arbiter_at = time.monotonic()
                if self.return_runtime is not None or data.get('mode')=='return' or getattr(self,'last_diagnostic_mode',None)=='return':
                    self.event('return_arbiter_output',status=data,received_monotonic_s=self.arbiter_at,received_ros_stamp=self.get_clock().now().nanoseconds/1e9)
                self.last_diagnostic_mode=data.get('mode')
                if data.get('fault') and data.get('commissioned') is True:self.latch_software_stop('arbiter_fault',fault_detail(data['fault']))
                pending=self.stop_reset_pending
                if pending is not None:
                    elapsed=self.arbiter_at-pending['requested_at']
                    if instance!=pending['instance_id'] or not 0<=elapsed<=2.:
                        self.stop_reset_pending=None;self.reset_arbiter=False
                        self.event('software_stop_reset_failed',reason='仲裁重启或解除确认超时',automatic_resume=False)
                    elif pending['sequence'] is not None and data.get('authority_sequence',-1)>=pending['sequence']:
                        if (data.get('owner')==self.control_owner and data.get('mode')=='stop' and
                                not data.get('fault') and data.get('graph_ready') is True and
                                data.get('output_v_m_s')==0. and data.get('output_w_rad_s')==0.):
                            current=self._snapshot(self.arbiter_at)
                            current['stationary_s']=self.stop_observation.duration(self.stop_observation.first)
                            blockers=motion_reset_reasons(current)
                            self.stop_reset_pending=None
                            if blockers:
                                self.latch_software_stop('reset_failed','底盘就绪确认期间条件变化');self.motion_recovery_required=True
                                self.event('software_stop_reset_failed',reason='确认期间条件已变化：'+'；'.join(blockers),automatic_resume=False)
                            else:
                                self.software_stop=False;self.motion_recovery_required=False;self.stop_cause=None
                                self.event('software_stop_released',automatic_resume=False,acknowledged_sequence=data['authority_sequence'])
                        else:
                            self.stop_reset_pending=None
                            self.event('software_stop_reset_failed',reason=data.get('fault') or data.get('graph_reason') or '仲裁未确认零速停止',automatic_resume=False)
                signature=(data.get('fault'),data.get('graph_ready'),data.get('graph_reason'))
                if signature!=getattr(self,'last_arbiter_guard_signature',None):
                    self.last_arbiter_guard_signature=signature
                    self.event('arbiter_guard_state',fault=data.get('fault'),graph_ready=data.get('graph_ready'),graph_reason=data.get('graph_reason'))
        except (ValueError, TypeError):
            with self.lock: self.arbiter_state = {}; self.arbiter_at = None

    def latch_software_stop(self,code,detail,now=None):
        """Stop current intent; ordinary operator/cancel stops do not latch a lock."""
        if now is None:now=time.monotonic()
        ordinary=code in ('operator_stop','panel_close','recording_failed','return_cancelled',
                         'return_controller_stop','manual_input_expired','arm_tool_hold')
        first=not getattr(self,'stop_cause',None)
        if not ordinary:self.software_stop=True
        if ordinary:
            self.cancel_requested=True
            self.manual=False;self.manual_target=(0.,0.)
            if self.return_active():self.return_state=self.return_runtime.stop(detail)
        if first:
            self.stop_cause=dict(code=code,detail=detail,source_monotonic_s=now,
                previous_control_mode=getattr(self,'control_mode','unknown'),
                gui_age_s=now-self.gui_at if getattr(self,'gui_at',None) is not None else None,
                latched=not ordinary)
            self.event('software_stop_latched' if not ordinary else 'motion_stop_requested',**self.stop_cause,automatic_resume=False)

    def gui_heartbeat(self, map_sha):
        with self.lock:
            self.gui_at = time.monotonic(); self.gui_map_sha = map_sha

    def return_active(self):
        return self.return_runtime is not None and self.return_runtime.runner.phase not in ('stopped', 'complete')

    def return_guard(self):
        if self.return_active():
            raise ValueError('返程已准备或进行中，请先停止返程并确认停稳')

    def return_heartbeat(self, held):
        with self.lock:
            if bool(held)!=self.return_held:self.event('return_hold_changed',held=bool(held),monotonic_s=time.monotonic(),ros_stamp=self.get_clock().now().nanoseconds/1e9,trial_id=self.return_trial_id,arbiter_instance=self.return_arbiter_instance)
            self.return_held = bool(held)
            if held: self.return_ever_held = True
            if self.return_runtime and self.return_ever_held:
                self.return_runtime.heartbeat(time.monotonic(), held)

    def start_return(self):
        """Operator click creates one immutable run session; timer never starts it."""
        with self.lock:
            runtime=self.return_runtime
            if getattr(self,'motion_recovery_required',False):raise ValueError('请先确认底盘就绪')
            if not isinstance(runtime,RawReturnRuntime) or runtime.activation_mode!='click':
                raise ValueError('尚未准备单击返程，或仲裁版本不匹配')
            if runtime.phase in ('stopped','complete'):raise ValueError('返程已停止，不能重新启动旧会话')
            if runtime.run_session is not None:return runtime.run_session
            now=time.monotonic()
            if runtime.phase!='ready' or self.software_stop or self.docking_hold:
                raise ValueError('返程已停止或停车锁定；需要重新准备')
            if self.arbiter_at is None or not 0<=now-self.arbiter_at<=.25 or self.arbiter_state.get('fault'):
                raise ValueError('独立仲裁未就绪，不能启动')
            state=self._snapshot(now);state['busy']=self.busy or self.server_active
            runtime.check_coordinates(state,now,starting=True)
            session=uuid.uuid4().hex
            runtime.start_run(now,session)
            self.event('return_run_started',run_session=session,trial_id=self.return_trial_id,automatic_resume=False)
            return session

    def return_keepalive(self,session):
        with self.lock:
            runtime=self.return_runtime
            if not runtime or runtime.activation_mode!='click':return False
            # GUI observation cannot renew a stalled backend controller.
            return runtime.phase not in ('stopped','complete') and runtime.run_session==session

    def stop_return(self, reason):
        with self.lock:
            if self.return_runtime:
                self.return_state = self.return_runtime.stop(reason)
                self.return_held = False
                self.latch_software_stop('return_cancelled',reason)

    def revalidate_saved_route(self,directory,map_path,confirmations):
        now=time.monotonic()
        with self.lock:
            if self.return_active():raise ValueError('请先停止当前返程')
            if (self.arbiter_at is None or now-self.arbiter_at>.25 or
                    not self.arbiter_state.get('driver_gate_verified')):
                raise ValueError('尚未核实新驱动门禁与当前控制链身份')
            snapshot=self._snapshot(now)
        return revalidate(directory,map_path,snapshot,confirmations,now)

    def prepare_return(self, plan, store_path, map_path, confirmations, *, short_test=True):
        plan=copy.deepcopy(plan)
        raw=plan.get('source')==SOURCE
        if not raw:raise ValueError('本模式必须重新记录同会话原始里程，不能执行旧地图轨迹')
        if not raw:
            document = MapDocument.load(map_path)
            if document.fingerprint != plan['map_sha256']:
                raise ValueError('地图文件已改变，请重新记录去程')
        with self.lock:
            now = time.monotonic()
            self.return_guard()
            state = self._snapshot(now)
            if (self.software_stop or getattr(self,'motion_recovery_required',False) or self.docking_hold or state['busy'] or self.manual or
                    self.manual_stop_pending):
                raise ValueError('导航/对接/操控尚未交接，或软件停车锁定')
            if (self.arbiter_at is None or now-self.arbiter_at > .25 or
                    self.arbiter_state.get('commissioned') is not True or self.arbiter_state.get('fault') or
                    self.arbiter_state.get('mode') != 'stop'):
                raise ValueError('独立底盘仲裁未就绪或尚未完成现场验收：'+str(self.arbiter_state))
            if self.arbiter_state.get('release_profile')=='short_only' and not short_test:
                raise ValueError('独立仲裁只允许短测，不能请求完整返程')
            # Explicit new preparation after navigation has ended may discard an
            # old cancellation marker; it cannot bypass a live goal or stop lock.
            obsolete_cancel=bool(self.cancel_requested)
            state['cancel_requested']=False
            if self.arbiter_state.get('release_profile')=='recorded_route' and short_test:
                raise ValueError('按轮返程使用完整记录，不启用旧短测模式')
            if not raw:plan['prepared_runtime_identity']=runtime_identity(state)
            certificate_id=plan.get('revalidation',{}).get('certificate_id')
            if certificate_id in self.used_revalidations:
                raise ValueError('本次重验已用于一次试验，请重新重验')
            length = sum(math.dist(a[:2], b[:2]) for a, b in zip(plan['poses'], plan['poses'][1:]))
            if self.arbiter_state.get('release_profile')=='supervised_trial':
                if self.arbiter_state.get('full_trial_consumed'):raise ValueError('本次完整试验授权已使用，不能自动复用')
                if short_test:raise ValueError('当前请求是短测，但本次授权为完整返程')
                limit=self.arbiter_state.get('full_trial_max_route_m',0)
                if type(limit) not in (float,int) or not math.isfinite(limit) or limit<=0:raise ValueError('尚未取得有效的路线长度授权')
                if length>limit:raise ValueError('实测路线长 {:.3f} 米，超过本次授权上限 {:.3f} 米'.format(length,limit))
            conditions = TestConditions(plan['cycle_id'], plan['goal_id'],
                **confirmations, speed_limit_m_s=self.arbiter_state.get("return_speed_m_s",.05), minimum_radius_m=1.78,
                max_time_s=3. if short_test else length/.03+15.,
                max_distance_m=.15 if short_test else length+.20,
                controller_id='pose-aware-supervised-return-v1')
            runtime = RawReturnRuntime(plan, conditions, store_path,feedback_speed_limit_m_s=self.arbiter_state.get('feedback_speed_limit_m_s'),activation_mode=self.arbiter_state.get('return_start_mode','hold')) if raw else ReturnRuntime(plan, conditions, store_path)
            if not raw and saved_grid_digest(map_path) != runtime.anchor['map_grid_sha']:
                raise ValueError('实际地图内容与保存地图不一致')
            runtime.check_coordinates(state, now, starting=True)
            if state.get('raw_stationary_s' if raw else 'stationary_s',0)<2 or state.get('raw_stationary_samples' if raw else 'stationary_samples',0)<3:
                raise ValueError('尚未观察到连续停稳')
            if certificate_id:self.used_revalidations.add(certificate_id)
            self.return_arbiter_instance=self.arbiter_state['instance_id']
            self.return_trial_id=uuid.uuid4().hex;self.return_short_test=bool(short_test)
            self.goal_status=None;self.goal_target=None;self.cancel_requested=False;self.stop_cause=None
            self.return_runtime = runtime
            self.return_held = self.return_ever_held = False
            self.return_prepared_at = now
            self.return_map_path = str(map_path)
            self.return_map_checked_at = now
            self.return_state = dict(phase='ready', reason=None, short_test=short_test)
        self.event('return_prepared', cycle_id=plan['cycle_id'], short_test=short_test,obsolete_cancel_cleared=obsolete_cancel,
                   detects_new_obstacles=False)

    def control_tick(self):
        with self.lock:
            now = time.monotonic()
            mode, v, w = 'stop', 0., 0.
            if self.stop_reset_pending is not None and not 0<=now-self.stop_reset_pending['requested_at']<=2.:
                self.stop_reset_pending=None;self.reset_arbiter=False
                self.event('software_stop_reset_failed',reason='解除确认超时，保持停车锁定',automatic_resume=False)
            gui_fresh = self.gui_at is not None and 0 <= now-self.gui_at <= .25
            state = self._snapshot(now)
            runtime = self.return_runtime
            click_return=isinstance(runtime,RawReturnRuntime) and runtime.activation_mode=='click'
            if click_return and runtime.run_session and runtime.phase not in ('stopped','complete') and not self.software_stop and not getattr(self,'motion_recovery_required',False) and not self.docking_hold:
                runtime.run_heartbeat(now,runtime.run_session)
            if self.return_active():
                if (not click_return and not gui_fresh) or self.software_stop or getattr(self,'motion_recovery_required',False) or self.docking_hold:
                    self.return_state = runtime.stop('保持式输入过期或底盘停车/占用状态')
                elif not isinstance(runtime,RawReturnRuntime) and now-self.return_map_checked_at >= .2:
                    self.return_map_checked_at = now
                    try:
                        if MapDocument.load(self.return_map_path).fingerprint != runtime.plan['map_sha256']:
                            self.return_state = runtime.stop('地图文件在返程期间改变')
                    except (OSError, ValueError, KeyError) as error:
                        self.return_state = runtime.stop('地图不可读：'+str(error))
                if runtime.runner.phase == 'ready' and not (runtime.run_session if isinstance(runtime,RawReturnRuntime) and runtime.activation_mode=='click' else self.return_ever_held):
                    if now-self.return_prepared_at > 30.:
                        self.return_state = runtime.stop('准备超时，请重新核对现场')
                elif runtime.runner.phase not in ('stopped', 'complete'):
                    a = self.arbiter_state
                    valid = (self.arbiter_at is not None and 0 <= now-self.arbiter_at <= .25 and
                             a.get('owner') == self.control_owner and a.get('commissioned') is True and
                             a.get('instance_id')==self.return_arbiter_instance and
                             not a.get('fault'))
                    if not valid:
                        self.return_state = runtime.stop('底盘仲裁状态失联或失效')
                    else:
                        # Return's own busy flag must not be confused with a navigation action.
                        state['busy'] = self.busy or self.server_active
                        self.return_state = runtime.tick(state, now, owns_control=a.get('mode') == 'return')
                        mode = 'return'
                        v, w = self.return_state.get('v', 0.), self.return_state.get('w', 0.)
            if runtime and runtime.runner.phase == 'stopped':
                self.latch_software_stop('return_controller_stop',getattr(runtime,'reason',None) or self.return_state.get('reason') or '返程控制器已停止',now)
            if runtime and runtime.runner.phase in ('stopped', 'complete'):
                mode, v, w = 'stop', 0., 0.
                self.event('return_finished', **self.return_state,elapsed_s=now-runtime.started if isinstance(runtime,RawReturnRuntime) and runtime.started is not None else None,software_stop=self.software_stop,raw_odom=state.get('raw_odom'))
                self.return_runtime = None
            elif not self.return_active() and not self.software_stop and not getattr(self,'motion_recovery_required',False) and not self.docking_hold and not self.cancel_requested:
                if self.manual and not self.busy and not self.server_active:
                    if gui_fresh and 0<=now-self.manual_stamp<=.25:
                        mode, (v, w) = 'manual', self.manual_target
                    else:self.latch_software_stop('manual_input_expired','键控输入或界面输入心跳过期',now)
                elif self.busy and getattr(self,'navigation_id',None):
                    mode = 'navigation'
            if mode in ('navigation','return'):
                if not gui_fresh and not getattr(self,'ui_late_reported',False):
                    self.ui_late_reported=True
                    self.event('ui_update_delayed',source_monotonic_s=now,gui_age_s=now-self.gui_at if self.gui_at is not None else None,control_mode=mode,control_callback_running=True)
                elif gui_fresh and getattr(self,'ui_late_reported',False):
                    self.ui_late_reported=False;self.event('ui_update_recovered',source_monotonic_s=now)
            if self.control_mode not in ('stop', mode) and mode != 'stop':
                # Arbiter requires a zero-output transition between owners.
                mode, v, w = 'stop', 0., 0.
            self.control_mode = mode
            self.control_sequence += 1
            packet = dict(owner=self.control_owner, sequence=self.control_sequence, mode=mode,
                          v=v, w=w, stamp=self.get_clock().now().nanoseconds/1e9,
                          reset=self.reset_arbiter,
                          reset_request_id=self.stop_reset_pending.get("request_id") if self.stop_reset_pending else None,
                          instance_id=self.return_arbiter_instance if mode=='return' else self.arbiter_state.get('instance_id'),
                          trial_id=self.return_trial_id,cycle_id=self.return_state.get('cycle_id') or (runtime.plan['cycle_id'] if runtime else None),
                          goal_id=self.return_state.get('goal_id') or (runtime.plan['goal_id'] if runtime else None),
                          route_length_m=sum(math.dist(a[:2],b[:2]) for a,b in zip(runtime.plan['poses'],runtime.plan['poses'][1:])) if runtime else None,
                          activation_mode=runtime.activation_mode if isinstance(runtime,RawReturnRuntime) else 'hold',
                          run_state='running' if isinstance(runtime,RawReturnRuntime) and runtime.run_session and runtime.phase not in ('stopped','complete') else 'inactive',
                          run_session=runtime.run_session if isinstance(runtime,RawReturnRuntime) else None,
                          run_sequence=runtime.run_sequence if isinstance(runtime,RawReturnRuntime) else 0,
                          run_lease_source='backend_controller' if click_return else 'hold_input',
                          held=bool(self.return_held),site_confirmed=bool(runtime is not None),short_test=self.return_short_test,
                          coordinate_mode=SOURCE if isinstance(runtime,RawReturnRuntime) else None,
                          raw_source_key=runtime.plan['scope']['source_key'] if isinstance(runtime,RawReturnRuntime) else None,
                          raw_epoch=runtime.plan['scope']['arbiter_raw_epoch'] if isinstance(runtime,RawReturnRuntime) else None)
            if self.reset_arbiter and self.stop_reset_pending is not None:
                self.stop_reset_pending['sequence']=self.control_sequence
            self.reset_arbiter = False
            if self.manual_stop_pending:
                self.manual_stop_pending = False
                self.event('manual_zero_sent')
        message = String(); message.data = json.dumps(packet)
        self.authority_pub.publish(message)
        if runtime is not None or mode=='return':self.event('return_authority_published',packet=packet,monotonic_s=time.monotonic(),software_stop=self.software_stop,raw_odom=state.get('raw_odom'))

    def raw_samples_since(self,sequence):
        with self.lock:return self.raw_tracker.since(sequence)

    def raw_snapshot(self,now):
        ros_now=self.get_clock().now().nanoseconds/1e9
        try:raw=self.raw_tracker.current(now,ros_now);error=None
        except ValueError as e:raw=None;error=str(e)
        latest=self.raw_tracker.latest or {}
        timing={k:latest.get(k) for k in ('sequence','source_stamp','received_at')}
        timing['checked_at']=now
        return dict(raw_odom=raw,raw_odom_error=error,raw_odom_timing=timing,raw_odom_rejection=self.raw_tracker.rejection,ros_now=ros_now,control_owner=self.control_owner,
            raw_stationary_since=self.raw_tracker.still_since,
            raw_stationary_s=now-self.raw_tracker.still_since if raw and self.raw_tracker.still_since is not None else 0.,
            raw_stationary_samples=self.raw_tracker.still_samples if raw else 0)
