"""Panel bridge integration; all chassis commands pass through the arbiter."""
import json
import math
import time
import uuid
from pathlib import Path
from std_msgs.msg import String
from return_runtime import ReturnRuntime
from model import MapDocument
from return_map import saved_grid_digest
from supervised_draft import TestConditions


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
        self.reset_arbiter = False
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
                self.arbiter_state = data; self.arbiter_at = time.monotonic()
                if data.get('fault') and data.get('commissioned') is True:self.software_stop=True
        except (ValueError, TypeError):
            with self.lock: self.arbiter_state = {}; self.arbiter_at = None

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
            self.return_held = bool(held)
            if held: self.return_ever_held = True
            if self.return_runtime and self.return_ever_held:
                self.return_runtime.heartbeat(time.monotonic(), held)

    def stop_return(self, reason):
        with self.lock:
            if self.return_runtime:
                self.return_state = self.return_runtime.stop(reason)
                self.return_held = False
                self.software_stop = True

    def prepare_return(self, plan, store_path, map_path, confirmations, *, short_test=True):
        document = MapDocument.load(map_path)
        if document.fingerprint != plan['map_sha256']:
            raise ValueError('地图文件已改变，请重新记录去程')
        now = time.monotonic()
        with self.lock:
            self.return_guard()
            state = self._snapshot(now)
            if (self.software_stop or self.docking_hold or state['busy'] or self.manual or
                    self.manual_stop_pending or self.cancel_requested):
                raise ValueError('导航/对接/操控尚未交接，或软件停车锁定')
            if (self.arbiter_at is None or now-self.arbiter_at > .25 or
                    self.arbiter_state.get('commissioned') is not True or self.arbiter_state.get('fault') or
                    self.arbiter_state.get('mode') != 'stop'):
                raise ValueError('独立底盘仲裁未就绪或尚未完成现场验收：'+str(self.arbiter_state))
            length = sum(math.dist(a[:2], b[:2]) for a, b in zip(plan['poses'], plan['poses'][1:]))
            conditions = TestConditions(plan['cycle_id'], plan['goal_id'],
                **confirmations, speed_limit_m_s=.05, minimum_radius_m=1.78,
                max_time_s=3. if short_test else length/.03+15.,
                max_distance_m=.15 if short_test else length+.20,
                controller_id='pose-aware-supervised-return-v1')
            runtime = ReturnRuntime(plan, conditions, store_path)
            if saved_grid_digest(map_path) != runtime.anchor['map_grid_sha']:
                raise ValueError('实际地图内容与保存地图不一致')
            runtime.check_coordinates(state, now, starting=True)
            if state['stationary_s'] < 2. or state['stationary_samples'] < 3:
                raise ValueError('尚未观察到连续停稳')
            self.goal_status=None;self.goal_target=None
            self.return_runtime = runtime
            self.return_held = self.return_ever_held = False
            self.return_prepared_at = now
            self.return_map_path = str(map_path)
            self.return_map_checked_at = now
            self.return_state = dict(phase='ready', reason=None, short_test=short_test)
        self.event('return_prepared', cycle_id=plan['cycle_id'], short_test=short_test,
                   detects_new_obstacles=False)

    def control_tick(self):
        now = time.monotonic()
        with self.lock:
            mode, v, w = 'stop', 0., 0.
            gui_fresh = self.gui_at is not None and 0 <= now-self.gui_at <= .25
            state = self._snapshot(now)
            runtime = self.return_runtime
            if self.return_active():
                if not gui_fresh or self.software_stop or self.docking_hold:
                    self.return_state = runtime.stop('面板心跳丢失或停车/对接锁定')
                elif now-self.return_map_checked_at >= .2:
                    self.return_map_checked_at = now
                    try:
                        if MapDocument.load(self.return_map_path).fingerprint != runtime.plan['map_sha256']:
                            self.return_state = runtime.stop('地图文件在返程期间改变')
                    except (OSError, ValueError, KeyError) as error:
                        self.return_state = runtime.stop('地图不可读：'+str(error))
                if runtime.runner.phase == 'ready' and not self.return_ever_held:
                    if now-self.return_prepared_at > 30.:
                        self.return_state = runtime.stop('准备超时，请重新核对现场')
                elif runtime.runner.phase not in ('stopped', 'complete'):
                    a = self.arbiter_state
                    valid = (self.arbiter_at is not None and 0 <= now-self.arbiter_at <= .25 and
                             a.get('owner') == self.control_owner and a.get('commissioned') is True and
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
                self.software_stop = True
            if runtime and runtime.runner.phase in ('stopped', 'complete'):
                mode, v, w = 'stop', 0., 0.
                self.event('return_finished', **self.return_state)
                self.return_runtime = None
            elif not self.return_active() and gui_fresh and not self.software_stop and not self.docking_hold:
                if self.manual and not self.busy and not self.server_active and now-self.manual_stamp <= .25:
                    mode, (v, w) = 'manual', self.manual_target
                elif self.busy or self.server_active:
                    mode = 'navigation'
            if not gui_fresh and self.control_mode != 'stop':
                self.software_stop = True
            if self.control_mode not in ('stop', mode) and mode != 'stop':
                # Arbiter requires a zero-output transition between owners.
                mode, v, w = 'stop', 0., 0.
            self.control_mode = mode
            self.control_sequence += 1
            packet = dict(owner=self.control_owner, sequence=self.control_sequence, mode=mode,
                          v=v, w=w, stamp=self.get_clock().now().nanoseconds/1e9,
                          reset=self.reset_arbiter)
            self.reset_arbiter = False
            if self.manual_stop_pending:
                self.manual_stop_pending = False
                self.event('manual_zero_sent')
        message = String(); message.data = json.dumps(packet)
        self.authority_pub.publish(message)
