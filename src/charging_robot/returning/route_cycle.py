"""Offline domain model. No ROS, process execution, serial access or motion output.

An integration adapter must supply real TF/odometry observations and fresh arm
readback. Return capabilities default to unavailable: this is not a controller.
"""
from dataclasses import dataclass, asdict
import json
import math
import os
from pathlib import Path
import uuid


class CycleError(ValueError):
    pass


@dataclass(frozen=True)
class Policy:
    # Provisional offline guards; not calibrated hardware tolerances.
    freshness_s: float = 1.0
    still_s: float = 2.0
    linear_still_m_s: float = .01
    angular_still_rad_s: float = .02
    max_trace_gap_s: float = 1.0
    max_pose_step_m: float = .75
    max_yaw_step_rad: float = .75
    start_xy_tolerance_m: float = .10
    start_yaw_tolerance_rad: float = .10
    return_xy_tolerance_m: float = .10
    return_yaw_tolerance_rad: float = .10


    def __post_init__(self):
        if not all(math.isfinite(x) and x > 0 for x in asdict(self).values()):
            raise CycleError('门限必须为有限正数')


@dataclass(frozen=True)
class Observation:
    map_sha256: str
    localization_epoch: str
    frame_id: str
    pose: tuple  # measured map->base pose, x[m], y[m], yaw[rad]
    source_stamp: float  # distinct measurement stamp; not the UI polling time
    received_at: float  # monotonic clock in this process
    pose_age_s: float
    odom_age_s: float
    scan_age_s: float
    v_m_s: float
    w_rad_s: float
    localized: bool
    still_since: float | None
    still_samples: int
    software_stop: bool = False
    manual_active: bool = False


@dataclass(frozen=True)
class ReverseCapabilities:
    controller_id: str = ''
    follow_path_available: bool = False
    reverse_only_verified: bool = False
    rear_collision_check_verified: bool = False
    feedback_tracking_verified: bool = False
    speed_limit_m_s: float | None = None

    def require(self):
        if not (self.controller_id and self.follow_path_available and
                self.reverse_only_verified and self.rear_collision_check_verified and
                self.feedback_tracking_verified):
            raise CycleError('缺少已验证的倒车路径跟踪/后方碰撞检查/实际反馈接口，返程保持禁用')
        if self.speed_limit_m_s is None or not math.isfinite(self.speed_limit_m_s) or self.speed_limit_m_s <= 0:
            raise CycleError('倒车速度尚未实机核定；不继承去程或Nav2负速度上限')


@dataclass(frozen=True)
class ArmReceipt:
    cycle_id: str
    invocation_id: str
    action: str
    started_at: float
    finished_at: float
    process_exited: bool
    returncode: int
    # Only the adapter's NEW GetAngle/RobotMode readback may set these fields.
    home_readback_at: float | None = None
    home_actual_verified: bool = False


def finite_pose(pose):
    return len(pose) == 3 and all(math.isfinite(v) for v in pose)


def angle_error(a, b):
    return abs(math.atan2(math.sin(a-b), math.cos(a-b)))


def near(a, b, xy, yaw):
    return math.hypot(a[0]-b[0], a[1]-b[1]) <= xy and angle_error(a[2], b[2]) <= yaw


class StartStore:
    """A persisted reference is NOT live localization and is never auto-published."""
    def __init__(self, path):
        self.path = Path(path)

    def read(self, map_sha256):
        if not self.path.exists():
            return None
        data = json.loads(self.path.read_text())
        if data.get('schema_version') != 1 or data.get('frame_id') != 'map' or data.get('map_sha256') != map_sha256:
            raise CycleError('保存起点与当前地图不匹配；保留原文件并要求手动重新设置')
        if not finite_pose(data['pose']):
            raise CycleError('保存起点无效')
        return data

    def write(self, map_sha256, pose, origin, cycle_id=None):
        if not finite_pose(pose):
            raise CycleError('起点必须为有限的地图坐标')
        self.path.parent.mkdir(parents=True, exist_ok=True)
        data = dict(schema_version=1, frame_id='map', map_sha256=map_sha256,
                    pose=list(pose), origin=origin, cycle_id=cycle_id,
                    usable_as_live_localization=False)
        temp = self.path.with_name(self.path.name+'.'+uuid.uuid4().hex+'.tmp')
        try:
            with temp.open('x') as stream:
                json.dump(data, stream, ensure_ascii=False, indent=2)
                stream.write('\n'); stream.flush(); os.fsync(stream.fileno())
            os.replace(temp, self.path)
        finally:
            temp.unlink(missing_ok=True)
        return data


class RouteCycle:
    def __init__(self, map_sha256, localization_epoch, store, policy=Policy()):
        if not map_sha256 or not localization_epoch:
            raise CycleError('必须绑定地图版本和本次定位会话')
        self.map_sha256 = map_sha256
        self.epoch = localization_epoch
        self.store = store
        self.policy = policy
        self.cycle_id = uuid.uuid4().hex
        self.phase = 'idle'
        self.trace = []
        self.outbound_goal_id = None
        self.return_goal_id = None
        self.arrived_at = None
        self.return_finished_at = None
        self.arm_request = None
        self.test_receipt = None
        self.home_receipt = None
        self.return_plan = None
        self.block_reason = None

    def _expect(self, phase):
        if self.phase != phase:
            raise CycleError('阶段不允许：当前 '+self.phase+'，需要 '+phase)

    def _block(self, reason):
        self.phase = 'blocked'; self.block_reason = reason
        raise CycleError(reason)

    def _observation(self, obs, now):
        p = self.policy
        if obs.map_sha256 != self.map_sha256 or obs.localization_epoch != self.epoch or obs.frame_id != 'map':
            self._block('地图或定位会话改变；不得继续使用旧轨迹/旧坐标')
        ages = (now-obs.received_at, obs.pose_age_s, obs.odom_age_s, obs.scan_age_s)
        if not obs.localized or not finite_pose(obs.pose) or not all(math.isfinite(x) and 0 <= x <= p.freshness_s for x in ages):
            self._block('缺少新鲜有效的定位/里程计/环境反馈')
        if not all(math.isfinite(x) for x in (obs.source_stamp, obs.v_m_s, obs.w_rad_s)):
            self._block('实际反馈数值无效')
        if obs.software_stop or obs.manual_active:
            self._block('软件停车或人工接管中；不得自动恢复任务')

    def _still(self, obs, now, after=None):
        self._observation(obs, now)
        p = self.policy
        since = obs.still_since
        if (since is None or not math.isfinite(since) or since > obs.received_at or
            obs.still_samples < 3 or abs(obs.v_m_s) > p.linear_still_m_s or
            abs(obs.w_rad_s) > p.angular_still_rad_s or
            obs.received_at-max(since, after if after is not None else since) < p.still_s):
            raise CycleError('尚未取得本阶段之后的连续停稳反馈')

    def set_manual_start(self, pose, operator_confirmed):
        self._expect('idle')
        if not operator_confirmed:
            raise CycleError('手动重设起点需要现场确认')
        return self.store.write(self.map_sha256, pose, 'manual_reference')

    def capture_actual_start(self, obs, now):
        self._expect('idle'); self._still(obs, now)
        return self.store.write(self.map_sha256, obs.pose, 'observed_stationary', self.cycle_id)

    def begin_outbound(self, obs, goal_id, now):
        self._expect('idle'); self._still(obs, now)
        reference = self.store.read(self.map_sha256)
        if reference is None:
            raise CycleError('请先保存起点或手动设置起点')
        p = self.policy
        if not near(obs.pose, reference['pose'], p.start_xy_tolerance_m, p.start_yaw_tolerance_rad):
            raise CycleError('实际定位不在保存起点附近；需要核实定位或手动重设，不能强套旧坐标')
        if not goal_id:
            raise CycleError('必须绑定本轮去程目标ID')
        self.outbound_goal_id = goal_id
        self.phase = 'outbound'
        self.record_outbound(obs, now)

    def record_outbound(self, obs, now):
        if self.phase not in ('outbound', 'arrival_check'):
            raise CycleError('本阶段不接受去程记录')
        self._observation(obs, now)
        if self.trace:
            previous = self.trace[-1]
            if obs.source_stamp == previous['source_stamp']:
                if list(obs.pose) != previous['pose']:
                    self._block('同一测量时间的坐标改变，反馈不可信')
                return False  # Repainting the UI must not invent another measurement.
            if obs.source_stamp < previous['source_stamp'] or obs.received_at <= previous['received_at']:
                self._block('测量时间倒退或乱序，轨迹不完整')
            if obs.received_at-previous['received_at'] > self.policy.max_trace_gap_s:
                self._block('去程反馈中断；不得跨缺失区间补画轨迹')
            if (math.dist(obs.pose[:2], previous['pose'][:2]) > self.policy.max_pose_step_m or
                    angle_error(obs.pose[2], previous['pose'][2]) > self.policy.max_yaw_step_rad):
                self._block('定位突跳，不能作为可信去程轨迹')
        self.trace.append(dict(pose=list(obs.pose), source_stamp=obs.source_stamp,
                               received_at=obs.received_at, observed_v_m_s=obs.v_m_s))
        return True

    def outbound_result(self, goal_id, status, now):
        self._expect('outbound')
        if not math.isfinite(now) or now < self.trace[-1]['received_at']:
            self._block('去程结果时间无效')
        if goal_id != self.outbound_goal_id or status != 'succeeded':
            self._block('本轮去程目标未成功；不得启动对接')
        self.arrived_at = now; self.phase = 'arrival_check'

    def request_test(self, obs, now, operator_docking_confirmed):
        self._expect('arrival_check'); self._still(obs, now, self.arrived_at)
        if not operator_docking_confirmed:
            raise CycleError('导航容差与停稳不代表可插入；必须现场确认停靠姿态及机械臂工作区')
        self.record_outbound(obs, now)  # Include measured stationary docking endpoint.
        self.phase = 'test_running'
        return self._arm_request('test', now)

    def _arm_request(self, action, now):
        # Data only. The existing INTERACTIVE command runner must execute it.
        self.arm_request = dict(cycle_id=self.cycle_id, invocation_id=uuid.uuid4().hex,
                                action=action, started_at=now, preserve_interactive_confirmation=True)
        return dict(self.arm_request)

    def arm_finished(self, receipt, now):
        request = self.arm_request
        if request is None or self.phase not in ('test_running', 'home_running'):
            raise CycleError('没有可接收结束结果的本轮机械臂请求')
        values = (receipt.started_at, receipt.finished_at, now)
        if (not all(math.isfinite(x) for x in values) or
            any(getattr(receipt,k) != request[k] for k in ('cycle_id','invocation_id','action','started_at')) or
            not receipt.process_exited or receipt.returncode != 0 or
            not receipt.started_at <= receipt.finished_at <= now or
            now-receipt.finished_at > self.policy.freshness_s):
            self._block('旧轮次/旧请求/过期或不确定的结束结果，保持底盘锁定')
        if receipt.action == 'test':
            self.test_receipt = receipt; self.phase = 'test_done'
        else:
            at = receipt.home_readback_at
            if (not receipt.home_actual_verified or at is None or not math.isfinite(at) or
                not receipt.started_at <= at <= now or now-at > self.policy.freshness_s):
                self._block('home退出码不代表实际回位；缺少本次调用后的新鲜关节/模式验收')
            self.home_receipt = receipt; self.phase = 'home_done'
        self.arm_request = None

    def request_home(self, now, mechanical_release_confirmed, dry_insertion_demo):
        self._expect('test_done')
        if not math.isfinite(now) or now < self.test_receipt.finished_at:
            self._block('home请求时间无效')
        if not dry_insertion_demo or not mechanical_release_confirmed:
            raise CycleError('本方案仅用于无电插入演示；先确认退枪路径及机构已释放')
        # test's subprocess MUST already have exited. Never launch home beside it.
        self.phase = 'home_running'
        return self._arm_request('home', now)

    def prepare_reverse(self, obs, now, capabilities):
        self._expect('home_done'); self._still(obs, now, self.home_receipt.finished_at)
        capabilities.require()
        if len(self.trace) < 2:
            raise CycleError('没有足够的实际去程测量')
        if any(item['observed_v_m_s'] < -self.policy.linear_still_m_s for item in self.trace):
            raise CycleError('去程包含倒车；不能声称整条反向路径都能纯倒车通过')
        poses = []
        for item in reversed(self.trace):
            pose = item['pose']
            if not poses or pose != poses[-1]:
                poses.append(list(pose))  # Preserve body yaw; do NOT add pi.
        if len(poses) < 2 or not any(math.dist(poses[0][:2], p[:2]) > .01 for p in poses[1:]):
            raise CycleError('记录没有形成有效的行驶路径')
        p = self.policy
        if not near(obs.pose, poses[0], p.return_xy_tolerance_m, p.return_yaw_tolerance_rad):
            self._block('底盘已偏离记录的充电停靠位置；禁止直接套用返程轨迹')
        self.return_goal_id = uuid.uuid4().hex
        self.return_plan = dict(cycle_id=self.cycle_id, goal_id=self.return_goal_id,
                                frame_id='map', map_sha256=self.map_sha256,
                                controller_id=capabilities.controller_id, poses=poses,
                                reverse_only=True, speed_limit_m_s=capabilities.speed_limit_m_s,
                                input_source='actual_map_pose_samples', executable=False)
        self.phase = 'return_ready'
        return dict(self.return_plan)

    def accept_return_handoff(self, goal_id):
        self._expect('return_ready')
        if goal_id != self.return_goal_id:
            raise CycleError('返程请求ID不匹配')
        self.phase = 'returning'

    def return_result(self, cycle_id, goal_id, status, finished_at, now,
                      actual_path_verified):
        """Adapter must monitor actual progress, cross-track error and timeout.

        This receipt alone does not implement that monitor or a controller.
        """
        self._expect('returning')
        if (cycle_id != self.cycle_id or goal_id != self.return_goal_id or
            status != 'succeeded' or not actual_path_verified or
            not all(math.isfinite(t) for t in (finished_at, now)) or
            not self.home_receipt.finished_at <= finished_at <= now or
            now-finished_at > self.policy.freshness_s):
            self._block('返程轮次/结果/实际路径反馈未确认')
        self.return_finished_at = finished_at
        self.phase = 'return_stop_check'

    def complete_return(self, obs, now):
        self._expect('return_stop_check')
        self._still(obs, now, self.return_finished_at)
        p = self.policy
        if not near(obs.pose, self.trace[0]['pose'], p.return_xy_tolerance_m, p.return_yaw_tolerance_rad):
            self._block('返程实际停点未达本轮起点验收范围')
        saved = self.store.write(self.map_sha256, obs.pose, 'observed_return_stop', self.cycle_id)
        self.phase = 'complete'
        return saved

    def stop_or_take_over(self, reason):
        self.phase = 'blocked'; self.block_reason = reason

    def export_record(self):
        return dict(schema_version=1, cycle_id=self.cycle_id, map_sha256=self.map_sha256,
                    localization_epoch=self.epoch, phase=self.phase, policy=asdict(self.policy),
                    trace=list(self.trace), return_plan=self.return_plan,
                    block_reason=self.block_reason, runtime_integrated=False)
