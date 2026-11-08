"""Feedback return state machine. No ROS, hardware, or automatic frame alignment."""
import copy
import math
from pathlib import Path
import yaml
from charging_robot.returning.chassis_route import observe
from charging_robot.returning.route_cycle import CycleError, StartStore, near
from charging_robot.returning.supervised_draft import SupervisedDraft
from charging_robot.returning.pose_aware_reverse import PoseAwareReversePursuit


def read_alignment_tolerance(path):
    """Read the existing AMCL validity horizon; never change navigation parameters."""
    try:
        data=yaml.safe_load(Path(path).read_bytes())
        tolerance=data['amcl']['ros__parameters']['transform_tolerance']
        if type(tolerance) in (int,float) and math.isfinite(tolerance) and tolerance>=0:
            return float(tolerance)
    except (OSError,ValueError,KeyError,TypeError):
        pass
    return None


def alignment_timing(ros_now,valid_until,received_age,tolerance):
    """AMCL stamps TF at laser measurement time + transform_tolerance.

    A future validity horizon is not a future sensor measurement. Receipt
    freshness and inferred measurement freshness are checked independently.
    """
    if (tolerance is None or not all(math.isfinite(x) for x in
            (ros_now,valid_until,received_age,tolerance)) or tolerance<0 or received_age<0):
        return math.inf,False,math.inf
    source_age=ros_now-(valid_until-tolerance)
    clock_valid=source_age>=-.05
    age=max(0.,source_age,received_age) if clock_valid else math.inf
    return age,clock_valid,source_age



def compose(a, b):
    c, s = math.cos(a[2]), math.sin(a[2])
    return (a[0]+c*b[0]-s*b[1], a[1]+s*b[0]+c*b[1],
            math.atan2(math.sin(a[2]+b[2]), math.cos(a[2]+b[2])))


def coordinate_sample(snapshot, now):
    """All values come from one TF lookup cycle; UI polling never refreshes them."""
    for key in ('pose', 'odom_pose', 'map_to_odom'):
        value = snapshot.get(key)
        if value is None or len(value) != 3 or not all(math.isfinite(v) for v in value):
            raise CycleError('缺少地图/里程计成对位姿，请重新记录完整去程')
    for key in ('pose_age', 'odom_tf_age', 'alignment_age', 'odom_age'):
        age = snapshot.get(key, math.inf)
        if not math.isfinite(age) or not 0 <= age <= .25:
            raise CycleError('成对坐标反馈过期')
    if snapshot.get('odom_frame') != 'odom_combined' or snapshot.get('base_frame') != 'base_footprint':
        raise CycleError('里程计或车身坐标帧不匹配')
    if not snapshot.get('odom_frame_valid', False):
        raise CycleError('里程计消息坐标帧未验证')
    if not near(compose(snapshot['map_to_odom'], snapshot['odom_pose']), snapshot['pose'], .005, .005):
        raise CycleError('map→odom→base 与 map→base 不一致')
    if not snapshot.get('map_grid_sha'):
        raise CycleError('尚未验证实际地图内容')
    stamp = snapshot.get('pose_stamp')
    if not isinstance(stamp, (int, float)) or not math.isfinite(stamp):
        raise CycleError('定位测量时间无效')
    return dict(source_stamp=stamp, map_grid_sha=snapshot['map_grid_sha'],pose=list(snapshot['pose']),
                odom_pose=list(snapshot['odom_pose']), map_to_odom=list(snapshot['map_to_odom']),
                odom_source_stamp=snapshot.get('odom_source_stamp',stamp),observed_w_rad_s=snapshot['velocity'][1])


def check_plan(plan):
    if (plan.get('source') != 'actual_map_pose_measurements' or
            plan.get('frame_id') != 'map' or plan.get('reverse_only') is not True):
        raise CycleError('返程必须使用实际去程位姿倒序')
    samples = plan.get('coordinate_samples', [])
    if len(samples) < 2:
        raise CycleError('旧记录缺少成对坐标证据；仅可离线回放，请重新记录去程')
    reverse = []
    previous = None
    for point in samples:
        if not all(math.isfinite(v) for v in
                   (*point['pose'], *point['odom_pose'], *point['map_to_odom'], point['source_stamp'])):
            raise CycleError('记录坐标存在无效数值')
        if not near(compose(point['map_to_odom'], point['odom_pose']), point['pose'], .005, .005):
            raise CycleError('记录坐标链不一致')
        if previous:
            if not 0 < point['source_stamp']-previous['source_stamp'] <= 1.:
                raise CycleError('成对轨迹时间重复、乱序或中断')
            if not near(previous['map_to_odom'], point['map_to_odom'], .08, .05):
                raise CycleError('去程定位校正突变，需核查记录')
        if point.get('map_grid_sha') != samples[0].get('map_grid_sha') or not point.get('map_grid_sha'):
            raise CycleError('记录期间地图内容变化或缺失')
        previous = point
    for point in reversed(samples):
        if not reverse or point['pose'] != reverse[-1]:
            reverse.append(point['pose'])
    if reverse != plan['poses']:
        raise CycleError('返程路径并非成对实测路径的原朝向倒序')
    if sum(math.dist(a[:2], b[:2]) for a, b in zip(reverse, reverse[1:])) < .05:
        raise CycleError('实测路径不足')
    for a, b in zip(reverse, reverse[1:]):
        if (b[0]-a[0])*math.cos(a[2])+(b[1]-a[1])*math.sin(a[2]) > .03:
            raise CycleError('去程方向不支持全程倒车返回')


class ReturnRuntime:
    def __init__(self, plan, conditions, store_path):
        self.plan = copy.deepcopy(plan)
        check_plan(self.plan)
        self.runner = SupervisedDraft(self.plan, conditions, StartStore(store_path))
        self.runner.follower = PoseAwareReversePursuit(
            self.plan['poses'], conditions.speed_limit_m_s, conditions.minimum_radius_m)
        self.anchor = self.plan['coordinate_samples'][-1]
        self.last_coordinate = None
        self.started = False

    def stop(self, reason):
        return self.runner.stop(reason)

    def heartbeat(self, now, held):
        return self.runner.heartbeat(self.plan['cycle_id'], self.plan['goal_id'], now, held)

    def check_coordinates(self, snapshot, now, *, starting=False):
        if snapshot.get('map_sha256') != self.plan['map_sha256']:
            raise CycleError('当前加载地图与记录地图不同')
        if snapshot.get('localization_epoch') != self.plan['localization_epoch']:
            raise CycleError('定位会话已变化；旧坐标不能自动恢复')
        from charging_robot.returning.session_revalidation import validate_binding, runtime_identity
        if self.plan.get('prepared_runtime_identity') is not None and runtime_identity(snapshot)!=self.plan['prepared_runtime_identity']:
            raise CycleError('准备后控制链进程身份改变')
        adopted=validate_binding(self.plan,snapshot,now,starting=starting)
        if adopted is not None:self.anchor=adopted
        point = coordinate_sample(snapshot, now)
        # Check an announced AMCL correction before its future validity horizon.
        latest=snapshot.get('latest_map_to_odom')
        if latest is not None:
            if len(latest)!=3 or not all(math.isfinite(v) for v in latest):
                raise CycleError('最新地图校正无效')
            if not near(latest,self.anchor['map_to_odom'],.08,.05):
                raise CycleError('最新地图校正相对记录停点跳变，停止返程')
        if point['map_grid_sha'] != self.anchor['map_grid_sha']:
            raise CycleError('实际地图消息内容与记录不同')
        # Never translate/rotate a stored path to hide an unexplained discrepancy.
        if not near(point['map_to_odom'], self.anchor['map_to_odom'], .08, .05):
            raise CycleError('定位坐标系相对记录停点漂移，禁止自动对齐旧路线')
        if starting and (not near(point['pose'], self.anchor['pose'], .10, .10) or
                         not near(point['odom_pose'], self.anchor['odom_pose'], .10, .10)):
            delta = math.dist(point['pose'][:2], self.anchor['pose'][:2])
            raise CycleError('当前位姿与实测停点不一致（地图差 {:.3f} m）'.format(delta))
        if self.last_coordinate:
            old = self.last_coordinate
            if point['source_stamp'] < old['source_stamp']:
                raise CycleError('坐标测量时间倒退')
            identity_keys=('source_stamp','pose','odom_pose','map_to_odom','map_grid_sha')
            if point['source_stamp'] == old['source_stamp'] and any(point[k]!=old[k] for k in identity_keys):
                raise CycleError('相同时间戳的坐标发生变化')
            if point['source_stamp'] > old['source_stamp']:
                dt = point['source_stamp']-old['source_stamp']
                limit = self.runner.conditions.speed_limit_m_s*dt+.025
                if (dt > .25 or math.dist(point['pose'][:2], old['pose'][:2]) > limit or
                        math.dist(point['odom_pose'][:2], old['odom_pose'][:2]) > limit):
                    raise CycleError('返程实测位姿跳变或中断')
        self.last_coordinate = point
        return observe(snapshot, self.plan['map_sha256'])

    def tick(self, snapshot, now, *, owns_control):
        try:
            if self.runner.phase in ('stopped', 'complete'):
                return self.runner.output('TERMINAL_ZERO')
            if snapshot.get('busy') or snapshot.get('docking_hold'):
                raise CycleError('导航或对接仍占有控制权')
            obs = self.check_coordinates(snapshot, now, starting=not self.started)
            if not self.started:
                if not owns_control:
                    return self.runner.output('AWAIT_EXCLUSIVE_HANDOFF')
                self.started = True
                return self.runner.start(obs, now)
            if self.runner.phase == 'goal_wait_still':
                return self.runner.observe_stop(obs, now, owns_control=owns_control)
            return self.runner.step(obs, now, owns_control=owns_control)
        except (ValueError, KeyError, TypeError, OverflowError, OSError) as error:
            return self.stop(str(error))
