"""Read-only coordinate capture for the passive recorder. No command output."""
import math
from pathlib import Path
import yaml
from charging_robot.returning.route_cycle import CycleError, near

def compose(a, b):
    c, s = math.cos(a[2]), math.sin(a[2])
    return (a[0]+c*b[0]-s*b[1], a[1]+s*b[0]+c*b[1],
            math.atan2(math.sin(a[2]+b[2]), math.cos(a[2]+b[2])))


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


def coordinate_sample(snapshot, now):
    """All values come from one TF lookup cycle; UI polling never refreshes them."""
    for key in ('pose', 'odom_pose', 'map_to_odom'):
        value = snapshot.get(key)
        if value is None or len(value) != 3 or not all(math.isfinite(v) for v in value):
            raise CycleError('缺少地图/里程计成对位姿，请重新记录完整去程')
    for key in ('pose_age', 'odom_tf_age', 'alignment_age', 'paired_odom_age'):
        age = snapshot.get(key, math.inf)
        if not math.isfinite(age) or not 0 <= age <= .25:
            raise CycleError('成对坐标反馈过期：{}={:.3f}s（上限0.250s）'.format(key,age))
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
    if not math.isfinite(snapshot.get('odom_source_stamp',math.nan)) or abs(stamp-snapshot['odom_source_stamp'])>.25:
        raise CycleError('地图位姿与里程计消息时间不同步')
    return dict(source_stamp=stamp, map_grid_sha=snapshot['map_grid_sha'],pose=list(snapshot['pose']),
                odom_pose=list(snapshot['odom_pose']), map_to_odom=list(snapshot['map_to_odom']),
                odom_source_stamp=snapshot.get('odom_source_stamp'),
                observed_w_rad_s=snapshot['velocity'][1])
