"""Motion ownership checks only. No map, route, tool-result or ROS dependencies."""
import math


def finite(value):
    return type(value) in (int, float) and math.isfinite(value)


def independent_tool_reason(s):
    """Reject known concurrent motion, not a missing base connection.

    Missing feedback is not proof of standstill: the operator confirms the
    physical workspace, while base motion remains independently stop-latched.
    """
    if s.get('busy') or s.get('navigation_busy') or s.get('return_active'):
        return '请先结束底盘导航或返程'
    if s.get('manual') or s.get('manual_stop_pending') or s.get('stop_reset_pending'):
        return '请先结束底盘操控或解除停车请求'
    age=s.get('odom_age');v=s.get('velocity')
    if finite(age) and 0<=age<=1 and isinstance(v,(tuple,list)) and len(v)==2:
        if all(finite(x) for x in v) and (abs(v[0])>.01 or abs(v[1])>.02):
            return '底盘反馈仍显示运动'
    a=s.get('arbiter',{});age=s.get('arbiter_age')
    if finite(age) and 0<=age<=.25:
        if a.get('mode') in ('navigation','manual','return'):
            return '底盘控制仍被运动任务占用'
        if any(finite(a.get(k)) and a[k]!=0. for k in ('output_v_m_s','output_w_rad_s')):
            return '底盘控制输出尚未归零'
    return None


def base_tool_reason(s, *, held=False):
    if s.get('busy') or s.get('return_active'):
        return '底盘导航或返程尚未结束'
    if s.get('manual') or s.get('manual_stop_pending'):
        return '请先退出键盘并等待停稳'
    if s.get('stop_reset_pending'):
        return '停车解除尚未确认'
    if held and s.get('docking_hold') is not True:
        return '底盘占用锁未保持'
    for name, label, limit in [('odom_age', '里程计', 1.), ('arbiter_age', '底盘控制反馈', .25)]:
        age = s.get(name)
        if not finite(age) or not 0 <= age <= limit:
            return label+'未就绪'
    a = s.get('arbiter', {})
    if (a.get('owner') != s.get('control_owner') or not s.get('control_owner') or
            a.get('mode') != 'stop' or a.get('graph_ready') is not True or
            a.get('driver_gate_verified') is not True or a.get('fault')):
        return '底盘控制权或停车状态未确认'
    if any(not finite(a.get(k)) or a[k] != 0. for k in ('output_v_m_s', 'output_w_rad_s')):
        return '底盘控制输出尚未归零'
    velocity = s.get('velocity')
    if (not isinstance(velocity, (list, tuple)) or len(velocity) != 2 or
            not all(finite(x) for x in velocity) or abs(velocity[0]) > .01 or abs(velocity[1]) > .02):
        return '底盘尚未停稳'
    quiet = s.get('stationary_s')
    if (not finite(quiet) or quiet < 2. or type(s.get('stationary_samples')) is not int or
            s['stationary_samples'] < 3):
        return '等待底盘连续停稳两秒'
    return None


def locked_base_ready(snapshot, now):
    """Verify the actual downstream stop before permitting the external runner."""
    a = snapshot.get('arbiter', {})
    raw = snapshot.get('raw_odom') or {}
    ages = [snapshot.get('arbiter_age'), snapshot.get('odom_age')]
    received = raw.get('received_at')
    source, ros_now = raw.get('source_stamp'), snapshot.get('ros_now')
    if not all(finite(v) for v in (received, source, ros_now, now)):
        return False
    ages += [now-received, ros_now-source]
    if any(not finite(v) or not 0 <= v <= .25 for v in ages):
        return False
    if (snapshot.get('docking_hold') is not True or snapshot.get('software_stop') is not False or
            snapshot.get('busy') is not False or snapshot.get('manual') is not False or
            snapshot.get('manual_stop_pending') is not False or snapshot.get('stop_reset_pending') is not False):
        return False
    if (a.get('owner') != snapshot.get('control_owner') or a.get('mode') != 'stop' or
            a.get('fault') or a.get('graph_ready') is not True or a.get('driver_gate_verified') is not True or
            a.get('raw_odom_ready') is not True):
        return False
    if any(not finite(a.get(k)) or a[k] != 0 for k in ('output_v_m_s','output_w_rad_s')):
        return False
    quiet = snapshot.get('raw_stationary_s')
    if (not finite(quiet) or quiet < 2 or type(snapshot.get('raw_stationary_samples')) is not int or
            snapshot['raw_stationary_samples'] < 3):
        return False
    for v in (snapshot.get('velocity'), raw.get('velocity')):
        if not isinstance(v, (list,tuple)) or len(v) not in (2,3) or not all(finite(x) for x in v):
            return False
        if abs(v[0]) > .01 or abs(v[-1]) > .02 or (len(v)==3 and abs(v[1])>.01):
            return False
    return True
