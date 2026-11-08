"""Observation only: no ROS calls and no motion or electrical authorization."""
import math

FRESH_SECONDS = 1.0
STILL_SECONDS = 2.0
LINEAR_LIMIT = .01
ANGULAR_LIMIT = .02


class StopObservation:
    def __init__(self):
        self.first = None
        self.last = None
        self.samples = 0

    def update(self, v, w, now):
        quiet = all(math.isfinite(x) for x in (v, w, now)) and abs(v) <= LINEAR_LIMIT and abs(w) <= ANGULAR_LIMIT
        if not quiet:
            self.first = None
            self.samples = 0
        else:
            if self.last is None or now <= self.last or now-self.last > FRESH_SECONDS:
                self.first = None
                self.samples = 0
            if self.first is None:
                self.first = now
            self.samples += 1
        self.last = now

    def duration(self, after):
        if self.first is None or self.last is None or after is None:
            return 0.
        return max(0., self.last-max(self.first, after))


def readiness(s, matching):
    """A software observation gate, never a claim that docking is safe."""
    if s.get('docking_hold'):
        return False, '对接锁定中：底盘操控已锁，机械臂状态需现场确认'
    if s.get('software_stop'):
        return False, '底盘软件停车锁定中；解除不会恢复对接许可'
    if not matching:
        return False, '请使用当前定位导航地图，并完成本面板的一次导航'
    if s.get('busy') or s.get('manual') or s.get('manual_stop_pending'):
        return False, '底盘任务或键盘操控尚未结束'
    if s.get('cancel_requested'):
        return False, '本次已请求取消，不能作为对接许可；需重新导航'
    status = s.get('goal_status')
    if status != 4:
        return False, {5:'导航已取消', 6:'导航失败'}.get(status, '等待本面板导航成功；未接受或结果未知不能放行')
    for field, label in [('pose_age','定位'),('odom_age','里程计'),('scan_age','雷达')]:
        age = s.get(field, math.inf)
        if not math.isfinite(age) or not 0 <= age <= FRESH_SECONDS:
            return False, label+'数据失效，不能确认车辆状态'
    pose, target = s.get('pose'), s.get('goal_target')
    if not pose or not target or not all(math.isfinite(v) for v in (*pose, *target)):
        return False, '当前定位或本次目标无效'
    quiet = s.get('stationary_s', 0.)
    if not math.isfinite(quiet) or quiet < STILL_SECONDS or s.get('stationary_samples', 0) < 3:
        return False, '导航成功仅代表Nav2容差；等待成功后连续停稳 {:.1f}/{:.1f} 秒'.format(max(0.,quiet),STILL_SECONDS)
    return True, '已观察到导航后连续停稳；仍需现场确认位置、朝向及机械臂工作区'


def goal_error(s):
    p, t = s.get('pose'), s.get('goal_target')
    if not p or not t or not all(math.isfinite(v) for v in (*p, *t)):
        return None
    return math.hypot(p[0]-t[0],p[1]-t[1]), abs(math.degrees(math.atan2(math.sin(p[2]-t[2]),math.cos(p[2]-t[2]))))
