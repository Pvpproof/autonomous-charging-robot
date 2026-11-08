"""Base ownership for independent arm tools, on the existing Bridge lock."""
import time
from arm_tools_guard import base_tool_reason,independent_tool_reason


class ArmToolsBridge:
    def begin_independent_arm_hold(self):
        with self.lock:
            reason=independent_tool_reason(self._snapshot(time.monotonic()))
            if reason:raise ValueError(reason)
            self.docking_hold=True
            self.latch_software_stop('arm_tool_hold','机械臂工具占用，停止底盘当前任务')
            self.manual=False;self.manual_target=(0.,0.)
            self.stop_reset_pending=None;self.reset_arbiter=False
        self.event('independent_arm_base_stop_latched',standstill_verified=False,
                   operator_confirmation_required=True,automatic_resume=False)

    def begin_arm_hold(self):
        with self.lock:
            self.return_guard()
            s = self._snapshot(time.monotonic())
            # Standstill is measured from odom, not from navigation success time.
            s['stationary_s'] = self.stop_observation.duration(self.stop_observation.first)
            reason = base_tool_reason(s)
            if reason:
                raise ValueError(reason)
            self.docking_hold = True
            self.arm_hold_requested_at = time.monotonic()
        self.event('arm_tool_base_held', automatic_resume=False)

    def arm_hold_snapshot(self):
        with self.lock:
            now = time.monotonic()
            s = self._snapshot(now)
            s['stationary_s'] = self.stop_observation.duration(self.stop_observation.first)
            at = getattr(self, 'arm_hold_requested_at', None)
            s['hold_feedback_confirmed'] = bool(at is not None and self.arbiter_at is not None and
                                                 self.arbiter_at >= at)
            return s

    def release_docking(self):
        with self.lock:
            self.return_guard()
            reason = independent_tool_reason(self._snapshot(time.monotonic()))
            if reason:
                raise ValueError(reason)
            # A terminal result is deliberately NOT consulted. Caller requires an
            # explicit physical-clear confirmation and no live tool processes.
            # An old cancellation flag must not require another navigation run.
            # A real software stop/fault remains latched and is reset separately.
            self.docking_hold = False
        self.event('arm_tool_base_released', operator_confirmed_arm_clear=True,
                   automatic_resume=False, route_preserved=True,software_stop_retained=self.software_stop)
