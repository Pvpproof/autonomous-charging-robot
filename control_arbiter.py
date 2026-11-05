"""Single downstream chassis command owner. Pure state machine; no ROS here."""
import math


class Arbiter:
    TIMEOUT = .25
    MODES = ('stop', 'navigation', 'manual', 'return')

    def __init__(self):
        self.owner = None
        self.sequence = -1
        self.packet_at = None
        self.mode = 'stop'
        self.packet = None
        self.nav = None
        self.nav_at = None
        self.acquired_at = None
        self.fault = None
        self.odom_at = None
        self.still_since = None
        self.odom_stamp = None
        self.nav_active = None

    def fail(self, reason):
        self.fault = reason
        self.mode = 'stop'
        return (0., 0.)

    def odom(self, v, w, stamp, now, frame_ok=True):
        if (not frame_ok or not all(math.isfinite(x) for x in (v, w, stamp, now)) or
                (self.odom_stamp is not None and stamp <= self.odom_stamp)):
            self.fail('invalid/repeated odometry'); return
        if self.odom_at is None or not 0 <= now-self.odom_at <= self.TIMEOUT:
            self.still_since = None
        self.odom_stamp, self.odom_at = stamp, now
        if abs(v) <= .01 and abs(w) <= .02:
            if self.still_since is None: self.still_since = now
        else:
            self.still_since = None

    def quiet(self, now):
        return (self.odom_at is not None and 0 <= now-self.odom_at <= self.TIMEOUT and
                self.still_since is not None and now-self.still_since >= 2.)

    def receive(self, packet, now, ros_now):
        try:
            owner, seq, mode = packet['owner'], packet['sequence'], packet['mode']
            v, w = packet['v'], packet['w']
            if (not isinstance(owner, str) or not owner or type(seq) is not int or
                    mode not in self.MODES or not all(math.isfinite(x) for x in (v, w, packet['stamp'])) or
                    not 0 <= ros_now-packet['stamp'] <= self.TIMEOUT):
                raise ValueError('invalid/stale authority packet')
            if self.owner is not None and owner != self.owner:
                # New panels must explicitly request a stationary reset; never take over in motion.
                if not (mode == 'stop' and packet.get('reset') is True and self.quiet(now)):
                    raise ValueError('competing authority owner')
                self.sequence = -1
            if seq <= self.sequence: raise ValueError('replayed authority sequence')
            if packet.get('reset') is True:
                if mode != 'stop' or not self.quiet(now) or v != 0 or w != 0:
                    raise ValueError('reset requires fresh continuous standstill')
                self.fault = None
            if mode == 'return' and (v > 0 or abs(v) > .05 or abs(w) > abs(v)/1.78+1e-9):
                raise ValueError('return command exceeds independent envelope')
            if mode == 'manual' and (abs(v) > .3 or abs(w) > abs(v)/1.78+1e-9):
                raise ValueError('manual command exceeds envelope')
            if mode != self.mode:
                if self.mode != 'stop' and mode != 'stop':
                    raise ValueError('control handoff must pass through stop')
                if mode == 'return' and (not self.quiet(now) or self.nav_active is not False):
                    raise ValueError('return handoff requires inactive navigation and standstill')
                self.acquired_at = now
                self.nav_at = None  # No buffered navigation command survives a handoff.
            self.owner, self.sequence = owner, seq
            self.packet_at, self.packet = now, dict(packet)
            if self.fault is None: self.mode = mode
        except (ValueError, KeyError, TypeError, OverflowError) as error:
            self.fail(str(error))

    def navigation(self, v, w, now):
        if not all(math.isfinite(x) for x in (v, w, now)):
            self.fail('invalid navigation velocity'); return
        self.nav, self.nav_at = (v, w), now

    def tick(self, now, *, graph_ok=True):
        if not graph_ok: return self.fail('unexpected chassis command graph')
        if self.fault: return (0., 0.)
        if self.mode == 'stop': return (0., 0.)
        if self.packet_at is None or not 0 <= now-self.packet_at <= self.TIMEOUT:
            return self.fail('panel authority heartbeat expired')
        if self.odom_at is None or not 0 <= now-self.odom_at <= self.TIMEOUT:
            return self.fail('odometry feedback expired')
        if self.mode == 'return' and self.nav_active is not False:
            return self.fail('navigation reacquired control during return')
        if self.mode == 'navigation':
            if self.nav_at is None or self.nav_at < self.acquired_at:
                return (0., 0.)
            if not 0 <= now-self.nav_at <= self.TIMEOUT:
                return self.fail('navigation command expired')
            return self.nav
        return self.packet['v'], self.packet['w']
