"""One explicit start, then a fresh controller lease; never resumes after cancellation.

The independent arbiter checks controller sequence *progress*, even if another publisher
replays an old run packet. This is not obstacle detection.
"""
import math
import re


class RunLeaseGate:
    TIMEOUT = .25

    def __init__(self):
        self.key = None
        self.sequence = None
        self.advanced_at = None
        self.closed = set()

    def close(self):
        if self.key is not None:
            self.closed.add(self.key)
        self.key = self.sequence = self.advanced_at = None

    def check(self, packet, now):
        if type(now) not in (int, float) or not math.isfinite(now):
            raise ValueError('invalid run lease clock')
        token = packet.get('run_session')
        sequence = packet.get('run_sequence')
        key = tuple(packet.get(k) for k in ('owner', 'trial_id', 'cycle_id', 'goal_id')) + (token,)
        if (packet.get('run_state') != 'running' or not isinstance(token, str) or
                re.fullmatch('[0-9a-f]{32}', token) is None or
                type(sequence) is not int or sequence < 1):
            raise ValueError('explicit click start/session lease missing')
        if self.key is None:
            if key in self.closed:
                raise ValueError('cancelled run session cannot resume')
            self.key = key
            self.sequence = sequence
            self.advanced_at = now
        elif key != self.key:
            raise ValueError('run session changed while active')
        elif not 0 <= now-self.advanced_at <= self.TIMEOUT:
            raise ValueError('controller run lease expired')
        elif sequence < self.sequence:
            raise ValueError('run lease sequence reversed')
        elif sequence > self.sequence:
            self.sequence = sequence
            self.advanced_at = now
        if not 0 <= now-self.advanced_at <= self.TIMEOUT:
            raise ValueError('controller run lease expired')
