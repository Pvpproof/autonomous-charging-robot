"""Fresh driver-origin status; startup lock is distinct from static readiness."""
import math
import re

class DriverWatchdogState:
    TIMEOUT=.25
    def __init__(self):
        self.state=None;self.received_at=None;self.error='driver watchdog state unavailable'
    @staticmethod
    def validate(data,now):
        if not isinstance(data,dict):raise ValueError('invalid driver watchdog state')
        if (data.get('version')!=2 or type(data.get('latched')) is not bool or
                not isinstance(data.get('session'),str) or re.fullmatch('(?:[0-9a-f]{32}|[0-9a-f]{8}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{12})',data['session']) is None or
                type(data.get('state_seq')) is not int or data['state_seq']<1 or
                type(data.get('monotonic_ns')) is not int):raise ValueError('driver watchdog v2 contract invalid')
        if not 0<=now-data['monotonic_ns']/1e9<=.25:raise ValueError('driver-origin watchdog state stale')
        for field in ('command_age_s','feedback_age_s'):
            value=data.get(field)
            if type(value) not in (float,int) or not math.isfinite(value) or (value<0 and not (value==-1 and data['latched'])):raise ValueError('driver watchdog age invalid')
        return data
    def observe(self,data,now):
        self.validate(data,now);previous=self.state
        changed=bool(previous and previous['session']!=data['session'])
        if previous and not changed and data['state_seq']<=previous['state_seq']:return False
        self.state=dict(data);self.received_at=now;self.error=None
        return changed
    def contract_ready(self,now):
        try:
            self.validate(self.state,now)
            return self.error is None and self.received_at is not None and 0<=now-self.received_at<=self.TIMEOUT
        except ValueError:return False
    def motion_ready(self,now):
        return self.contract_ready(now) and self.state['latched'] is False and max(self.state['command_age_s'],self.state['feedback_age_s'])<=self.TIMEOUT
