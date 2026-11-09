"""Controller-origin progress evidence. No ROS counter or joint-change heuristic."""
import math
import time

class SourceProgress:
    def __init__(self, clock=time.monotonic):
        self.clock=clock;self.session=None;self.tick=None;self.stamp=0.;self.count=0;self.fault=None;self.rx_ns=None
    def accept(self, d):
        if self.fault:raise RuntimeError(self.fault)
        try:
            if d.get('feedback_schema')!=1 or d.get('source_valid') is not True:
                raise ValueError('missing/invalid receiver evidence')
            session=d['source_session'];tick=d['controller_timer'];age=d['source_progress_age_s']
            rx=d['source_progress_monotonic_ns']
            if type(rx) is not int or rx<=0 or not 0<=self.clock()-rx/1e9<=.4:
                raise ValueError('source receive CLOCK_MONOTONIC age (same host required)')
            if not isinstance(session,str) or not session:raise ValueError('source session')
            if type(tick) is not int or not 0<tick<2**64:raise ValueError('controller timestamp')
            if type(age) not in (int,float) or not math.isfinite(age) or not 0<=age<=.4:
                raise ValueError('receiver source age')
            if self.session is not None and session!=self.session:
                self.fault='controller receiver session changed; restart consumer after inspection';raise ValueError(self.fault)
            if self.tick is not None and tick<self.tick:
                self.fault='controller timestamp regressed; restart consumer after inspection';raise ValueError(self.fault)
            if tick==self.tick:return False
            if self.rx_ns is not None and rx<=self.rx_ns:
                self.fault='receiver progress time did not advance with controller timestamp'
                raise ValueError(self.fault)
            self.session=session;self.tick=tick;self.rx_ns=rx;self.stamp=self.clock();self.count+=1
            return True
        except Exception as e:raise RuntimeError('source feedback invalid: '+str(e)) from e
    def check(self):
        if (self.fault or self.count<2 or self.clock()-self.stamp>.4 or self.rx_ns is None
                or not 0<=self.clock()-self.rx_ns/1e9<=.4):
            raise RuntimeError(self.fault or 'controller source progress missing/frozen')
    @property
    def token(self):return (self.session,self.tick)
