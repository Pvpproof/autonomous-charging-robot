"""Offline candidate: reverse pursuit with measured body-yaw feedback.

Does not alter the recorded path or loosen ReturnMonitor's error thresholds.
This numerical helper is used by the supervised return candidate.
Its gains have not been validated on a physical vehicle.
"""
import bisect
from offline_reverse import ReversePursuit, wrap


class PoseAwareReversePursuit(ReversePursuit):
    def __init__(self,path,speed,minimum_radius,lookahead=.35,heading_gain=1.):
        super().__init__(path,speed,minimum_radius,lookahead)
        self.heading_gain=heading_gain

    def control(self,actual_pose):
        v,w,done,error=super().control(actual_pose)
        if done:return v,w,done,error
        i=min(len(self.path)-2,max(0,bisect.bisect_right(self.s,self.progress)-1))
        a,b=self.path[i:i+2]
        t=(self.progress-self.s[i])/max(1e-9,self.s[i+1]-self.s[i])
        reference_yaw=a[2]+t*wrap(b[2]-a[2])
        w+=abs(v)*self.heading_gain*wrap(reference_yaw-actual_pose[2])
        w=max(-abs(v)/self.radius,min(abs(v)/self.radius,w))
        return v,w,done,error
