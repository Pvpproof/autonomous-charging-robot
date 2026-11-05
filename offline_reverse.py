"""Offline kinematic experiment only. No ROS, sockets or device API.

Returns numerical control values; this module performs no device I/O.
The supervised return candidate uses these helpers, without hardware validation.
"""
import bisect
import math


def wrap(angle):return math.atan2(math.sin(angle),math.cos(angle))


class ReversePursuit:
    def __init__(self,path,speed,minimum_radius,lookahead=.35):
        self.path=path;self.speed=speed;self.radius=minimum_radius;self.lookahead=lookahead
        assert speed>0 and minimum_radius>0
        self.s=[0.]
        for a,b in zip(path,path[1:]):self.s.append(self.s[-1]+math.dist(a[:2],b[:2]))
        self.progress=0.

    def control(self,actual_pose):
        # Recompute from actual pose on every simulation step, not saved commands.
        choices=[]
        for i,(a,b) in enumerate(zip(self.path,self.path[1:])):
            dx,dy=b[0]-a[0],b[1]-a[1];length=self.s[i+1]-self.s[i]
            if length<1e-9:continue
            t=max(0.,min(1.,((actual_pose[0]-a[0])*dx+(actual_pose[1]-a[1])*dy)/length**2))
            progress=self.s[i]+t*length
            if self.progress-.02<=progress<=self.progress+.25:
                choices.append((math.hypot(actual_pose[0]-a[0]-t*dx,actual_pose[1]-a[1]-t*dy),progress))
        if not choices:raise ValueError('lost continuous path projection')
        error,progress=min(choices);self.progress=max(self.progress,progress)
        if error>.15:raise ValueError('cross track limit exceeded')
        end=self.path[-1];distance=math.dist(actual_pose[:2],end[:2]);yaw_error=abs(wrap(actual_pose[2]-end[2]))
        if distance<.04 and yaw_error<.05:return 0.,0.,True,error
        target_s=min(self.s[-1],self.progress+self.lookahead)
        i=min(len(self.path)-2,max(0,bisect.bisect_right(self.s,target_s)-1))
        a,b=self.path[i:i+2];t=(target_s-self.s[i])/max(1e-9,self.s[i+1]-self.s[i])
        dx,dy=a[0]+t*(b[0]-a[0])-actual_pose[0],a[1]+t*(b[1]-a[1])-actual_pose[1]
        # Travel heading is body yaw + pi; body yaw itself stays unchanged.
        lateral=math.sin(actual_pose[2])*dx-math.cos(actual_pose[2])*dy
        travel_curvature=2*lateral/max(.0025,dx*dx+dy*dy)
        travel_curvature=max(-1/self.radius,min(1/self.radius,travel_curvature))
        speed=min(self.speed,max(.015,distance*.6))
        return -speed,speed*travel_curvature,False,error


def forward_path(curvature,total=4.,ds=.02):
    pose=(0.,0.,0.);path=[pose]
    for step in range(round(total/ds)):
        k=curvature(step*ds);theta=pose[2]+k*ds/2
        pose=(pose[0]+ds*math.cos(theta),pose[1]+ds*math.sin(theta),wrap(pose[2]+k*ds))
        path.append(pose)
    return path


def experiment(name,curvature,feedback=True):
    recorded=forward_path(curvature);reverse=list(reversed(recorded))
    reference=ReversePursuit(reverse,.1,1.78)
    actual=(reverse[0][0],reverse[0][1]+.025,wrap(reverse[0][2]+.015))
    ideal=reverse[0];max_error=0.;trace=[];dt=.1
    for step in range(2000):
        v,w,done,error=reference.control(actual if feedback else ideal)
        max_error=max(max_error,error)
        if done:
            return dict(case=name,feedback=feedback,simulated_seconds=round(step*dt,2),
                endpoint_error_m=math.dist(actual[:2],reverse[-1][:2]),
                yaw_error_rad=abs(wrap(actual[2]-reverse[-1][2])),
                max_measured_cross_track_m=max_error,steps=step,
                hardware_validated=False)
        # A small unmodelled yaw-rate bias makes real feedback materially useful.
        actual=(actual[0]+v*math.cos(actual[2])*dt,actual[1]+v*math.sin(actual[2])*dt,
                wrap(actual[2]+(w+.0015)*dt))
        ideal=(ideal[0]+v*math.cos(ideal[2])*dt,ideal[1]+v*math.sin(ideal[2])*dt,
               wrap(ideal[2]+w*dt))
    raise ValueError(name+': no convergence')
