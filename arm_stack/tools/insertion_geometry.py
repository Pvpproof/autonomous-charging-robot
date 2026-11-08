"""Pure geometry checks for the actual ServoJ joint interpolation (metres/radians)."""
import math
import numpy as np
from scipy.spatial.transform import Rotation

def check_tcp(T,origin,axis,R,depth):
    off=T[:3,3]-origin;along=float(off@axis);lateral=float(np.linalg.norm(off-axis*along))
    angular=float(Rotation.from_matrix(R.T@T[:3,:3]).magnitude())
    if lateral>.0005 or angular>math.radians(.25) or along<-.001 or along>depth+.001:
        raise RuntimeError('插补路径偏离轴线：横向%.3fmm 角度%.3f度 轴向%.3fmm'%(lateral*1000,math.degrees(angular),along*1000))
    return along

def segment_samples(a,b):
    a=np.asarray(a,float);b=np.asarray(b,float)
    if a.shape!=(6,) or b.shape!=(6,) or not np.isfinite([a,b]).all():raise RuntimeError('关节路径无效')
    if max(abs(b-a))>math.radians(3):raise RuntimeError('插入路径关节跳变超过3度')
    # Sample the joint-linear path, including midpoint even for tiny changes.
    n=max(2,int(math.ceil(max(abs(b-a))/math.radians(.1))))
    fractions=sorted({i/n for i in range(1,n+1)} | {.5})
    return [a+(b-a)*f for f in fractions]

def axis_report(T,socket_axis,depth):
    axis=T[:3,2];s=np.asarray(socket_axis,float);s=s/np.linalg.norm(s)
    angle=math.degrees(math.acos(float(np.clip(axis@s,-1,1))))
    return dict(policy='compensated_tool_z',axis_base=axis.tolist(),socket_axis_base=s.tolist(),
        tool_socket_angle_deg=angle,travel_mm=depth*1000,
        transverse_travel_mm=float(np.linalg.norm(axis-s*float(axis@s))*depth*1000),
        transverse_at_50mm_mm=float(np.linalg.norm(axis-s*float(axis@s))*.05*1000))


def seeded_cartesian_path(start,depth,solve,max_step_m=.001):
    """Refine Cartesian distances, not coarse joint chords, using previous IK seed."""
    q=np.asarray(start,float)
    if q.shape!=(6,) or not np.isfinite(q).all():raise RuntimeError('插入起点关节无效')
    if not math.isfinite(depth) or not 0<depth<=.080 or not .0001<=max_step_m<=.002:
        raise RuntimeError('插入细化步长/深度无效')
    count=int(math.ceil(depth/max_step_m));path=[q.copy()]
    for i in range(1,count+1):
        qi=np.asarray(solve(depth*i/count,path[-1].copy()),float)
        if qi.shape!=(6,) or not np.isfinite(qi).all():raise RuntimeError('第%d个细化IK点无效'%i)
        delta=np.degrees(qi-path[-1]);j=int(np.argmax(abs(delta)))
        if abs(delta[j])>3:
            raise RuntimeError('细化路径第%d/%d点J%d跳变%.3f度（上限3度，步长%.3fmm）；未生成轨迹'%
                               (i,count,j+1,delta[j],depth/count*1000))
        path.append(qi)
    return path
