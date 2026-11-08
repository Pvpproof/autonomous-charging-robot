#!/usr/bin/env python3
"""Plan only: full Cartesian path, then validate joint-interpolated TCP samples."""
import json, math, time, uuid
from insertion_limits import INSERTION_DEPTH_M, MAX_INSERTION_DEPTH_MM
import numpy as np
import rclpy
from scipy.spatial.transform import Rotation
from geometry_msgs.msg import Pose
from moveit_msgs.srv import GetCartesianPath,GetStateValidity,GetPositionIK
from insert_force import Runner,ROOT,atomic
from prepare_insertion_target import prepare
from insertion_geometry import check_tcp,segment_samples,axis_report,seeded_cartesian_path

def plan(n):
    dest=ROOT/'pending_insertion_trajectory.json';dest.unlink(missing_ok=True)
    prepare();source=json.loads((ROOT/'last_dispatched_trajectory.json').read_text())
    names=source['joint_names'];n.mode();q=n.current();T=n.pose(q,names);L=n.pose(q,names,raw=True)
    expected=np.array(source['endpoint_validation']['target_tcp_base'],float)
    if (np.linalg.norm(T[:3,3]-expected[:3,3])>.001 or
        Rotation.from_matrix(expected[:3,:3].T@T[:3,:3]).magnitude()>math.radians(.5)):
        raise RuntimeError('实际TCP不在已验收的gun预备位，请重新gun-plan/execute')
    depth=INSERTION_DEPTH_M;axis=T[:3,2];origin=T[:3,3];R=T[:3,:3];quat=Rotation.from_matrix(L[:3,:3]).as_quat()
    c=n.create_client(GetCartesianPath,'/compute_cartesian_path')
    req=GetCartesianPath.Request();req.header.frame_id='base_link';req.group_name='nova2_group';req.link_name='Link6'
    req.start_state.joint_state.name=names;req.start_state.joint_state.position=list(q)
    req.max_step=.001;req.jump_threshold=1.5;req.avoid_collisions=True
    # Explicit collinear waypoints; do not substitute an RRT joint-goal plan.
    for distance in np.linspace(.002,depth,40):
        p=Pose();xyz=L[:3,3]+axis*distance
        p.position.x,p.position.y,p.position.z=map(float,xyz)
        p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w=map(float,quat);req.waypoints.append(p)
    out=n.call(c,req)
    if out.error_code.val!=1 or out.fraction<1.-1e-9:raise RuntimeError('笛卡尔路径不完整，fraction='+str(out.fraction))
    jt=out.solution.joint_trajectory
    if set(jt.joint_names)!=set(names) or len(jt.points)<2:raise RuntimeError('笛卡尔关节路径无效')
    coarse_count=len(jt.points)
    coarse_start=np.array([jt.points[0].positions[jt.joint_names.index(k)] for k in names])
    if max(abs(coarse_start-q))>math.radians(.1):raise RuntimeError('笛卡尔路径起点不匹配')
    print('[直线细化] MoveIt返回%d个重采样点；重新按1mm笛卡尔目标逐点求IK，不沿粗关节弦直接插值。'%coarse_count,flush=True)
    solved=[0]
    def solve(distance,seed):
        req=GetPositionIK.Request();r=req.ik_request
        r.group_name='nova2_group';r.ik_link_name='Link6';r.avoid_collisions=True;r.timeout.sec=1
        r.robot_state.joint_state.name=names;r.robot_state.joint_state.position=list(seed)
        r.pose_stamped.header.frame_id='base_link';p=r.pose_stamped.pose;xyz=L[:3,3]+axis*distance
        p.position.x,p.position.y,p.position.z=map(float,xyz)
        p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w=map(float,quat)
        result=n.call(n.ik,req)
        if result.error_code.val!=1:raise RuntimeError('直线细化IK失败：轴向%.2fmm code=%s'%(distance*1000,result.error_code.val))
        lookup=dict(zip(result.solution.joint_state.name,result.solution.joint_state.position))
        if not all(k in lookup for k in names):raise RuntimeError('细化IK缺少关节')
        qi=np.array([lookup[k] for k in names],float)
        solved[0]+=1
        if solved[0]%20==0:print('[直线细化] 已求解%d/%d个1mm目标'%(solved[0],round(depth/.001)),flush=True)
        return qi
    path=seeded_cartesian_path(q,depth,solve)
    endpoint_poses=[T]
    validity=n.create_client(GetStateValidity,'/check_state_validity');checks=0;previous_along=-.001
    for segment_index,(a,b) in enumerate(zip(path,path[1:]),1):
        for qi in segment_samples(a,b):
            Ti=n.pose(qi,names);along=check_tcp(Ti,origin,axis,R,depth)
            if along<previous_along-.0001:raise RuntimeError('笛卡尔插入路径发生轴向回退')
            previous_along=along
            vr=GetStateValidity.Request();vr.group_name='nova2_group';vr.robot_state.joint_state.name=names;vr.robot_state.joint_state.position=list(qi)
            if not n.call(validity,vr).valid:raise RuntimeError('插补采样状态碰撞/约束不满足')
            checks+=1
        endpoint_poses.append(Ti)
        if segment_index%20==0:print('[路径验收] %d/%d段通过'%(segment_index,len(path)-1),flush=True)
    end=endpoint_poses[-1]
    if np.linalg.norm(end[:3,3]-(origin+axis*depth))>.001:raise RuntimeError('插入终点未到75mm')
    # Use bounded Cartesian speed; no optimistic MoveIt timing from a joint-goal plan.
    points=[];last=q;elapsed=0.
    for i,qi in enumerate(path):
        if i:
            elapsed+=max(.05,np.linalg.norm(endpoint_poses[i][:3,3]-endpoint_poses[i-1][:3,3])/.005,
                         float(max(abs(qi-last)))/math.radians(10))
        points.append(dict(positions=qi.tolist(),velocities=[],accelerations=[],time_from_start=dict(sec=int(elapsed),nanosec=int((elapsed%1)*1e9))))
        last=qi
    data=dict(source);data.update(stage='insertion',plan_id='insertion-'+uuid.uuid4().hex,
        source_gun_plan_id=source['plan_id'],planned_at=time.time(),insertion_depth_mm=depth*1000.0,points=points,
        path_validation=dict(method='cartesian_full_path_joint_interpolation_samples',fraction=float(out.fraction),
            sample_count=checks,coarse_moveit_points=coarse_count,refined_points=len(path),
            refinement='seeded_cartesian_1mm',max_local_joint_delta_deg=float(np.max(abs(np.degrees(np.diff(path,axis=0))))),max_lateral_mm=.5,max_rotation_deg=.25,joint_sample_deg=.1),
        insertion_start_tcp_base=T.tolist(),insertion_axis=axis_report(T,-np.array(source['T_base_from_socket'])[:3,2],depth),
        planning_ready=True)
    if max(abs(n.current()-q))>math.radians(.1):raise RuntimeError('规划期间机械臂位置改变')
    atomic(dest,data);print('INSERTION_CARTESIAN_READY 75mm; samples=%d; speed<=5mm/s'%checks)
    print(json.dumps(data['insertion_axis'],ensure_ascii=False))

def main():
    rclpy.init();n=Runner()
    try:plan(n);return 0
    except Exception as e:print('INSERT_PLAN_REFUSED:',e);return 2
    finally:n.destroy_node();rclpy.shutdown()
if __name__=='__main__':raise SystemExit(main())
