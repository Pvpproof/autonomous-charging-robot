"""Nominal gun TCP checks, in meters/radians. No robot or ROS side effects."""
import math
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation

JOINT_TOLERANCE_RAD=0.0005
MAX_POSITION_ERROR_M=0.001
MAX_ROTATION_ERROR_DEG=0.5
TCP_OFFSET_M=np.array([0.,0.,.123])
TCP_ROTATION=Rotation.from_euler('z',-90.,degrees=True).as_matrix()

def matrix(value):
    T=np.asarray(value,dtype=float)
    if (T.shape!=(4,4) or not np.isfinite(T).all() or not np.allclose(T[3],[0,0,0,1])
            or not np.allclose(T[:3,:3].T@T[:3,:3],np.eye(3),atol=1e-6)
            or abs(np.linalg.det(T[:3,:3])-1)>1e-6):
        raise ValueError('枪头验收变换矩阵无效')
    return T

def tcp_from_link6(value):
    T=matrix(value).copy();T[:3,3]+=T[:3,:3]@TCP_OFFSET_M
    T[:3,:3]=T[:3,:3]@TCP_ROTATION
    return T

def target_from_socket(value):
    S=matrix(value);T=np.eye(4)
    T[:3,3]=S[:3,3]+S[:3,2]*.050
    T[:3,:3]=np.column_stack((-S[:3,0],S[:3,1],-S[:3,2]))
    return T


COMPENSATION_CONFIG=Path('/opt/charging-robot-local/nova2_tools/gun_lateral_compensation.json')

def checked_compensation(value):
    if not isinstance(value,dict) or value.get('frame')!='socket_minus_x':
        raise ValueError('Invalid lateral compensation frame')
    if type(value.get('enabled')) is not bool:
        raise ValueError('Invalid lateral compensation switch')
    mm=value.get('left_mm')
    if isinstance(mm,bool) or not isinstance(mm,(int,float)) or not math.isfinite(mm) or abs(mm)>10:
        raise ValueError('Lateral compensation must be finite and within 10 mm')
    angle=value.get('right_yaw_deg',0.0)
    if isinstance(angle,bool) or not isinstance(angle,(int,float)) or not math.isfinite(angle) or abs(angle)>10:
        raise ValueError('Angular compensation must be finite and within 10 degrees')
    return dict(enabled=value['enabled'],left_mm=float(mm),frame='socket_minus_x',right_yaw_deg=float(angle))

def load_compensation():
    return checked_compensation(json.loads(COMPENSATION_CONFIG.read_text()))

def compensated_target(value,compensation):
    target=target_from_socket(value)
    c=checked_compensation(compensation)
    if c['enabled']:
        target[:3,3]-=matrix(value)[:3,0]*(c['left_mm']/1000.)
        # TCP fixed; local +X is left, so negative rotation about +Y turns right.
        target[:3,:3]=target[:3,:3]@Rotation.from_euler('y',-c['right_yaw_deg'],degrees=True).as_matrix()
    return target


def evaluate(actual,target):
    A=matrix(actual);T=matrix(target)
    position=float(np.linalg.norm(A[:3,3]-T[:3,3]))
    rotation=float(np.degrees(Rotation.from_matrix(T[:3,:3].T@A[:3,:3]).magnitude()))
    return dict(passed=position<=MAX_POSITION_ERROR_M and rotation<=MAX_ROTATION_ERROR_DEG,
        position_error_mm=position*1000,rotation_error_deg=rotation,
        position_error_base_mm=((A[:3,3]-T[:3,3])*1000).tolist(),
        max_position_error_mm=MAX_POSITION_ERROR_M*1000,max_rotation_error_deg=MAX_ROTATION_ERROR_DEG,
        target_tcp_base=T.tolist(),actual_tcp_base=A.tolist(),
        tcp_definition_mm_deg=[0,0,123,0,0,-90])

def require(actual,target):
    result=evaluate(actual,target)
    if not result['passed']:
        raise ValueError(f"枪头末端验收失败：位置误差{result['position_error_mm']:.3f}mm（上限1mm），方向误差{result['rotation_error_deg']:.3f}度（上限0.5度）；未导出可执行轨迹")
    return result

def validate_saved(data,socket):
    evidence=data.get('endpoint_validation')
    if not isinstance(evidence,dict) or evidence.get('passed') is not True:
        raise ValueError('枪头轨迹缺少末端验收，请重新gun-plan')
    if data.get('pose_source') == 'fused_icp_v3':
        fusion=data.get('fusion',{})
        if fusion.get('enabled') is not True or fusion.get('method')!='weighted_icp_v3':
            raise ValueError('Fusion evidence missing; regenerate gun-plan')
        ref=fusion.get('icp_reference',{})
        if (not data.get('cycle_id') or fusion.get('cycle_id')!=data['cycle_id']
                or ref.get('cycle_id')!=data['cycle_id'] or ref.get('execution_verified') is not True
                or not ref.get('camera_plan_id') or not ref.get('camera_request_id')):
            raise ValueError('Fusion lacks committed current-cycle ICP provenance')
        v3=matrix(fusion.get('T_base_from_socket_v3'))
        icp=matrix(fusion.get('T_base_from_socket_icp'))
        if not np.allclose(icp,matrix(ref.get('T_base_from_socket')),atol=1e-9,rtol=0):raise ValueError('ICP evidence inconsistent')
        if not np.allclose(v3,matrix(socket),atol=1e-9,rtol=0):
            raise ValueError('Fusion V3 input differs from frozen measurement')
        if fusion.get('icp_weight')!=0.30 or fusion.get('v3_weight')!=0.70:
            raise ValueError('Unsupported fusion weights')
        delta=Rotation.from_matrix(v3[:3,:3].T@icp[:3,:3])
        if np.linalg.norm(v3[:3,3]-icp[:3,3])>0.015 or np.degrees(delta.magnitude())>8.0:
            raise ValueError('Fusion input disagreement exceeds gate')
        fused=v3.copy()
        fused[:3,3]=0.70*v3[:3,3]+0.30*icp[:3,3]
        rotation_icp_weight=float(fusion.get("rotation_icp_weight",0.30))
        if rotation_icp_weight!=.50:raise ValueError('Unsupported rotation fusion weight')
        fused[:3,:3]=v3[:3,:3]@Rotation.from_rotvec(rotation_icp_weight*delta.as_rotvec()).as_matrix()
        if not np.allclose(fused,matrix(data.get('T_base_from_socket')),atol=1e-9,rtol=0):
            raise ValueError('Saved fusion target differs from recomputed target')
        socket=fused
    compensation=data.get("lateral_compensation")
    target=(target_from_socket(socket) if compensation is None else compensated_target(socket,compensation))
    if not np.allclose(matrix(evidence.get('target_tcp_base')),target,atol=1e-9,rtol=0):
        raise ValueError('枪头末端验收目标与冻结插座不匹配')
    if evidence.get('joint_names')!=data.get('joint_names') or evidence.get('joint_positions')!=data['points'][-1]['positions']:
        raise ValueError('枪头轨迹终点与验收关节不匹配')
    require(evidence.get('actual_tcp_base'),target)
    return target

def link6_from_controller(values):
    v=np.asarray(values,dtype=float)
    if v.shape!=(6,) or not np.isfinite(v).all():raise ValueError('GetPose反馈格式无效')
    T=np.eye(4);T[:3,3]=v[:3]/1000.
    T[:3,:3]=Rotation.from_euler('xyz',v[3:],degrees=True).as_matrix()
    return T
