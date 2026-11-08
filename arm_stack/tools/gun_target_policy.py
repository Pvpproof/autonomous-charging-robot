"""Frozen same-cycle fusion only. No ROS or motion side effects."""
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from gun_pose_validation import matrix
CONFIG=Path('/opt/charging-robot-local/nova2_tools/gun_target_policy.json')

def fuse(target,cycle,policy=None):
    c=json.loads(CONFIG.read_text()) if policy is None else policy
    if c.get('fusion_mode') not in ('icp_v3','v3_only'):raise ValueError('Invalid fusion_mode')
    if (c.get('position_icp_weight')!=.30 or c.get('rotation_icp_weight')!=.50
            or c.get('max_position_delta_mm')!=15. or c.get('max_rotation_delta_deg')!=8.
            or c.get('insertion_axis')!='compensated_tool_z'):raise ValueError('Unsupported gun target policy')
    if not cycle.get('cycle_id') or target.get('cycle_id')!=cycle['cycle_id']:raise ValueError('V3冻结目标不属于本轮，请重新cam-plan/execute/gun-plan')
    v3=matrix(target['T_base_from_socket']);f=dict(enabled=False,method='v3_only',mode=c['fusion_mode'],
        icp_weight=.30,v3_weight=.70,rotation_icp_weight=.50,rotation_v3_weight=.50,
        cycle_id=cycle['cycle_id'],T_base_from_socket_v3=v3.tolist())
    if c['fusion_mode']=='v3_only':return v3.copy(),f
    ref=target.get('icp_reference',{})
    if (ref.get('cycle_id')!=cycle['cycle_id'] or not ref.get('camera_plan_id')
            or not ref.get('camera_request_id') or ref.get('execution_verified') is not True):
        raise ValueError('ICP缺少本轮已执行cam-plan的绑定信息，请重新cam-plan/execute/gun-plan')
    icp=matrix(ref['T_base_from_socket']);delta=Rotation.from_matrix(v3[:3,:3].T@icp[:3,:3])
    distance=float(np.linalg.norm(v3[:3,3]-icp[:3,3]));angle=float(np.degrees(delta.magnitude()))
    f.update(icp_reference=ref,T_base_from_socket_icp=icp.tolist(),icp_position_delta_mm=distance*1000,
        icp_rotation_delta_deg=angle,icp_measurement_time=ref['measurement_time'])
    if distance>.015 or angle>8:
        f['method']='v3_only_icp_disagreement';return v3.copy(),f
    result=v3.copy();result[:3,3]=.7*v3[:3,3]+.3*icp[:3,3]
    result[:3,:3]=v3[:3,:3]@Rotation.from_rotvec(.5*delta.as_rotvec()).as_matrix()
    f.update(enabled=True,method='weighted_icp_v3',gate='15mm/8deg')
    return result,f
