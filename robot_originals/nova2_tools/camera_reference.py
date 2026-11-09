"""Validate frozen ICP provenance against an actually executed camera approach."""
import json,math
from pathlib import Path
import numpy as np
from gun_pose_validation import matrix

def executed_camera_reference(root):
    root=Path(root)
    def read(name):
        try:
            value=json.loads((root/name).read_text())
            if not isinstance(value,dict):raise ValueError('not object')
            return value
        except (OSError,ValueError) as error:
            raise RuntimeError('相机绑定记录不可读 '+name+'；请检查本轮cam-plan执行结果') from error
    cycle=read('workflow_cycle.json');icp=read('latest_camera_pose_selection.json')
    executed=read('last_dispatched_trajectory.json');feedback=read('latest_camera_execution_feedback.json')
    if not cycle.get('cycle_id') or icp.get('cycle_id')!=cycle['cycle_id']:
        raise RuntimeError('ICP周期与当前流程不匹配；请重新cam-plan/execute')
    if icp.get('status')!='accepted' or icp.get('stage')!='camera':
        raise RuntimeError('本轮ICP稳定测量未通过')
    if (executed.get('stage')!='camera' or not icp.get('camera_plan_id') or not icp.get('camera_request_id')
            or executed.get('plan_id')!=icp['camera_plan_id']
            or executed.get('request_id')!=icp['camera_request_id']):
        raise RuntimeError('ICP与已提交cam-plan的轨迹/请求ID不匹配')
    # Match plan_flow_guard's CAMERA eligibility. planning_ready is the ICP's
    # independent docking flag; it is deliberately not a camera-approach gate.
    if (executed.get('schema_version')!=2 or executed.get('pose_valid') is not True
            or executed.get('pose_fresh') is not True or executed.get('quality_level')!='full_geometry'
            or executed.get('selected_distance_m') not in (.140,.130)):
        raise RuntimeError('已执行相机轨迹不符合完整几何140/130mm接近条件')
    if (feedback.get('stage')!='camera' or feedback.get('plan_id')!=executed['plan_id']
            or feedback.get('passed') is not True or feedback.get('status')!='verified'):
        raise RuntimeError('本轮cam-plan尚未实际执行并通过到位验收')
    selected=icp.get('selected',{})
    try:
        T=matrix(selected['T_base_from_socket']);committed=matrix(executed['T_base_from_socket'])
        stamp=selected['stamp'];planned=executed['planned_at'];checked=feedback['checked_at']
        if any(isinstance(x,bool) or not isinstance(x,(int,float)) or not math.isfinite(x) for x in (stamp,planned,checked)):
            raise ValueError('时间戳格式无效')
        if not np.allclose(T,committed,atol=1e-9,rtol=0):raise ValueError('ICP矩阵与执行轨迹不一致')
        if stamp!=executed.get('measurement_time'):raise ValueError('ICP测量标识与执行轨迹不一致')
        if checked<planned:raise ValueError('到位验收早于轨迹规划')
    except (KeyError,ValueError,TypeError) as error:
        raise RuntimeError('相机绑定几何/验收记录不一致：'+str(error)) from error
    return dict(cycle_id=cycle['cycle_id'],camera_plan_id=icp['camera_plan_id'],
        camera_request_id=icp['camera_request_id'],execution_verified=True,
        T_base_from_socket=T.tolist(),measurement_time=stamp)
