#!/usr/bin/env python3
"""Freeze this cycle's saved ICP base pose for gun planning; no robot commands."""
import json,time,uuid
from pathlib import Path
import numpy as np
from gun_pose_validation import matrix
from stable_pose_selection import atomic_json
ROOT=Path('/opt/charging-robot-local/charging_socket_vision/results/realtime_runs')
def freeze(root=ROOT):
    selection=json.loads((root/'latest_camera_pose_selection.json').read_text())
    cycle=json.loads((root/'workflow_cycle.json').read_text())
    camera=json.loads((root/'last_dispatched_trajectory.json').read_text())
    if selection.get('status')!='accepted' or selection.get('selected_at',0)<cycle.get('updated_at',0):
        raise ValueError('本轮没有有效的cam-plan ICP记录，请先完成相机阶段')
    from charge_workflow_state import assert_idle
    assert_idle()
    # Camera approach eligibility is geometric, unlike gun endpoint validation.
    # planning_ready deliberately remains false for unverified docking accuracy.
    if (camera.get('stage')!='camera' or camera.get('schema_version')!=2
            or camera.get('pose_valid') is not True or camera.get('pose_fresh') is not True
            or camera.get('quality_level')!='full_geometry'
            or camera.get('selected_distance_m') not in (0.140,0.130)
            or camera.get('planned_at',0)<cycle.get('updated_at',0)):
        raise ValueError('本轮尚无符合140/130mm接近条件的已提交相机轨迹')
    T=matrix(selection['selected']['T_base_from_socket'])
    if not np.allclose(T,matrix(camera.get('T_base_from_socket')),atol=1e-9,rtol=0):
        raise ValueError('保存ICP与相机轨迹的目标不一致')
    token='frozen-icp-'+uuid.uuid4().hex;now=time.time()
    common=dict(request_id=token,plan_id=token,measurement_time=selection['selected']['stamp'],
                frozen_at=now,cycle_id=cycle['cycle_id'],icp_camera_plan_id=camera['plan_id'])
    target=dict(common,schema_version=1,stage='camera_socket_target',pose_source='icp_cam_plan',
                pose_valid=True,T_base_from_socket=T.tolist(),pose_selection=selection.get('statistics'))
    request=dict(common,status='complete',source='icp_cam_plan')
    atomic_json(root/'frozen_pnp_v3_socket_target.json',target)
    atomic_json(root/'frozen_pnp_v3_request.json',request)
    print('[ICP目标] 使用本轮cam-plan保存的ICP；位置和方向均不融合V3。',flush=True)
    return target,request
if __name__=='__main__':
    try:freeze()
    except Exception as e:raise SystemExit('ICP_FREEZE_REFUSED: '+str(e))
