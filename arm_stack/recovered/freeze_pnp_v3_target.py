#!/usr/bin/env python3
"""Validate and mark the latest RGB PnP V3 target as frozen for gun-plan."""
from publication_paths import vision_path, hand_eye_path
import json
import sys
import time
import uuid
from stable_pose_selection import collect
from pathlib import Path

root = Path(str(vision_path('results/realtime_runs')))
target_path = root / 'latest_camera_socket_target.json'
request_path = root / 'latest_camera_plan_request.json'
frozen_target_path = root / 'frozen_pnp_v3_socket_target.json'
frozen_request_path = root / 'frozen_pnp_v3_request.json'


def atomic_write(path, value):
    temp = path.with_name(path.name + '.tmp')
    temp.write_text(json.dumps(value, indent=2))
    temp.replace(path)

try:
    def read_target():
        request = json.loads(request_path.read_text())
        target = json.loads(target_path.read_text())
        if target.get('pose_source') != 'rgb_pnp_v3' or target.get('pose_valid') is not True:
            raise ValueError('invalid_visual:'+str(request.get('error','V3无有效测量')))
        if (request.get('source')!='rgb_pnp_v3' or request.get('status')!='complete'
                or request.get('request_id')!=target.get('request_id')
                or request.get('plan_id')!=target.get('plan_id')
                or request.get('measurement_time')!=target.get('measurement_time')):
            raise ValueError('invalid_request:V3桥接测量记录不匹配')
        identity=(target['request_id'],target['plan_id'])
        if read_target.identity is not None and identity!=read_target.identity:
            raise RuntimeError('取值期间V3桥接重启，请重新gun-plan')
        read_target.identity=identity
        return {'stamp':target.get('measurement_time'),'matrix':target.get('T_base_from_socket'),
                'target':target,'request':request}
    read_target.identity=None
    # Never leave a previous frozen target available if this attempt fails.
    frozen_target_path.unlink(missing_ok=True)
    frozen_request_path.unlink(missing_ok=True)
    chosen, stats=collect(read_target,'gun',root/'latest_gun_pose_selection.json')
    target=chosen['target'];request=chosen['request'];now=time.time()
    age=now-chosen['stamp']
    token='frozen-v3-'+uuid.uuid4().hex
    target['live_request_id']=target['request_id'];target['live_plan_id']=target['plan_id']
    request['live_request_id']=request['request_id'];request['live_plan_id']=request['plan_id']
    target['request_id']=request['request_id']=token
    target['plan_id']=request['plan_id']=token
    target['pose_selection']=request['pose_selection']=stats
    target['frozen_at'] = now
    request['frozen_at'] = now
    request['frozen_measurement_time'] = target['measurement_time']
    request['frozen_age_s'] = age
    from camera_reference import executed_camera_reference
    reference=executed_camera_reference(root)
    target['cycle_id']=request['cycle_id']=reference['cycle_id']
    target['icp_reference']=request['icp_reference']=reference
    atomic_write(frozen_target_path, target)
    atomic_write(frozen_request_path, request)
    print(f"FROZEN request={request['request_id']} age_s={age:.3f}")
except Exception as error:
    print(f'FREEZE_REFUSED: {error}', file=sys.stderr)
    sys.exit(2)
