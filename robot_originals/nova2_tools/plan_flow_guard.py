"""File-only planning status checks; never imports ROS or sends commands."""
import json
import math
import sys
import time
import uuid
from pathlib import Path
from gun_pose_validation import validate_saved

ROOT = Path('/home/nvidia/charging_socket_vision/results/realtime_runs')
REQUEST = ROOT / 'latest_camera_plan_request.json'


def atomic_json(path, data):
    temporary = path.with_name(path.name + '.' + uuid.uuid4().hex + '.tmp')
    temporary.write_text(json.dumps(data, indent=2))
    temporary.replace(path)


def begin():
    for name in ('pending_trajectory.json', 'pending_camera_trajectory.json', 'pending_gun_trajectory.json'):
        (ROOT / name).unlink(missing_ok=True)
    token = 'camera-request-' + uuid.uuid4().hex
    (ROOT / 'frozen_pnp_v3_request.json').unlink(missing_ok=True)
    (ROOT / 'frozen_pnp_v3_socket_target.json').unlink(missing_ok=True)
    atomic_json(REQUEST, {'request_id': token, 'requested_at': time.time(),
                          'status': 'planning', 'stage': 'camera'})
    return token


def finish(token, status, **details):
    current = json.loads(REQUEST.read_text())
    if current['request_id'] != token:
        raise RuntimeError('A newer cam-plan request superseded this planner')
    current.update(status=status, updated_at=time.time(), **details)
    atomic_json(REQUEST, current)


def validate(data, now=None):
    now = time.time() if now is None else now
    stage = data.get('stage')
    if stage not in ('camera', 'gun'):
        raise RuntimeError('Unknown trajectory stage; regenerate the plan')
    if not REQUEST.exists():
        raise RuntimeError('Latest camera request status missing; run cam-plan again')
    if REQUEST.exists():
        request = json.loads(REQUEST.read_text())
        if stage == 'camera':
            if request.get('status') != 'complete':
                raise RuntimeError('Latest cam-plan is not complete: ' + str(request.get('status'))
                                   + '; old trajectory refused; ' + str(request.get('error', '')))
            if data.get('request_id') != request.get('request_id') or data.get('plan_id') != request.get('plan_id'):
                raise RuntimeError('Trajectory does not belong to the latest completed cam-plan')
            # Current ICP is geometric-only: allow camera approach, never docking.
            if (data.get('schema_version') != 2 or data.get('pose_valid') is not True
                    or data.get('pose_fresh') is not True or data.get('quality_level') != 'full_geometry'
                    or data.get('selected_distance_m') not in (0.140, 0.130)):
                raise RuntimeError('Camera approach requires fresh full geometry at 140/130mm')
        elif stage == 'gun':
            if data.get('pose_source') in ('rgb_pnp_v3', 'fused_icp_v3'):
                frozen_path = ROOT / 'frozen_pnp_v3_request.json'
                if not frozen_path.exists():
                    raise RuntimeError('Frozen RGB PnP V3 request is missing')
                request = json.loads(frozen_path.read_text())
                if request.get('status') != 'complete' or request.get('source') != 'rgb_pnp_v3':
                    raise RuntimeError('Frozen RGB PnP V3 measurement is invalid')
                if (data.get('camera_request_id') != request.get('request_id')
                        or data.get('camera_plan_id') != request.get('plan_id')):
                    raise RuntimeError('Gun trajectory does not match the frozen RGB PnP V3 measurement')
            else:
                if request.get('status') != 'complete':
                    raise RuntimeError('Latest camera measurement is not valid; gun trajectory refused')
                if (data.get('camera_request_id') != request.get('request_id')
                        or data.get('camera_plan_id') != request.get('plan_id')):
                    raise RuntimeError('Gun trajectory does not belong to the latest camera measurement')
    if stage == 'camera' and not data.get('request_id'):
        raise RuntimeError('Legacy camera trajectory has no request ID; run cam-plan again')
    stamp = data.get('planned_at')
    if isinstance(stamp, bool) or not isinstance(stamp, (int, float)) or not math.isfinite(stamp):
        raise RuntimeError('Plan timestamp missing or invalid; replan')
    if not data.get('points'):
        raise RuntimeError('Empty trajectory refused')
    if stage == 'gun' and data.get('planning_ready') is not True:
        raise RuntimeError('Gun trajectory is not marked as a successful plan')
    names = data.get('joint_names')
    if not isinstance(names, list) or len(names) != 6 or set(names) != {f'joint{i}' for i in range(1, 7)}:
        raise RuntimeError('Trajectory joint names invalid')
    previous = -1.0
    for point in data['points']:
        for key in ('positions', 'velocities', 'accelerations'):
            values = point.get(key, [])
            if (not isinstance(values, list) or (len(values) != 6 if key == 'positions' else len(values) not in (0, 6))
                    or any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in values)):
                raise RuntimeError('Malformed trajectory: ' + key)
        stamp = point.get('time_from_start', {})
        sec, nano = stamp.get('sec'), stamp.get('nanosec')
        if type(sec) is not int or type(nano) is not int or sec < 0 or not 0 <= nano < 1000000000:
            raise RuntimeError('Invalid trajectory time')
        current = sec + nano / 1e9
        if current <= previous:
            raise RuntimeError('Trajectory time must increase')
        previous = current
    if previous <= 0:
        raise RuntimeError('Trajectory duration must be positive')
    if stage=='gun':
        target=json.loads((ROOT/'frozen_pnp_v3_socket_target.json').read_text())
        if target.get('request_id')!=data.get('camera_request_id') or target.get('plan_id')!=data.get('camera_plan_id'):
            raise RuntimeError('枪头末端验收与冻结目标ID不匹配')
        if not data.get('cycle_id') or data['cycle_id']!=target.get('cycle_id'):raise RuntimeError('枪头目标周期不匹配，请重新规划')
        if data.get('fusion',{}).get('enabled') and data['fusion'].get('icp_reference')!=target.get('icp_reference'):raise RuntimeError('ICP绑定不匹配')
        validate_saved(data,target.get('T_base_from_socket'))
    return data


if __name__ == '__main__':
    try:
        if sys.argv[1:] == ['begin']:
            print(begin())
        elif sys.argv[1:] == ['status']:
            if not (ROOT / 'pending_trajectory.json').exists():
                from workflow_progress import missing
                raise RuntimeError(missing())
            data = json.loads((ROOT / 'pending_trajectory.json').read_text())
            validate(data)
            print('Plan complete; execution eligibility check passed (no motion sent)')
        else:
            raise RuntimeError('Expected begin or status')
    except Exception as error:
        print('PLAN_STATUS_REFUSED:', error, file=sys.stderr)
        sys.exit(2)


def validate_start_positions(data, actual_radians, tolerance_rad=math.radians(0.5)):
    expected = data['points'][0]['positions']
    if len(actual_radians) != len(expected) or any(not math.isfinite(x) for x in actual_radians):
        raise RuntimeError('Invalid actual robot joints')
    errors = [abs(a-b) for a,b in zip(actual_radians, expected)]
    if max(errors) > tolerance_rad:
        raise RuntimeError(f'机械臂已偏离轨迹起点 {math.degrees(max(errors)):.3f}度（允许0.5度），请重新规划')
    return math.degrees(max(errors))
