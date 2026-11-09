#!/usr/bin/env python3
"""Check the current cycle's frozen target and verified gun execution."""
import json
import sys
from pathlib import Path

ROOT = Path('/home/nvidia/charging_socket_vision/results/realtime_runs')


def prepare(root=ROOT):
    def read(name):
        try:
            value = json.loads((root / name).read_text())
        except (OSError, ValueError) as error:
            raise RuntimeError(f'本轮状态文件 {name} 不可读，请重新完成 gun-plan 和 execute') from error
        if not isinstance(value, dict):
            raise RuntimeError(f'本轮状态文件 {name} 格式无效')
        return value

    cycle = read('workflow_cycle.json')
    request = read('frozen_pnp_v3_request.json')
    target = read('frozen_pnp_v3_socket_target.json')
    selection = read('latest_camera_pose_selection.json')
    trajectory = read('last_dispatched_trajectory.json')
    feedback = read('latest_gun_execution_feedback.json')
    if cycle.get('status') != 'completed':
        raise RuntimeError('本轮枪头阶段尚未完成，请先完成 gun-plan 和 execute')
    if (trajectory.get('stage') != 'gun' or trajectory.get('planning_ready') is not True
            or trajectory.get('insertion_depth_mm', 0) != 0 or not trajectory.get('plan_id')):
        raise RuntimeError('本轮没有有效的枪头预备位轨迹')
    if (feedback.get('stage') != 'gun' or feedback.get('status') != 'verified'
            or feedback.get('passed') is not True or feedback.get('plan_id') != trajectory['plan_id']):
        raise RuntimeError('枪头轨迹尚未实际执行并通过到位检查')
    if (request.get('status') != 'complete' or request.get('source') != 'rgb_pnp_v3'
            or target.get('pose_source') != 'rgb_pnp_v3' or target.get('pose_valid') is not True
            or not request.get('request_id') or not request.get('plan_id')
            or target.get('request_id') != request['request_id']
            or target.get('plan_id') != request['plan_id']
            or trajectory.get('camera_request_id') != request['request_id']
            or trajectory.get('camera_plan_id') != request['plan_id']):
        raise RuntimeError('本轮冻结视觉目标与已执行的枪头轨迹不匹配')
    if selection.get('stage') != 'camera' or selection.get('status') != 'accepted':
        raise RuntimeError('本轮 ICP 测量记录不可用')
    print('INSERTION_SOURCE_CURRENT', trajectory['plan_id'])


if __name__ == '__main__':
    try:
        prepare()
    except Exception as error:
        print('INSERTION_SOURCE_REFUSED:', error, file=sys.stderr)
        sys.exit(2)
