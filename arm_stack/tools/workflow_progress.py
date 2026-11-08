"""Reconstructed file-only planning progress, not the missing field original.

Observed interfaces: begin camera|gun, fail REASON, wait camera|gun, missing().
No ROS, subprocess, hardware connection, trajectory execution or automatic resume.
"""
import argparse
import importlib
import json
import math
import os
from pathlib import Path
import time
import uuid
from insertion_runtime import atomic

RECONSTRUCTED = True
STAGES = ('camera', 'gun')

def root_path():
    value = os.environ.get('CHARGING_WORKFLOW_ROOT', '')
    path = Path(value)
    if not value or not path.is_absolute() or not path.is_dir():
        raise RuntimeError('Set CHARGING_WORKFLOW_ROOT to an existing reviewed absolute runtime directory')
    return path

def read(path):
    value = json.loads(Path(path).read_text())
    if not isinstance(value, dict):
        raise ValueError('Expected JSON object: ' + Path(path).name)
    return value

def cycle(root):
    value = read(root / 'workflow_cycle.json').get('cycle_id')
    if not isinstance(value, str) or not value:
        raise RuntimeError('Current workflow cycle_id is missing; no cycle is invented')
    return value

def begin(stage, root=None, now=None):
    if stage not in STAGES:
        raise ValueError('Unknown planning stage')
    root = root_path() if root is None else Path(root)
    record = dict(schema_version=1, implementation='reconstructed-publication-v1',
                  attempt_id=uuid.uuid4().hex, stage=stage, cycle_id=cycle(root),
                  started_at=time.time() if now is None else now, status='pending')
    atomic(root / 'latest_planning_attempt.json', record)
    return record

def fail(reason, root=None):
    root = root_path() if root is None else Path(root)
    record = read(root / 'latest_planning_attempt.json')
    record.update(status='failed', reason=str(reason))
    atomic(root / 'latest_planning_attempt.json', record)
    return record

def missing(root=None):
    root = root_path() if root is None else Path(root)
    try:
        record = read(root / 'latest_planning_attempt.json')
    except (OSError, ValueError):
        return 'No readable planning attempt; regenerate the plan (reconstructed status helper)'
    return 'Planning attempt ' + str(record.get('status', 'unknown')) + ': ' + str(record.get('reason', 'no eligible trajectory yet'))

def default_validator(data, root):
    # Reuse the recovered eligibility checker. Never replace it with file-exists success.
    guard = importlib.import_module('plan_flow_guard')
    if Path(guard.ROOT).resolve() != root.resolve():
        raise RuntimeError('Planning validator and progress helper use different runtime roots')
    return guard.validate(data)

def inspect_attempt(stage, root, attempt_id, validator=None, now=None):
    root = Path(root)
    state = read(root / 'latest_planning_attempt.json')
    if state.get('attempt_id') != attempt_id or state.get('stage') != stage:
        raise RuntimeError('Planning attempt superseded or stage mismatch')
    if state.get('cycle_id') != cycle(root):
        raise RuntimeError('Workflow cycle changed')
    if state.get('status') != 'pending':
        raise RuntimeError('Attempt is not pending: ' + str(state.get('reason', state.get('status'))))
    plan_path = root / 'pending_trajectory.json'
    if not plan_path.exists():
        return None
    data = read(plan_path)
    stamp = data.get('planned_at')
    if isinstance(stamp, bool) or not isinstance(stamp, (int, float)) or not math.isfinite(stamp):
        raise RuntimeError('Invalid plan timestamp')
    if stamp < state['started_at']:
        return None  # Old output cannot complete a new attempt.
    current_time = time.time() if now is None else now
    if stamp > current_time or data.get('stage') != stage or data.get('cycle_id') != state['cycle_id']:
        raise RuntimeError('Plan time, stage or cycle mismatch')
    if not isinstance(data.get('plan_id'), str) or not data['plan_id']:
        raise RuntimeError('Plan identity missing')
    (default_validator if validator is None else validator)(data, root)
    # Detect replaced attempts/plans during validation; no completion is persisted.
    if read(root / 'latest_planning_attempt.json') != state or read(plan_path) != data or cycle(root) != state['cycle_id']:
        raise RuntimeError('Planning data changed during validation')
    return data

def wait(stage, root=None, timeout=60., validator=None, clock=time.monotonic, sleep=time.sleep):
    if stage not in STAGES or not math.isfinite(timeout) or not 0 < timeout <= 600:
        raise ValueError('Invalid stage or timeout (0 < seconds <= 600)')
    root = root_path() if root is None else Path(root)
    attempt = read(root / 'latest_planning_attempt.json')['attempt_id']
    deadline = clock() + timeout
    while clock() < deadline:
        plan = inspect_attempt(stage, root, attempt, validator=validator)
        if plan is not None:
            return plan
        sleep(min(.1, max(0., deadline-clock())))
    raise RuntimeError('Planning wait timed out; no execution authorized')

def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    sub=parser.add_subparsers(dest='command',required=True)
    sub.add_parser('begin').add_argument('stage',choices=STAGES)
    sub.add_parser('fail').add_argument('reason')
    waiter=sub.add_parser('wait');waiter.add_argument('stage',choices=STAGES);waiter.add_argument('--timeout',type=float,default=60.)
    sub.add_parser('missing')
    args=parser.parse_args(argv)
    try:
        if args.command=='begin':print(begin(args.stage)['attempt_id'])
        elif args.command=='fail':fail(args.reason)
        elif args.command=='missing':print(missing());return 2
        else:
            plan=wait(args.stage,timeout=args.timeout)
            print('Reconstructed status: planning validation passed, plan_id='+plan['plan_id']+'; no motion sent')
        return 0
    except (Exception,KeyboardInterrupt) as error:
        print('WORKFLOW_PROGRESS_REFUSED:',error)
        return 2

if __name__=='__main__':raise SystemExit(main())
