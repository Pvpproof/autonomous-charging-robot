#!/usr/bin/env python3
"""Explicit CLI for one test -> verified home sequence. Importing does nothing.

This runner never starts chassis return. It reports evidence to the caller,
which retains control of the chassis lock and raw-odometry session.
"""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import shlex
import signal
import subprocess
import sys
import time
import uuid

RESULTS_ROOT = '/home/nvidia/charging_socket_vision/results/realtime_runs'
HOME_POSE = '/home/nvidia/nova2_tools/initial_arm_pose.yaml'


def atomic(path, data):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + '.' + uuid.uuid4().hex + '.tmp')
    temporary.write_text(json.dumps(data, ensure_ascii=False, indent=2, allow_nan=False))
    temporary.replace(path)


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def finite(value):
    return type(value) in (int, float) and math.isfinite(value)


def read_object(path, optional=False):
    path = Path(path)
    if optional and not path.exists():
        return {}
    value = json.loads(path.read_text())
    require(isinstance(value, dict), '状态文件不是对象: ' + path.name)
    return value


class SequenceCancelled(RuntimeError):
    pass


class SequenceRunner:
    def __init__(self, sequence_id, invocation_id, receipt, mode='manual', cancel_file=None,
                 command=None, results_root=None, home_pose=None, popen=subprocess.Popen,
                 clock=time.monotonic, wall=time.time, sleep=time.sleep, cancel_grace=2.0):
        require(bool(sequence_id) and bool(invocation_id), '必须提供本次sequence/invocation ID')
        require(mode in ('automatic', 'manual'), '无效运行模式')
        self.sequence_id, self.invocation_id = sequence_id, invocation_id
        self.path, self.mode = Path(receipt), mode
        self.cancel_file = (Path(cancel_file) if cancel_file else
            self.path.with_name(self.path.stem + '.cancel-' + uuid.uuid4().hex + '.json'))
        self.command = shlex.split(command or os.environ.get('CHARGE_ROBOT_SEQUENCE_COMMAND') or str(Path.home() / 'bin/charge_robot'))
        require(bool(self.command), '机械臂命令为空')
        self.command[0] = os.path.expanduser(self.command[0])
        self.root = Path(results_root or os.environ.get('CHARGE_ROBOT_SEQUENCE_RESULTS_ROOT', RESULTS_ROOT))
        self.home_pose = Path(home_pose or os.environ.get('CHARGE_ROBOT_HOME_POSE', HOME_POSE))
        self.popen, self.clock, self.wall, self.sleep = popen, clock, wall, sleep
        self.cancel_grace = cancel_grace
        self.cancel_reason = None
        self.data = dict(schema_version=1, sequence_id=sequence_id, invocation_id=invocation_id,
            mode=mode, started_at=clock(), started_wall_at=wall(), status='running',
            test_success=False, home_success=False, home_verified=False, stowed=False,
            retracted=False, stow_evidence=None, exit_code=None, stopped_confirmed=False,
            cancel_file=str(self.cancel_file))

    def cancelled(self):
        return bool(self.cancel_reason or (self.cancel_file and self.cancel_file.exists()))

    def check_cancel(self):
        if self.cancelled():
            raise SequenceCancelled(self.cancel_reason or '收到取消文件；未确认实际停稳')

    def persist(self):
        atomic(self.path, self.data)

    def child_env(self, action, call_id):
        env = dict(os.environ)
        for key in ('CHARGE_ROBOT_DEMO_AUTO', 'CHARGE_ROBOT_HOME_RECEIPT',
                    'CHARGE_ROBOT_HOME_REQUEST_ID', 'CHARGE_ROBOT_HOME_INVOCATION_ID',
                    'CHARGE_ROBOT_EXPECT_CYCLE_ID', 'CHARGE_ROBOT_EXPECT_INSERTION_PLAN_ID',
                    'CHARGE_ROBOT_SEQUENCE_CANCEL_FILE'):
            env.pop(key, None)
        if self.mode == 'automatic':
            env['CHARGE_ROBOT_DEMO_AUTO'] = '1'
        if self.cancel_file:
            env['CHARGE_ROBOT_SEQUENCE_CANCEL_FILE'] = str(self.cancel_file)
        env['CHARGE_ROBOT_SEQUENCE_ID'] = self.sequence_id
        env['CHARGE_ROBOT_CALL_ID'] = call_id
        if action == 'home':
            env.update(CHARGE_ROBOT_HOME_RECEIPT=str(self.home_receipt),
                       CHARGE_ROBOT_HOME_REQUEST_ID=call_id,
                       CHARGE_ROBOT_HOME_INVOCATION_ID=call_id,
                       CHARGE_ROBOT_EXPECT_CYCLE_ID=self.test_evidence['cycle_id'],
                       CHARGE_ROBOT_EXPECT_INSERTION_PLAN_ID=self.test_evidence['insertion_plan_id'])
        return env

    def write_cancel_marker(self):
        # Create without replacing an earlier UI/operator marker. The child
        # checks existence, so its cooperative gate precedes the shell signal.
        try:
            self.cancel_file.parent.mkdir(parents=True, exist_ok=True)
            try:
                with self.cancel_file.open('x', encoding='utf-8') as stream:
                    json.dump(dict(sequence_id=self.sequence_id, invocation_id=self.invocation_id,
                                   reason=self.cancel_reason or 'runner interrupt requested',
                                   requested_at=self.clock()), stream, ensure_ascii=False, allow_nan=False)
            except FileExistsError:
                if not self.cancel_file.exists():
                    raise
            self.data['cooperative_cancel_available'] = True
        except OSError as error:
            self.data.update(cooperative_cancel_available=False,
                             cancel_file_write_error=str(error))

    def interrupt_child(self, child):
        # Interrupting a process is not an arm stop acknowledgement. Never kill
        # a controller process or silently promote process exit to physical stop.
        self.data.update(status='uncertain', cancelled=True, stopped_confirmed=False,
                         child_pid=getattr(child, 'pid', None))
        self.write_cancel_marker()
        if child.poll() is None:
            try:
                child.send_signal(signal.SIGINT)
                self.data['interrupt_requested'] = True
            except OSError as error:
                self.data['interrupt_error'] = str(error)
            deadline = self.clock() + self.cancel_grace
            while child.poll() is None and self.clock() < deadline:
                self.sleep(.05)
        self.data['child_may_still_be_running'] = child.poll() is None
        self.data['child_exit_code'] = child.poll()
        raise SequenceCancelled(self.cancel_reason or '已请求中断，实际停车未确认')

    def call(self, action):
        self.check_cancel()
        call_id = uuid.uuid4().hex
        self.data[action + '_invocation_id'] = call_id
        self.data[action + '_started_at'] = self.clock()
        self.data[action + '_started_wall_at'] = self.wall()
        if action == 'home':
            self.home_receipt = self.path.with_name(self.path.stem + '.home-' + call_id + '.json')
        self.persist()
        child = self.popen(self.command + [action], env=self.child_env(action, call_id))
        while True:
            if self.cancelled():
                self.interrupt_child(child)
            try:
                code = child.poll()
                if code is not None:
                    break
                self.sleep(.05)
            except KeyboardInterrupt:
                self.cancel_reason = '操作者中断等待；实际停车未确认'
                self.interrupt_child(child)
        self.data[action + '_finished_at'] = self.clock()
        self.data[action + '_finished_wall_at'] = self.wall()
        self.data[action + '_exit_code'] = code
        self.check_cancel()
        require(code == 0, action + '返回失败；不自动重试或继续下一动作')
        return call_id

    def validate_test(self, previous_cycle, previous_plan):
        cycle = read_object(self.root / 'workflow_cycle.json')
        inserted = read_object(self.root / 'last_inserted_trajectory.json')
        report = read_object(self.root / 'latest_insertion_execution_feedback.json')
        gun = read_object(self.root / 'last_dispatched_trajectory.json')
        gun_feedback = read_object(self.root / 'latest_gun_execution_feedback.json')
        request = read_object(self.root / 'frozen_pnp_v3_request.json')
        target = read_object(self.root / 'frozen_pnp_v3_socket_target.json')
        cycle_id, plan_id = cycle.get('cycle_id'), inserted.get('plan_id')
        require(cycle.get('status') == 'completed' and bool(cycle_id) and cycle_id != previous_cycle,
                'test没有产生本轮新的completed工作周期')
        require(inserted.get('stage') == 'insertion' and inserted.get('planning_ready') is True
                and bool(plan_id) and plan_id != previous_plan and inserted.get('cycle_id') == cycle_id,
                '插入轨迹不是本次周期的新轨迹')
        require(report.get('stage') == 'insertion' and report.get('executor') == 'cartesian_once'
                and report.get('status') == 'succeeded' and report.get('action_status') == 4
                and report.get('error_code') == 0 and report.get('motion_commands_sent') is True
                and report.get('plan_id') == plan_id and report.get('retracted', False) is False,
                '本次插入结果未明确成功或记录已被替换/退回')
        require(finite(report.get('finished_at'))
                and self.data['test_started_wall_at'] <= report['finished_at'] <= self.data['test_finished_wall_at'],
                '插入结果不属于本次test调用时间窗')
        require(finite(report.get('endpoint_max_error_deg')) and 0 <= report['endpoint_max_error_deg'] <= .1,
                '插入实际终点反馈不合格')
        actual = report.get('final_joint_rad')
        require(isinstance(actual, list) and len(actual) == 6 and all(finite(x) for x in actual),
                '插入实际关节反馈无效')
        require(gun.get('stage') == 'gun' and gun.get('cycle_id') == cycle_id
                and bool(gun.get('plan_id')) and inserted.get('source_gun_plan_id') == gun['plan_id'],
                '插入来源枪头轨迹/周期不匹配')
        require(gun_feedback.get('stage') == 'gun' and gun_feedback.get('status') == 'verified'
                and gun_feedback.get('passed') is True and gun_feedback.get('plan_id') == gun['plan_id'],
                '枪头预备位实际反馈未通过')
        require(request.get('status') == 'complete' and request.get('source') == 'rgb_pnp_v3'
                and target.get('pose_source') == 'rgb_pnp_v3' and target.get('pose_valid') is True
                and target.get('cycle_id') == cycle_id and bool(request.get('request_id'))
                and bool(request.get('plan_id')), '本轮视觉目标无效')
        require(all(d.get('camera_request_id') == request['request_id']
                    and d.get('camera_plan_id') == request['plan_id'] for d in (gun, inserted))
                and target.get('request_id') == request['request_id']
                and target.get('plan_id') == request['plan_id'], '视觉请求链不一致')
        return dict(cycle_id=cycle_id, insertion_plan_id=plan_id, gun_plan_id=gun['plan_id'],
                    inserted_sha256=hashlib.sha256((self.root / 'last_inserted_trajectory.json').read_bytes()).hexdigest(),
                    insertion_finished_wall_at=report['finished_at'])

    def validate_home(self, call_id):
        home = read_object(self.home_receipt)
        cycle = read_object(self.root / 'workflow_cycle.json')
        report = read_object(self.root / 'latest_insertion_execution_feedback.json')
        require(home.get('sequence_id') == self.sequence_id and home.get('invocation_id') == call_id
                and home.get('request_id') == call_id, 'home回执不是本次调用')
        require(home.get('exit_code') == 0 and home.get('status') == 'verified'
                and home.get('verified') is True and home.get('home_verified') is True
                and home.get('evidence_type') == 'service_getangle_robotmode'
                and home.get('robot_mode') == 5 and home.get('consecutive_samples') == 3,
                'home未完成实际服务反馈验收')
        require(finite(home.get('started_at')) and finite(home.get('finished_at'))
                and self.data['home_started_at'] <= home['started_at'] <= home['finished_at'] <= self.clock()
                and self.clock() - home['finished_at'] <= 5, 'home回执过期或调用时间不符')
        samples = home.get('samples')
        require(isinstance(samples, list) and len(samples) == 3, '缺少home连续3次反馈')
        previous = home['started_at']
        for sample in samples:
            require(sample.get('robot_mode') == 5 and finite(sample.get('max_error_deg'))
                    and 0 <= sample['max_error_deg'] <= .5 and finite(sample.get('measured_at'))
                    and previous < sample['measured_at'] <= home['finished_at'], 'home反馈不是本次连续到位服务反馈')
            previous = sample['measured_at']
        require(hashlib.sha256(self.home_pose.read_bytes()).hexdigest() == home.get('target_pose_sha256'),
                'home目标姿态不匹配或已变更')
        require(cycle.get('cycle_id') == self.test_evidence['cycle_id'] and cycle.get('status') == 'home_ready',
                'home_ready不属于本次周期')
        require(hashlib.sha256((self.root / 'last_inserted_trajectory.json').read_bytes()).hexdigest()
                == self.test_evidence['inserted_sha256'], 'home期间原插入轨迹已被替换')
        require(report.get('plan_id') == self.test_evidence['insertion_plan_id']
                and report.get('stage') == 'insertion' and report.get('executor') == 'cartesian_once'
                and report.get('action_status') == 4 and report.get('error_code') == 0
                and report.get('motion_commands_sent') is True
                and report.get('finished_at') == self.test_evidence['insertion_finished_wall_at']
                and report.get('status') == 'succeeded' and report.get('retracted') is True
                and report.get('retraction_status') == 'succeeded'
                and report.get('retraction_method') == 'position_cartesian'
                and finite(report.get('retracted_at'))
                and self.data['home_started_wall_at'] <= report['retracted_at'] <= self.data['home_finished_wall_at'],
                '本次插入尚未明确沿原插入路径退回；禁止自动HOME覆盖')
        return dict(evidence_type='service_getangle_robotmode', source_session_verified=False,
                    home_invocation_id=call_id, request_id=home['request_id'],
                    measured_at=samples[-1]['measured_at'], samples=3, robot_mode=5,
                    max_error_deg=max(x['max_error_deg'] for x in samples),
                    target_pose_sha256=home['target_pose_sha256'],
                    cycle_id=cycle['cycle_id'], insertion_plan_id=report['plan_id'],
                    retraction_method=report['retraction_method'], retracted_wall_at=report['retracted_at'])

    def run(self):
        self.persist()
        try:
            self.check_cancel()
            previous_cycle = read_object(self.root / 'workflow_cycle.json', True).get('cycle_id')
            previous_plan = read_object(self.root / 'last_inserted_trajectory.json', True).get('plan_id')
            previous_report = read_object(self.root / 'latest_insertion_execution_feedback.json', True)
            prior_may_be_inserted = (previous_report.get('motion_commands_sent') is True
                or previous_report.get('executor') in ('contact_steps', 'cartesian_once'))
            rejected_without_motion = (previous_report.get('status') == 'rejected'
                and previous_report.get('motion_commands_sent') is False)
            require(not (prior_may_be_inserted and not rejected_without_motion
                         and previous_report.get('retracted') is not True),
                    '存在尚未确认退回的旧插入状态；禁止自动开始新test')
            self.call('test')
            self.test_evidence = self.validate_test(previous_cycle, previous_plan)
            self.data.update(test_success=True, test_evidence=self.test_evidence)
            self.persist()
            self.check_cancel()
            # Re-read immediately before home; the shell auto-home guard also
            # checks these exact IDs under its existing command lock.
            require(read_object(self.root / 'workflow_cycle.json').get('cycle_id') == self.test_evidence['cycle_id']
                    and hashlib.sha256((self.root / 'last_inserted_trajectory.json').read_bytes()).hexdigest()
                    == self.test_evidence['inserted_sha256'], 'test之后流程已被其他调用替换')
            home_id = self.call('home')
            self.data['home_success'] = True
            self.check_cancel()
            evidence = self.validate_home(home_id)
            self.check_cancel()
            self.data.update(status='verified', home_verified=True, stowed=True,
                             retracted=True, stow_evidence=evidence, exit_code=0)
        except (Exception, KeyboardInterrupt) as error:
            self.data.update(status='uncertain', error=str(error) or type(error).__name__,
                             home_verified=False, stowed=False, retracted=False,
                             stopped_confirmed=False, exit_code=2)
        self.data.update(finished_at=self.clock(), finished_wall_at=self.wall())
        # A cancellation arriving just before publication must defeat success.
        if self.cancelled():
            self.data.update(status='uncertain', cancelled=True, exit_code=2,
                             home_verified=False, stowed=False, retracted=False,
                             stopped_confirmed=False)
        self.persist()
        return self.data['exit_code']


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mode', choices=('automatic', 'manual'), default='manual')
    parser.add_argument('--sequence-id', required=True)
    parser.add_argument('--invocation-id', required=True)
    parser.add_argument('--receipt', type=Path, required=True)
    parser.add_argument('--cancel-file', type=Path)
    args = parser.parse_args(argv)
    runner = SequenceRunner(args.sequence_id, args.invocation_id, args.receipt,
                            mode=args.mode, cancel_file=args.cancel_file)
    old = {}
    def cancel(signum, frame):
        runner.cancel_reason = '收到中断信号%d；实际停车未确认' % signum
    try:
        for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
            old[sig] = signal.signal(sig, cancel)
        return runner.run()
    finally:
        for sig, handler in old.items():
            signal.signal(sig, handler)


if __name__ == '__main__':
    sys.exit(main())
