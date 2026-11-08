#!/usr/bin/env python3
"""One independent legacy test OR home. No routing, auto-home or auto-return."""
import argparse
import fcntl
import os
from pathlib import Path
import subprocess
import sys
import time
from arm_process_guard import active_arm_tools, scan_arm_process_domains
from model import write_json


def command(action):
    if action not in ('test', 'home'):
        raise ValueError('只接受test或home')
    configured=os.environ.get('CHARGING_DOCKING_COMMAND','').strip()
    executable=Path(configured).expanduser() if configured else Path.home()/'bin/charge_robot'
    return [str(executable), action]


def child_env(environment=None):
    env = dict(os.environ if environment is None else environment)
    for name in ('CHARGE_ROBOT_DEMO_AUTO', 'CHARGE_ROBOT_HOME_RECEIPT',
                 'CHARGE_ROBOT_HOME_REQUEST_ID', 'CHARGE_ROBOT_HOME_INVOCATION_ID',
                 'CHARGE_ROBOT_EXPECT_CYCLE_ID', 'CHARGE_ROBOT_EXPECT_INSERTION_PLAN_ID'):
        env.pop(name, None)
    env.update(ROS_DOMAIN_ID='67', ROS_LOCALHOST_ONLY='1')
    return env


def run(action, state, invocation, *, run_command=subprocess.run,
        scanner=scan_arm_process_domains, active_scanner=active_arm_tools, prepare_display=None):
    args = command(action)
    def record(phase, **fields):
        write_json(state, dict(invocation=invocation, action=action, phase=phase,
                              time=time.time(), physical_stop_verified=False, **fields))
    record('checking')
    try:
        if active_scanner(exclude_pid=os.getpid()):
            raise ValueError('另一个机械臂工具仍在运行；未启动')
        scanner()  # Reject reuse of arm nodes in the base ROS domain.
        if action=='test' and prepare_display is not None:
            display=prepare_display()
            record('checking',navigation_display=display)
            print('导航显示处理：'+str(display),flush=True)
        record('running')
        result = run_command(args, env=child_env(), check=False)
        record('returned' if result.returncode == 0 else 'uncertain', returncode=result.returncode)
        return result.returncode
    except (KeyboardInterrupt, OSError, ValueError, RuntimeError) as error:
        record('uncertain', reason=str(error) or '人工中断；实体状态未确认')
        return 2


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=('test', 'home'))
    parser.add_argument('--state', type=Path, required=True)
    parser.add_argument('--invocation', required=True)
    args = parser.parse_args(argv)
    # Held for the entire terminal lifetime. No command starts on import/help.
    with open('/tmp/charging-arm-tool-'+str(os.getuid())+'.lock', 'a') as lease:
        try:
            fcntl.flock(lease, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            write_json(args.state, dict(invocation=args.invocation, action=args.action,
                                       phase='uncertain', reason='已有机械臂工具运行'))
            return 2
        print('charge_robot '+args.action+' · 原版逐段确认；不自动执行其他工具或返程。', flush=True)
        from navigation_display import close_navigation_rviz
        result = run(args.action, args.state, args.invocation,prepare_display=close_navigation_rviz)
        print('\n工具已返回；不代表机械臂已停止或收拢。底盘占用保持，请现场确认。', flush=True)
        try:
            input('按 Enter 关闭此终端：')
        except (KeyboardInterrupt, EOFError):
            pass
        return result


if __name__ == '__main__':
    sys.exit(main())
