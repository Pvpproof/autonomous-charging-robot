#!/usr/bin/env python3
"""Interactive handoff only; opening this program never runs a robot command."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import time
import uuid

COMMAND = os.environ.get('CHARGING_DOCKING_COMMAND', str(Path.home()/'bin'/'charge_robot'))


def record(path, phase, **fields):
    data = dict(phase=phase, time=time.time(), **fields)
    temp = path.with_suffix('.tmp')
    temp.write_text(json.dumps(data, ensure_ascii=False)+'\n')
    temp.replace(path)


def run_command(action, path, cycle_id):
    request=dict(cycle_id=cycle_id,invocation_id=uuid.uuid4().hex,
                 action=action,started_at=time.monotonic())
    record(path, action+'-running', **request, process_exited=False)
    print('\n原工作流仍逐段要求人工确认。Ctrl+C仅请求中断，不能代表机械臂已经停止。',flush=True)
    try:
        result = subprocess.run([COMMAND, action], check=False)
    except KeyboardInterrupt:
        record(path, 'uncertain', **request, process_exited=False, reason='人工中断，实体状态未确认')
        print('\n已中断等待；请核实机械臂与充电口状态，必要时使用实体急停。')
        return
    except OSError as error:
        record(path, 'launch-failed', **request, process_exited=False, reason=str(error))
        print('命令未能启动：',error)
        return
    phase = action+'-returned' if result.returncode == 0 else 'uncertain'
    record(path, phase, **request, finished_at=time.monotonic(),
           process_exited=True, returncode=result.returncode, home_actual_verified=False)
    if result.returncode:
        print('命令未成功，返回码 {}；不自动重试、不自动home，实际停止需现场核实。'.format(result.returncode))
    elif action == 'test':
        print('test返回成功：仅说明运动工作流完成，未验证锁止、电气连接或充电完成。')
    else:
        print('home返回成功；仍需现场确认枪头退出、机械臂收拢后再解除底盘对接锁。')


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--state',type=Path,required=True)
    parser.add_argument('--cycle-id',default=uuid.uuid4().hex)
    args=parser.parse_args()
    path=args.state
    print('充电机器人 · 人工对接终端\n底盘软件停车不控制机械臂。不要关闭导航面板；外部底盘控制源仍需现场禁用。')
    record(path,'ready',cycle_id=args.cycle_id)
    while True:
        try:
            choice=input('\n输入 test 开始逐段确认，home 退回/回位，q 关闭终端（不解除底盘锁）：').strip()
            if choice == 'q':
                record(path,'console-closed');return
            if choice == 'test':
                if input('确认车辆未移动、定位及姿态满足要求、工作区安全，输入 TEST：').strip() == 'TEST':
                    run_command('test',path,args.cycle_id)
            elif choice == 'home':
                print('home可能先拔枪。请确认充电已安全停止并断电、锁扣已解除、退回路径安全。')
                if input('上述条件均已在现场确认，输入 已断电并解锁：').strip() == '已断电并解锁':
                    run_command('home',path,args.cycle_id)
            else:
                print('没有执行命令。只接受 test、home 或 q。')
        except (KeyboardInterrupt,EOFError):
            record(path,'uncertain',reason='终端输入中断；实际设备状态需现场核实')
            print('\n终端退出，底盘对接锁保持；请核实实际设备状态。');return


if __name__=='__main__':main()
