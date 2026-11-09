#!/usr/bin/env python3
"""File-only workflow progress. No robot or camera commands."""
import json,sys,time
from pathlib import Path
from stable_pose_selection import atomic_json
from plan_flow_guard import validate
ROOT=Path('/home/nvidia/charging_socket_vision/results/realtime_runs')
ATTEMPT=ROOT/'latest_planning_attempt.json'
PLANNER_STARTUP_GRACE_S=30.0
def read(path):
    try:return json.loads(path.read_text())
    except (OSError,ValueError):return {}
def begin(stage):
    if stage=='gun':(ROOT/'latest_gun_prepose_plan_only.json').unlink(missing_ok=True)
    atomic_json(ATTEMPT,dict(stage=stage,status='planning',started_at=time.time()))
def finish(status,error=None):
    d=read(ATTEMPT);d.update(status=status,updated_at=time.time())
    if error:d['error']=error
    atomic_json(ATTEMPT,d)
def missing():
    a=read(ATTEMPT);cycle=read(ROOT/'workflow_cycle.json')
    # A fresh cycle invalidates previous attempt messages.
    if a.get('started_at',0)>=cycle.get('updated_at',0):
        if a.get('status')=='planning':return '规划仍在进行，请等待规划命令返回成功。'
        if a.get('status')=='failed':return '最近规划失败：'+str(a.get('error','查看规划窗口'))+'；修正后重新规划。'
    d=read(ROOT/'last_dispatched_trajectory.json')
    if d:return '上一条轨迹已提交执行，不能重复执行；请查看执行结果并规划下一阶段。'
    return '没有待执行轨迹；请先成功完成 cam-plan 或 gun-plan。'
def wait(stage,timeout=100):
    deadline=time.monotonic()+timeout;started=read(ATTEMPT).get('started_at',time.time())
    while time.monotonic()<deadline:
        p=read(ROOT/'pending_trajectory.json')
        request=read(ROOT/'latest_camera_plan_request.json')
        camera_complete=(stage!='camera' or (request.get('status')=='complete'
                         and request.get('plan_id')==p.get('plan_id')))
        if p.get('stage')==stage and p.get('planned_at',0)>=started and camera_complete:
            validate(p);finish('ready')
            print('[规划成功] 轨迹已保存并通过执行资格检查。下一步：charge_robot execute',flush=True)
            return
        if stage=='camera':
            d=read(ROOT/'latest_camera_plan_request.json')
            if d.get('requested_at',0)>=started:
                if d.get('status')=='failed':raise RuntimeError(str(d.get('error')))
                if d.get('status')=='complete' and d.get('planning_ready') is False:
                    raise RuntimeError('ICP几何质量未达到可执行要求；当前仅生成预览，请重新采集后 cam-plan')
        else:
            d=read(ROOT/'latest_gun_prepose_plan_only.json')
            path=ROOT/'latest_gun_prepose_plan_only.json'
            if path.exists() and d.get('planning_status')=='failed':
                raise RuntimeError(str(d.get('error')))
        # Distinguish a planner that never started/crashed from one still calculating.
        if time.time()-started>PLANNER_STARTUP_GRACE_S:
            from charge_workflow_state import pids
            name='plan_camera_pre_align_nearest_ik.py' if stage=='camera' else 'plan_charging_gun_prepose.py'
            if not pids('/home/nvidia/nova2_tools/'+name):
                raise RuntimeError('规划进程已退出或窗口未启动，请查看对应窗口错误')
        time.sleep(.2)
    raise RuntimeError('等待规划超时；请查看规划窗口后重新规划')
def main():
    action=sys.argv[1]
    if action=='begin':begin(sys.argv[2])
    elif action=='fail':finish('failed',sys.argv[2])
    elif action=='missing':print('[无法执行] '+missing())
    elif action=='wait':wait(sys.argv[2])
    else:raise ValueError(action)
if __name__=='__main__':
    try:main()
    except Exception as e:
        finish('failed',str(e));print('[规划未完成] '+str(e),file=sys.stderr);sys.exit(2)
