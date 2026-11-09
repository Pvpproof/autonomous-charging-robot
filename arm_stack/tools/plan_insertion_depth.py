#!/usr/bin/env python3
"""Plan-only axial insertion candidates. No ROS and no robot commands."""
import json,sys,time
from pathlib import Path
import numpy as np
ROOT=Path('/opt/charging-robot-local/charging_socket_vision/results/realtime_runs')
PROFILE=Path('/opt/charging-robot-local/nova2_tools/insertion_profile.json')
OUT=ROOT/'latest_insertion_depth_candidates.json'
def load_target():
    paths=[ROOT/'latest_gun_prepose_plan_only.json',ROOT/'last_dispatched_trajectory.json']
    for p in paths:
        if p.exists():
            d=json.loads(p.read_text()); T=d.get('target_tcp_base') or d.get('endpoint_validation',{}).get('target_tcp_base')
            if T is not None:
                a=np.asarray(T,float)
                if a.shape==(4,4): return d,a
    raise RuntimeError('没有可用的枪头目标；请先完成 gun-plan')
def main():
    cfg=json.loads(PROFILE.read_text()); d,T=load_target(); z=T[:3,2].copy(); rows=[]
    for depth in cfg['candidate_depths_mm']:
        c=T.copy(); c[:3,3]=T[:3,3]+z*(float(depth)/1000.)
        rows.append({'depth_from_prepose_mm':float(depth),'tcp_position_base_mm':(c[:3,3]*1000.).tolist(),'orientation_base':c[:3,:3].tolist(),'motion_commands_sent':False})
    out={'read_only':True,'motion_commands_sent':False,'source_plan_id':d.get('plan_id'),'source_stage':d.get('stage'),'reference':'existing_50mm_prepose_tcp_target','positive_direction':'current gun TCP +Z / toward socket','profile':cfg,'candidates':rows,'generated_at':time.time()}
    OUT.write_text(json.dumps(out,indent=2,ensure_ascii=False)); print(json.dumps(out,indent=2,ensure_ascii=False))
if __name__=='__main__':
    try: main()
    except Exception as e: print('INSERTION_PLAN_REFUSED:',e); sys.exit(2)
