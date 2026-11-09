#!/usr/bin/env python3
"""Collect a normalized multi-view socket template from the live ICP output.

The realtime ICP process remains the matcher. This tool only reads its latest
pose/quality and target point cloud, checks short-term stability, transforms
the camera cloud into the common socket frame, and saves one named sample.
No robot, camera, or motion command is issued here.
"""
import argparse, json, math, re, sys, time
from pathlib import Path
import numpy as np
import open3d as o3d

ROOT=Path('/opt/charging-robot-local/charging_socket_vision')
POSE=ROOT/'results/realtime_runs/latest_pose_readonly.json'
CLOUD=ROOT/'results/realtime_runs/latest_target_camera.ply'
OUT=ROOT/'template_library'

def load_pose():
    d=json.loads(POSE.read_text())
    T=np.asarray(d['T_camera_from_socket_raw'],float)
    if T.shape!=(4,4): raise RuntimeError('pose matrix shape is not 4x4')
    return d,T

def pose_delta(T0,T1):
    dp=float(np.linalg.norm(T0[:3,3]-T1[:3,3]))
    R=T0[:3,:3].T@T1[:3,:3]
    c=float(np.clip((np.trace(R)-1)/2,-1,1))
    da=math.degrees(math.acos(c))
    return dp,da

def wait_stable(args):
    samples=[]
    deadline=time.time()+args.timeout
    while time.time()<deadline:
        try:
            d,T=load_pose()
            age=time.time()-float(d.get('timestamp',0))
            fit=float(d.get('fitness',0)); rmse=float(d.get('rmse_m',99))
            if age<=args.max_age and d.get('status') in ('tracking','stable') and fit>=args.min_fitness and rmse<=args.max_rmse:
                samples.append((time.time(),d,T))
                if len(samples)>args.samples: samples.pop(0)
                if len(samples)>=args.samples:
                    base=samples[0][2]; ok=True
                    for _,_,t in samples[1:]:
                        dp,da=pose_delta(base,t)
                        if dp>args.pos_tol or da>args.angle_tol: ok=False; break
                    if ok: return samples[-1][1],samples[-1][2],samples
            else:
                samples.clear()
        except Exception:
            samples.clear()
        time.sleep(args.period)
    raise RuntimeError('在规定时间内没有得到稳定且质量合格的 ICP 结果')

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--name',required=True,help='sample name, e.g. front, left15, upper_right')
    ap.add_argument('--out',type=Path,default=OUT)
    ap.add_argument('--samples',type=int,default=8)
    ap.add_argument('--period',type=float,default=.25)
    ap.add_argument('--timeout',type=float,default=30.)
    ap.add_argument('--max-age',type=float,default=1.5)
    ap.add_argument('--min-fitness',type=float,default=.45)
    ap.add_argument('--max-rmse',type=float,default=.015)
    ap.add_argument('--pos-tol',type=float,default=.004,help='stability translation tolerance in meters')
    ap.add_argument('--angle-tol',type=float,default=2.0,help='stability rotation tolerance in degrees')
    args=ap.parse_args()
    if not re.fullmatch(r'[A-Za-z0-9_.-]+',args.name): raise SystemExit('name must use letters, numbers, _, ., - only')
    d,T,samples=wait_stable(args)
    if not CLOUD.exists(): raise RuntimeError(f'missing live target cloud: {CLOUD}')
    cloud=o3d.io.read_point_cloud(str(CLOUD))
    if len(cloud.points)<80: raise RuntimeError(f'target cloud too small: {len(cloud.points)}')
    # P_socket = inv(T_camera_from_socket) * P_camera.
    socket_cloud=__import__("copy").deepcopy(cloud).transform(np.linalg.inv(T))
    sample=args.out/args.name; sample.mkdir(parents=True,exist_ok=False)
    o3d.io.write_point_cloud(str(sample/'target_socket.ply'),socket_cloud,write_ascii=False,compressed=False)
    o3d.io.write_point_cloud(str(sample/'target_camera.ply'),cloud,write_ascii=False,compressed=False)
    metadata={
      'name':args.name,'source':'live_realtime_socket_pose.py',
      'normalized_frame':'socket_model_frame','motion_commands_sent':False,
      'T_camera_from_socket':T.tolist(),
      'fitness':float(d.get('fitness',0)),'rmse_m':float(d.get('rmse_m',99)),
      'status':d.get('status'),'source_timestamp':d.get('timestamp'),
      'stable_samples':len(samples),'cloud_points':len(socket_cloud.points),
      'model_frame':{'x':'right','y':'up','z':'outward','origin':'PE axis at front mouth plane'},
      'files':['target_socket.ply','target_camera.ply','metadata.json']}
    (sample/'metadata.json').write_text(json.dumps(metadata,indent=2,ensure_ascii=False))
    print(json.dumps(metadata,indent=2,ensure_ascii=False))
    print('SAVED_TEMPLATE',sample)
    return 0

if __name__=='__main__':
    try: sys.exit(main())
    except Exception as e: print('COLLECT_REFUSED:',e,file=sys.stderr); sys.exit(2)
