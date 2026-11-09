#!/usr/bin/env python3
import json, re, sys, time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from dobot_msgs_v4.srv import GetPose, MovJ
from scipy.spatial.transform import Rotation

POSE_JSON=Path('/home/nvidia/charging_socket_vision/results/realtime_runs/latest_pose_readonly.json')
HAND_EYE=Path('/home/nvidia/charging_robot_tools/calibration/hand_eye_calibration_result_ros_24mm.npz')
DIST=0.150


def rot_dobot(rx,ry,rz):
    rx,ry,rz=np.deg2rad([rx,ry,rz]); cx,sx=np.cos(rx),np.sin(rx); cy,sy=np.cos(ry),np.sin(ry); cz,sz=np.cos(rz),np.sin(rz)
    return np.array([[cz*cy,cz*sy*sx-sz*cx,cz*sy*cx+sz*sx],[sz*cy,sz*sy*sx+cz*cx,sz*sy*cx-cz*sx],[-sy,cy*sx,cy*cx]])

class Execute(Node):
    def __init__(self):
        super().__init__('execute_camera_pre_align_movj_confirmed')
        self.getpose=self.create_client(GetPose,'/dobot_bringup_ros2/srv/GetPose')
        self.movj=self.create_client(MovJ,'/dobot_bringup_ros2/srv/MovJ')

    def candidate(self):
        if not self.getpose.wait_for_service(8): raise RuntimeError('GetPose unavailable')
        f=self.getpose.call_async(GetPose.Request(user=0,tool=0)); rclpy.spin_until_future_complete(self,f,timeout_sec=8); res=f.result()
        if res is None or res.res!=0: raise RuntimeError(f'GetPose failed: {res}')
        vals=[float(x) for x in re.findall(r'[-+]?\d+(?:\.\d+)?',res.robot_return)]
        if len(vals)<6: raise RuntimeError(res.robot_return)
        Tbg=np.eye(4); Tbg[:3,:3]=rot_dobot(*vals[3:6]); Tbg[:3,3]=np.asarray(vals[:3])/1000.0
        with np.load(HAND_EYE) as d:
            Tgc=np.eye(4); Tgc[:3,:3]=np.asarray(d['R_cam2gripper'],float); Tgc[:3,3]=np.asarray(d['t_cam2gripper'],float).reshape(3)
        if np.linalg.norm(Tgc[:3,3])>2:Tgc[:3,3]/=1000.0
        d=json.loads(POSE_JSON.read_text()); age=time.time()-float(d.get('timestamp',0))
        if d.get('status')!='tracking' or float(d.get('fitness',0))<.80 or float(d.get('rmse_m',1))>.012: raise RuntimeError(f'ICP quality not safe: status={d.get("status")} fitness={d.get("fitness")} rmse_m={d.get("rmse_m")}')
        if age>5: raise RuntimeError(f'ICP pose is stale: age={age:.2f}s')
        Tcs=np.asarray(d['T_camera_from_socket_raw'],float); Tbs=Tbg@Tgc@Tcs; Rs=Tbs[:3,:3]; cam=Tbs[:3,3]+Rs[:,2]*DIST; Rc=np.column_stack((Rs[:,0],-Rs[:,1],-Rs[:,2])); Tbc=np.eye(4); Tbc[:3,:3]=Rc; Tbc[:3,3]=cam; Tgoal=Tbc@np.linalg.inv(Tgc)
        rpy=Rotation.from_matrix(Tgoal[:3,:3]).as_euler('xyz',degrees=True); out=np.array([*Tgoal[:3,3]*1000,*rpy])
        return out, vals, d, age

    def execute_after_confirmation(self, target, current, d, age):
        print('\n真实预对准目标（MovJ pose）:')
        print('  x y z(mm)   =', ' '.join(f'{v:.3f}' for v in target[:3]))
        print('  rx ry rz(deg)=', ' '.join(f'{v:.3f}' for v in target[3:]))
        print(f'  distance={DIST*1000:.0f}mm  velocity=10%  acceleration=10%')
        print('  ICP:', d.get('status'), 'fitness=', d.get('fitness'), 'rmse_mm=', float(d.get('rmse_m',0))*1000, f'age={age:.2f}s')
        print('  current GetPose=', current)
        print('\n确认工作区安全、急停可触及、无工具动作后，输入 EXECUTE 才发送一次 MovJ。')
        try: answer=input('输入 EXECUTE 继续，其他内容取消: ').strip()
        except EOFError: answer=''
        if answer!='EXECUTE': print('已取消，未发送运动命令。'); return 2
        if not self.movj.wait_for_service(8): raise RuntimeError('MovJ service unavailable')
        req=MovJ.Request(); req.mode=False; req.a,req.b,req.c,req.d,req.e,req.f=map(float,target); req.param_value=['user=0','tool=0','a=10','v=10','cp=100']
        print('发送 MovJ：仅一次，10%速度/10%加速度...')
        f=self.movj.call_async(req); rclpy.spin_until_future_complete(self,f,timeout_sec=12); res=f.result()
        print('MovJ response:', res)
        if res is None or res.res!=0: return 3
        print('MovJ accepted. 请继续观察机械臂，必要时立即按急停。')
        return 0

def main():
    rclpy.init(); n=Execute()
    try:
        target,current,d,age=n.candidate(); rc=n.execute_after_confirmation(target,current,d,age)
    except Exception as e:
        print('执行前检查失败，未发送运动命令：',e); rc=2
    n.destroy_node(); rclpy.shutdown(); return rc
if __name__=='__main__': sys.exit(main())
