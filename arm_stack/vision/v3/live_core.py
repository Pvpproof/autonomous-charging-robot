"""Image-based live session state. No hardware, UI or robot imports."""
import json
import cv2
import numpy as np
import socket_pnp as p


def track(previous,current,points,cfg):
    a=np.asarray(points,np.float32).reshape(-1,1,2)
    if a.shape!=(7,1,2) or not np.isfinite(a).all():return None
    b,s,_=cv2.calcOpticalFlowPyrLK(previous,current,a,None,winSize=(41,41),maxLevel=3)
    if b is None or s is None or not s.all() or not np.isfinite(b).all():return None
    back,s2,_=cv2.calcOpticalFlowPyrLK(current,previous,b,None,winSize=(41,41),maxLevel=3)
    if back is None or s2 is None or not s2.all() or not np.isfinite(back).all():return None
    if np.max(np.linalg.norm(back-a,axis=2))>cfg['max_forward_backward_error_px']:return None
    if np.max(np.linalg.norm(b-a,axis=2))>cfg['max_feature_motion_px']:return None
    return b.reshape(-1,2).astype(float)


class PoseSession:
    def __init__(self,camera,cfg,settings):
        self.camera=camera;self.cfg=cfg;self.settings=settings;self.epoch=0
        self.last_frame=None;self.last_sensor_stamp=None
        self.reset()

    def reset(self):
        self.epoch+=1;self.uv=None;self.previous=None;self.confirmation=p.Confirmation()
        self.anchor_gray=None;self.anchor_uv=None;self.trusted_pose=None
        self.verified_at=None;self.measurement_at=None;self.result={'geometry_valid':False,'reason':'searching'}
        self.source='none';self.reference_age=None

    def advance(self,image,now,frame_id,sensor_stamp,recovery=None):
        if self.last_frame is not None and (frame_id<=self.last_frame or sensor_stamp<=self.last_sensor_stamp):
            self.reset();self.result={'geometry_valid':False,'reason':'duplicate_or_reversed_frame'}
            return False
        self.last_frame=frame_id;self.last_sensor_stamp=sensor_stamp
        gray=cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
        result={'geometry_valid':False,'reason':'searching'}
        if self.uv is not None and self.previous is not None:
            self.uv=track(self.anchor_gray if self.anchor_gray is not None else self.previous,
                          gray,self.anchor_uv if self.anchor_uv is not None else self.uv,self.cfg)
            # This candidate is specifically for stationary final measurements.
            # Visible motion invalidates the anchor, rather than forcing a stale pose.
            if self.uv is not None and self.anchor_uv is not None and np.max(np.linalg.norm(self.uv-self.anchor_uv,axis=1))>1.0:
                self.reset();self.result={'geometry_valid':False,'reason':'static_anchor_scene_moved'}
                self.previous=gray;self.measurement_at=now
                return False
            if self.uv is not None:
                result=p.solve_pose(self.uv,self.camera,self.cfg);self.source='current_frame_optical_flow'
            if not result.get('geometry_valid'):
                self.uv=None;self.anchor_gray=None;self.anchor_uv=None;self.trusted_pose=None;self.confirmation.reset()
        if recovery is not None:
            reference_age=now-recovery['submitted_at']
            if (recovery['epoch']==self.epoch and 0<=reference_age<=self.settings['maximum_recovery_reference_age_s']
                and recovery['result'].get('geometry_valid') and recovery['points'] is not None):
                points=track(recovery['gray'],gray,recovery['points'],self.cfg)
                candidate=p.solve_pose(points,self.camera,self.cfg) if points is not None else {'geometry_valid':False}
                correction_ok=True
                if candidate.get('geometry_valid') and result.get('geometry_valid'):
                    dt,dr=p.pose_delta(np.asarray(result['best']['T_camera_from_socket']),np.asarray(candidate['best']['T_camera_from_socket']))
                    correction_ok=dt<=self.settings['correction_translation_m'] and dr<=self.settings['correction_rotation_deg']
                if candidate.get('geometry_valid') and correction_ok:
                    if self.uv is None:self.confirmation.reset()
                    if self.anchor_uv is None:
                        self.uv=points;result=candidate
                    self.verified_at=now
                    self.source='recovery_retracked_on_current_frame';self.reference_age=reference_age
        if self.uv is not None and (self.verified_at is None or now-self.verified_at>self.settings['maximum_unverified_s']):
            self.uv=None;self.anchor_gray=None;self.anchor_uv=None;self.trusted_pose=None;self.confirmation.reset();result={'geometry_valid':False,'reason':'independent_verification_expired'}
        # A large discontinuity cancels the trusted track instead of blending it.
        if self.trusted_pose is not None and result.get('geometry_valid'):
            dt,dr=p.pose_delta(self.trusted_pose,
                               np.asarray(result['best']['T_camera_from_socket']))
            if dt>0.002 or dr>2.0:
                self.confirmation.reset()
                result={'geometry_valid':False,'reason':'pose_jump_rejected',
                        'translation_jump_m':dt,'rotation_jump_deg':dr}
        self.result=result;self.previous=gray;self.measurement_at=now
        confirmed=self.confirmation.update(result,now,self.cfg)
        if confirmed:
            self.trusted_pose=np.asarray(result['best']['T_camera_from_socket']).copy()
        if confirmed and self.anchor_uv is None and self.uv is not None:
            self.anchor_uv=self.uv.copy();self.anchor_gray=gray.copy()
        return confirmed

    def payload(self,now):
        fresh=self.measurement_at is not None and 0<=now-self.measurement_at<=self.cfg['max_measurement_age_s']
        calibrated=self.camera.get('distortion_model','assumed_none')!='assumed_none'
        verified=self.verified_at is not None and 0<=now-self.verified_at<=self.settings['maximum_unverified_s']
        valid=bool(fresh and calibrated and verified and self.result.get('geometry_valid') and self.result.get('in_working_range') and self.confirmation.count>=self.cfg['confirm_frames'])
        best=self.result.get('best');reason=self.result['reason']
        if self.result.get('geometry_valid'):
            if not fresh:reason='measurement_expired'
            elif not verified:reason='independent_verification_expired'
            elif not calibrated:reason='missing_distortion_metadata'
            elif not self.result.get('in_working_range'):reason='outside_130_150mm_working_range'
            elif not valid:reason='confirming_fresh_frames'
            else:reason='visual_pose_valid_read_only'
        matrix=None if best is None else best['T_camera_from_socket']
        xyz_rotvec=None
        if matrix is not None:
            T=np.asarray(matrix);xyz_rotvec=T[:3,3].tolist()+cv2.Rodrigues(T[:3,:3])[0].ravel().tolist()
        return {'schema_version':3,'read_only':True,'pose_valid':valid,'planning_ready':False,'reason':reason,
                'unit':'m','camera_frame':'realsense_color_optical_frame',
                'frame_convention':'P_camera = T_camera_from_socket @ P_socket',
                'T_camera_from_socket':matrix if valid else None,'candidate_T_camera_from_socket':matrix,
                'pose6d_xyz_m_rotvec_rad':xyz_rotvec if valid else None,
                'candidate_pose6d_xyz_m_rotvec_rad':xyz_rotvec,
                'points_px':None if self.uv is None else self.uv.tolist(),
                'measurement_time_host':self.measurement_at,'write_time_host':now,
                'source':self.source,'confirmation_frames':self.confirmation.count,
                'recovery_reference_age_s':self.reference_age,
                'age_s':None if self.measurement_at is None else now-self.measurement_at,
                'frame_id':self.last_frame,'sensor_timestamp_ms':self.last_sensor_stamp,'quality':self.result}


def write_json(path,value):
    path.parent.mkdir(parents=True,exist_ok=True)
    temporary=path.with_suffix('.tmp')
    temporary.write_text(json.dumps(value,indent=2,allow_nan=False),encoding='utf8')
    temporary.replace(path)


def worker_main(requests,responses):
    from automatic_recovery import estimate
    cv2.setNumThreads(1)
    while True:
        item=requests.get()
        if item is None:return
        try:
            uv,result,diagnostics=estimate(item['image'],item['camera'],item['cfg'])
            responses.put({'job':item['job'],'points':uv,'result':result,'source':diagnostics['source']})
        except Exception as error:
            responses.put({'job':item['job'],'points':None,'result':{'geometry_valid':False,'reason':'worker_error'},'error':repr(error)})
