"""D435i RGB-only read-only pose viewer. No ROS or robot connection."""
import argparse
import json
import multiprocessing as mp
import queue
import time
from pathlib import Path
import cv2
import numpy as np
import socket_pnp as p
from live_core import PoseSession, worker_main, write_json


class RecoveryWorker:
    def __init__(self,timeout):
        self.ctx=mp.get_context('spawn');self.timeout=timeout;self.pending=None;self.job=0
        self.start()
    def start(self):
        self.requests=self.ctx.Queue(maxsize=1);self.responses=self.ctx.Queue(maxsize=1)
        self.process=self.ctx.Process(target=worker_main,args=(self.requests,self.responses),daemon=True)
        self.process.start()
    def stop(self):
        try:self.requests.put_nowait(None)
        except queue.Full:pass
        self.process.join(.2)
        if self.process.is_alive():self.process.terminate();self.process.join(.5)
        for q in (self.requests,self.responses):q.cancel_join_thread();q.close()
        self.pending=None
    def submit(self,image,camera,cfg,epoch,stamp):
        if self.pending is not None:return
        self.job+=1
        self.pending={'job':self.job,'gray':cv2.cvtColor(image,cv2.COLOR_BGR2GRAY),
                      'epoch':epoch,'submitted_at':stamp,'started_monotonic':time.monotonic()}
        try:self.requests.put_nowait({'job':self.job,'image':image.copy(),'camera':camera,'cfg':cfg})
        except queue.Full:self.pending=None
    def poll(self):
        if not self.process.is_alive() or (self.pending is not None and time.monotonic()-self.pending['started_monotonic']>self.timeout):
            self.stop();self.start()
            return {'error':'recovery_worker_timeout_or_exit','result':{'geometry_valid':False}}
        try:answer=self.responses.get_nowait()
        except queue.Empty:return None
        if self.pending is None or answer['job']!=self.pending['job']:return None
        answer.update(self.pending);self.pending=None
        return answer


def verify_camera_model(rs,ci,camera):
    p.camera_parameters(camera)
    probes=np.array([[120,100],[320,240],[520,380]],float)
    expected=np.array([rs.rs2_deproject_pixel_to_point(ci,point.tolist(),1.)[:2] for point in probes])
    actual=(p.undistort_pixels(probes,camera)-[ci.ppx,ci.ppy])/[ci.fx,ci.fy]
    if not np.isfinite(actual).all() or np.max(abs(actual-expected))>1e-4:
        raise ValueError('Distortion conversion differs from RealSense SDK; save the error and camera metadata')


def main():
    root=Path(__file__).resolve().parent
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config',type=Path,default=root/'pnp_near.json')
    parser.add_argument('--settings',type=Path,default=root/'live_settings.json')
    parser.add_argument('--output',type=Path,default=root/'live_results')
    parser.add_argument('--fps',type=int,choices=[6,15,30],help='Override color FPS if supported by camera')
    parser.add_argument('--no-display',action='store_true',help='Run headless; stop with SIGINT or SIGTERM')
    args=parser.parse_args();cfg=p.load_config(args.config)
    settings=json.loads(args.settings.read_text(encoding='utf-8-sig'))
    if args.fps:settings['fps']=args.fps
    if not 0<settings['verification_interval_s']<settings['maximum_unverified_s']:
        raise ValueError('Verification interval must be positive and below expiry')
    if not 0<settings['maximum_recovery_reference_age_s']<settings['worker_timeout_s']:
        raise ValueError('Recovery reference expiry must be below worker timeout')
    args.output.mkdir(parents=True,exist_ok=True)
    latest=args.output/'latest_pose_readonly.json'
    def invalid(reason):write_json(latest,{'pose_valid':False,'planning_ready':False,'T_camera_from_socket':None,'reason':reason,'write_time_host':time.time()})
    invalid('starting')
    # Import only inside main: spawned recovery process does not load camera SDK.
    import pyrealsense2 as rs
    cv2.setNumThreads(1)
    pipeline=rs.pipeline();configuration=rs.config()
    configuration.enable_stream(rs.stream.color,640,480,rs.format.bgr8,settings['fps'])
    started=False;worker=None;frame_count=0;window='Socket RGB PnP | S save | R relock | Q quit'
    print('RGB PnP V3.1: 640x480, RGB only, read-only, no robot interface',flush=True)
    exit_reason="exception_or_interrupt"
    try:
        print("[START] Opening RGB camera",flush=True)
        profile=pipeline.start(configuration);started=True
        print("[START] Camera started",flush=True)
        ci=profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
        camera={'width':ci.width,'height':ci.height,'fx':ci.fx,'fy':ci.fy,'cx':ci.ppx,'cy':ci.ppy,
                'coeffs':list(ci.coeffs),'distortion_model':str(ci.model).replace('distortion.','')}
        write_json(args.output/'camera.json',camera);verify_camera_model(rs,ci,camera)
        print('[START] Intrinsics verified; warming up 20 frames',flush=True)
        for _ in range(20):pipeline.wait_for_frames(timeout_ms=5000)
        print("[START] Warm-up complete",flush=True)
        session=PoseSession(camera,cfg,settings);worker=RecoveryWorker(settings['worker_timeout_s'])
        print("[START] Recovery process started; press Q or Esc to quit",flush=True)
        last_progress=0.
        last_frame_time=time.time();last_submit=-float('inf');last_notice='';read_errors=0
        placeholder=np.zeros((480,640,3),np.uint8)
        cv2.putText(placeholder,'Waiting for RGB frames',(30,100),0,.8,(0,255,255),2)
        if not args.no_display:
            cv2.imshow(window,placeholder)
        while True:
            try:
                frames=pipeline.wait_for_frames(timeout_ms=200);frame=frames.get_color_frame()
                read_errors=0
            except RuntimeError as error:
                read_errors+=1
                if time.time()-last_frame_time>cfg['max_measurement_age_s']:
                    session.reset();invalid('camera_frame_timeout')
                    if not args.no_display:
                        timeout_view=placeholder.copy();cv2.putText(timeout_view,'Frame timeout - invalid',(30,150),0,.7,(0,0,255),2);cv2.imshow(window,timeout_view)
                if not args.no_display and cv2.waitKey(1)&255 in (27,ord('q')):
                    exit_reason='quit_key_during_timeout';break
                if read_errors>=10:raise RuntimeError('Camera produced no usable frames; check USB/camera ownership') from error
                continue
            if not frame:continue
            stamp=time.time();last_frame_time=stamp
            image=np.asanyarray(frame.get_data()).copy()
            if image.shape!=(480,640,3):raise ValueError('Camera did not return native 640x480 BGR')
            completed=worker.poll()
            if completed is not None and 'error' in completed:
                print(completed['error'],flush=True);last_notice=completed['error'];completed=None
            session.advance(image,stamp,int(frame.get_frame_number()),float(frame.get_timestamp()),completed)
            if completed is not None:last_notice=completed.get('source',completed['result'].get('reason',''))
            if worker.pending is None and (session.uv is None or stamp-last_submit>=settings['verification_interval_s']):
                worker.submit(image,camera,cfg,session.epoch,stamp);last_submit=stamp
            payload=session.payload(time.time());payload['recovery_busy']=worker.pending is not None;payload['recovery_notice']=last_notice
            write_json(latest,payload)
            view=p.draw_result(image,session.uv,session.result,camera)
            text='VISUAL VALID / READ ONLY' if payload['pose_valid'] else payload['reason']
            cv2.putText(view,text,(10,440),0,.48,(0,255,0) if payload['pose_valid'] else (0,255,255),1)
            activity=('BACKGROUND CHECK' if session.uv is not None else 'SEARCHING') if worker.pending else 'TRACKING'
            cv2.putText(view,activity+' / Q quit S save R relock',(10,465),0,.48,(0,255,255),1)
            key=-1
            if not args.no_display:
                cv2.imshow(window,view);key=cv2.waitKey(1)&255
            if key in (27,ord('q')):
                exit_reason='quit_key';break
            if key==ord('r'):
                session.reset();invalid('manual_relock');last_submit=-float('inf')
            if key==ord('s') and not args.no_display:
                name=time.strftime('%Y%m%d_%H%M%S')+f'_f{frame_count:06d}'
                cv2.imwrite(str(args.output/(name+'_rgb.png')),image);cv2.imwrite(str(args.output/(name+'_overlay.png')),view)
                write_json(args.output/(name+'_camera.json'),camera);write_json(args.output/(name+'_pose.json'),payload)
                print('Saved',name,flush=True)
            frame_count+=1
            # Some GUI backends do not report window visibility reliably.
            # Exit only via Q/Esc or Ctrl+C; never infer closure from this property.
            if time.time()-last_progress>=2.:
                print('[LIVE] frames={} status={} recovery_busy={}'.format(frame_count,payload['reason'],worker.pending is not None),flush=True)
                last_progress=time.time()
    except KeyboardInterrupt:
        exit_reason='keyboard_interrupt'
    except Exception as error:
        exit_reason='exception:'+type(error).__name__
        print('[ERROR]',repr(error),flush=True)
        raise
    finally:
        print('[EXIT]',exit_reason,flush=True)
        invalid('camera_stopped:'+exit_reason)
        if worker is not None:worker.stop()
        if started:pipeline.stop()
        if not args.no_display:
            cv2.destroyAllWindows()


if __name__=='__main__':
    mp.freeze_support()
    main()
