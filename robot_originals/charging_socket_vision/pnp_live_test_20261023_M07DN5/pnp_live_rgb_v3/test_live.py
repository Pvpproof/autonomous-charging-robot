"""Hardware-free state, mocked camera lifecycle and real worker-process checks."""
import json
import sys
import tempfile
import time
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
import cv2
import numpy as np
import socket_pnp as p
from live_core import PoseSession
import run_live


def main():
    root=Path(__file__).resolve().parent;cfg=p.load_config(root/'pnp_near.json')
    settings=json.loads((root/'live_settings.json').read_text())
    image=cv2.imread(str(root/'test_data/rgb.png'))
    camera=json.loads((root/'test_data/camera.json').read_text())
    # Explicit mocked calibration for state testing; NOT a calibration of this photo.
    camera.update(distortion_model='none',coeffs=[0]*5)
    uv,result=p.automatic_points(image,camera,cfg)
    assert result['geometry_valid']
    checks=[]
    def check(name,condition):
        if not condition:raise AssertionError(name)
        checks.append(name)
    def recovery(session,stamp):
        return {'points':uv,'result':result,'gray':cv2.cvtColor(image,cv2.COLOR_BGR2GRAY),
                'epoch':session.epoch,'submitted_at':stamp}
    session=PoseSession(camera,cfg,settings)
    session.advance(image,10.,1,100.,recovery(session,9.9))
    check('one frame not valid',not session.payload(10.)['pose_valid'])
    session.advance(image,10.1,2,200.);session.advance(image,10.2,3,300.)
    check('three fresh frames confirmed',session.payload(10.2)['pose_valid'])
    check('never planning ready',not session.payload(10.2)['planning_ready'])
    check('expired measurement invalidates matrix',session.payload(11.)['T_camera_from_socket'] is None)
    session.advance(image,10.3,3,300.)
    check('duplicate sensor frame clears pose',not session.payload(10.3)['pose_valid'] and session.uv is None)
    old=recovery(session,11.);session.reset();session.advance(image,11.1,4,400.,old)
    check('pre-reset worker result ignored',session.uv is None)
    session.advance(image,21.,5,500.,recovery(session,10.))
    check('old recovery reference ignored',session.uv is None)
    session.advance(image,21.1,6,600.,recovery(session,21.))
    session.advance(image,28.,7,700.)
    check('independent verification expiry forces search',session.uv is None)
    unknown=PoseSession({k:v for k,v in camera.items() if k!='distortion_model'},cfg,settings)
    unknown.advance(image,30.,1,100.,recovery(unknown,29.9))
    unknown.advance(image,30.1,2,200.);unknown.advance(image,30.2,3,300.)
    check('missing calibration cannot become valid',not unknown.payload(30.2)['pose_valid'])

    # Actual spawn/queue/worker computation, but with a saved image only.
    worker=run_live.RecoveryWorker(settings['worker_timeout_s'])
    try:
        worker.submit(image,camera,cfg,1,time.time());deadline=time.monotonic()+15;answer=None
        while answer is None and time.monotonic()<deadline:
            answer=worker.poll();time.sleep(.02)
        check('spawned recovery worker returns saved-image candidate',answer is not None and answer['result'].get('geometry_valid'))
        worker.submit(image,camera,cfg,1,time.time())
        worker.pending['started_monotonic']=time.monotonic()-20
        check('worker timeout restarts without pose adoption','error' in worker.poll() and worker.pending is None)
    finally:worker.stop()
    check('worker stopped',not worker.process.is_alive())

    # SDK and GUI mocks: validates orchestration, not USB support or window rendering.
    ci=SimpleNamespace(width=640,height=480,fx=camera['fx'],fy=camera['fy'],ppx=camera['cx'],ppy=camera['cy'],coeffs=[0]*5,model='none')
    calls=[]
    class Frame:
        def __init__(self,n):self.n=n
        def get_data(self):return image
        def get_frame_number(self):return self.n
        def get_timestamp(self):return self.n*1000/15
    class Pipeline:
        def __init__(self):self.n=0;self.stopped=False
        def start(self,config):
            return SimpleNamespace(get_stream=lambda _:SimpleNamespace(as_video_stream_profile=lambda:SimpleNamespace(get_intrinsics=lambda:ci)))
        def wait_for_frames(self,timeout_ms):
            self.n+=1;return SimpleNamespace(get_color_frame=lambda:Frame(self.n))
        def stop(self):self.stopped=True
    pipeline=Pipeline()
    sdk=SimpleNamespace(pipeline=lambda:pipeline,config=lambda:SimpleNamespace(enable_stream=lambda *args:calls.append(args)),
                        stream=SimpleNamespace(color='color'),format=SimpleNamespace(bgr8='bgr8'),
                        rs2_deproject_pixel_to_point=lambda _,xy,z:[(xy[0]-ci.ppx)/ci.fx*z,(xy[1]-ci.ppy)/ci.fy*z,z])
    class MockWorker:
        def __init__(self,_):self.pending=None
        def submit(self,img,cam,config,epoch,stamp):self.pending={**recovery(SimpleNamespace(epoch=epoch),stamp),'source':'mock'}
        def poll(self):answer=self.pending;self.pending=None;return answer
        def stop(self):self.pending=None
    runs=root/'test_runs';runs.mkdir(exist_ok=True)
    with tempfile.TemporaryDirectory(dir=str(runs)) as directory:
        argv=['run_live.py','--output',directory]
        with patch.dict(sys.modules,{'pyrealsense2':sdk}),patch.object(sys,'argv',argv),patch.object(run_live,'RecoveryWorker',MockWorker),patch.object(cv2,'imshow'),patch.object(cv2,'waitKey',side_effect=[-1,ord('s'),-1,ord('q')]),patch.object(cv2,'destroyAllWindows'),patch.object(cv2,'getWindowProperty',return_value=1):
            run_live.main()
        saved=list(Path(directory).glob('*_rgb.png'))
        latest=json.loads((Path(directory)/'latest_pose_readonly.json').read_text())
        check('camera uses native RGB-only profile',calls==[('color',640,480,'bgr8',15)])
        check('save button stores RGB and metadata',len(saved)==1 and len(list(Path(directory).glob('*_camera.json')))==1)
        check('exit stops camera and invalidates output',pipeline.stopped and not latest['pose_valid'] and latest['T_camera_from_socket'] is None)
        pipeline.stopped=False
        with patch.dict(sys.modules,{'pyrealsense2':sdk}),patch.object(sys,'argv',argv),patch.object(run_live,'verify_camera_model',side_effect=ValueError('test calibration failure')),patch.object(cv2,'destroyAllWindows'):
            try:run_live.main()
            except ValueError:pass
            else:raise AssertionError('Expected calibration error')
        latest=json.loads((Path(directory)/'latest_pose_readonly.json').read_text())
        check('startup calibration failure stops camera and clears output',pipeline.stopped and not latest['pose_valid'])
    report={'scope':'Mock camera lifecycle, real saved-image worker, no physical camera or GUI validation',
            'checks_passed':len(checks),'checks':checks}
    (root/'test_results.json').write_text(json.dumps(report,indent=2),encoding='utf8')
    print(json.dumps(report,indent=2))


if __name__=='__main__':main()
