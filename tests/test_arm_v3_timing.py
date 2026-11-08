"""Unmodified real Confirmation/PoseSession/collector, synthetic time/geometry only."""
import copy,json
from pathlib import Path
import sys,tempfile,types,unittest
from unittest.mock import patch,Mock
import numpy as np
ROOT=Path(__file__).resolve().parents[1]/'arm_stack';sys.path.insert(0,str(ROOT/'vision/v3'))
import socket_pnp as p
import live_core
import stable_pose_selection as selection
import run_live
CFG=p.load_config(ROOT/'examples/pnp_near.example.json')
SETTINGS=json.loads((ROOT/'examples/live_settings.example.json').read_text())
T=np.eye(4);T[2,3]=.14
RESULT=dict(geometry_valid=True,in_working_range=True,reason='synthetic_geometry',best=dict(T_camera_from_socket=T.tolist(),rmse_px=.1))
IMAGE=np.zeros((48,64,3),np.uint8);UV=np.zeros((7,2))

class TimingTests(unittest.TestCase):
 def setUp(self):
  self.track=patch.object(live_core,'track',side_effect=lambda *a:UV.copy());self.track.start()
  self.solve=patch.object(p,'solve_pose',side_effect=lambda *a:copy.deepcopy(RESULT));self.solve.start()
 def tearDown(self):self.solve.stop();self.track.stop()
 def session(self):return live_core.PoseSession({'distortion_model':'none'},CFG,SETTINGS)
 def frame(self,s,t,index,recovery=False,reference_age=.1):
  answer=dict(submitted_at=t-reference_age,epoch=s.epoch,result=copy.deepcopy(RESULT),points=UV.copy(),gray=np.zeros((48,64),np.uint8)) if recovery else None
  return s.advance(IMAGE,t,index,index*100.,answer)
 def test_fifteen_fps_three_frames_become_valid(self):
  s=self.session()
  for i in range(3):self.frame(s,10+i/15,i+1,recovery=i==0)
  self.assertTrue(s.payload(10+2/15)['pose_valid']);self.assertEqual(s.confirmation.count,3)
 def test_confirmation_window_is_adjacent_gap_not_total_three_frame_span(self):
  s=self.session()
  for i in range(3):self.frame(s,10+i*.49,i+1,recovery=i==0)
  self.assertTrue(s.payload(10.98)['pose_valid']);self.assertGreater(.49*2,CFG['confirmation_window_s'])
 def test_gap_over_half_second_never_reaches_three(self):
  s=self.session()
  for i in range(10):
   self.frame(s,10+i*.501,i+1,recovery=i in (0,4,8));self.assertEqual(s.confirmation.count,1)
   self.assertFalse(s.payload(10+i*.501)['pose_valid'])
 def test_half_second_boundary_is_inclusive(self):
  c=p.Confirmation()
  self.assertFalse(c.update(RESULT,10.,CFG));self.assertFalse(c.update(RESULT,10.5,CFG));self.assertTrue(c.update(RESULT,11.,CFG))
 def test_processing_delay_can_expire_otherwise_confirmed_pose(self):
  s=self.session()
  for i in range(3):self.frame(s,10+i/15,i+1,recovery=i==0)
  stamp=10+2/15
  self.assertTrue(s.payload(stamp+.499)['pose_valid'])
  late=s.payload(stamp+.501);self.assertFalse(late['pose_valid']);self.assertEqual(late['reason'],'measurement_expired');self.assertIsNone(late['T_camera_from_socket'])
 def test_two_second_background_checks_do_not_limit_frame_confirmation_rate(self):
  s=self.session();valid=0
  for i in range(106):
   stamp=10+i/15;self.frame(s,stamp,i+1,recovery=i%30==0)
   if i>=2:self.assertTrue(s.payload(stamp)['pose_valid']);valid+=1
  self.assertEqual(valid,104)
 def test_recovery_older_than_eight_seconds_is_not_adopted(self):
  s=self.session();self.frame(s,20.,1,recovery=True,reference_age=8.001)
  self.assertIsNone(s.uv);self.assertFalse(s.payload(20.)['pose_valid'])
 def test_independent_check_expiry_forces_search(self):
  s=self.session()
  for i in range(3):self.frame(s,10+i/15,i+1,recovery=i==0)
  self.frame(s,16.001,4)
  self.assertIsNone(s.uv);self.assertEqual(s.result['reason'],'independent_verification_expired')
 def test_worker_timeout_restarts_without_using_result(self):
  worker=object.__new__(run_live.RecoveryWorker);worker.timeout=12.;worker.pending={'started_monotonic':0.};worker.process=types.SimpleNamespace(is_alive=lambda:True)
  worker.stop=Mock();worker.start=Mock()
  with patch.object(run_live.time,'monotonic',return_value=12.001):answer=worker.poll()
  self.assertEqual(answer['error'],'recovery_worker_timeout_or_exit');worker.stop.assert_called_once();worker.start.assert_called_once()
 def collect(self,hz):
  clock=types.SimpleNamespace(now=1000.)
  fake_time=types.SimpleNamespace(time=lambda:clock.now,monotonic=lambda:clock.now,sleep=lambda seconds:setattr(clock,'now',clock.now+seconds))
  def reader():
   index=int((clock.now-1000.)*hz+1e-7)
   return dict(stamp=1000.+index/hz,matrix=T.tolist())
  with tempfile.TemporaryDirectory() as tmp,patch.object(selection,'time',fake_time):
   report=Path(tmp)/'selection.json'
   try:
    chosen,stats=selection.collect(reader,'gun',report);out=dict(accepted=True,stats=stats)
   except RuntimeError as error:out=dict(accepted=False,reason=str(error))
   out['simulated_elapsed_s']=clock.now-1000.;return out
 def test_gun_collector_accepts_fifteen_and_six_hz(self):
  for hz in (15.,6.):
   result=self.collect(hz);self.assertTrue(result['accepted'],result);self.assertLess(result['simulated_elapsed_s'],3.)
 def test_gun_collector_five_hz_cannot_fit_fifteen_samples_in_window(self):
  result=self.collect(5.);self.assertFalse(result['accepted']);self.assertIn('not_enough_distinct_measurements',result['reason']);self.assertGreaterEqual(result['simulated_elapsed_s'],25.)
 def test_gun_collector_threshold_is_fourteen_intervals_over_two_point_five_seconds(self):
  policy=selection.POLICIES['gun'];self.assertEqual((policy['min_samples']-1)/policy['window_s'],5.6)
  samples=[dict(stamp=1000.+i/5.6,matrix=T.tolist()) for i in range(15)]
  chosen,stats=selection.select(samples,policy);self.assertEqual(stats['inlier_samples'],15);self.assertAlmostEqual(stats['span_s'],2.5)

if __name__=='__main__':unittest.main(verbosity=2)
