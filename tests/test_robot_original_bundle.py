"""Exact original bundle and isolated copy tests; no services/devices started."""
import importlib.util,json,hashlib,tempfile,unittest
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1]/'robot_originals'
spec=importlib.util.spec_from_file_location('copy_originals',ROOT/'deploy_originals.py');copy=importlib.util.module_from_spec(spec);spec.loader.exec_module(copy)
class BundleTests(unittest.TestCase):
 def test_all_original_hashes(self):self.assertEqual(len(copy.manifest_files(ROOT)),599)
 def test_originals_are_unmodified_field_implementations(self):
  for name in ['workflow_progress.py','insertion_runtime.py','nova2_real_model.launch.py']:
   self.assertNotIn('RECONSTRUCTED = True',(ROOT/'nova2_tools'/name).read_text())
 def test_handeye_numerically_well_formed(self):
  with np.load(ROOT/'charging_robot_tools/calibration/hand_eye_calibration_result_ros_24mm.npz',allow_pickle=False) as d:
   R=d['R_cam2gripper'];self.assertEqual(R.shape,(3,3));np.testing.assert_allclose(R.T@R,np.eye(3),atol=1e-6);self.assertAlmostEqual(np.linalg.det(R),1.,places=6);self.assertTrue(np.isfinite(d['t_cam2gripper']).all())
 def test_dobot_license_retained(self):self.assertIn('Copyright (c) 2026 Dobot',(ROOT/'nova2_ros2_ws/src/dobot_ros2_v4/LICENSE').read_text())
 def fixture(self):
  tmp=tempfile.TemporaryDirectory();self.addCleanup(tmp.cleanup);d=Path(tmp.name);b=d/'bundle';b.mkdir();(b/'a').write_text('source');(b/'source-manifest.json').write_text(json.dumps({'files':[{'path':'a','sha256':hashlib.sha256(b'source').hexdigest()}]}));return b,d/'target'
 def test_preview_does_not_write(self):
  b,t=self.fixture();self.assertEqual(copy.restore(b,t),{'same':0,'replace':0,'add':1});self.assertFalse(t.exists())
 def test_apply_in_temp_and_preserve_backup(self):
  b,t=self.fixture();t.mkdir();(t/'a').write_text('old');result=copy.restore(b,t,True);self.assertEqual((t/'a').read_text(),'source');self.assertEqual((Path(result['backup'])/'a').read_text(),'old');self.assertFalse(result['services_started'])
 def test_source_corruption_refused(self):
  b,t=self.fixture();(b/'a').write_text('bad')
  with self.assertRaises(ValueError):copy.restore(b,t,True)
 def test_symlink_target_refused(self):
  b,t=self.fixture();t.mkdir();(t/'a').symlink_to(b/'a')
  with self.assertRaises(ValueError):copy.restore(b,t,True)
if __name__=='__main__':unittest.main(verbosity=2)
