"""Historical arm publication checks; no node, camera or motion entry imported."""
import ast
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import numpy as np

ROOT=Path(__file__).resolve().parents[1]/'arm_stack/recovered'
spec=importlib.util.spec_from_file_location('publication_paths',ROOT/'publication_paths.py')
paths=importlib.util.module_from_spec(spec);spec.loader.exec_module(paths)

class PublicationTests(unittest.TestCase):
    def test_manifest_and_syntax(self):
        rows=json.loads((ROOT/'source-manifest.json').read_text())['sources']
        self.assertEqual(len(rows),8)
        for row in rows:
            data=(ROOT/row['file']).read_bytes();ast.parse(data.decode())
            self.assertEqual(hashlib.sha256(data).hexdigest(),row['published_sha256'])
            self.assertFalse(row['current_field_verified'])

    def test_missing_environment_refuses_without_fallback(self):
        with patch.dict(os.environ,{},clear=True):
            with self.assertRaises(RuntimeError):paths.vision_path()
            with self.assertRaises(RuntimeError):paths.hand_eye_path()

    def test_existing_explicit_paths(self):
        with tempfile.TemporaryDirectory() as tmp:
            f=Path(tmp)/'synthetic.npz';f.write_bytes(b'not calibration')
            with patch.dict(os.environ,CHARGING_SOCKET_VISION_ROOT=tmp,CHARGING_HAND_EYE_FILE=str(f)):
                self.assertEqual(paths.vision_path('results/test.json'),Path(tmp)/'results/test.json')
                self.assertEqual(paths.hand_eye_path(),f)

    def test_relative_or_missing_local_paths_refused(self):
        for value in ('relative-path','/nonexistent-synthetic-arm-path'):
            with patch.dict(os.environ,CHARGING_SOCKET_VISION_ROOT=value):
                with self.assertRaises(ValueError):paths.vision_path()

    def test_parent_and_absolute_suffix_refused(self):
        for suffix in ('../outside','/outside'):
            with self.assertRaises(ValueError):paths.vision_path(suffix)

    def test_no_personal_paths_or_calibration_payload(self):
        for p in ROOT.glob('*.py'):
            self.assertNotIn('/home/',p.read_text())
        self.assertFalse(list(ROOT.glob('*.npz')))
        self.assertFalse(list(ROOT.glob('*.ply')))

    def test_rotation_conventions_agree_without_importing_nodes(self):
        outputs=[]
        for name in ('plan_camera_pre_align_nearest_ik.py','icp_cloud_bridge.py','pnp_v3_pose_bridge.py'):
            tree=ast.parse((ROOT/name).read_text());fn=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='rot_dobot')
            ns={'np':np};exec(compile(ast.Module(body=[fn],type_ignores=[]),'<pure-rotation>','exec'),ns)
            output=ns['rot_dobot'](10,20,30);outputs.append(output)
            np.testing.assert_allclose(output.T@output,np.eye(3),atol=1e-12)
            self.assertAlmostEqual(np.linalg.det(output),1.)
        for output in outputs[1:]:np.testing.assert_allclose(output,outputs[0])

if __name__=='__main__':unittest.main(verbosity=2)
