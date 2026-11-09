"""Read-only source provenance and syntax checks; no device modules imported."""
import ast,hashlib,json,subprocess,unittest,xml.etree.ElementTree as ET
from pathlib import Path
REPO=Path(__file__).resolve().parents[1];ARM=REPO/'arm_stack'
MANIFEST=json.loads((ARM/'field-source-manifest.json').read_text())
class FieldSourceTests(unittest.TestCase):
 def test_every_public_copy_matches_manifest(self):
  for row in MANIFEST['files']:
   p=REPO/row['public_path'];self.assertEqual(hashlib.sha256(p.read_bytes()).hexdigest(),row['published_sha256'],str(p))
 def test_python_sources_parse_without_import(self):
  for row in MANIFEST['files']:
   p=REPO/row['public_path']
   if p.suffix=='.py':ast.parse(p.read_text(),filename=str(p))
 def test_shell_sources_syntax_only(self):
  for row in MANIFEST['files']:
   p=REPO/row['public_path']
   if p.suffix=='.sh':self.assertEqual(subprocess.run(['bash','-n',str(p)],capture_output=True).returncode,0,str(p))
 def test_originals_replace_reconstruction_markers(self):
  for name in ['workflow_progress.py','insertion_runtime.py','nova2_real_model.launch.py','start_nova2_real_model.sh','start_nova2_moveit_plan_only.sh']:
   self.assertNotIn('RECONSTRUCTED = True',(ARM/'tools'/name).read_text())
  self.assertFalse((ARM/'tools/insertion_limits.py').exists())
 def test_workflow_original_interface_and_timeout(self):
  t=ast.parse((ARM/'tools/workflow_progress.py').read_text());f={n.name:n for n in t.body if isinstance(n,ast.FunctionDef)}
  self.assertTrue({'begin','finish','missing','wait'}<=set(f));self.assertEqual(ast.literal_eval(f['wait'].args.defaults[-1]),100)
 def test_original_runtime_exports_runner_and_atomic(self):
  t=ast.parse((ARM/'tools/insertion_runtime.py').read_text());self.assertTrue(any(isinstance(n,ast.ClassDef) and n.name=='Runner' for n in t.body));self.assertTrue(any(isinstance(n,ast.FunctionDef) and n.name=='atomic' for n in t.body))
 def test_current_planner_depth_and_metadata(self):
  t=ast.parse((ARM/'tools/plan_insertion_cartesian.py').read_text());f=next(n for n in t.body if getattr(n,'name',None)=='plan');d=next(n for n in ast.walk(f) if isinstance(n,ast.Assign) and any(isinstance(x,ast.Name) and x.id=='depth' for x in n.targets));self.assertEqual(ast.literal_eval(d.value),.075)
  field=next(n for n in ast.walk(f) if isinstance(n,ast.keyword) and n.arg=='insertion_depth_mm');self.assertEqual(ast.literal_eval(field.value),75.)
 def test_template_is_the_approved_asset(self):
  p=ARM/'vision/model/socket_frame_and_rim_2mm_m.ply';text=(ARM/'vision/icp/realtime_socket_pose.py').read_text();self.assertIn(hashlib.sha256(p.read_bytes()).hexdigest(),text)
 def test_simulation_package_metadata_and_resources(self):
  p=ARM/'simulation/charging_dock_sim';xml=ET.parse(p/'package.xml').getroot();self.assertEqual(xml.findtext('license'),'Apache-2.0');self.assertTrue((p/'resource/charging_dock_sim').is_file())
  for name in ['auto_dock','plan_docking','scene_publisher']:self.assertTrue((p/'charging_dock_sim'/(name+'.py')).is_file())
 def test_no_embedded_private_calibration_or_device_address(self):
  model=(ARM/'tools/nova2_real_model.launch.py').read_text();self.assertNotIn('camera_frame_static_tf',model)
  for row in MANIFEST['files']:
   p=REPO/row['public_path']
   if p.suffix in ('.py','.sh','.json','.yaml'):self.assertNotRegex(p.read_text(),r'/home/nvidia/|192\.168\.\d+\.\d+|BEGIN .*PRIVATE KEY')
if __name__=='__main__':unittest.main(verbosity=2)
