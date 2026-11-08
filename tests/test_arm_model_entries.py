"""No ROS launch is executed: fake ros2 records wrapper argv."""
import ast,importlib.util,os,subprocess,tempfile,unittest
from pathlib import Path
TOOLS=Path(__file__).resolve().parents[1]/'arm_stack/tools'
# Extract pure validation without importing launch/ROS modules.
source=ast.parse((TOOLS/'nova2_real_model.launch.py').read_text())
ns={'os':os,'Path':Path,'ET':__import__('xml.etree.ElementTree',fromlist=['ElementTree'])}
fn=next(n for n in source.body if isinstance(n,ast.FunctionDef) and n.name=='model_configuration')
exec(compile(ast.Module(body=[fn],type_ignores=[]),'<model-validation>','exec'),ns)
class ModelTests(unittest.TestCase):
 def setUp(self):
  self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup);self.root=Path(self.tmp.name);self.urdf=self.root/'synthetic.urdf';self.urdf.write_text('<robot name="synthetic"><link name="base_link"/></robot>')
 def test_explicit_model_and_feedback(self):
  text,topic=ns['model_configuration']({'CHARGING_ARM_URDF':str(self.urdf)});self.assertIn('synthetic',text);self.assertEqual(topic,'/joint_states')
 def test_missing_or_relative_model_refused(self):
  for env in ({},{'CHARGING_ARM_URDF':'relative'}):
   with self.assertRaises(RuntimeError):ns['model_configuration'](env)
 def test_malformed_and_unexpanded_xacro_refused(self):
  for text in ('<wrong/>','<robot xmlns:xacro="xacro"><link name="a"/><xacro:include/></robot>'):
   self.urdf.write_text(text)
   with self.assertRaises(ValueError):ns['model_configuration']({'CHARGING_ARM_URDF':str(self.urdf)})
 def test_model_launch_has_no_static_site_tf_or_joint_generator(self):
  s=(TOOLS/'nova2_real_model.launch.py').read_text();self.assertNotIn('static_transform_publisher',s);self.assertNotIn('joint_state_publisher',s)
 def test_wrappers_only_dispatch_expected_nonexecution_args(self):
  setup=self.root/'setup.bash';setup.write_text('export PATH="'+str(self.root)+':$PATH"\n')
  fake=self.root/'ros2';fake.write_text('#!/bin/sh\nprintf "%s\\n" "$@" > "$CAPTURE"\n');fake.chmod(0o755)
  capture=self.root/'args';env=dict(os.environ,CHARGING_ARM_SETUP=str(setup),CHARGING_ARM_URDF=str(self.urdf),CAPTURE=str(capture))
  for script in ('start_nova2_real_model.sh','start_nova2_moveit_plan_only.sh'):
   result=subprocess.run(['bash',str(TOOLS/script)],env=env,capture_output=True,text=True);self.assertEqual(result.returncode,0,result.stderr);args=capture.read_text().splitlines()
   if 'moveit' in script:self.assertEqual(args,['launch','nova2_moveit','move_group.launch.py','allow_trajectory_execution:=false'])
   else:self.assertEqual(args,['launch',str(TOOLS/'nova2_real_model.launch.py')])
 def test_wrappers_require_explicit_setup(self):
  for script in ('start_nova2_real_model.sh','start_nova2_moveit_plan_only.sh'):
   result=subprocess.run(['bash',str(TOOLS/script)],env={'PATH':os.environ['PATH']},capture_output=True,text=True);self.assertNotEqual(result.returncode,0)
if __name__=='__main__':unittest.main(verbosity=2)
