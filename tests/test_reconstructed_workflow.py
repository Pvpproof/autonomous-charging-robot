"""Temporary-file-only reconstruction tests; no ROS or execution imports."""
import json,math,sys,tempfile,unittest
from pathlib import Path
from unittest.mock import patch
TOOLS=Path(__file__).resolve().parents[1]/'arm_stack/tools'
sys.path.insert(0,str(TOOLS))
import insertion_runtime as runtime
import workflow_progress as progress

class ReconstructionTests(unittest.TestCase):
 def setUp(self):
  self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup);self.root=Path(self.tmp.name)
  runtime.atomic(self.root/'workflow_cycle.json',{'cycle_id':'cycle'})
  self.state=progress.begin('camera',self.root,now=100.)
  self.plan={'stage':'camera','cycle_id':'cycle','plan_id':'new','planned_at':101.}
 def write(self,**changes):
  plan=dict(self.plan,**changes);runtime.atomic(self.root/'pending_trajectory.json',plan);return plan
 def check(self,validator=lambda data,root:None):return progress.inspect_attempt('camera',self.root,self.state['attempt_id'],validator,now=102.)
 def test_atomic_roundtrip_and_invalid_values_preserve_previous(self):
  p=self.root/'data.json';runtime.atomic(p,{'old':1})
  with self.assertRaises(ValueError):runtime.atomic(p,{'bad':float('nan')})
  self.assertEqual(json.loads(p.read_text()),{'old':1});self.assertFalse(list(self.root.glob('*.tmp')))
 def test_missing_parent_and_unknown_motion_api_refuse(self):
  with self.assertRaises(FileNotFoundError):runtime.atomic(self.root/'missing/a.json',{})
  with self.assertRaises(AttributeError):getattr(runtime,'execute')
 def test_begin_preserves_old_trajectory_but_never_accepts_it(self):
  p=self.write(planned_at=99.);self.assertIsNone(self.check());self.assertEqual(json.loads((self.root/'pending_trajectory.json').read_text()),p)
 def test_valid_current_plan_requires_validator(self):
  self.write();seen=[];result=self.check(lambda data,root:seen.append(data['plan_id']));self.assertEqual(seen,['new']);self.assertEqual(result['plan_id'],'new')
 def test_validator_failure_is_not_success(self):
  self.write()
  def refuse(*a):raise RuntimeError('eligibility failed')
  with self.assertRaisesRegex(RuntimeError,'eligibility'):self.check(refuse)
 def test_superseded_attempt_failed_attempt_and_cycle_change_refuse(self):
  self.write();progress.fail('cancelled',self.root)
  with self.assertRaises(RuntimeError):self.check()
  progress.begin('camera',self.root,now=100.)
  with self.assertRaises(RuntimeError):self.check()
  runtime.atomic(self.root/'workflow_cycle.json',{'cycle_id':'other'})
  with self.assertRaises(RuntimeError):self.check()
 def test_wrong_stage_cycle_and_future_time_refuse(self):
  for changes in ({'stage':'gun'},{'cycle_id':'other'},{'planned_at':103.},{'planned_at':True},{'plan_id':''}):
   self.write(**changes)
   with self.assertRaises(RuntimeError):self.check()
 def test_mutation_during_validation_refused(self):
  self.write()
  with self.assertRaisesRegex(RuntimeError,'changed'):self.check(lambda *a:self.write(plan_id='replaced'))
 def test_bounded_wait_timeout(self):
  clock=[0.]
  def sleep(dt):clock[0]+=dt
  with self.assertRaisesRegex(RuntimeError,'timed out'):progress.wait('camera',self.root,timeout=.3,clock=lambda:clock[0],sleep=sleep)
  self.assertLessEqual(clock[0],.31)
 def test_missing_environment_and_malformed_files_refuse(self):
  with patch.dict('os.environ',{},clear=True):
   with self.assertRaises(RuntimeError):progress.root_path()
  (self.root/'pending_trajectory.json').write_text('{')
  with self.assertRaises(ValueError):self.check()
 def test_unknown_cli_command_and_missing_status_nonzero(self):
  with self.assertRaises(SystemExit):progress.main(['execute'])
  with patch.dict('os.environ',{'CHARGING_WORKFLOW_ROOT':str(self.root)}):self.assertEqual(progress.main(['missing']),2)

if __name__=='__main__':unittest.main(verbosity=2)
