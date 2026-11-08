"""The uncalibrated path classifier is gone; integrity checks stay. Synthetic only."""
import inspect,tempfile,unittest
from pathlib import Path
import raw_odom_mode as raw
from _recording_helpers import session_fixture,move,finish
from test_raw_mode import make_plan,conditions,snap
class ModelRemovalTests(unittest.TestCase):
 def test_no_path_classifier_or_warning_remains(self):
  s=inspect.getsource(raw)
  for token in ('HeadingCone','path_heading_cone','HEADING_OBSERVATION_BUDGET','curvature_diagnostic','超出现有模型'):self.assertNotIn(token,s)
 def test_tight_complete_route_can_create_candidate(self):
  with tempfile.TemporaryDirectory() as tmp:
   t,now,s=session_fixture(tmp);now=move(t,now,s,.042,.09,50);finish(t,now,s);self.assertTrue(s.return_eligible,s.status());raw.check_raw_plan(s.return_candidate());self.assertNotIn('模型',s.status())
 def test_tight_plan_can_prepare_without_changing_controller(self):
  p,t,now=make_plan(curve=1.)
  with tempfile.TemporaryDirectory() as tmp:
   raw.check_raw_plan(p);r=raw.RawReturnRuntime(p,conditions(p),Path(tmp)/'start.json');self.assertEqual(r.phase,'ready')
 def test_real_feedback_loss_still_invalidates_recording(self):
  with tempfile.TemporaryDirectory() as tmp:
   t,now,s=session_fixture(tmp);now=move(t,now,s,.042,.09,30);v=snap(t,now);v['raw_odom']['received_at']-=1;s.update(v,'meta',now);self.assertEqual(s.phase,'blocked');self.assertFalse(s.return_eligible)
 def test_incomplete_route_does_not_gain_return_qualification(self):
  with tempfile.TemporaryDirectory() as tmp:
   t,now,s=session_fixture(tmp);move(t,now,s,.042,.09,30)
   with self.assertRaises(ValueError):s.return_candidate()
 def test_reverse_outbound_is_not_assigned_to_reverse_only_follower(self):
  with tempfile.TemporaryDirectory() as tmp:
   t,now,s=session_fixture(tmp);now=move(t,now,s,-.1,0.,30);finish(t,now,s);self.assertFalse(s.return_eligible);self.assertEqual(s.return_qualification['code'],'outbound_reverse')
if __name__=='__main__':unittest.main(verbosity=2)
