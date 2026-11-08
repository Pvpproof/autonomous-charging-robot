import unittest,pathlib,sys,copy,math
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parents[1]/'arm_stack/tools'))
from arm_readiness import Alignment,JOINTS,joint_values
class AlignmentTests(unittest.TestCase):
 def setUp(self):
  self.g=Alignment();self.s=dict(valid=True,source_session='a',controller_timer=1,source_progress_monotonic_ns=10000000000);self.q=dict.fromkeys(JOINTS,1.)
 def accept(self,scene=None,**kwargs):
  data=dict(feedback=self.s,joints=self.q,joints_received=10.,scene=self.q if scene is None else scene,query_started=10.,query_finished=10.,request_session='a',now=10.);data.update(kwargs);return self.g.accept(**data)
 def advance(self):self.s['controller_timer']+=1
 def test_service_present_initial_zero_state_never_ready(self):
  for _ in range(5):self.advance();self.assertFalse(self.accept(dict.fromkeys(JOINTS,0.)))
  self.assertEqual(self.g.count,0)
 def test_requires_three_distinct_source_ticks_after_alignment(self):
  self.assertFalse(self.accept());self.advance();self.assertFalse(self.accept());self.advance();self.assertTrue(self.accept())
 def test_duplicate_tick_cannot_accumulate_success(self):
  for _ in range(10):self.assertFalse(self.accept())
  self.assertEqual(self.g.count,1)
 def test_stale_joints_source_or_scene_reply_refused(self):
  for args in [dict(joints_received=9.),dict(query_started=9.),dict(query_finished=11.),dict(now=11.)]:
   self.g=Alignment();self.assertFalse(self.accept(**args));self.assertEqual(self.g.count,0)
 def test_mismatch_resets_consecutive_samples(self):
  self.accept();self.advance();self.accept(dict.fromkeys(JOINTS,0.));self.advance();self.assertFalse(self.accept());self.assertEqual(self.g.count,1)
 def test_invalid_feedback_and_missing_or_nan_scene_refused(self):
  self.s['valid']=False;self.assertFalse(self.accept());self.s['valid']=True
  self.assertFalse(self.accept({}));self.assertFalse(self.accept(dict.fromkeys(JOINTS,float('nan'))))
 def test_session_change_latches_without_auto_resume(self):
  self.accept();self.s['source_session']='b';self.assertFalse(self.accept(request_session='b'));self.assertIsNotNone(self.g.fault)
  self.s['source_session']='a';self.advance();self.assertFalse(self.accept())
 def test_source_timer_regression_latches(self):
  self.accept();self.s['controller_timer']=0;self.assertFalse(self.accept());self.assertIsNotNone(self.g.fault)
 def test_cross_session_query_is_refused(self):self.assertFalse(self.accept(request_session='old'))
 def test_joint_parser_rejects_duplicates_missing_and_nonfinite(self):
  for names,positions in [(JOINTS[:-1],[0]*5),(JOINTS+(JOINTS[0],),[0]*7),(JOINTS,[float('nan')]*6)]:
   with self.assertRaises(ValueError):joint_values(names,positions)
 def test_revolute_angles_are_not_silently_wrapped(self):
  other=dict(self.q);other['joint6']+=2*math.pi;self.assertFalse(self.accept(other))
if __name__=='__main__':unittest.main(verbosity=2)
