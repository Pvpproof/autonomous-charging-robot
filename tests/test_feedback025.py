import unittest,tempfile,copy,struct
from pathlib import Path
from dataclasses import replace
from release_policy import ReleasePolicy,TrialLimiter
from raw_odom_mode import RawReturnRuntime
from test_full_trial import full_config
from test_release_policy import packet,config
from test_raw_mode import make_plan,conditions,snap

def relaxed_config():
 c=full_config();c['feedback_speed_limit_m_s']=.25;return c
class Feedback025Tests(unittest.TestCase):
 def output(self,measured,command=(-.15,0.)):
  g=TrialLimiter(ReleasePolicy(relaxed_config()));g.claim_trial=lambda *args:True
  out=g.filter('return',packet(short_test=False,route_length_m=1.),command,10.,odom_pose=(0,0),odom_stamp=10.,odom_age=0.,measured_velocity=(measured,0.))
  return out,g
 def test_configuration_is_bounded_and_default_short_unchanged(self):
  p=ReleasePolicy(relaxed_config());self.assertTrue(p.enabled,p.errors);self.assertEqual(p.return_speed,.15);self.assertEqual(p.feedback_speed_limit,.25)
  self.assertEqual(ReleasePolicy(full_config()).feedback_speed_limit,.18)
  self.assertEqual(ReleasePolicy(config()).feedback_speed_limit,.06)
  for v in [float('nan'),True,.10,.41]:
   c=relaxed_config();c['feedback_speed_limit_m_s']=v;self.assertFalse(ReleasePolicy(c).enabled)
  c=config();c['feedback_speed_limit_m_s']=.25;self.assertFalse(ReleasePolicy(c).enabled)
 def test_independent_accepts_old_boundary_and_stops_real_new_excess(self):
  field_value=struct.unpack('f',struct.pack('f',.18))[0]
  for speed in [field_value,.181,.20,.249,.25]:
   out,g=self.output(-speed);self.assertEqual(out,(-.15,0.));self.assertIsNone(g.fault)
  for speed in [.251,.26,.30]:
   out,g=self.output(-speed);self.assertEqual(out,(0.,0.));self.assertIn('measured return speed exceeded',g.fault)
 def test_runtime_uses_same_bound(self):
  plan,t,now=make_plan();c=replace(conditions(plan),speed_limit_m_s=.15)
  with tempfile.TemporaryDirectory() as tmp:
   rt=RawReturnRuntime(plan,c,Path(tmp)/'map.json',feedback_speed_limit_m_s=.25)
   for v in [-.18000000715255737,-.20,-.25]:
    s=snap(t,now);s['raw_odom']['velocity'][0]=v;rt.check_coordinates(s,now,True)
   s=snap(t,now);s['raw_odom']['velocity'][0]=-.251
   with self.assertRaisesRegex(ValueError,'实测速度异常'):rt.check_coordinates(s,now,True)
 def test_relaxed_feedback_does_not_raise_command_limit_or_allow_forward_motion(self):
  for command,measured in [((-.151,0.),-.20),((-.20,0.),-.20),((-.15,.09),-.20),((-.15,0.),.02)]:
   out,g=self.output(measured,command);self.assertEqual(out,(0.,0.));self.assertIsNotNone(g.fault)
if __name__=='__main__':unittest.main(verbosity=2)
