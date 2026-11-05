import unittest,struct,tempfile,copy,json
from pathlib import Path
from dataclasses import replace
from release_policy import ReleasePolicy,TrialLimiter,feedback_speed_exceeded
from raw_odom_mode import RawReturnRuntime
from test_full_trial import full_config
from test_release_policy import packet
from test_raw_mode import make_plan,conditions,snap

def config():
 c=full_config();c['feedback_speed_limit_m_s']=.4;return c
class Feedback040Tests(unittest.TestCase):
 def test_float32_boundary_and_next_value(self):
  boundary=struct.unpack('!f',struct.pack('!f',.4))[0];bits=struct.unpack('!I',struct.pack('!f',boundary))[0];above=struct.unpack('!f',struct.pack('!I',bits+1))[0]
  for v in [.2540000081062317,.35,.399,.4,boundary]:self.assertFalse(feedback_speed_exceeded(-v,.4))
  for v in [above,.401,.41]:self.assertTrue(feedback_speed_exceeded(-v,.4))
 def test_both_consumers_share_explicit_threshold(self):
  plan,t,now=make_plan();c=replace(conditions(plan),speed_limit_m_s=.15)
  with tempfile.TemporaryDirectory() as tmp:
   rt=RawReturnRuntime(plan,c,Path(tmp)/'ref.json',feedback_speed_limit_m_s=.4)
   for v,ok in [(.2540000081062317,True),(.35,True),(.4000000059604645,True),(.401,False)]:
    g=TrialLimiter(ReleasePolicy(config()));g.claim_trial=lambda *a:True
    out=g.filter('return',packet(short_test=False,route_length_m=1.),(-.15,0.),now,odom_pose=(0,0),odom_stamp=now,odom_age=0.,measured_velocity=(-v,0))
    self.assertEqual(out==(-.15,0),ok);s=snap(t,now);s['raw_odom']['velocity'][0]=-v
    if ok:rt.check_coordinates(s,now,True)
    else:
     with self.assertRaisesRegex(ValueError,'实测速度异常'):rt.check_coordinates(s,now,True)
 def test_target_speed_and_configuration_caps_remain(self):
  p=ReleasePolicy(config());self.assertTrue(p.enabled,p.errors);self.assertEqual(p.return_speed,.15);self.assertEqual(p.feedback_speed_limit,.4)
  for v in [.401,float('nan'),True]:
   c=config();c['feedback_speed_limit_m_s']=v;self.assertFalse(ReleasePolicy(c).enabled)
  g=TrialLimiter(p);g.claim_trial=lambda *a:True
  out=g.filter('return',packet(short_test=False,route_length_m=1.),(-.2,0),10.,odom_pose=(0,0),odom_stamp=10.,odom_age=0.,measured_velocity=(-.3,0));self.assertEqual(out,(0.,0.));self.assertIn('envelope',g.fault)
if __name__=='__main__':unittest.main(verbosity=2)
