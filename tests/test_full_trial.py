import unittest,copy
from release_policy import ReleasePolicy,TrialLimiter
from test_release_policy import config,packet

def full_config():
 c=config('supervised_trial');c['return_speed_m_s']=.15
 c['full_trial_authorization']=dict(authorization_id='abcdef0123456789',operator_approved=True,one_run=True,allow_full_route=True,user_selected_speed_m_s=.15,max_route_m=3.,max_time_s=90.)
 return c
class FullTrialTests(unittest.TestCase):
 def tick(self,g,t=10.,pose=(0.,0.),**kwargs):
  p=packet(short_test=False,route_length_m=.3);p.update(kwargs.pop('p',{}))
  return g.filter('return',p,kwargs.pop('command',(-.15,0.)),t,odom_pose=pose,odom_stamp=t,odom_age=0.,measured_velocity=(-.15,0.))
 def guard(self):
  g=TrialLimiter(ReleasePolicy(full_config()));g.claim_trial=lambda *args:True;return g
 def test_full_trial_is_explicit_authorization_not_fabricated_field_acceptance(self):
  c=full_config();p=ReleasePolicy(c);self.assertTrue(p.enabled,p.errors);self.assertNotIn('full_route_acceptance_receipt',c)
  for field in ['authorization_id','operator_approved','one_run','allow_full_route','user_selected_speed_m_s','max_route_m','max_time_s']:
   bad=copy.deepcopy(c);del bad['full_trial_authorization'][field];self.assertFalse(ReleasePolicy(bad).enabled,field)
  for key,value in [('max_route_m',30.),('max_time_s',900.),('authorization_id','../bad-path'),('user_selected_speed_m_s',.3)]:
   bad=copy.deepcopy(c);bad['full_trial_authorization'][key]=value;self.assertFalse(ReleasePolicy(bad).enabled)
 def test_claim_required_before_motion_and_zero_acquisition_does_not_consume(self):
  g=self.guard();calls=[];g.claim_trial=lambda *args:calls.append(args) or True
  self.assertEqual(self.tick(g,command=(0.,0.)),(0.,0.));self.assertEqual(calls,[])
  self.assertEqual(self.tick(g,10.05),(-.15,0.));self.assertEqual(len(calls),1);self.assertTrue(g.full_trial_consumed)
  self.tick(g,10.1);self.assertEqual(len(calls),1)
  g=self.guard();g.claim_trial=None;self.assertEqual(self.tick(g),(0.,0.));self.assertIn('claim unavailable',g.fault)
  g=self.guard();g.claim_trial=lambda *args:False;self.assertEqual(self.tick(g),(0.,0.));self.assertIn('consumed',g.fault)
 def test_reset_cannot_reuse_one_full_trial_or_change_route(self):
  g=self.guard();self.tick(g);g.filter('stop',{},(0.,0.),10.05,odom_pose=None,odom_stamp=0,odom_age=0,measured_velocity=(0,0));g.reset(quiet=True,explicit_reset=True)
  self.assertEqual(self.tick(g,10.1,p={'trial_id':'new'}),(0.,0.));self.assertIn('consumed',g.fault)
  g=self.guard();self.tick(g);self.assertEqual(self.tick(g,10.05,p={'route_length_m':.4}),(0.,0.));self.assertIn('length changed',g.fault)
 def test_full_trial_has_route_and_time_watchdogs_instead_of_short_limits(self):
  g=self.guard()
  for i in range(80):out=self.tick(g,10+i*.05,pose=(-i*.005,0.))
  self.assertIsNone(g.fault);self.assertEqual(out,(-.15,0.));self.assertGreater(g.distance,.15)
  for i in range(80,120):
   out=self.tick(g,10+i*.05,pose=(-i*.005,0.))
   if g.fault:break
  self.assertIn('watchdog',g.fault);self.assertEqual(out,(0.,0.))
  c=full_config();c['full_trial_authorization']['max_time_s']=1.;g=TrialLimiter(ReleasePolicy(c));g.claim_trial=lambda *args:True
  for i in range(25):out=self.tick(g,10+i*.05)
  self.assertIn('watchdog',g.fault)
if __name__=='__main__':unittest.main(verbosity=2)
