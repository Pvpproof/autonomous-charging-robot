import math,unittest
from charging_robot.returning.release_policy import ReleasePolicy, TrialLimiter

def config(profile='short_only'):
 return dict(profile=profile,deployment_id='test-deployment',physical_stop_evidence={'user_confirmed':True,'agent_tested':False},
  site_authorization={k:True for k in ['operator_present','route_cleared','arm_stowed','other_controls_handed_over']})
def packet(**change):
 p=dict(owner='panel',trial_id='trial',cycle_id='cycle',goal_id='goal',held=True,site_confirmed=True,short_test=True);p.update(change);return p
class ReleaseTests(unittest.TestCase):
 def guard(self):return TrialLimiter(ReleasePolicy(config()))
 def tick(self,g,now=10.,pose=(0.,0.),stamp=None,p=None,command=(-.05,0.),velocity=(-.05,0.),age=0.,mode='return'):
  return g.filter(mode,packet() if p is None else p,command,now,odom_pose=pose,odom_stamp=now if stamp is None else stamp,odom_age=age,measured_velocity=velocity)
 def test_old_global_boolean_cannot_enable_motion(self):
  p=ReleasePolicy({'commissioned':True});self.assertFalse(p.enabled)
 def test_default_and_malformed_configs_stay_disabled(self):
  for c in [{},None,[],{'profile':'unknown'},dict(config(),site_authorization='wrong')]:self.assertFalse(ReleasePolicy(c).enabled)
 def test_user_confirmed_physical_stop_is_not_claimed_agent_tested(self):
  c=config();self.assertTrue(ReleasePolicy(c).enabled);self.assertFalse(c['physical_stop_evidence']['agent_tested'])
 def test_full_route_requires_matching_real_acceptance(self):
  c=config('supervised_route');self.assertFalse(ReleasePolicy(c).enabled)
  c['full_route_acceptance_receipt']={k:True for k in ['short_trial_passed','hold_release_passed','software_stop_passed','producer_loss_passed','command_silence_stop_passed','operator_approved']}
  c['full_route_acceptance_receipt']['deployment_id']='test-deployment';self.assertTrue(ReleasePolicy(c).enabled)
  c['full_route_acceptance_receipt']['synthetic']=True;self.assertFalse(ReleasePolicy(c).enabled)
 def test_short_only_rejects_full_route_even_with_valid_hold(self):
  g=self.guard();self.assertEqual(self.tick(g,p=packet(short_test=False)),(0.,0.));self.assertIn('full-route',g.fault)
 def test_independent_three_second_limit_cannot_be_extended(self):
  g=self.guard()
  for i in range(62):out=self.tick(g,10.+i*.05,pose=(0.,0.))
  self.assertEqual(out,(0.,0.));self.assertIn('short trial limit',g.fault)
 def test_independent_distance_can_stop_before_time_limit(self):
  g=self.guard()
  for i in range(35):out=self.tick(g,10.+i*.05,pose=(-i*.005,0.))
  self.assertEqual(out,(0.,0.));self.assertGreaterEqual(g.distance,.15);self.assertLess(1.75,3.)
 def test_hold_release_missing_site_and_invalid_ids_latch(self):
  for p in [packet(held=False),packet(site_confirmed=False),packet(trial_id='')]:
   g=self.guard();self.assertEqual(self.tick(g,p=p),(0.,0.));self.assertIsNotNone(g.fault)
 def test_consumed_trial_cannot_resume_after_stop_and_reset(self):
  g=self.guard();self.tick(g);self.tick(g,10.05,mode='stop',command=(0.,0.))
  self.assertTrue(g.reset(quiet=True,explicit_reset=True));self.assertEqual(self.tick(g,10.1),(0.,0.));self.assertIn('consumed',g.fault)
 def test_arbiter_own_loop_stall_is_not_refreshed_by_producer(self):
  g=self.guard();self.tick(g);self.assertEqual(self.tick(g,10.3),(0.,0.));self.assertIn('loop stalled',g.fault)
 def test_odom_loss_jump_backwards_and_duplicate_mutation_refuse(self):
  for fault in ['loss','jump','backwards','duplicate']:
   g=self.guard();self.tick(g)
   if fault=='loss':out=self.tick(g,10.05,age=.3)
   if fault=='jump':out=self.tick(g,10.05,pose=(-.1,0.))
   if fault=='backwards':out=self.tick(g,10.05,stamp=9.)
   if fault=='duplicate':out=self.tick(g,10.05,pose=(-.001,0.),stamp=10.)
   self.assertEqual(out,(0.,0.));self.assertIsNotNone(g.fault)
 def test_speed_turn_envelope_and_wrong_motion_direction_refuse(self):
  for command,velocity in [((-.1,0.),(-.05,0.)),((-.05,.1),(-.05,0.)),((.01,0.),(-.05,0.)),((-.05,0.),(.03,0.)),((-.05,0.),(-.08,0.))]:
   g=self.guard();self.assertEqual(self.tick(g,command=command,velocity=velocity),(0.,0.));self.assertIsNotNone(g.fault)
 def test_zero_handoff_does_not_consume_motion_time(self):
  g=self.guard()
  for i in range(30):self.tick(g,10.+i*.05,command=(0.,0.))
  self.assertIsNone(g.started_at);self.assertEqual(self.tick(g,11.5),(-.05,0.))
 def test_same_trial_identity_cannot_change_mid_motion(self):
  g=self.guard();self.tick(g);self.assertEqual(self.tick(g,10.05,p=packet(goal_id='other')),(0.,0.))
 def test_explicit_quiet_reset_required(self):
  g=self.guard();g.fail('test');self.assertFalse(g.reset(quiet=False,explicit_reset=True));self.assertIsNotNone(g.fault)
if __name__=='__main__':unittest.main(verbosity=2)
