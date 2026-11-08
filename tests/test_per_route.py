"""Synthetic per-route policy; public configuration stays disabled."""
import unittest
from release_policy import ReleasePolicy,TrialLimiter
from test_release_policy import packet
def config():
    return dict(profile='recorded_route',return_start_mode='click',return_speed_m_s=.15,
                feedback_speed_limit_m_s=.4)


def intent(index=1,**extra):
    p=packet(short_test=False,route_length_m=2.,activation_mode='click',run_state='running',
             run_session=f'{index:032x}',run_sequence=1,held=False,cycle_id='cycle-'+str(index),
             goal_id='return-'+str(index),trial_id='run-'+str(index))
    p.update(extra);return p


class PerRoutePolicyTests(unittest.TestCase):
    def limiter(self):return TrialLimiter(ReleasePolicy(config(),isolated_test=True))
    def apply(self,g,p,now=10.,x=0.,v=0.):
        return g.filter('return',p,(-.15,0.),now,odom_pose=(x,0),odom_stamp=now,odom_age=0.,measured_velocity=(v,0.))
    def test_old_global_consumed_flag_and_missing_claim_callback_do_not_block_new_routes(self):
        g=self.limiter();g.full_trial_consumed=True
        self.assertIsNone(g.claim_trial);self.assertEqual(self.apply(g,intent()),(-.15,0.))
        g.filter('stop',{},(0.,0.),10.05,odom_pose=(0,0),odom_stamp=10.05,odom_age=0,measured_velocity=(0,0))
        self.assertEqual(self.apply(g,intent(2),10.1),(-.15,0.))
    def test_previous_run_packet_cannot_resume_after_stop(self):
        g=self.limiter();p=intent();self.apply(g,p)
        g.filter('stop',{},(0.,0.),10.05,odom_pose=(0,0),odom_stamp=10.05,odom_age=0,measured_velocity=(0,0))
        self.assertEqual(self.apply(g,p,10.1),(0.,0.));self.assertTrue(g.fault)
    def test_no_legacy_four_meter_trial_cap_but_route_length_must_be_valid_and_immutable(self):
        self.assertEqual(self.apply(self.limiter(),intent(route_length_m=5.)),(-.15,0.))
        for length in (float('nan'),0.,-.1,True):
            self.assertEqual(self.apply(self.limiter(),intent(route_length_m=length)),(0.,0.))
        g=self.limiter();self.apply(g,intent());self.assertEqual(self.apply(g,intent(route_length_m=3.,run_sequence=2),10.05),(0.,0.))
    def test_route_watchdog_keeps_independent_distance_boundary(self):
        g=self.limiter();p=intent(route_length_m=.1)
        for i in range(60):
            p['run_sequence']=i+1;out=self.apply(g,p,10+i*.05,x=-i*.0075,v=-.15)
            if g.fault:break
        self.assertEqual(out,(0.,0.));self.assertIn('route/time watchdog',g.fault)
    def test_stale_feedback_or_controller_lease_still_stops(self):
        g=self.limiter();p=intent();self.apply(g,p)
        for i in range(1,8):out=self.apply(g,p,10+i*.05)
        self.assertEqual(out,(0.,0.));self.assertIn('lease',g.fault)
        g=self.limiter();out=g.filter('return',intent(),(-.15,0.),10,odom_pose=(0,0),odom_stamp=9,odom_age=1,measured_velocity=(0,0))
        self.assertEqual(out,(0.,0.));self.assertTrue(g.fault)
    def test_short_mode_is_not_silently_used_for_daily_route(self):
        g=self.limiter();self.assertEqual(self.apply(g,intent(short_test=True)),(0.,0.))
    def test_velocity_envelope_is_not_relaxed(self):
        g=self.limiter();p=intent()
        self.assertEqual(g.filter('return',p,(-.16,0.),10,odom_pose=(0,0),odom_stamp=10,odom_age=0,measured_velocity=(0,0)),(0.,0.))

if __name__=="__main__":unittest.main(verbosity=2)
