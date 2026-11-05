import unittest,copy
from driver_watchdog_client import DriverWatchdogState

def state(now=10.,**kw):
 d=dict(version=2,session='12345678-1234-1234-1234-123456789abc',state_seq=1,latched=True,reason='startup_reset_required',monotonic_ns=int(now*1e9),command_age_s=-1.,feedback_age_s=-1.)
 d.update(kw);return d
class DriverStateTests(unittest.TestCase):
 def test_static_contract_allows_startup_lock_but_motion_never_does(self):
  d=DriverWatchdogState();d.observe(state(),10.);self.assertTrue(d.contract_ready(10.));self.assertFalse(d.motion_ready(10.))
  d.observe(state(10.05,state_seq=2,latched=False,command_age_s=.01,feedback_age_s=.01),10.05);self.assertTrue(d.motion_ready(10.05))
 def test_old_ready_source_timestamp_cannot_look_fresh_on_arrival(self):
  with self.assertRaisesRegex(ValueError,'stale'):DriverWatchdogState().observe(state(),10.251)
 def test_duplicate_state_does_not_renew_freshness(self):
  d=DriverWatchdogState();d.observe(state(),10.);d.observe(state(10.2),10.2);self.assertFalse(d.contract_ready(10.251))
 def test_reset_or_new_session_is_distinct(self):
  d=DriverWatchdogState();self.assertFalse(d.observe(state(),10.));self.assertTrue(d.observe(state(10.1,session='f'*32),10.1))
 def test_invalid_contract_rejected(self):
  for kw in [dict(version=1),dict(session='bad'),dict(state_seq=True),dict(latched=1),dict(monotonic_ns=0),dict(command_age_s=float('nan')),dict(latched=False)]:
   with self.assertRaises(ValueError):DriverWatchdogState().observe(state(**kw),10.)
if __name__=='__main__':unittest.main(verbosity=2)
