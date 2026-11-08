"""Deterministic contention regression, no ROS executor or device."""
import unittest
from unittest.mock import patch
from _current_fixtures import Base
from test_raw_mode import make_plan
class ClockOrderingTests(unittest.TestCase):
 def test_tick_samples_time_after_obtaining_control_lock(self):
  p,t,now=make_plan();b=Base(t,now);clock=[now];b.busy=True;b.navigation_id='own';b.cancel_requested=False
  class Lock:
   def __enter__(self):clock[0]=now+.1;b.gui_at=now+.1
   def __exit__(self,*args):pass
  b.lock=Lock()
  with patch('charging_robot.returning.return_bridge.time.monotonic',side_effect=lambda:clock[0]):b.control_tick()
  self.assertEqual(b.packets[-1]['mode'],'navigation');self.assertFalse(b.software_stop);self.assertFalse(any(n=='ui_update_delayed' for n,_ in b.events))
if __name__=='__main__':unittest.main(verbosity=2)
