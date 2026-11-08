"""Offscreen independent tools and base buttons, with all process launches faked."""
import json,unittest
from PyQt5 import QtCore
from _current_fixtures import WindowTests as Fixture
class PanelTests(Fixture):
 def test_initialization_does_not_launch_tools(self):
  self.assertEqual(self.launches,[]);self.assertIn('test',self.w.arm_test_button.text());self.assertIn('home',self.w.arm_home_button.text())
 def test_offline_return_is_disabled(self):
  self.w.offline=True;self.w.update_return_ui();self.assertFalse(self.w.return_button.isEnabled())
 def test_click_is_idempotent_and_focus_loss_does_not_cancel(self):
  p=self.w.return_hold;p.setEnabled(True);starts=[];stops=[];p.start_requested.disconnect();p.cancel_requested.disconnect();p.start_requested.connect(lambda:starts.append(1));p.cancel_requested.connect(lambda:stops.append(1));p.button.click();p.button.click();p.eventFilter(self.w,QtCore.QEvent(QtCore.QEvent.ApplicationDeactivate));self.assertEqual(starts,[1]);self.assertEqual(stops,[]);self.assertTrue(p.running)
 def test_close_cancels_click_return(self):
  p=self.w.return_hold;stops=[];p.cancel_requested.disconnect();p.cancel_requested.connect(lambda:stops.append(1));p.set_running(True);p.eventFilter(self.w,QtCore.QEvent(QtCore.QEvent.Close));self.assertEqual(stops,[1]);self.assertFalse(p.running)
 def test_active_return_disables_conflicting_controls(self):
  self.w.latest=dict(return_active=True,return_state={'phase':'running'},arbiter={'commissioned':True,'return_start_mode':'click'});self.w.update_return_ui()
  for b in (self.w.manual_button,self.w.go_button,self.w.apply_initial_button,self.w.disconnect_button,self.w.arm_test_button,self.w.arm_home_button):self.assertFalse(b.isEnabled())
 def test_stop_button_is_available_during_return(self):
  self.w.latest=dict(return_active=True,return_state={'phase':'running'},arbiter={'commissioned':True,'return_start_mode':'click'});self.w.update_return_ui();self.assertTrue(self.w.emergency_button.isEnabled())
if __name__=='__main__':unittest.main(verbosity=2)
