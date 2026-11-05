import os
os.environ['QT_QPA_PLATFORM']='offscreen'
import importlib
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch
from PyQt5 import QtWidgets, QtCore, QtTest
import app as panel

QT=QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

class PanelTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.old=panel.ROOT;panel.ROOT=Path(self.tmp.name)
        self.window=panel.Window(offline=True)
    def tearDown(self):
        QT.removeEventFilter(self.window.return_hold)
        self.window.allow_close=True;self.window.close();self.window.deleteLater()
        QtCore.QCoreApplication.sendPostedEvents(None,QtCore.QEvent.DeferredDelete)
        QT.processEvents();panel.ROOT=self.old;self.tmp.cleanup()
    def test_full_trial_displays_selected_speed_and_disables_short_toggle(self):
        self.window.latest['arbiter']={'release_profile':'supervised_trial','return_speed_m_s':.15,'commissioned':True}
        self.window.update_return_ui()
        self.assertFalse(self.window.return_short.isChecked())
        self.assertFalse(self.window.return_short.isEnabled())
        self.assertIn('0.15',self.window.return_note.text())
    def test_offline_return_disabled_and_default_short_bound(self):
        self.assertFalse(self.window.return_button.isEnabled())
        self.assertFalse(self.window.return_hold.isEnabled())
        self.assertTrue(self.window.return_short.isChecked())
        self.assertNotIn('ros_bridge',sys.modules)
        self.assertIn('无后向自动避障',self.window.return_note.text())
    def test_click_start_once_release_keeps_running_and_deactivate_cancels(self):
        pad=self.window.return_hold;pad.setEnabled(True);starts=[];stops=[]
        pad.start_requested.disconnect();pad.cancel_requested.disconnect()
        pad.start_requested.connect(lambda:starts.append('start'))
        pad.cancel_requested.connect(lambda:stops.append('stop'))
        pad.button.click();pad.button.click()
        self.assertEqual(starts,['start']);self.assertTrue(pad.running)
        pad.eventFilter(pad.button,QtCore.QEvent(QtCore.QEvent.FocusOut))
        self.assertTrue(pad.running)
        pad.eventFilter(self.window,QtCore.QEvent(QtCore.QEvent.ApplicationDeactivate))
        self.assertFalse(pad.running);self.assertEqual(stops,['stop'])
    def test_reset_ui_waits_for_ack_instead_of_reporting_unlocked(self):
        class Stub:
            def release_stop(self):pass
            def snapshot(self):return dict(software_stop=True,stop_reset_pending=True,velocity=(0.,0.),odom_age=0.)
        self.window.bridge=Stub();self.window.offline=False
        try:
            with patch.object(self.window,'confirm',return_value=True):self.window.release_stop()
            self.assertTrue(self.window.latest['software_stop'])
            self.assertFalse(self.window.release_stop_button.isEnabled())
            self.assertIn('等待解除确认',self.window.stop_note.text())
        finally:self.window.bridge=None;self.window.offline=True
    def test_short_only_cannot_uncheck_short_trial(self):
        self.window.latest=dict(arbiter={'release_profile':'short_only','commissioned':True})
        self.window.return_short.setChecked(False);self.window.update_return_ui()
        self.assertTrue(self.window.return_short.isChecked())
        self.assertFalse(self.window.return_short.isEnabled())
    def test_emergency_button_stays_available_during_return(self):
        seen=[]
        class Stub:
            def emergency_stop(self):seen.append('stop')
        self.window.bridge=Stub();self.window.offline=False
        self.window.latest=dict(return_active=True,return_state={'phase':'running'},arbiter={'commissioned':True,'return_start_mode':'click'})
        self.window.update_return_ui()
        self.assertTrue(self.window.emergency_button.isEnabled())
        try:
            self.window.emergency_button.click()
            self.assertEqual(seen,['stop'])
        finally:
            self.window.bridge=None;self.window.offline=True
    def test_active_return_disables_conflicting_controls(self):
        self.window.latest=dict(return_active=True,return_state={'phase':'running'},arbiter={'commissioned':True,'return_start_mode':'click'})
        self.window.update_return_ui()
        self.assertTrue(self.window.return_hold.isEnabled())
        for button in [self.window.manual_button,self.window.go_button,self.window.apply_initial_button,
                       self.window.disconnect_button,self.window.docking_button]:self.assertFalse(button.isEnabled())

if __name__=='__main__':unittest.main(verbosity=2)
