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
    def test_hold_release_focus_loss_and_window_deactivation(self):
        hold=self.window.return_hold;hold.setEnabled(True)
        seen=[];hold.held_changed.connect(seen.append)
        hold.set_held(True)
        hold.eventFilter(hold.button,QtCore.QEvent(QtCore.QEvent.FocusOut))
        self.assertFalse(hold.held)
        hold.set_held(True)
        hold.eventFilter(self.window,QtCore.QEvent(QtCore.QEvent.WindowDeactivate))
        self.assertEqual(seen,[True,False,True,False])
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
        self.window.latest=dict(return_active=True,return_state={'phase':'running'},arbiter={'commissioned':True})
        self.window.update_return_ui()
        self.assertTrue(self.window.emergency_button.isEnabled())
        try:
            self.window.emergency_button.click()
            self.assertEqual(seen,['stop'])
        finally:
            self.window.bridge=None;self.window.offline=True
    def test_active_return_disables_conflicting_controls(self):
        self.window.latest=dict(return_active=True,return_state={'phase':'running'},arbiter={'commissioned':True})
        self.window.update_return_ui()
        self.assertTrue(self.window.return_hold.isEnabled())
        for button in [self.window.manual_button,self.window.go_button,self.window.apply_initial_button,
                       self.window.disconnect_button,self.window.docking_button]:self.assertFalse(button.isEnabled())

if __name__=='__main__':unittest.main(verbosity=2)
