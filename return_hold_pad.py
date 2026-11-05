"""Hold UI component; emits state only and never connects to a robot."""
from PyQt5 import QtCore, QtWidgets


class HoldPad(QtWidgets.QWidget):
    held_changed=QtCore.pyqtSignal(bool)

    def __init__(self,parent=None):
        super().__init__(parent);self.held=False
        layout=QtWidgets.QVBoxLayout(self)
        self.button=QtWidgets.QPushButton('按住自动沿实测路线倒车 · 松开即停止')
        self.button.setAutoRepeat(False);layout.addWidget(self.button)
        self.button.pressed.connect(lambda:self.set_held(True))
        self.button.released.connect(lambda:self.set_held(False))
        QtWidgets.QApplication.instance().installEventFilter(self)

    def set_held(self,value):
        value=bool(value)
        if value!=self.held:self.held=value;self.held_changed.emit(value)

    def eventFilter(self,watched,event):
        if (event.type()==QtCore.QEvent.ApplicationDeactivate or
            (event.type() in (QtCore.QEvent.WindowDeactivate,QtCore.QEvent.Hide,QtCore.QEvent.Close) and watched==self.window()) or
            (event.type()==QtCore.QEvent.FocusOut and watched==self.button)):
            self.set_held(False)
        return False

    def closeEvent(self,event):
        self.set_held(False);super().closeEvent(event)
