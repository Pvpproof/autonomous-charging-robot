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


class ReturnStartPad(QtWidgets.QWidget):
    """A click requests start once; release does not cancel. Hiding stops it."""
    start_requested=QtCore.pyqtSignal()
    cancel_requested=QtCore.pyqtSignal()

    def __init__(self,parent=None):
        super().__init__(parent);self.running=False
        layout=QtWidgets.QVBoxLayout(self)
        self.button=QtWidgets.QPushButton('点击启动沿实测轨迹返程')
        self.button.setAutoRepeat(False);layout.addWidget(self.button)
        self.button.clicked.connect(self._start)
        QtWidgets.QApplication.instance().installEventFilter(self)

    def _start(self):
        if not self.running:
            self.set_running(True)
            self.start_requested.emit()

    def set_running(self,value):
        self.running=bool(value)
        self.button.setEnabled(not self.running)
        self.button.setText('返程运行中 · 点击停止返程可取消' if self.running else '点击启动沿实测轨迹返程')

    def eventFilter(self,watched,event):
        if self.running and (event.type()==QtCore.QEvent.ApplicationDeactivate or
                (event.type() in (QtCore.QEvent.Hide,QtCore.QEvent.Close) and watched==self.window())):
            self.cancel_requested.emit()
            self.set_running(False)
        return False
