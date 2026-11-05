"""Panel input adapter using YCHD's installed keyboard direction table."""
from PyQt5 import QtCore, QtWidgets

# Steering follows the installed Ackermann planner's existing radius.
TURN_RADIUS_M = 1.78
SPEEDS_M_S = (.05, .10, .15)


def official_key(keys):
    forward = ('w' in keys) - ('s' in keys)
    turn = ('a' in keys) - ('d' in keys)
    if forward == 0:
        return None
    return {(1, 0): 'i', (1, 1): 'u', (1, -1): 'o',
            (-1, 0): ',', (-1, 1): 'm', (-1, -1): '.'}[(forward, turn)]


def velocity(keys, speed, bindings):
    key = official_key(keys)
    if key is None:
        return 0., 0.
    x, turn = bindings[key]
    return float(x * speed), float(turn * speed / TURN_RADIUS_M)


class KeyboardPad(QtWidgets.QLabel):
    released = QtCore.pyqtSignal(str)
    keymap = {QtCore.Qt.Key_W: 'w', QtCore.Qt.Key_Up: 'w',
              QtCore.Qt.Key_S: 's', QtCore.Qt.Key_Down: 's',
              QtCore.Qt.Key_A: 'a', QtCore.Qt.Key_Left: 'a',
              QtCore.Qt.Key_D: 'd', QtCore.Qt.Key_Right: 'd'}

    def __init__(self):
        super().__init__('键盘控制未开启')
        self.armed = False
        self.pressed = set()
        self.setFocusPolicy(QtCore.Qt.StrongFocus)
        self.setMinimumHeight(52)
        self.setAlignment(QtCore.Qt.AlignCenter)
        self.setStyleSheet('padding:8px;border:1px solid #36516c;border-radius:6px;')

    def arm(self):
        self.pressed.clear(); self.armed = True
        self.setText('W / ↑ 前进   S / ↓ 后退   配合 A/D 或 ←/→ 转向\n松键停车 · 空格 / Esc 退出 · 手动不自动避障')
        self.setStyleSheet('padding:8px;border:2px solid #14b8a6;border-radius:6px;background:#12413d;')
        self.setFocus(QtCore.Qt.OtherFocusReason)

    def disarm(self, reason):
        was_armed = self.armed
        self.armed = False; self.pressed.clear()
        self.setText('键盘控制未开启 · '+reason)
        self.setStyleSheet('padding:8px;border:1px solid #36516c;border-radius:6px;')
        if was_armed:self.released.emit(reason)

    def keyPressEvent(self, event):
        if not self.armed:return super().keyPressEvent(event)
        if event.isAutoRepeat():event.accept();return
        if event.key() in (QtCore.Qt.Key_Space, QtCore.Qt.Key_Escape) or event.modifiers() & (QtCore.Qt.ControlModifier | QtCore.Qt.AltModifier | QtCore.Qt.MetaModifier):
            self.disarm('已请求停车');event.accept();return
        if event.key() in self.keymap:
            self.pressed.add(event.key());event.accept();return
        self.pressed.clear();event.accept()

    def keyReleaseEvent(self, event):
        if not self.armed:return super().keyReleaseEvent(event)
        if event.isAutoRepeat():event.accept();return
        self.pressed.discard(event.key());event.accept()

    def keys(self):
        return {self.keymap[k] for k in self.pressed}

    def focusOutEvent(self, event):
        self.disarm('焦点已离开操控区')
        super().focusOutEvent(event)
