"""Independent terminal tools. Tool outcomes never grant or revoke a route."""
from charging_robot.paths import PROJECT_ROOT
import json
from pathlib import Path
import time
import uuid
from PyQt5 import QtCore, QtWidgets as W
from charging_robot.arm.arm_tools_guard import base_tool_reason, independent_tool_reason
from charging_robot.arm.arm_process_guard import active_arm_tools
from charging_robot.shared.model import write_json


class DetachedTool:
    def __init__(self,pid):self.pid=pid;self.active=True
    def state(self):return 2 if self.active else 0


class ArmToolsUi:
    def init_arm_tools(self):
        self.arm_process = None
        self.arm_invocation = None
        self.arm_action = None
        self.arm_waiting_hold = False
        self.arm_waiting_pause = False
        self.arm_status_path = None
        self.arm_deadline = None
        self.arm_external_active=False;self.arm_scan_at=-float('inf');self.arm_launch_grace=0.

    def build_arm_tools_ui(self, layout):
        group = W.QGroupBox('独立工具');layout.addWidget(group)
        box = W.QVBoxLayout(group)
        self.arm_test_button = self.button('charge_robot test', lambda:self.start_arm_tool('test'), box, True)
        self.arm_home_button = self.button('charge_robot home', lambda:self.start_arm_tool('home'), box)
        self.arm_note = W.QLabel('点击后打开终端，沿用原版确认流程。')
        self.arm_note.setWordWrap(True);box.addWidget(self.arm_note)
        self.docking_release_button = self.button('已收拢，释放底盘', self.release_arm_hold, box)
        self.docking_release_button.setToolTip('现场确认收拢后释放工具占用；不额外生成普通停车锁，真实控制故障需先处理。')
        warning = W.QLabel('底盘停车不控制机械臂。');box.addWidget(warning)
        self.docking_button = self.arm_test_button  # Common base-conflict button list.

    def arm_tool_busy(self):
        return bool(self.arm_waiting_hold or self.arm_waiting_pause or
                    (self.arm_process is not None and self.arm_process.state()!=QtCore.QProcess.NotRunning) or self.arm_external_active)

    def _arm_base_snapshot(self):
        if self.bridge:
            return self.bridge.arm_hold_snapshot()
        return self.latest

    def _hold_base(self,independent=False):
        if self.bridge:
            if independent:self.bridge.begin_independent_arm_hold()
            else:self.bridge.begin_arm_hold()
        self.docking_active = True
        write_json(self.docking_marker, dict(active=True, session=self.session,
                                             scope='independent_arm_tools'))
        self.keyboard.disarm('机械臂工具占用底盘')

    def start_arm_tool(self, action):
        if action not in ('test','home'):
            raise ValueError('未知机械臂工具')
        if self.offline or self.cleanup:
            return
        if self.arm_tool_busy():
            self.error('已有工具或暂停请求运行中');return
        try:
            reason=independent_tool_reason(self._arm_base_snapshot())
            if reason:raise ValueError(reason)
            if active_arm_tools():raise ValueError('检测到另一个机械臂工具仍在运行')
            text=('打开独立 charge_robot '+action+'？原版终端会逐段确认。\n'
                  '请现场确认底盘与机械臂停稳、其他控制已停用、工作区安全。\n底盘反馈缺失时不代表已验证停稳；软件停车不控制机械臂。')
            if action=='home':text+='\nhome可能先拔枪：确认已断电、解锁，退回路径安全。'
            if not self.confirm(text):return
            self._hold_base(independent=True)
            self.arm_action=action;self.arm_invocation=uuid.uuid4().hex
            self.arm_status_path=self.logdir/('arm-tool-'+self.arm_invocation+'.json')
            self._launch_arm_tool()
            self.log('independent_arm_tool_requested',action=action,invocation=self.arm_invocation)
        except (OSError,ValueError,RuntimeError) as error:
            self.arm_waiting_hold=False
            self.error('工具未启动：'+str(error))
        self.update_arm_tools_ui()

    def _launch_arm_tool(self):
        script=PROJECT_ROOT/'arm_tool_terminal.py'
        ok,pid=QtCore.QProcess.startDetached('gnome-terminal',
            ['--wait','--title=charge_robot '+self.arm_action,'--','/usr/bin/python3',
             str(script),self.arm_action,'--state',str(self.arm_status_path),
             '--invocation',self.arm_invocation],str(script.parent))
        if not ok:raise OSError('无法打开独立终端；底盘继续保持停车')
        self.arm_process=DetachedTool(pid);self.arm_launch_grace=time.monotonic()+5.
        self.arm_note.setText(self.arm_action+' 独立终端运行中；关闭面板不会停止机械臂')

    def refresh_arm_presence(self):
        now=time.monotonic()
        if now-self.arm_scan_at<.5:return
        self.arm_scan_at=now
        try:rows=active_arm_tools()
        except OSError:
            self.arm_external_active=True;return
        self.arm_external_active=bool(rows)
        process=self.arm_process
        if process is not None and process.state()!=0 and not rows and now>=self.arm_launch_grace:
            process.active=False
            self._arm_tool_finished(self.arm_invocation,None,None)

    def _arm_tool_error(self, token, error):
        if token!=self.arm_invocation:return
        self.arm_note.setText('终端启动或通信异常；底盘保持占用')
        self.log('independent_arm_terminal_error',invocation=token,error=int(error))

    def _arm_tool_finished(self, token, code, status):
        if token!=self.arm_invocation:return
        self.arm_note.setText('终端已退出；确认收拢后可释放底盘')
        self.log('independent_arm_terminal_closed',invocation=token,exit_code=code,
                 physical_stop_verified=False,route_preserved=True)
        self.update_arm_tools_ui()

    def tick_arm_tools(self):
        self.refresh_arm_presence()
        if self.arm_waiting_hold:
            s=self._arm_base_snapshot();reason=base_tool_reason(s,held=True)
            if reason or time.monotonic()>self.arm_deadline:
                self.arm_waiting_hold=False
                self.arm_note.setText('未启动：'+str(reason or '停车确认超时')+'；底盘保持占用')
            elif s.get('hold_feedback_confirmed'):
                self.arm_waiting_hold=False
                if self.arm_action is None:
                    try:
                        self.bridge.request_nav2_pause();self.arm_waiting_pause=True
                    except ValueError as error:self.arm_note.setText('导航暂停未完成：'+str(error))
                else:self._launch_arm_tool()
        if self.arm_waiting_pause:
            p=self.bridge.snapshot().get('nav2_pause',{})
            if p.get('phase') in ('authorized','failed','cancelled'):
                self.arm_waiting_pause=False
                self.arm_note.setText('导航计算已暂停；底盘保持占用' if p['phase']=='authorized' else
                                      '导航暂停失败：'+str(p.get('reason')))
        # Status is informational only: never clears a lock, cancels a route,
        # starts another tool or issues any base command.
        if self.arm_tool_busy() and self.arm_status_path and self.arm_status_path.exists():
            try:
                d=json.loads(self.arm_status_path.read_text())
                if d.get('invocation')==self.arm_invocation:
                    labels={'checking':'检查已有进程','running':'终端运行中','returned':'命令返回；关闭终端后确认收拢',
                            'uncertain':'命令失败或中断；现场核实机械臂'}
                    self.arm_note.setText(labels.get(d.get('phase'),'等待终端状态'))
            except (OSError,ValueError):pass
        self.update_arm_tools_ui()

    def release_arm_hold(self):
        if self.offline or not self.docking_active:return
        if self.arm_tool_busy():self.error('先结束工具终端或等待暂停请求结束');return
        try:
            if active_arm_tools():raise ValueError('仍有机械臂工具进程；请先结束并确认实际停止')
            if not self.confirm('确认机械臂已经停止并收拢、枪头完全退出、线缆安全，释放底盘？\n'
                                '不要求test/home成功；释放后不会自动移动。'):return
            if self.bridge:self.bridge.release_docking()
            self.docking_active=False
            write_json(self.docking_marker,dict(active=False,session=self.session))
            self.arm_note.setText('工具占用已释放；底盘软件停车需单独解除')
            self.log('independent_arm_base_released',route_preserved=True,automatic_resume=False)
        except (OSError,ValueError,RuntimeError) as error:self.error(str(error))
        self.update_navigation_ui();self.update_return_ui()

    def pause_navigation_compute(self):
        if self.offline or not self.bridge or self.cleanup:return
        if self.arm_tool_busy():self.error('工具或暂停请求运行中');return
        if not self.confirm('只暂停导航计算，保留定位、底盘与原始里程计；不启动机械臂。'):return
        try:
            self._hold_base();self.arm_action=None;self.arm_status_path=None
            self.arm_waiting_hold=True;self.arm_deadline=time.monotonic()+3.
        except (OSError,ValueError) as error:self.error(str(error))

    def note_base_stop(self, reason):
        # Base stop/cancel actions do not drive, restart or cancel arm scripts.
        # A tool not yet launched is different: operator interruption must not
        # leave a delayed launch queued behind a later feedback callback.
        if self.arm_waiting_hold or self.arm_waiting_pause:
            if self.arm_waiting_pause and self.bridge:
                self.bridge.cancel_nav2_pause(reason)
            self.arm_waiting_hold=False;self.arm_waiting_pause=False
            self.arm_invocation=None
            self.arm_note.setText('等待已取消；未启动机械臂，底盘保持占用')
        if self.arm_tool_busy():
            self.log('base_action_during_arm_tool',reason=reason,arm_stop_confirmed=False)

    def update_arm_tools_ui(self):
        if not hasattr(self,'arm_test_button'):return
        self.refresh_arm_presence()
        reason=independent_tool_reason(self._arm_base_snapshot())
        eligible=not self.offline and not self.cleanup and not self.arm_tool_busy() and reason is None
        if not self.arm_tool_busy() and not self.docking_active:
            self.arm_note.setText('离线预览' if self.offline else reason or '点击后打开独立终端，沿用逐段确认。')
        for button in (self.arm_test_button,self.arm_home_button):
            button.setEnabled(eligible);button.setToolTip(reason or '打开独立终端；不启动其他模块')
        self.docking_release_button.setEnabled(self.docking_active and not self.offline and not self.arm_tool_busy())
        self.pause_nav_button.setEnabled(not self.offline and self.mode=='navigation' and eligible and
                                        base_tool_reason(self._arm_base_snapshot()) is None and
                                        self.latest.get('nav2_pause',{}).get('phase','idle')=='idle')
        if self.docking_active:
            for button in (self.manual_button,self.connect_button,self.disconnect_button,self.nav_button,
                           self.mapping_button,self.initial_button,self.apply_initial_button,self.go_button,
                           self.free_go_button,self.recovery_button,self.stop_button,self.return_button):
                button.setEnabled(False)
