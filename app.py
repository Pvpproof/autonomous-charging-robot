#!/usr/bin/env python3
"""Independent charging-navigation desktop. Hardware starts only on a user click."""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import sys
import time
from PyQt5 import QtCore, QtGui, QtWidgets as W
from canvas import MapCanvas
from teleop import KeyboardPad, SPEEDS_M_S
from process_control import find_services, remaining, send_signal, shutdown_targets
from model import MapDocument, Stations, write_json, import_map_copy
from docking_state import readiness, goal_error
from power_status import power_text
from route_cycle import StartStore, CycleError
from chassis_route import ChassisSession
from raw_odom_mode import RawChassisSession,SOURCE
from navigation_status import navigation_reasons,button_status
from session_revalidation import load_source
from return_hold_pad import ReturnStartPad

ROOT=Path(__file__).resolve().parent


class Window(W.QMainWindow):
    def __init__(self,offline=False):
        super().__init__()
        self.offline=offline;self.bridge=None;self.backend=None;self.base_process=None;self.saver=None;self.mode=None
        self.view_mode='saved';self.live_document=None;self.active_fingerprint=None
        self.initial_pose=None;self.initial_map=None;self.backend_failure='';self.stopping_backend=False
        self.nav_output_buffer=''
        self.cleanup=None;self.allow_close=False
        self.docking_active=False;self.docking_process=None;self.chassis_session=None
        self.imported_return_plan=None
        self.document=None;self.stations=None;self.target=None;self.free_goal=None;self.live_seq=-1;self.latest={}
        self.pending_goal=None;self.active_map=None;self.last_state=None;self.latest_map_seen=0
        self.form_dirty=False;self.loading_fields=False;self.editing_index=-1
        self.graph_ready=False;self.session=time.strftime('%Y%m%d_%H%M%S')+'_'+str(os.getpid())
        for name in ('maps','logs','config'): (ROOT/name).mkdir(exist_ok=True)
        self.docking_marker=ROOT/'config'/'docking_hold.json'
        if self.docking_marker.exists():
            try:self.docking_active=bool(json.loads(self.docking_marker.read_text())['active'])
            except (OSError,ValueError,KeyError):self.docking_active=True
        self.logdir=ROOT/'logs'/self.session;self.logdir.mkdir()
        self.events=(self.logdir/'events.jsonl').open('a',encoding='utf-8')
        self.console=(self.logdir/'backend.log').open('a',encoding='utf-8')
        self.setWindowTitle('充电导航工作台'+(' · 离线预览' if offline else ''))
        self.resize(1380,880);self.setMinimumSize(1100,760)
        self.build_ui()
        self.cleanup_timer=QtCore.QTimer(self);self.cleanup_timer.timeout.connect(self.poll_cleanup);self.cleanup_timer.setInterval(250)
        if not offline:
            from ros_bridge import Bridge
            self.bridge=Bridge(docking_hold=self.docking_active)
        self.manual_timer=QtCore.QTimer(self);self.manual_timer.timeout.connect(self.manual_heartbeat);self.manual_timer.start(50)
        QtWidgetsApp=W.QApplication.instance()
        QtWidgetsApp.applicationStateChanged.connect(self.application_state)
        self.refresh_maps()
        prefs=ROOT/'config'/'panel.json'
        if prefs.exists():
            try:
                previous=json.loads(prefs.read_text())['last_map']
                if Path(previous).exists():self.load_map(previous)
            except (ValueError,KeyError,OSError) as error:self.log('无法恢复上次地图：'+str(error))
        self.timer=QtCore.QTimer(self);self.timer.timeout.connect(self.tick);self.timer.start(200)
        self.update_navigation_ui()
        self.log('面板已启动。编辑地图和点位不会发送运动命令。')

    def button(self,text,slot,layout,primary=False):
        b=W.QPushButton(text);b.clicked.connect(slot)
        if primary:b.setProperty('primary',True)
        layout.addWidget(b);return b

    def build_ui(self):
        self.setStyleSheet('''QMainWindow,QWidget {background:#101b28;color:#dbe6f2;font-size:13px;}
        QGroupBox {border:1px solid #304358;border-radius:8px;margin-top:12px;padding:12px 8px 8px;}
        QGroupBox::title {subcontrol-origin:margin;left:12px;color:#70d7c0;font-weight:bold;}
        QPushButton {background:#253a50;border:1px solid #36516c;border-radius:5px;padding:7px;}
        QPushButton:hover {background:#36536e;} QPushButton:disabled {color:#61758c;}
        QPushButton[primary=true] {background:#0f766e;border-color:#14b8a6;}
        QLineEdit,QDoubleSpinBox,QComboBox,QListWidget,QPlainTextEdit {background:#172638;border:1px solid #304358;border-radius:4px;padding:4px;}
        QListWidget::item:selected {background:#145f67;} QLabel#title {font-size:23px;font-weight:bold;color:#f0faf8;}
        QSplitter::handle {background:#304358;}''')
        central=W.QWidget();self.setCentralWidget(central);outer=W.QVBoxLayout(central)
        header=W.QHBoxLayout();outer.addLayout(header)
        title=W.QLabel('充电导航工作台');title.setObjectName('title');header.addWidget(title)
        header.addStretch()
        self.power_label=W.QLabel(power_text({}));self.power_label.setWordWrap(True);self.power_label.setMinimumWidth(345)
        self.power_label.setToolTip('只读 /PowerVoltage 与 /PowerVoltage48V，两路分别显示，互不替代。48V接口按原发布器解释为V，但报文未校验CRC/地址/功能码，寄存器含义尚待厂家确认。消息无时间戳，超过3秒未收到标为过期；新鲜不等于真实或准确，不换算电量百分比。')
        header.addWidget(self.power_label)
        self.state=W.QLabel('地图未加载');self.state.setWordWrap(True);self.state.setMinimumWidth(460);header.addWidget(self.state)
        stoprow=W.QHBoxLayout();outer.addLayout(stoprow)
        self.emergency_button=W.QPushButton('■  底盘紧急停车');self.emergency_button.setMinimumHeight(64);self.emergency_button.setMinimumWidth(240)
        self.emergency_button.setStyleSheet('QPushButton {background:#c81e1e;color:white;border:3px solid #ffb4b4;font-size:25px;font-weight:bold;border-radius:8px;} QPushButton:hover {background:#ef2929;} QPushButton:pressed {background:#8b0000;}')
        self.emergency_button.clicked.connect(self.emergency_stop);stoprow.addWidget(self.emergency_button)
        self.stop_note=W.QLabel('底盘软件停车：立即发零速度、取消导航并锁定操控。\n不控制机械臂；不能替代实体急停。');self.stop_note.setWordWrap(True);stoprow.addWidget(self.stop_note,1)
        self.release_stop_button=W.QPushButton('解除停车锁定');self.release_stop_button.clicked.connect(self.release_stop);self.release_stop_button.setEnabled(False);stoprow.addWidget(self.release_stop_button)
        splitter=W.QSplitter();outer.addWidget(splitter,1)
        left=W.QWidget();left.setMaximumWidth(365);left.setMinimumWidth(320);side=W.QVBoxLayout(left);side.setContentsMargins(0,0,6,0)
        scroll=W.QScrollArea();scroll.setWidgetResizable(True);scroll.setWidget(left);scroll.setMinimumWidth(340);splitter.addWidget(scroll)
        devicebox=W.QGroupBox('设备连接 · 各功能共用');side.addWidget(devicebox);dl=W.QHBoxLayout(devicebox)
        self.connect_button=self.button('连接底盘与雷达',self.start_devices,dl,True)
        self.disconnect_button=self.button('断开设备',self.stop_devices,dl)
        mapbox=W.QGroupBox('01  地图与工作模式');side.addWidget(mapbox);ml=W.QVBoxLayout(mapbox)
        self.map_combo=W.QComboBox();ml.addWidget(self.map_combo)
        row=W.QHBoxLayout();ml.addLayout(row)
        self.button('加载选中',self.load_selected,row);self.button('导入地图…',self.import_map,row)
        self.mapping_toggle=W.QPushButton('▶ 重新建图（可选）');self.mapping_toggle.setCheckable(True);ml.addWidget(self.mapping_toggle)
        self.mapping_tools=W.QWidget();mapping_layout=W.QVBoxLayout(self.mapping_tools);mapping_layout.setContentsMargins(0,0,0,0)
        ml.addWidget(self.mapping_tools);self.mapping_tools.hide()
        self.mapping_toggle.toggled.connect(self.mapping_tools.setVisible)
        self.mapping_toggle.toggled.connect(lambda expanded:self.mapping_toggle.setText(('▼ ' if expanded else '▶ ')+'重新建图（可选）'))
        self.map_name=W.QLineEdit(time.strftime('charging_%Y%m%d_%H%M%S'));self.map_name.setPlaceholderText('新地图名称');mapping_layout.addWidget(self.map_name)
        self.mapping_button=self.button('开始重新建图',lambda:self.start_backend('mapping'),mapping_layout,True)
        self.save_map_button=self.button('保存当前地图',self.save_map,mapping_layout)
        row=W.QHBoxLayout();ml.addLayout(row)
        self.nav_button=W.QPushButton('启动本图定位导航');self.nav_button.clicked.connect(lambda:self.start_backend('navigation'))
        self.stop_button=self.button('停止当前模式',self.stop_backend,row)
        self.recovery_button=self.button('关闭已有建图 / 定位服务',lambda:self.cleanup_services(),ml)
        self.recovery_button.setToolTip('关闭本次或上次面板遗留的建图、Nav2 服务；保留底盘和雷达')
        self.map_label=W.QLabel('尚未加载地图');self.map_label.setWordWrap(True);ml.addWidget(self.map_label)
        self.sensors=W.QLabel('雷达 —   里程计 —   定位 —');self.sensors.setWordWrap(True);ml.addWidget(self.sensors)
        stations=W.QGroupBox('03  充电点与预停靠');side.addWidget(stations);sl=W.QVBoxLayout(stations)
        self.list=W.QListWidget();self.list.setMaximumHeight(115);self.list.currentRowChanged.connect(self.select_station);sl.addWidget(self.list)
        row=W.QHBoxLayout();sl.addLayout(row)
        self.button('新增',self.add_station,row);self.button('删除',self.delete_station,row);self.button('保存点位',self.save_stations,row,True)
        self.button('点击地图新增充电停靠点',lambda:self.tool('new_station'),sl)
        self.button('当前位置新增充电停靠点',lambda:self.capture_pose(new=True),sl)
        note=W.QLabel('以下坐标仅编辑充电点，不用于车辆初始定位。');note.setWordWrap(True);sl.addWidget(note)
        self.station_editor=W.QWidget();sl.addWidget(self.station_editor);sl=W.QVBoxLayout(self.station_editor);sl.setContentsMargins(0,0,0,0)
        self.station_editor.setEnabled(False)
        self.name=W.QLineEdit();self.name.setPlaceholderText('充电点名称');sl.addWidget(self.name)
        self.socket_valid=W.QCheckBox('已设置充电口标记');sl.addWidget(self.socket_valid)
        row=W.QHBoxLayout();sl.addLayout(row);self.sx=self.number(row,'X');self.sy=self.number(row,'Y')
        self.dock_valid=W.QCheckBox('已设置底盘预停靠位');sl.addWidget(self.dock_valid)
        row=W.QHBoxLayout();sl.addLayout(row);self.dx=self.number(row,'X');self.dy=self.number(row,'Y')
        row=W.QHBoxLayout();sl.addLayout(row);self.yaw=self.number(row,'朝向°',-180,180)
        self.button('应用数值 / 名称',self.apply_fields,sl)
        self.name.textEdited.connect(self.mark_form)
        for field in (self.sx,self.sy,self.dx,self.dy,self.yaw):field.valueChanged.connect(self.mark_form)
        self.socket_valid.toggled.connect(self.mark_form);self.dock_valid.toggled.connect(self.mark_form)
        row=W.QHBoxLayout();sl.addLayout(row)
        self.button('地图标充电口',lambda:self.tool('socket'),row)
        self.button('拖出预停靠朝向',lambda:self.tool('dock'),row)
        self.button('记录当前底盘位置为预停靠位',self.capture_pose,sl)
        self.point_note=W.QLabel('紫色：充电口   蓝色箭头：预停靠位');self.point_note.setWordWrap(True);sl.addWidget(self.point_note)
        nav=W.QGroupBox('02  定位与导航 · 无需充电点');side.insertWidget(2,nav);nl=W.QVBoxLayout(nav)
        self.nav_help=W.QLabel();self.nav_help.setWordWrap(True);nl.addWidget(self.nav_help)
        nl.addWidget(self.nav_button)
        self.initial_button=self.button('在地图上设置初始位置',lambda:self.tool('initial'),nl)
        self.apply_initial_button=self.button('应用初始位置（不移动车辆）',self.apply_initial_pose,nl,True)
        self.initial_note=W.QLabel('尚未标记车辆初始位置');self.initial_note.setWordWrap(True);nl.addWidget(self.initial_note)
        self.goal_button=self.button('在地图选择导航目标',lambda:self.tool('goal'),nl)
        self.free_go_button=self.button('导航到选定目标',self.navigate_free,nl,True)
        self.go_button=self.button('前往选中预停靠位',self.navigate,nl,True)
        self.go_status=W.QLabel();self.go_status.setWordWrap(True);nl.addWidget(self.go_status)
        self.record_route=W.QCheckBox('记录实际去程（同会话底盘原始里程）');nl.addWidget(self.record_route)
        self.route_note=W.QLabel('定位后勾选记录并导航；返程使用本轮原始里程，不套地图起点参考。');self.route_note.setWordWrap(True);nl.addWidget(self.route_note)
        self.return_button=W.QPushButton('准备沿实测轨迹返程');self.return_button.clicked.connect(self.prepare_return);self.return_button.setEnabled(False);nl.addWidget(self.return_button)
        self.import_return_button=self.button('旧地图轨迹不可用于原始里程模式',self.import_return_route,nl);self.import_return_button.setEnabled(False)
        self.return_short=W.QCheckBox('首次短测：最多 3 秒 / 实测 0.15 米');self.return_short.setChecked(True);nl.addWidget(self.return_short)
        self.return_run_session=None;self.return_hold=ReturnStartPad(self);self.return_hold.start_requested.connect(self.start_return);self.return_hold.cancel_requested.connect(self.stop_return);self.return_hold.setEnabled(False);nl.addWidget(self.return_hold)
        self.return_stop=W.QPushButton('停止返程');self.return_stop.clicked.connect(self.stop_return);nl.addWidget(self.return_stop)
        self.return_note=W.QLabel('人工持续监护，无后向自动避障。记录路径无法发现新进入的障碍。');self.return_note.setWordWrap(True);nl.addWidget(self.return_note)
        self.cancel_button=self.button('取消导航',self.cancel_navigation,nl)
        self.nav_label=W.QLabel('可直接选目标导航，也可前往已保存充电点。');self.nav_label.setWordWrap(True);nl.addWidget(self.nav_label)
        self.nav_failure_note=W.QLabel();self.nav_failure_note.setWordWrap(True);self.nav_failure_note.setStyleSheet('color:#fbbf24;');nl.addWidget(self.nav_failure_note)
        dock=W.QGroupBox('04  导航后对接 · 人工接管');side.addWidget(dock);dkl=W.QVBoxLayout(dock)
        self.docking_note=W.QLabel();self.docking_note.setWordWrap(True);dkl.addWidget(self.docking_note)
        self.docking_conditions=W.QCheckBox('现场确认停靠姿态及工作区安全')
        self.docking_conditions.setToolTip('软件停稳观察不证明可对接；需确认相机能看到充电口、机械臂可达，外部底盘控制已停用')
        dkl.addWidget(self.docking_conditions)
        self.docking_conditions.toggled.connect(self.update_docking_ui)
        self.docking_button=self.button('锁定底盘并打开对接终端',self.open_docking_console,dkl,True)
        self.docking_result=W.QLabel('test/home 不会自动启动；原工作流仍逐段确认。')
        self.docking_result.setWordWrap(True);dkl.addWidget(self.docking_result)
        self.docking_release_button=self.button('现场确认机械臂已收拢，解除对接锁',self.release_docking,dkl)
        boundary=W.QLabel('home 可能先拔枪，必须先确认断电、解锁及退回路径。底盘停车不停止机械臂。')
        boundary.setWordWrap(True);dkl.addWidget(boundary)
        side.addStretch()
        right=W.QWidget();rl=W.QVBoxLayout(right);rl.setContentsMargins(4,0,0,0);splitter.addWidget(right)
        row=W.QHBoxLayout();rl.addLayout(row)
        self.hint=W.QLabel('中键拖图 · 滚轮缩放 · 右键取消当前标点');row.addWidget(self.hint,1)
        self.button('实时建图画面',lambda:self.set_view('live'),row)
        self.button('地图 / 点位编辑',lambda:self.set_view('saved'),row)
        self.button('平移 / 取消绘制',lambda:self.tool(None),row);self.button('适应地图',lambda:self.canvas.fit_map(),row)
        manual_box=W.QGroupBox('键盘操控 · YCHD 底盘');ml=W.QVBoxLayout(manual_box)
        manual_row=W.QHBoxLayout();ml.addLayout(manual_row)
        self.speed_combo=W.QComboBox()
        for speed in SPEEDS_M_S:self.speed_combo.addItem('{:.2f} m/s'.format(speed),speed)
        manual_row.addWidget(W.QLabel('速度'));manual_row.addWidget(self.speed_combo)
        self.manual_button=self.button('开启键盘控制',self.enable_keyboard,manual_row,True)
        self.button('停车并退出键盘',lambda:self.keyboard.disarm('用户停车'),manual_row)
        self.keyboard=KeyboardPad();self.keyboard.released.connect(self.keyboard_released);ml.addWidget(self.keyboard)
        self.steering_note=W.QLabel('键盘意图 / 里程计反馈：等待数据');self.steering_note.setWordWrap(True);ml.addWidget(self.steering_note)
        rl.addWidget(manual_box)
        self.canvas=MapCanvas();rl.addWidget(self.canvas,1)
        self.canvas.poseDrawn.connect(self.pose_drawn)
        self.canvas.operationCanceled.connect(self.cancel_map_operation)
        self.canvas.cursorWorld.connect(lambda x,y:self.statusBar().showMessage('地图坐标 X {:.3f} m   Y {:.3f} m'.format(x,y)))
        self.pose_label=W.QLabel('底盘位姿：等待定位');rl.addWidget(self.pose_label)
        self.logview=W.QPlainTextEdit();self.logview.setReadOnly(True);self.logview.setMaximumBlockCount(700);self.logview.setMaximumHeight(130);rl.addWidget(self.logview)
        self.log_path=W.QLabel('日志：'+str(self.logdir));self.log_path.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse);rl.addWidget(self.log_path)
        splitter.setSizes([350,1000])

    def number(self,layout,label,minimum=-10000,maximum=10000):
        layout.addWidget(W.QLabel(label));n=W.QDoubleSpinBox();n.setRange(minimum,maximum);n.setDecimals(3);n.setSingleStep(.05);layout.addWidget(n);return n
    def log(self,text,**fields):
        self.logview.appendPlainText(time.strftime('%H:%M:%S')+'  '+text)
        self.events.write(json.dumps({'time':time.time(),'monotonic_s':time.monotonic(),'text':text,**fields},ensure_ascii=False)+'\n');self.events.flush()
    def error(self,text):self.log(text);W.QMessageBox.warning(self,'操作未完成',text)
    def confirm(self,text):return W.QMessageBox.question(self,'请确认',text,W.QMessageBox.Yes|W.QMessageBox.No,W.QMessageBox.No)==W.QMessageBox.Yes
    def refresh_maps(self):
        self.map_combo.clear()
        for path in sorted((ROOT/'maps').glob('*.yaml'),reverse=True):self.map_combo.addItem(path.stem,str(path))
    def mark_form(self,*args):
        if not self.loading_fields and self.editing_index>=0:self.form_dirty=True
    def dirty_guard(self):
        if self.form_dirty and not self.apply_fields():return False
        if not self.stations or not self.stations.dirty:return True
        answer=W.QMessageBox.question(self,'点位尚未保存','先保存当前点位修改？',W.QMessageBox.Save|W.QMessageBox.Discard|W.QMessageBox.Cancel,W.QMessageBox.Save)
        if answer==W.QMessageBox.Cancel:return False
        return self.save_stations() if answer==W.QMessageBox.Save else True
    def load_selected(self):
        path=self.map_combo.currentData()
        if path:self.load_map(path)
    def import_map(self):
        path,_=W.QFileDialog.getOpenFileName(self,'导入已有地图副本',str(ROOT/'maps'),'地图 (*.yaml *.yml)')
        if path:
            try:copied=import_map_copy(path,ROOT/'maps')
            except (OSError,ValueError,KeyError) as error:self.error(str(error));return
            self.refresh_maps();self.load_map(copied)
    def load_map(self,path,after_save=False):
        if not self.dirty_guard():return False
        try:doc=MapDocument.load(path);stations=Stations(doc)
        except (OSError,ValueError,KeyError) as error:self.error(str(error));return False
        self.document=doc;self.stations=stations;self.target=None;self.free_goal=None;self.canvas.tool(None)
        selected=self.map_combo.findData(doc.yaml_path)
        if selected>=0:self.map_combo.setCurrentIndex(selected)
        if not after_save:self.view_mode='saved'
        if self.view_mode=='saved':self.canvas.set_document(doc)
        self.refresh_station_list();self.map_label.setText('{}\n{} × {} 格 · {:.3f} m/格'.format(Path(path).name,doc.width,doc.height,doc.resolution))
        write_json(ROOT/'config'/'panel.json',{'last_map':str(Path(path).resolve())})
        if self.initial_map!=(doc.yaml_path,doc.fingerprint):
            self.initial_pose=None;self.initial_map=None
            try:
                saved=self.start_store().read(doc.fingerprint)
                if saved:
                    self.initial_pose=tuple(saved['pose']);self.initial_map=(doc.yaml_path,doc.fingerprint)
                    self.log('已恢复保存的起点参考；尚未应用定位，请核实现车位置',pose_m_rad=self.initial_pose)
            except (OSError,ValueError,KeyError) as error:self.log('无法恢复起点参考：'+str(error))
        self.update_navigation_ui()
        self.log('已加载地图 '+str(path),map_sha256=doc.fingerprint)
        return True
    def editing(self):
        if not self.stations:self.error('请先加载一张已保存的地图。');return False
        if self.view_mode!='saved':self.set_view('saved')
        return True
    def navigation_map_matches(self):
        return bool(self.mode=='navigation' and self.document and
                    self.document.yaml_path==self.active_map and
                    self.document.fingerprint==self.active_fingerprint)
    def set_view(self,view):
        doc=self.live_document if view=='live' else self.document
        if doc is None:self.error('尚无实时建图画面。' if view=='live' else '请先加载已保存地图。');return
        self.view_mode=view;self.canvas.tool(None);self.canvas.set_document(doc)
        self.hint.setText('实时建图显示' if view=='live' else '已载入地图与点位编辑（不改变导航正在使用的地图）')
        self.repaint_map()
    def refresh_station_list(self,index=0):
        self.list.blockSignals(True);self.list.clear()
        if self.stations:
            for p in self.stations.points:self.list.addItem(p['name'])
        self.list.blockSignals(False)
        if self.list.count():self.list.setCurrentRow(min(index,self.list.count()-1))
        else:self.select_station(-1)
    def select_station(self,index):
        if self.form_dirty and self.editing_index >= 0:
            if not self.apply_fields():
                self.list.blockSignals(True);self.list.setCurrentRow(self.editing_index);self.list.blockSignals(False)
                return
            self.list.blockSignals(True);self.list.setCurrentRow(index);self.list.blockSignals(False)
        self.loading_fields=True;self.editing_index=index
        self.station_editor.setEnabled(index>=0)
        self.canvas.tool(None)
        p=self.stations.points[index] if self.stations and index>=0 else {'name':'','socket':None,'pre_dock':None}
        self.name.setText(p['name']);self.socket_valid.setChecked(p['socket'] is not None);self.dock_valid.setChecked(p['pre_dock'] is not None)
        for obj,x,y in [(p['socket'],self.sx,self.sy),(p['pre_dock'],self.dx,self.dy)]:
            x.setValue(obj['x_m'] if obj else 0);y.setValue(obj['y_m'] if obj else 0)
        self.yaw.setValue(math.degrees(p['pre_dock']['yaw_rad']) if p['pre_dock'] else 0)
        self.loading_fields=False;self.form_dirty=False
        self.repaint_map()
    def add_station(self):
        if not self.editing():return
        if self.form_dirty and not self.apply_fields():return
        name,ok=W.QInputDialog.getText(self,'新增充电点','名称',text='充电位{:02d}'.format(len(self.stations.points)+1))
        if ok:
            try:index=self.stations.add(name)
            except ValueError as error:self.error(str(error));return
            self.refresh_station_list(index)
    def delete_station(self):
        if not self.editing():return
        i=self.list.currentRow()
        if i>=0 and self.confirm('删除充电点“{}”？'.format(self.stations.points[i]['name'])):
            self.form_dirty=False;self.stations.delete(i);self.refresh_station_list(max(0,i-1))
    def apply_fields(self):
        if not self.editing():return False
        i=self.editing_index
        if i<0:self.error('请先新增或选择充电点。');return False
        socket={'x_m':self.sx.value(),'y_m':self.sy.value()} if self.socket_valid.isChecked() else None
        dock={'x_m':self.dx.value(),'y_m':self.dy.value(),'yaw_rad':math.radians(self.yaw.value())} if self.dock_valid.isChecked() else None
        try:self.stations.update(i,self.name.text(),socket,dock)
        except ValueError as error:self.error(str(error));return False
        self.form_dirty=False
        self.list.item(i).setText(self.stations.points[i]['name'])
        self.repaint_map();return True
    def save_stations(self):
        if not self.stations:return False
        if self.form_dirty and not self.apply_fields():return False
        try:self.stations.save()
        except OSError as error:self.error(str(error));return False
        self.log('点位已保存 '+str(self.stations.path));return True
    def tool(self,mode):
        if mode in ('socket','dock'):
            if not self.editing():return
            if self.list.currentRow()<0:self.error('请先选择充电点。');return
        if mode in ('initial','goal','new_station') and not self.document:self.error('请先加载地图。');return
        if mode=='initial' and self.latest.get('busy'):self.error('请先取消导航再重设定位。');return
        if mode in ('initial','goal','new_station'):self.set_view('saved')
        self.canvas.tool(mode)
        self.hint.setText({'socket':'点击充电口位置。','dock':'按住预停靠位置，向车头方向拖动后松开。',
            'new_station':'点击新增底盘停靠点（默认朝向地图 +X）；拖动可指定车头朝向，创建后可修改并保存。',
            'goal':'在目标位置按住，拖出到达朝向；再点击“导航到选定目标”。',
            'initial':'按底盘 base_footprint 参考点标位置，拖出车头朝向；黄色车体轮廓应与现场对应，不只是中心点落在空白处。',None:'中键拖图 · 滚轮缩放 · 右键取消当前标点。'}[mode])
    def cancel_map_operation(self):
        self.tool(None);self.hint.setText('已取消当前标点；已保存点位保持不变。中键可拖动地图。')

    def pose_drawn(self,mode,x,y,yaw):
        if mode=='new_station':
            if not self.editing():return
            if self.form_dirty and not self.apply_fields():return
            number=len(self.stations.points)+1
            names={point['name'] for point in self.stations.points}
            while '充电位{:02d}'.format(number) in names:number+=1
            name='充电位{:02d}'.format(number)
            index=self.stations.add(name)
            self.stations.update(index,name,None,{'x_m':x,'y_m':y,'yaw_rad':yaw})
            self.refresh_station_list(index);self.tool(None)
            self.save_stations();return
        if mode=='goal':
            self.free_goal=(x,y,yaw);self.target=self.free_goal;self.tool(None)
            self.log('已选择导航目标，尚未发送',x_m=x,y_m=y,yaw_rad=yaw);return
        if mode=='initial':
            if self.latest.get('busy'):self.error('请先取消导航再重设定位。');return
            self.initial_pose=(x,y,yaw);self.initial_map=(self.document.yaml_path,self.document.fingerprint)
            self.tool(None);self.update_navigation_ui();self.repaint_map()
            try:self.start_store().write(self.document.fingerprint,self.initial_pose,'manual_reference')
            except (OSError,ValueError) as error:self.error('已标记，但起点参考保存失败：'+str(error));return
            self.log('已保存车辆起点参考，尚未应用定位',pose_m_rad=self.initial_pose);return
        if not self.editing():return
        if mode=='socket':self.socket_valid.setChecked(True);self.sx.setValue(x);self.sy.setValue(y)
        else:self.dock_valid.setChecked(True);self.dx.setValue(x);self.dy.setValue(y);self.yaw.setValue(math.degrees(yaw))
        self.apply_fields();self.tool(None)
    def start_store(self):
        return StartStore(ROOT/'config'/'starts'/(self.document.fingerprint+'.json'))

    def apply_initial_pose(self):
        if self.docking_guard():return
        if self.initial_pose is None:self.error('请先在地图上标记车辆位置和车头朝向。');return
        if self.offline or not self.navigation_map_matches():self.error('位置已标记；请先启动本图定位导航。');return
        if not self.latest.get('initial_ready',False):self.error('定位节点尚未准备好接收初始位置，请查看启动状态。');return
        if self.latest.get('busy') or self.keyboard.armed or self.latest.get('manual') or self.latest.get('manual_stop_pending'):
            self.error('请先结束导航或键盘控制，待车辆停稳后应用初始位置。');return
        x,y,yaw=self.initial_pose
        if self.confirm('确认车辆已停稳，实际位置为 X {:.2f} m，Y {:.2f} m，朝向 {:.1f}°？\n只更新定位，不移动车辆。'.format(x,y,math.degrees(yaw))):
            self.nav_failure_note.clear()
            self.bridge.commands.put(('initial',self.initial_pose))
            self.log('已请求应用初始位置，请检查雷达与地图是否重合',pose_m_rad=self.initial_pose)

    def capture_pose(self,new=False):
        if not self.editing():return
        if not self.navigation_map_matches() or self.latest.get('pose_age',math.inf)>1 or self.latest.get('odom_age',math.inf)>1:
            self.error('需要当前地图的实时定位和里程计。');return
        v,w=self.latest['velocity']
        if abs(v)>.01 or abs(w)>.02 or self.latest['busy']:self.error('请在底盘停稳、导航结束后记录。');return
        if not self.confirm('确认当前定位与实车一致，将当前位置保存为预停靠位？'):return
        self.pose_drawn('new_station' if new or self.list.currentRow()<0 else 'dock',*self.latest['pose'])

    def emergency_stop(self):
        if self.bridge and not self.offline:
            self.bridge.emergency_stop()
            self.keyboard.disarm('紧急停车');self.latest['software_stop']=True
            self.log('紧急停车已请求：持续零速度、取消导航、锁定面板运动；等待实际停稳')
        else:self.stop_note.setText('离线预览：没有连接底盘，未发送停车命令。');return
        self.update_navigation_ui()

    def release_stop(self):
        if not self.bridge or self.offline:return
        if not self.confirm('确认车辆已经停稳、现场可以继续操作，解除软件停车锁定？\n解除后不会自动恢复键盘操控或导航任务。'):return
        try:self.bridge.release_stop()
        except ValueError as error:self.error(str(error));return
        self.latest=self.bridge.snapshot();self.log('已请求解除停车锁定，等待底盘与仲裁零速确认；不会自动恢复任务');self.update_navigation_ui()

    def application_state(self,state):
        if state!=QtCore.Qt.ApplicationActive:self.keyboard.disarm('窗口失去激活')

    def enable_keyboard(self):
        if self.docking_guard():return
        if self.offline or not self.bridge or self.latest.get('odom_age',math.inf)>1:
            self.error('请先连接底盘并等待里程计就绪；不需要启动建图或导航。');return
        if self.latest.get('busy'):
            self.error('请先取消导航，等待任务结束和底盘停稳，再开启键盘。');return
        if not self.confirm('开启键盘操控底盘？\n保持机械臂收拢，关闭其他键盘/遥控发送程序。手动控制不自动避障。\nW/S 前进后退，配合 A/D 转向；松键停车。'):return
        try:self.bridge.enable_manual()
        except ValueError as error:self.error(str(error));return
        self.keyboard.arm()
        self.log('键盘控制已开启',speed_m_s=self.speed_combo.currentData())

    def keyboard_released(self,reason):
        if self.bridge:self.bridge.disable_manual(reason)
        self.log('键盘控制退出：'+reason)

    def manual_heartbeat(self):
        if self.bridge:
            self.bridge.gui_heartbeat(self.document.fingerprint if self.navigation_map_matches() else None)
            if self.return_run_session:self.bridge.return_keepalive(self.return_run_session)
        if not self.keyboard.armed:return
        if self.latest.get('odom_age',math.inf)>1:
            self.keyboard.disarm('底盘反馈已中断');return
        if not self.bridge.update_manual(self.keyboard.keys(),self.speed_combo.currentData()):
            self.keyboard.disarm('控制已停止，请重新开启')

    def start_devices(self):
        if self.docking_guard():return
        if self.cleanup:self.error('正在关闭服务，请等待结束。');return False
        if self.offline:self.error('离线预览不启动硬件。');return False
        if self.base_process:return True
        if not self.graph_ready:self.error('正在读取 ROS 节点状态，请稍后再试。');return False
        if self.latest.get('odom_age',math.inf)<=1 and self.latest.get('scan_age',math.inf)<=1:
            self.log('底盘与雷达已连接，复用现有官方驱动。');return True
        occupied=[n for n in self.latest.get('nodes',[]) if n in ('lslidar_driver_node','ychd_robot_node','ychd_robot')]
        if occupied:self.error('已有部分设备驱动正在运行，请先检查对应窗口：'+', '.join(occupied));return False
        if not all(Path(p).exists() for p in ('/dev/ychd_controller','/dev/ychd_lidar')):
            self.error('底盘或雷达串口未连接。');return False
        proc=QtCore.QProcess(self);self.base_process=proc
        proc.setProcessChannelMode(proc.MergedChannels);proc.setWorkingDirectory(str(ROOT))
        proc.readyReadStandardOutput.connect(lambda:self.process_output(proc))
        proc.finished.connect(self.devices_finished);proc.errorOccurred.connect(lambda e:self.process_error(proc,e))
        args=['ros2','launch',str(ROOT/'navigation.launch.py'),'mode:=devices']
        proc.start('/usr/bin/setsid',args);self.log('连接底盘与雷达',command=args)
        return True

    def stop_devices(self):
        if self.docking_guard():return
        if not self.base_process:
            self.error('本面板没有启动设备进程；外部驱动请在原窗口关闭。');return
        if self.latest.get('busy'):
            self.error('请先取消正在执行的导航任务，待底盘停稳后断开。');return
        self.keyboard.disarm('断开设备')
        if not self.confirm('确认底盘已停稳，断开本面板的底盘与雷达连接？\n建图/定位进程不会被关闭，但将失去实时传感器数据。'):return
        pid=int(self.base_process.processId())
        if pid:os.killpg(pid,signal.SIGINT);self.log('请求断开设备',pid=pid)

    def devices_finished(self,code,status):
        self.base_process=None;self.keyboard.disarm('设备连接已结束')
        self.log('设备进程已退出',exit_code=code)

    def start_backend(self,mode):
        if self.docking_guard():return
        if self.cleanup:self.error('正在关闭服务，请等待结束。');return
        if self.offline:self.error('离线预览不启动硬件。');return
        if self.backend:
            self.error('建图和定位共享 map 坐标系，请先停止当前建图/定位模式再切换。');return
        if not self.graph_ready:self.error('正在读取 ROS 节点状态，请稍后再试。');return
        occupied=[n for n in self.latest.get('nodes',[]) if n in ('slam_gmapping','slam_toolbox','amcl','map_server','controller_server','nav2_container','planner_server','bt_navigator')]
        if occupied:self.error('已有建图/定位服务，请点击“关闭已有建图 / 定位服务”后重启：'+', '.join(occupied));return
        if not all(Path(p).exists() for p in ('/dev/ychd_controller','/dev/ychd_lidar')):
            self.error('底盘或雷达串口未连接。');return
        if mode=='navigation' and not self.document:self.error('请先加载地图。');return
        if not self.confirm('启动'+('建图' if mode=='mapping' else '定位导航')+'？设备未连接时将自动连接。\n启动本身不会发送导航目标。'):return
        if not self.start_devices():return
        self.backend_failure='';self.stopping_backend=False;self.nav_output_buffer='';self.nav_failure_note.clear()
        self.bridge.reset_view()
        self.view_mode='live' if mode=='mapping' else 'saved'
        if mode=='navigation':self.canvas.set_document(self.document)
        self.active_fingerprint=self.document.fingerprint if mode=='navigation' else None
        self.mode=mode;self.active_map=self.document.yaml_path if mode=='navigation' else None
        self.live_seq=-1;self.latest_map_seen=0;self.target=self.free_goal if mode=='navigation' else None;self.canvas.tool(None)
        args=['ros2','launch',str(ROOT/'navigation.launch.py'),'mode:='+mode]
        if self.active_map:args.append('map:='+self.active_map)
        proc=QtCore.QProcess(self);self.backend=proc
        proc.setProcessChannelMode(proc.MergedChannels);proc.setWorkingDirectory(str(ROOT))
        proc.readyReadStandardOutput.connect(lambda:self.process_output(proc))
        proc.finished.connect(self.backend_finished)
        proc.errorOccurred.connect(lambda e:self.process_error(proc,e))
        proc.start('/usr/bin/setsid',args)
        self.update_navigation_ui();self.log('启动模式 '+mode,command=args)
    def process_output(self,proc):
        text=bytes(proc.readAllStandardOutput()).decode(errors='replace')
        self.console.write(text);self.console.flush()
        if proc is self.backend:
            self.nav_output_buffer+=text
            lines=self.nav_output_buffer.split('\n');self.nav_output_buffer=lines.pop()
            for line in lines:
                if 'Starting point in lethal space' in line:
                    self.nav_failure_note.setText('无法规划：当前车体起始轮廓与代价地图障碍冲突。请核对初始定位、雷达与地图重合及车体尺寸；改终点不能解决起点冲突。')
                    self.log('导航规划失败：起点碰撞',raw=line.strip())
                elif 'goal is occupied' in line:
                    self.nav_failure_note.setText('无法规划：目标位置被占用。请核对目标位置与车头朝向。')
                    self.log('导航规划失败：目标占用',raw=line.strip())
        for line in text.splitlines():
            self.logview.appendPlainText(line)
            if proc is self.backend and ('[ERROR]' in line or 'exception' in line.lower()):self.backend_failure=line.strip()
    def process_error(self,proc,error):
        if error==QtCore.QProcess.FailedToStart:
            self.error('进程无法启动：'+proc.errorString())
            if proc is self.backend:self.backend_failure=proc.errorString();self.backend=None;self.mode=None
            if proc is self.base_process:self.base_process=None
            if proc is self.saver:self.saver=None
    def stop_backend(self):
        if self.docking_guard():return
        self.keyboard.disarm('停止底盘模式')
        if not self.backend:return
        if self.saver:self.error('地图仍在保存，请等待保存结果。');return
        if self.latest.get('busy'):self.cancel_navigation();self.error('已请求取消导航；等待结束并确认底盘停稳后，再停止当前模式。');return
        if not self.confirm('停止当前建图/定位进程？\n底盘与雷达保持连接。建图请先保存地图。'):return
        pid=int(self.backend.processId())
        if pid:self.stopping_backend=True;os.killpg(pid,signal.SIGINT);self.state.setText('正在停止');self.log('请求停止模式',pid=pid)
    def backend_finished(self,code,status):
        self.log('模式已退出',exit_code=code);self.backend=None;self.mode=None;self.active_map=None
        if self.stopping_backend:self.backend_failure=''
        elif code and not self.backend_failure:self.backend_failure='启动进程退出，返回码 '+str(code)
        self.stopping_backend=False;self.canvas.tool(None)
        self.active_fingerprint=None
        if self.document:self.set_view('saved')
        self.update_navigation_ui()
    def save_map(self):
        if self.mode!='mapping' or not self.backend:self.error('请先启动建图。');return
        if not self.latest_map_seen:self.error('尚未收到本次建图的地图。');return
        if self.latest.get('scan_age',math.inf)>1 or self.latest.get('odom_age',math.inf)>1:
            self.error('雷达或里程计已中断，请恢复后保存地图。');return
        if self.saver:return
        name=self.map_name.text().strip()
        if not name or Path(name).name!=name or name in ('.','..'):self.error('请填写地图名称，不含目录。');return
        prefix=ROOT/'maps'/name
        if Path(str(prefix)+'.yaml').exists() or Path(str(prefix)+'.pgm').exists():
            self.error('地图名称已存在，请换一个新名称，保留旧地图和点位。');return
        self.save_prefix=prefix
        proc=QtCore.QProcess(self);self.saver=proc
        proc.setProcessChannelMode(proc.MergedChannels);proc.readyReadStandardOutput.connect(lambda:self.process_output(proc))
        proc.finished.connect(self.map_saved);proc.errorOccurred.connect(lambda e:self.process_error(proc,e))
        proc.start('ros2',['run','nav2_map_server','map_saver_cli','-f',str(prefix),
                          '--ros-args','-p','save_map_timeout:=10.0'])
        self.log('正在保存地图 '+str(prefix))
    def map_saved(self,code,status):
        self.saver=None
        if code!=0:self.error('地图保存失败，请查看运行日志。');return
        self.refresh_maps()
        if self.load_map(str(self.save_prefix)+'.yaml',after_save=True):
            self.hint.setText('地图已保存。点击“地图 / 点位编辑”即可标注，建图可继续运行。')
            self.log('地图保存成功；可停止建图。')
    def navigate(self):
        if self.form_dirty and not self.apply_fields():return
        i=self.list.currentRow()
        if not self.stations or i<0 or not self.stations.points[i]['pre_dock']:
            self.error('请选择已设置预停靠位的充电点。');return
        p=self.stations.points[i];d=p['pre_dock']
        self.navigate_pose((d['x_m'],d['y_m'],d['yaw_rad']),p['name']+'预停靠位')

    def navigate_free(self):
        if self.free_goal is None:self.error('请先在地图选择目标位置和到达朝向。');return
        self.navigate_pose(self.free_goal,'地图选定目标')

    def navigate_pose(self,pose,label):
        if self.docking_guard():return
        if self.keyboard.armed or self.latest.get('manual') or self.latest.get('manual_stop_pending'):
            self.error('请先退出键盘控制，待底盘停稳后再导航。');return
        if self.offline or not self.navigation_map_matches():
            self.error('请启动定位导航，并选择它正在使用的地图后再发目标。');return
        if self.latest.get('pose_age',math.inf)>1 or self.latest.get('scan_age',math.inf)>1 or self.latest.get('odom_age',math.inf)>1:
            self.error('定位、雷达或里程计尚未就绪，请先设置初始位置并检查显示。');return
        if self.latest.get('busy'):self.error('已有导航任务，请先取消或等待完成。');return
        if not self.confirm('前往“{}”？\nX {:.2f} m，Y {:.2f} m，朝向 {:.1f}°\n确认定位正确、底盘通道畅通、工具已收拢。'.format(label,pose[0],pose[1],math.degrees(pose[2]))):return
        request_id=None
        self.imported_return_plan=None
        if self.chassis_session:
            self.chassis_session.stop('新导航请求，旧记录不自动恢复',time.monotonic());self.chassis_session=None
        if self.record_route.isChecked():
            self.latest=self.bridge.snapshot()
            try:
                self.chassis_session=RawChassisSession(self.document.fingerprint,self.start_store().path,self.logdir,self.latest,time.monotonic(),self.bridge.raw_samples_since)
                request_id=self.chassis_session.goal_id
            except (OSError,ValueError,KeyError) as error:self.error('无法开始实际轨迹记录：'+str(error));return
        try:self.bridge.request_goal(pose,request_id=request_id)
        except ValueError as error:self.error(str(error));return
        self.nav_failure_note.clear()
        self.target=pose
        self.log('请求前往 '+label,pose_m_rad=pose,map_sha256=self.document.fingerprint)
    def update_route(self):
        if not self.chassis_session:
            self.update_return_ui();return
        session=self.chassis_session
        try:
            sha=self.document.fingerprint if self.navigation_map_matches() else 'map-changed'
            session.update(self.latest,sha,time.monotonic())
            if session.core.phase=='outbound_stopped' and not session.candidate_attempted:
                try:session.return_candidate()
                except CycleError as error:self.log('不能生成纯倒车返程候选：'+str(error))
            self.route_note.setText(session.status())
        except (OSError,ValueError,KeyError) as error:
            session.core.stop_or_take_over('记录失败：'+str(error));self.route_note.setText(session.status())
        if session.core.phase=='blocked' and isinstance(session,RawChassisSession) and not getattr(session,'stop_requested',False):
            session.stop_requested=True
            if self.bridge:self.bridge.emergency_stop()
            self.log('原始去程记录失败，已请求停车：'+str(session.block_reason))
        self.update_return_ui()

    def import_return_route(self):
        self.error('原始里程模式必须同面板、同里程会话重新录去程，不能导入旧地图轨迹。');return
        if self.offline or not self.bridge or not self.document:return
        if self.docking_guard():return
        selected,_=W.QFileDialog.getOpenFileName(self,'选择原去程记录 cycle.json',str(ROOT/'logs'),'去程记录 (cycle.json)')
        if not selected:return
        try:
            source=load_source(Path(selected).parent,self.document.yaml_path)
            old=source['plan']
            if not self.confirm('将原轨迹几何保留，并绑定当前的新会话。\n来源轮次 '+old['cycle_id']+'\n请独立核实：车辆确在原记录停点附近、地标/激光与地图重合、通道清空、机构收拢、人员持续监护，物理断电由现场人员负责。\n不会平移旧路线、不会自动移动；重验后30秒内准备。'):return
            confirmations={key:True for key in ('at_recorded_stop_verified','landmark_alignment_checked',
                'operator_present','route_cleared','arm_stowed','physical_stop_user_managed')}
            self.imported_return_plan=self.bridge.revalidate_saved_route(Path(selected).parent,self.document.yaml_path,confirmations)
            receipt=self.logdir/('revalidated-'+self.imported_return_plan['cycle_id']+'.json')
            write_json(receipt,self.imported_return_plan)
            self.log('旧轨迹已显式重验，尚未执行',source_cycle_id=old['cycle_id'],new_cycle_id=self.imported_return_plan['cycle_id'],receipt=str(receipt))
            self.latest=self.bridge.snapshot();self.update_return_ui()
        except (OSError,ValueError,KeyError,TypeError) as error:
            self.imported_return_plan=None;self.error('旧轨迹重验失败：'+str(error))

    def prepare_return(self):
        if self.offline or not self.bridge or (not self.chassis_session and not self.imported_return_plan):return
        if self.docking_guard():return
        try:
            plan=self.imported_return_plan or self.chassis_session.return_candidate()
            if not self.confirm('按同会话原始里程轨迹闭环倒车，保持原车身朝向。地图仅用于去程导航。轮滑或被推动无法靠这套里程保证识别。\n请现场逐项确认：人员持续监护、通道清空、机械臂和枪头收拢、线缆无牵拉、实体断电按钮可及，其他遥控已交接。\n没有后向自动避障；记录轨迹无法发现新进入的障碍。\n'+('短测：0.05 m/s，最多3秒或实测0.15米，先到即请求停车。' if self.return_short.isChecked() else '完整返程：{:.2f} m/s，点击启动，点击停止返程取消；全程保持现场监护。'.format(self.latest.get('arbiter',{}).get('return_speed_m_s',.05)))):return
            confirmations={name:True for name in ('operator_present','route_cleared','physical_stop_ready',
                'arm_stowed','exclusive_control_confirmed','no_rear_detection_acknowledged')}
            self.bridge.prepare_return(plan,self.start_store().path,self.document.yaml_path,
                confirmations,short_test=self.return_short.isChecked())
            self.return_run_session=None
            self.log('返程已准备，尚未运动；点击启动才开始')
            self.latest=self.bridge.snapshot();self.update_return_ui()
        except (OSError,ValueError,KeyError,TypeError) as error:self.error('不能准备返程：'+str(error))

    def start_return(self):
        if not self.bridge:return
        try:
            self.return_run_session=self.bridge.start_return()
            self.log('已请求启动返程；停止、急停或故障后不自动续行')
        except (ValueError,TypeError) as error:
            self.return_hold.set_running(False);self.error('不能启动返程：'+str(error))

    def stop_return(self):
        self.return_run_session=None;self.return_hold.set_running(False)
        if self.bridge:self.bridge.stop_return('操作员停止返程')
        self.log('已请求停止返程，不自动恢复')

    def update_return_ui(self):
        state=self.latest.get('return_state',{})
        active=self.latest.get('return_active',False)
        arbiter=self.latest.get('arbiter',{})
        eligible=(not self.offline and (self.imported_return_plan is not None or (self.chassis_session is not None and
            self.chassis_session.core.phase=='outbound_stopped')) and
            not active and not self.latest.get('busy') and not self.latest.get('software_stop') and
            not self.latest.get('manual') and not self.docking_active and not self.cleanup and
            self.latest.get('arbiter_age',math.inf)<=.25 and arbiter.get('commissioned') is True and
            not arbiter.get('fault') and arbiter.get('mode')=='stop' and
            not (arbiter.get('release_profile')=='supervised_trial' and arbiter.get('full_trial_consumed')) and
            arbiter.get('raw_odom_ready') is True and self.latest.get('raw_odom') is not None and
            not self.latest.get('raw_odom_error'))
        self.return_button.setEnabled(eligible)
        short_only=arbiter.get('release_profile')=='short_only'
        full_trial=arbiter.get('release_profile')=='supervised_trial'
        if short_only:self.return_short.setChecked(True)
        if full_trial:self.return_short.setChecked(False)
        self.return_short.setEnabled(not active and not short_only and not full_trial)
        self.import_return_button.setEnabled(False)
        self.return_hold.setEnabled(active and arbiter.get('return_start_mode')=='click')
        if not active:self.return_run_session=None
        self.return_hold.set_running(bool(self.return_run_session))
        self.return_stop.setEnabled(active)
        labels={'idle':'未准备','ready':'已准备，等待点击启动','running':'正在沿实测轨迹倒车',
            'goal_wait_still':'已到记录起点，等待实测停稳','complete':'已回到本轮里程起点并停稳',
            'stopped':'返程已停止，需重新核查'}
        text=labels.get(state.get('phase'),'未准备')
        if state.get('reason'):text+='：'+str(state['reason'])
        if full_trial:text+='\n一次完整试验 · {:.2f} m/s · 单击启动 / 点击停止取消；运行后本次授权不再复用。'.format(arbiter.get('return_speed_m_s',.05))
        if not arbiter.get('commissioned'):text+='\n尚未完成现场验收，实车输出禁用。'
        if self.latest.get('raw_odom_error'):text+='\n原始里程未就绪：'+str(self.latest['raw_odom_error'])
        self.return_button.setToolTip(text)
        self.return_note.setText(text+'\n同会话原始里程闭环；轮滑无独立位置验证。人工持续监护，无后向自动避障。')
        if active:
            for button in (self.manual_button,self.connect_button,self.disconnect_button,self.nav_button,
                self.mapping_button,self.initial_button,self.apply_initial_button,self.go_button,
                self.free_go_button,self.recovery_button,self.stop_button,self.docking_button):button.setEnabled(False)

    def cancel_navigation(self):
        if self.latest.get('return_active'):self.stop_return();return
        if self.bridge:self.bridge.commands.put(('cancel',()));self.log('请求取消导航；等待服务器确认')
    def update_navigation_ui(self):
        loaded=self.document is not None;matching=self.navigation_map_matches()
        busy=self.latest.get('busy',False)
        stopped=self.latest.get('software_stop',False)
        manual=self.keyboard.armed or self.latest.get('manual',False) or self.latest.get('manual_stop_pending',False)
        reset_pending=bool(self.latest.get('stop_reset_pending'))
        self.release_stop_button.setEnabled(stopped and not self.offline and not reset_pending)
        self.manual_button.setEnabled(not self.offline and not stopped and self.latest.get('odom_age',math.inf)<=1 and not busy and not self.keyboard.armed)
        if stopped:
            v,w=self.latest.get('velocity',(0.,0.))
            observed=self.latest.get('odom_age',math.inf)<=1 and abs(v)<=.01 and abs(w)<=.02
            self.stop_note.setText('软件停车已锁定 · '+('里程计显示静止，请现场确认' if observed else '尚未确认停稳，请现场检查；必要时使用实体急停')+'\n持续发送零速度；解除后不会自动恢复任务。')
        if reset_pending:self.stop_note.setText('软件停车保持锁定 · 正在等待解除确认，请勿重复点击。\n底盘与仲裁确认前不允许运动，确认后也不自动恢复任务。')
        elif not stopped:self.stop_note.setText('底盘软件停车：立即发零速度、取消导航并锁定操控。\n不控制机械臂；不能替代实体急停。')
        localized=matching and self.latest.get('pose_age',math.inf)<=1.
        initial_ready=matching and self.latest.get('initial_ready',False)
        ready=localized and self.latest.get('nav_ready',False) and self.latest.get('scan_age',math.inf)<=1 and self.latest.get('odom_age',math.inf)<=1
        map_text='地图已加载：'+Path(self.document.yaml_path).name if loaded else '地图未加载'
        if self.offline:phase='离线预览';hint='可编辑地图、充电点并标记位置；实车定位需正常模式启动。'
        elif self.cleanup:phase='正在关闭服务';hint='等待导航取消和进程退出；完成后可重新启动。'
        elif self.backend_failure and self.mode is None:phase='定位 / 建图启动失败';hint=self.backend_failure+'\n地图仍已加载；修正后重新启动定位导航。'
        elif self.mode=='mapping':phase='正在建图';hint='初始位置可先标记。实际应用需要先停止建图，再启动本图定位导航。'
        elif self.mode=='navigation' and not matching:phase='正在定位另一张地图';hint='导航正在使用 '+Path(self.active_map).name+'；请加载该地图，或停止当前模式后切换。'
        elif matching and not initial_ready:phase='定位节点启动中';hint='可先标记初始位置；等待定位节点就绪后应用。若启动失败，原因会显示在这里。'
        elif matching and not localized:phase='等待车辆初始位置';hint='在地图标记车辆位置与车头朝向，再点“应用初始位置”。'
        elif matching and not ready:phase='定位已有反馈，等待导航 / 传感器';hint='检查雷达、里程计及导航服务；暂不发送导航目标。'
        elif matching:phase='定位就绪';hint='先检查雷达与地图重合，再选目标位置和朝向，确认后导航。'
        elif self.mode is None and any(n in self.latest.get('nodes',[]) for n in ('amcl','map_server','nav2_container','slam_gmapping','slam_toolbox')):
            phase='检测到已有建图 / 定位服务';hint='可能由上次面板或其他窗口启动。点击“关闭已有建图 / 定位服务”，结束后重新启动本图定位。'
        else:phase='定位未启动';hint='地图已加载，可先标车辆位置；点击“启动本图定位导航”后应用初始位置。' if loaded else '先加载已有地图；也可以独立使用设备连接和键盘操控。'
        if stopped:phase='软件停车锁定 · '+phase
        self.state.setText(map_text+'\n'+phase);self.nav_help.setText(hint)
        self.state.setToolTip(hint)
        self.initial_button.setEnabled(loaded and not busy)
        self.initial_button.setToolTip('先结束当前导航任务' if busy else '只标记位置和朝向；加载地图后即可使用，无需充电点')
        self.goal_button.setEnabled(loaded)
        can_apply=not self.offline and initial_ready and self.initial_pose is not None and not busy and not manual
        self.apply_initial_button.setEnabled(can_apply)
        self.apply_initial_button.setToolTip('标记位置后启动本图定位，待车辆停稳再应用' if not can_apply else '发布初始定位，不发运动命令')
        if self.initial_pose:
            x,y,yaw=self.initial_pose
            self.initial_note.setText('已标记 X {:.2f} m · Y {:.2f} m · {:.1f}°\n已保存参考，不代表当前定位。核实现车位置后再应用，并检查激光重合。'.format(x,y,math.degrees(yaw)))
        else:self.initial_note.setText('尚未标记车辆初始位置；在地图按住并拖出车头朝向。')
        can_go=not self.offline and ready and not busy and not manual and not stopped
        self.free_go_button.setEnabled(can_go and self.free_goal is not None)
        i=self.list.currentRow()
        self.go_button.setEnabled(can_go and self.stations is not None and i>=0 and self.stations.points[i]['pre_dock'] is not None)
        self.refresh_go_status()

        self.update_docking_ui()

    def refresh_go_status(self):
        reasons=navigation_reasons(self.latest,offline=self.offline,loaded=self.document is not None,
            matching=self.navigation_map_matches(),mode=self.mode,cleanup=bool(self.cleanup),
            docking=self.docking_active,manual=self.keyboard.armed or self.latest.get('manual',False) or
            self.latest.get('manual_stop_pending',False),backend_failure=self.backend_failure)
        free=list(reasons)
        if self.free_goal is None:free.append('请先在地图选择目标位置和朝向')
        station=list(reasons);i=self.list.currentRow()
        if self.stations is None or i<0:station.append('请先选择一个充电点')
        elif self.stations.points[i]['pre_dock'] is None:station.append('选中充电点尚未设置预停靠位')
        free_text=button_status(self.free_go_button.isEnabled(),free)
        station_text=button_status(self.go_button.isEnabled(),station)
        self.free_go_button.setToolTip(free_text);self.go_button.setToolTip(station_text)
        self.go_status.setText('选定目标：'+free_text+'\n预停靠位：'+station_text)

    def docking_guard(self):
        if self.docking_active:
            self.error('对接锁定中：先结束对接终端，现场确认枪头已退出、机械臂已收拢，再解除对接锁。底盘停车按钮仍可使用。')
            return True
        return False

    def update_docking_ui(self):
        if not hasattr(self,'docking_button'):return
        allowed,reason=readiness(self.latest,self.navigation_map_matches())
        if self.offline:reason='离线预览：不连接机器人，不允许打开运动工作流'
        if self.docking_active:reason='对接锁定中：本面板持续发零速度；机械臂状态请看终端并现场核实'
        error=goal_error(self.latest) if self.latest.get('pose_age',math.inf)<=1 else None
        if error:reason+='\n当前相对本次目标误差 {:.3f} m / {:.1f}°（仅定位估计）'.format(*error)
        self.docking_note.setText(reason)
        if not allowed and self.docking_conditions.isChecked():
            self.docking_conditions.blockSignals(True);self.docking_conditions.setChecked(False);self.docking_conditions.blockSignals(False)
        eligible=allowed and not self.offline and not self.cleanup and not self.docking_active
        self.docking_conditions.setEnabled(eligible)
        self.docking_button.setEnabled(eligible and self.docking_conditions.isChecked())
        running=self.docking_process is not None and self.docking_process.state()!=QtCore.QProcess.NotRunning
        self.docking_release_button.setEnabled(self.docking_active and not running and not self.offline)
        if self.docking_active:
            for button in (self.manual_button,self.connect_button,self.disconnect_button,self.nav_button,self.mapping_button,self.initial_button,self.apply_initial_button,self.go_button,self.free_go_button,self.recovery_button,self.stop_button):button.setEnabled(False)
        path=self.logdir/'docking-state.json'
        if self.docking_active and path.exists():
            try:
                data=json.loads(path.read_text());phase=data.get('phase')
                messages={'ready':'终端已打开，尚未执行命令','test-running':'test 命令进行中（可能正在等待人工确认）',
                    'test-returned':'test 返回成功；仅运动流程完成，未验证充电或锁止',
                    'home-running':'home 命令进行中；可能先退枪，实际状态需现场核实',
                    'home-returned':'home 返回成功；请现场确认机械臂收拢',
                    'uncertain':'命令中断/失败，实体状态未确认；不自动重试',
                    'launch-failed':'命令启动失败；未解除底盘对接锁',
                    'console-closed':'终端已关闭；底盘对接锁仍保持'}
                self.docking_result.setText(messages.get(phase,'对接终端状态未知，请现场核实'))
            except (OSError,ValueError):self.docking_result.setText('无法读取对接终端状态，保持对接锁；请现场核实')

    def open_docking_console(self):
        if self.offline or not self.bridge or self.docking_active or self.cleanup:return
        self.latest=self.bridge.snapshot()
        allowed,reason=readiness(self.latest,self.navigation_map_matches())
        if not allowed or not self.docking_conditions.isChecked():self.error(reason+'；需要现场勾选确认');return
        if not self.confirm('只锁定底盘并打开人工终端，不自动执行test。\n确认位置朝向合适、机械臂工作区安全、其他底盘控制源已停用。原test仍逐段确认，home需先确认断电解锁。'):return
        try:self.bridge.begin_docking()
        except ValueError as error:self.error(str(error));return
        self.docking_active=True;self.latest=self.bridge.snapshot()
        try:write_json(self.docking_marker,{'active':True,'session':self.session})
        except OSError as error:
            self.error('无法保存对接锁，未打开终端；本面板仍保持锁定：'+str(error));self.update_docking_ui();return
        self.keyboard.disarm('对接锁定');self.docking_conditions.setChecked(False)
        proc=QtCore.QProcess(self);self.docking_process=proc
        proc.errorOccurred.connect(self.docking_console_error)
        proc.finished.connect(self.docking_console_finished)
        proc.start('gnome-terminal',['--wait','--title=充电机器人 · 人工对接', '--', '/usr/bin/python3',str(ROOT/'docking_console.py'),'--state',str(self.logdir/'docking-state.json')])
        self.log('已锁定本面板底盘操控，打开人工对接终端；没有自动运行test或home')
        self.update_navigation_ui();self.update_docking_ui()

    def docking_console_error(self,error):
        if error==QtCore.QProcess.FailedToStart:
            self.docking_result.setText('无法打开终端，底盘对接锁仍保持；检查桌面会话与gnome-terminal')
            self.log('对接终端启动失败；保持对接锁')
            self.update_docking_ui()

    def docking_console_finished(self,code,status):
        self.log('对接终端已退出，不代表机械臂停止；对接锁保持',exit_code=code)
        self.update_docking_ui()

    def release_docking(self):
        if not self.docking_active or not self.bridge:return
        if self.docking_process is not None and self.docking_process.state()!=QtCore.QProcess.NotRunning:
            self.error('请先在对接终端完成操作并退出；不能把关闭窗口当成机械臂停止。');return
        if not self.confirm('解除底盘对接锁会允许后续导航。\n请现场确认：机械臂已实际停止并收拢，枪头已完全退出，线缆无牵拉风险。解除不会自动移动。'):return
        try:self.bridge.release_docking()
        except ValueError as error:self.error(str(error));return
        self.docking_active=False;self.latest=self.bridge.snapshot()
        try:write_json(self.docking_marker,{'active':False,'session':self.session})
        except OSError as error:self.error('本次已解除对接锁，但保存失败；重开面板可能仍锁定：'+str(error))
        self.docking_result.setText('已由现场人员确认解除对接锁；不会自动导航或恢复任务')
        self.log('现场确认后解除底盘对接锁，无自动恢复')
        self.update_navigation_ui();self.update_docking_ui()

    def repaint_map(self):
        overlay=(self.view_mode=='live' and self.mode=='mapping') or (self.view_mode=='saved' and self.navigation_map_matches())
        pose=self.latest.get('pose') if overlay and self.latest.get('pose_age',math.inf)<=1 else None
        scan=self.latest.get('scan_world',[]) if overlay and self.latest.get('scan_age',math.inf)<=1 else []
        points=self.stations.points if self.stations and self.view_mode=='saved' else []
        self.canvas.draw(points,self.list.currentRow(),pose,scan,self.latest.get('path',[]) if overlay and self.mode=='navigation' else [],self.target if self.view_mode=='saved' else None,initial=self.initial_pose if self.view_mode=='saved' else None)
    def tick(self):
        if self.bridge:
            self.latest=self.bridge.snapshot();self.graph_ready=self.latest['graph_ready']
            while not self.bridge.events.empty():
                event=self.bridge.events.get_nowait();self.log(event.pop('event'),**event)
            s=self.latest
            if self.mode=='mapping' and s['grid'] is not None and s['grid_seq']!=self.live_seq:
                self.live_document=MapDocument.from_grid(s['grid'])
                if self.view_mode=='live':self.canvas.set_document(self.live_document,fit=self.live_seq<0)
                self.live_seq=s['grid_seq'];self.latest_map_seen=time.monotonic()
            self.sensors.setText('雷达 {:.1f} Hz {}\n里程计 {} · 定位 {}'.format(s['scan_hz'],'✓' if s['scan_age']<=1 else '无新数据',
                '✓' if s['odom_age']<=1 else '等待','✓' if s['pose_age']<=1 else '等待初始位置/TF'))
            if s['pose'] and s['pose_age']<=1:
                x,y,a=s['pose'];self.pose_label.setText('底盘 X {:.3f} m   Y {:.3f} m   朝向 {:.1f}°   v {:.3f} m/s'.format(x,y,math.degrees(a),s['velocity'][0]))
            else:self.pose_label.setText('底盘位姿：暂无新鲜 map → base_footprint 定位')
            status=s['status']+(' · 剩余 {:.2f} m'.format(s['distance']) if s['distance'] is not None else '')
            self.nav_label.setText(status)
            if self.target and s.get('goal_status')==4 and s['pose_age']<=1:
                p=s['pose'];err=math.hypot(p[0]-self.target[0],p[1]-self.target[1]);angle=math.atan2(math.sin(p[2]-self.target[2]),math.cos(p[2]-self.target[2]))
                self.nav_label.setText('导航成功（Nav2容差）· 定位误差 {:.3f} m / {:.1f}°\n不是对接许可；等待连续停稳并现场确认'.format(err,abs(math.degrees(angle))))
        self.power_label.setText(power_text(self.latest.get('power',{}),self.latest.get('power48',{})))
        target=self.latest.get('manual_target',(0.,0.)) if self.latest.get('manual') else (0.,0.)
        if not self.offline and self.latest.get('odom_age',math.inf)<=1:
            v,w=self.latest['velocity']
            self.steering_note.setText('键盘意图 v {:.3f} m/s · ω {:+.3f} rad/s\n里程计反馈 v {:.3f} m/s · ω {:+.3f} rad/s（非舵角实测）'.format(*target,v,w))
        else:self.steering_note.setText('键盘意图 / 里程计反馈不可用；不是舵角测量')
        self.manual_button.setEnabled(not self.offline and self.latest.get('odom_age',math.inf)<=1 and not self.latest.get('busy',False) and not self.keyboard.armed)
        self.connect_button.setEnabled(not self.offline and self.base_process is None)
        self.disconnect_button.setEnabled(self.base_process is not None)
        self.speed_combo.setEnabled(not self.keyboard.armed)
        self.mapping_button.setEnabled(not self.offline and self.backend is None)
        self.nav_button.setEnabled(not self.offline and self.backend is None and self.document is not None)
        self.save_map_button.setEnabled(self.mode=='mapping' and self.saver is None)
        self.stop_button.setEnabled(self.backend is not None)
        self.update_navigation_ui()
        self.cancel_button.setEnabled(bool(self.latest.get('busy')))
        if self.stations:self.point_note.setText(('● 有未保存修改' if self.stations.dirty else '✓ 点位已保存 / 无修改')+'\n紫色充电口 · 蓝色预停靠')
        if self.cleanup:
            for button in (self.connect_button,self.disconnect_button,self.nav_button,self.mapping_button,self.manual_button,self.initial_button,self.apply_initial_button,self.go_button,self.free_go_button,self.recovery_button,self.stop_button):button.setEnabled(False)
        else:self.recovery_button.setEnabled(not self.offline)
        self.update_docking_ui()
        self.update_route()
        self.update_return_ui()
        self.repaint_map()
    def cleanup_services(self,exit_after=False):
        if self.docking_guard():return
        if self.cleanup:return
        if self.offline and not exit_after:self.error('离线预览不会关闭真实服务。');return
        if self.saver:self.error('地图正在保存，请等待保存结束再关闭服务。');return
        group=int(self.base_process.processId()) if exit_after and self.base_process else None
        processes=find_services(ROOT,device_group=group) if not self.offline else []
        if not processes:
            if exit_after:self.allow_close=True;QtCore.QTimer.singleShot(0,self.close)
            else:self.log('未发现残留建图/定位进程，可重新启动。')
            return
        listing='\n'.join('{}  {}'.format(p['pid'],' '.join(p['argv'][:3])) for p in processes)
        action='停止以下建图/定位及本面板设备进程，并退出面板？' if exit_after else '关闭以下建图 / 定位服务？底盘、雷达连接保留。'
        if not self.confirm(action+'\n如有导航任务将先请求取消。确认车辆可安全停稳。未保存的实时建图会丢失，已保存地图和点位保留。\n'+listing):return
        self.keyboard.disarm('关闭服务')
        self.cleanup={'processes':processes,'exit':exit_after,'phase':'cancel','deadline':time.monotonic()+8.}
        self.stopping_backend=True
        if self.latest.get('busy'):self.cancel_navigation()
        self.cleanup_timer.start();self.log('准备关闭服务',exit_after=exit_after,pids=[p['pid'] for p in processes])
        self.update_navigation_ui()

    def poll_cleanup(self):
        job=self.cleanup
        if not job:return
        if job['phase']=='cancel':
            v,w=self.latest.get('velocity',(0.,0.))
            moving=self.latest.get('odom_age',math.inf)<=1 and (abs(v)>.01 or abs(w)>.02)
            if self.latest.get('busy') or moving or self.latest.get('manual_stop_pending'):
                if time.monotonic()>job['deadline']:
                    self.cleanup_timer.stop();self.cleanup=None;self.stopping_backend=False
                    self.error('导航取消或停稳尚未确认，未关闭控制进程。请先现场停稳并检查导航状态，再重试关闭。')
                return
            self.log('提交服务退出',results=send_signal(shutdown_targets(job['processes'])))
            job.update(phase='wait',deadline=time.monotonic()+20.);return
        alive=remaining(job['processes'])
        if not alive:
            self.cleanup_timer.stop();self.cleanup=None;self.backend_failure=''
            self.log('所选服务进程已退出；已保存地图和点位保留')
            if self.bridge:self.bridge.reset_view()
            self.update_navigation_ui()
            if job['exit']:self.allow_close=True;self.close()
            return
        if time.monotonic()<job['deadline']:return
        self.cleanup_timer.stop()
        if job['phase']=='force':
            self.cleanup=None;self.stopping_backend=False
            self.error('仍有进程未退出：'+', '.join(str(p['pid']) for p in alive));return
        if self.latest.get('busy'):
            self.cleanup=None;self.stopping_backend=False;self.error('检测到导航任务，未强制关闭。请先取消任务并停稳。');return
        if not self.confirm('以下进程未正常退出：'+', '.join(str(p['pid']) for p in alive)+'\n确认车辆已停稳，强制关闭这些进程？不会删除已保存地图或点位。'):
            self.cleanup=None;self.stopping_backend=False;return
        self.log('强制关闭残留进程',results=send_signal(alive,signal.SIGKILL))
        job.update(phase='force',deadline=time.monotonic()+3.);self.cleanup_timer.start()

    def closeEvent(self,event):
        if self.docking_active:
            event.ignore();self.docking_guard();return
        if not self.allow_close:
            event.ignore()
            if self.cleanup:return
            if not self.dirty_guard():return
            self.cleanup_services(exit_after=True);return
        self.keyboard.disarm('关闭窗口')
        self.timer.stop();self.manual_timer.stop();self.cleanup_timer.stop()
        if self.bridge:self.bridge.close()
        self.events.close();self.console.close();event.accept()


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--offline',action='store_true');args=parser.parse_args()
    app=W.QApplication(sys.argv);app.setApplicationName('Charging Navigation')
    lock=QtCore.QLockFile('/tmp/charging_navigation_panel.lock');lock.setStaleLockTime(0)
    if not lock.tryLock(0):W.QMessageBox.information(None,'面板已运行','请使用已打开的充电导航工作台。');return
    window=Window(args.offline);window.show();sys.exit(app.exec_())

if __name__=='__main__':main()
