"""Describe actual control state; GUI delay is not operator takeover."""
import copy
FLAGS=('software_stop','motion_recovery_required','manual','manual_stop_pending','docking_hold','cancel_requested')

def control_evidence(s):
    result={k:bool(s.get(k)) for k in FLAGS}
    result.update(stop_cause=copy.deepcopy(s.get('stop_cause')),gui_age_s=s.get('gui_age_s'),
                  control_mode=s.get('control_mode'),arbiter_fault=s.get('arbiter',{}).get('fault'))
    return result

def recording_block_reason(s):
    if s.get('software_stop'):
        cause=s.get('stop_cause') or {}
        return '软件停车：'+str(cause.get('detail') or '来源未记录')
    if s.get('cancel_requested'):return '当前去程已请求停止或取消'
    if s.get('motion_recovery_required'):return '底盘运动尚未放行'
    if s.get('manual'):return '键盘控制已开启，去程控制权已改变'
    if s.get('manual_stop_pending'):return '键盘停车请求尚未完成'
    if s.get('docking_hold'):return '机械臂工具占用底盘'
    return None

FAULT_LABELS={
    'panel authority heartbeat expired':'后台底盘控制指令中断',
    'navigation command expired':'Nav2速度指令中断',
    'odometry feedback expired':'底盘里程反馈中断',
    'unexpected chassis command graph':'底盘控制源或连接图异常',
    'navigation reacquired control during return':'返程期间出现导航控制冲突',
}

def fault_detail(reason):return FAULT_LABELS.get(reason,'底盘控制异常：'+str(reason))
