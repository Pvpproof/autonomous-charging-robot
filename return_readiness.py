"""Explain return availability from observations only; never issues commands."""
import math
from raw_odom_mode import scope,sample_now
from route_cycle import near


def fresh(value,limit):
    return type(value) in (int,float) and math.isfinite(value) and 0<=value<=limit


def return_reasons(s,recording,*,now,offline=False,cleanup=False,docking=False,starting=False):
    reasons=[]
    if offline:reasons.append('离线预览不能返程')
    if cleanup:reasons.append('正在关闭服务')
    if recording is None:reasons.append('尚无完整去程记录')
    else:
        phase=recording.core.phase
        if phase=='blocked':reasons.append('去程记录已失效：'+str(getattr(recording,'block_reason','请重新录制')))
        elif phase!='outbound_stopped':reasons.append('去程尚未完整到达并停稳')
        elif not getattr(recording,'return_eligible',True):
            reasons.append('暂不能返程：'+getattr(recording,'return_qualification',{}).get('reason','未通过返程检查'))
        result=s.get('return_state',{})
        if result.get('phase')=='complete' and result.get('cycle_id')==getattr(recording,'cycle_id',None):
            reasons.append('本轮返程已完成，请重新录去程')
    active=s.get('return_active',False)
    if active and not starting:reasons.append('返程已准备或运行中')
    if starting and (not active or s.get('return_state',{}).get('phase')!='ready'):
        reasons.append('返程尚未就绪')
    if s.get('motion_recovery_required'):reasons.append('底盘运动未放行，请确认底盘就绪')
    if s.get('software_stop'):reasons.append('软件停车锁定中')
    if s.get('stop_reset_pending'):reasons.append('等待解除停车确认')
    if s.get('manual') or s.get('manual_stop_pending'):reasons.append('请先退出键盘并等待停稳')
    if docking or s.get('docking_hold'):reasons.append('底盘被机械臂工具占用，请确认收拢后释放')
    if s.get('navigation_busy',bool(s.get('busy') and not active)):reasons.append('导航任务尚未结束')
    a=s.get('arbiter',{})
    if not fresh(s.get('arbiter_age'),.25):reasons.append('底盘控制反馈未就绪或已过期')
    elif a.get('commissioned') is not True:reasons.append('返程配置不可用：'+str(a.get('release_errors') or '等待控制器'))
    elif a.get('fault'):reasons.append('底盘故障：'+str(a['fault']))
    elif a.get('graph_ready') is not True or a.get('driver_gate_verified') is not True:
        reasons.append('底盘控制链尚未核实')
    elif a.get('owner')!=s.get('control_owner'):reasons.append('底盘控制权尚未交接给本面板')
    elif a.get('mode')!='stop':reasons.append('底盘控制权尚未交接到停车状态')
    if a.get('release_profile')=='supervised_trial' and a.get('full_trial_consumed'):
        reasons.append('仍在旧一次性试验模式：授权已使用')
    if a.get('return_start_mode')!='click':reasons.append('当前控制器未启用单击返程')
    if s.get('raw_odom_error'):reasons.append('原始里程异常：'+str(s['raw_odom_error']))
    elif a.get('raw_odom_ready') is not True or not s.get('raw_odom'):
        reasons.append('原始里程尚未就绪')
    else:
        try:
            p=sample_now(s,now)
            expected=getattr(recording,'scope',None) if recording is not None else None
            if expected is not None and scope(s)!=expected:
                reasons.append('里程或控制会话已变化，请重新录制')
            samples=getattr(recording,'samples',None) if recording is not None else None
            if samples and not near(p['pose'],samples[-1]['pose'],.10,.10):
                reasons.append('车辆已离开本轮记录停点')
        except (ValueError,KeyError,TypeError):reasons.append('原始里程反馈过期或来源信息不完整')
    if not fresh(s.get('scan_age'),.25):reasons.append('雷达反馈未就绪或已过期')
    quiet=s.get('raw_stationary_s',0)
    if not (type(quiet) in (int,float) and math.isfinite(quiet) and quiet>=2 and
            type(s.get('raw_stationary_samples')) is int and s['raw_stationary_samples']>=3):
        reasons.append('等待底盘连续停稳两秒')
    return list(dict.fromkeys(reasons))
