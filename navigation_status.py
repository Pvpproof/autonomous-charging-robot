"""Explain existing navigation gates. Does not enable buttons or issue commands."""
import math


def navigation_reasons(snapshot, *, offline, loaded, matching, mode, cleanup,
                       docking, manual, backend_failure=''):
    reasons=[]
    if offline:reasons.append('离线预览不能发送实车导航目标')
    if cleanup:reasons.append('正在关闭服务，请等待进程退出')
    if docking:reasons.append('底盘对接锁定中，请先完成对接并现场确认解除')
    if snapshot.get('return_active'):reasons.append('返程已准备或进行中，请先停止并确认停稳')
    if snapshot.get('motion_recovery_required'):reasons.append('面板已恢复，底盘运动未放行；请确认底盘就绪')
    if snapshot.get('software_stop'):reasons.append('软件停车已锁定，请停稳后手动解除')
    if snapshot.get('busy'):reasons.append('已有导航或底盘任务尚未结束')
    if manual:reasons.append('键盘操控或停车请求尚未结束，请先退出并等待停稳')
    if not loaded:reasons.append('尚未加载地图')
    elif not matching:
        if mode=='mapping':reasons.append('当前是建图模式，需先保存地图并切换定位导航')
        elif mode=='navigation':reasons.append('当前显示地图与正在定位的地图不一致')
        elif backend_failure:reasons.append('定位导航启动失败：'+backend_failure)
        else:reasons.append('尚未启动本图定位导航')
    elif not offline:
        if not snapshot.get('nav_ready'):
            reasons.append('导航服务尚未就绪；等待规划/控制组件初始化，持续未就绪请查看运行日志')
            if backend_failure:reasons.append('运行日志提示：'+backend_failure)
        for field,label in [('pose_age','定位'),('scan_age','雷达'),('odom_age','里程计')]:
            age=snapshot.get(field,math.inf)
            if not age<=1.:
                if not math.isfinite(age):reasons.append('尚无可用的'+label+'反馈')
                else:reasons.append('{}反馈已过期（{:.2f}s > 1.00s）'.format(label,age))
    return reasons


def button_status(enabled,reasons):
    if reasons:return '暂不可用：'+'；'.join(reasons)
    return '可用：确认后发送导航目标' if enabled else '等待界面状态刷新；请查看运行日志'
