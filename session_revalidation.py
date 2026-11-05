"""Explicit saved-route revalidation. Original geometry and provenance stay intact."""
import copy
import hashlib
import json
import math
from pathlib import Path
import uuid
from route_cycle import CycleError,near
from model import MapDocument
from return_map import saved_grid_digest

FILES=('cycle.json','actual-trace.jsonl','return-candidate.json')
CONFIRMATIONS=('at_recorded_stop_verified','landmark_alignment_checked','operator_present',
               'route_cleared','arm_stowed','physical_stop_user_managed')


def canonical(value):
    return json.dumps(value,sort_keys=True,separators=(',',':'),allow_nan=False).encode()


def digest(value):return hashlib.sha256(canonical(value)).hexdigest()


def load_source(directory,map_path):
    from return_runtime import check_plan
    directory=Path(directory).resolve()
    hashes={}
    for name in FILES:
        p=directory/name
        if p.is_symlink():raise CycleError('来源记录不能是符号链接')
        hashes[name]=hashlib.sha256(p.read_bytes()).hexdigest()
    cycle=json.loads((directory/'cycle.json').read_text())
    plan=json.loads((directory/'return-candidate.json').read_text())
    trace=[json.loads(x) for x in (directory/'actual-trace.jsonl').read_text().splitlines() if x.strip()]
    if (cycle.get('phase')!='outbound_stopped' or cycle.get('coordinate_error') is not None or
            cycle.get('trace')!=trace or cycle.get('coordinate_samples')!=plan.get('coordinate_samples') or
            len(trace)!=len(cycle.get('coordinate_samples',[])) or len(trace)<2 or
            plan.get('executable') is not False):raise CycleError('来源去程不完整或配对检查点不一致')
    if not all(cycle.get(k)==plan.get(k) for k in ('cycle_id','map_sha256','localization_epoch')):
        raise CycleError('来源轮次、地图或定位会话不匹配')
    last_moving=None
    for a,b in zip(trace,plan['coordinate_samples']):
        if a['pose']!=b['pose'] or a['source_stamp']!=b['source_stamp']:
            raise CycleError('来源配对未逐点绑定实际轨迹')
        if not math.isfinite(a['observed_v_m_s']) or a['observed_v_m_s']<-.01:
            raise CycleError('来源去程包含倒车或无效速度')
        angular=b.get('observed_w_rad_s')
        odom_stamp=b.get('odom_source_stamp')
        if (not isinstance(angular,(int,float)) or not math.isfinite(angular) or
                not isinstance(odom_stamp,(int,float)) or not math.isfinite(odom_stamp) or
                abs(a['source_stamp']-odom_stamp)>.25):
            raise CycleError('来源缺少新鲜角速度/里程计时间证据')
        if abs(a['observed_v_m_s'])>.01 or abs(angular)>.02:last_moving=a['received_at']
    for a,b in zip(trace,trace[1:]):
        if not 0<b['received_at']-a['received_at']<=1.:
            raise CycleError('来源反馈中断或乱序')
    if last_moving is None or trace[-1]['received_at']-last_moving<2.:
        raise CycleError('来源没有完整的实际行驶与末尾连续停稳证据')
    check_plan(plan)
    document=MapDocument.load(map_path)
    grid=saved_grid_digest(map_path)
    if document.fingerprint!=plan['map_sha256'] or any(p['map_grid_sha']!=grid for p in plan['coordinate_samples']):
        raise CycleError('来源地图与当前保存地图不一致')
    return dict(directory=str(directory),files=hashes,plan=plan,source_digest=digest(hashes))


def runtime_identity(snapshot):
    identity=snapshot.get('runtime_identity')
    if not isinstance(identity,dict):raise CycleError('缺少当前进程与仲裁身份')
    for key in ('boot_id','driver_identity','localizer_identity','arbiter_instance','panel_epoch'):
        if not identity.get(key):raise CycleError('当前运行身份未核实：'+key)
    if identity['panel_epoch']!=snapshot.get('localization_epoch'):
        raise CycleError('运行身份与定位会话不符')
    return copy.deepcopy(identity)


def revalidate(directory,map_path,snapshot,confirmations,now):
    from return_runtime import coordinate_sample
    if not all(confirmations.get(k) is True for k in CONFIRMATIONS):
        raise CycleError('需要本轮现场位置、地标重合、监护和机构/通道确认')
    if snapshot.get('busy') or snapshot.get('manual') or snapshot.get('manual_stop_pending') or snapshot.get('docking_hold'):
        raise CycleError('当前控制任务尚未交接')
    if snapshot.get('software_stop'):raise CycleError('软件停车仍锁定；不自动解除')
    if (snapshot.get('stationary_since') is None or snapshot.get('stationary_samples',0)<3 or
            not math.isfinite(now) or now-snapshot['stationary_since']<2 or
            abs(snapshot['velocity'][0])>.01 or abs(snapshot['velocity'][1])>.02):
        raise CycleError('没有当前新鲜连续停稳证据')
    source=load_source(directory,map_path)
    point=coordinate_sample(snapshot,now)
    original=source['plan']
    if snapshot.get('map_sha256')!=original['map_sha256'] or point['map_grid_sha']!=original['coordinate_samples'][-1]['map_grid_sha']:
        raise CycleError('实时地图与来源记录不一致')
    if not near(point['pose'],original['poses'][0],.10,.10):
        raise CycleError('当前实际停点不在原记录终点范围内；不平移旧路线')
    identity=runtime_identity(snapshot)
    plan=copy.deepcopy(original)
    source_geometry=digest(original['poses'])
    plan.update(cycle_id=uuid.uuid4().hex,goal_id=uuid.uuid4().hex,
                localization_epoch=snapshot['localization_epoch'],executable=False,
                source_cycle_id=original['cycle_id'],source_localization_epoch=original['localization_epoch'])
    cert=dict(schema_version=1,certificate_id=uuid.uuid4().hex,issued_at=now,expires_at=now+30.,
              source_directory=source['directory'],source_files=source['files'],source_digest=source['source_digest'],
              source_geometry_sha256=source_geometry,source_cycle_id=original['cycle_id'],
              source_epoch=original['localization_epoch'],new_epoch=snapshot['localization_epoch'],
              runtime_identity=identity,execution_anchor=point,confirmations=copy.deepcopy(confirmations),
              new_cycle_id=plan['cycle_id'],new_goal_id=plan['goal_id'],
              physical_estop_evidence='user-verified direct power cut; not tested by this program')
    cert['integrity_sha256']=digest(cert)
    plan['revalidation']=cert
    return plan


def validate_binding(plan,snapshot,now,*,starting):
    cert=plan.get('revalidation')
    if not cert:return None
    body=copy.deepcopy(cert);integrity=body.pop('integrity_sha256',None)
    if digest(body)!=integrity:raise CycleError('会话重验记录完整性失败')
    if (cert['new_epoch']!=plan['localization_epoch'] or cert['new_cycle_id']!=plan['cycle_id'] or
            cert['new_goal_id']!=plan['goal_id'] or cert['source_geometry_sha256']!=digest(plan['poses'])):
        raise CycleError('重验身份或原始几何被改变')
    if runtime_identity(snapshot)!=cert['runtime_identity']:
        raise CycleError('重验后驱动、定位、面板或仲裁身份改变')
    if starting and not cert['issued_at']<=now<=cert['expires_at']:
        raise CycleError('会话重验已过期，请重新检查现场')
    if not all(cert['confirmations'].get(k) is True for k in CONFIRMATIONS):
        raise CycleError('重验现场确认缺失')
    directory=Path(cert['source_directory'])
    hashes={name:hashlib.sha256((directory/name).read_bytes()).hexdigest() for name in FILES}
    if hashes!=cert['source_files'] or digest(hashes)!=cert['source_digest']:
        raise CycleError('来源记录在重验后被修改')
    return copy.deepcopy(cert['execution_anchor'])
