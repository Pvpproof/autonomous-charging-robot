"""Stationary pose selection. No ROS, camera, or motion commands."""
import json
import math
import time
import uuid
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation

POLICIES = {
    'camera': dict(min_samples=6, min_span_s=1.0, window_s=3.0, timeout_s=12.0,
                   translation_m=0.003, rotation_deg=3.0, min_fraction=0.85),
    'gun': dict(min_samples=15, min_span_s=1.0, window_s=2.5, timeout_s=25.0,
                translation_m=0.0015, rotation_deg=1.5, min_fraction=0.85),
}


def atomic_json(path, data):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_name(path.name + '.' + uuid.uuid4().hex + '.tmp')
    tmp.write_text(json.dumps(data, indent=2, allow_nan=False))
    tmp.replace(path)


def matrix(value):
    m = np.asarray(value, dtype=float)
    if (m.shape != (4, 4) or not np.isfinite(m).all()
            or not np.allclose(m[3], [0, 0, 0, 1])
            or not np.allclose(m[:3,:3].T @ m[:3,:3], np.eye(3), atol=1e-3)
            or abs(np.linalg.det(m[:3,:3]) - 1) > 1e-3):
        raise ValueError('Invalid rigid transform')
    return m


def select(samples, policy):
    """Select a measured medoid; never average separated rotations."""
    if len(samples) < policy['min_samples']:
        raise ValueError('not_enough_distinct_measurements')
    stamps = np.array([s['stamp'] for s in samples], float)
    if not np.isfinite(stamps).all() or len(set(stamps)) != len(stamps) or np.any(np.diff(stamps) <= 0):
        raise ValueError('measurement_timestamps_not_unique_increasing')
    if stamps[-1] - stamps[0] < policy['min_span_s']:
        raise ValueError('measurement_span_too_short')
    transforms = np.array([matrix(s['matrix']) for s in samples])
    xyz = transforms[:,:3,3]
    dt = np.linalg.norm(xyz[:,None,:] - xyz[None,:,:], axis=-1)
    rotations = transforms[:,:3,:3]
    dr = np.empty_like(dt)
    for i, r in enumerate(rotations):
        dr[i] = np.degrees(Rotation.from_matrix(r.T @ rotations).magnitude())
    neighbors = (dt <= policy['translation_m']) & (dr <= policy['rotation_deg'])
    counts = neighbors.sum(1)
    # Minimize distance among equally supported candidates; stable deterministic tie-break.
    costs = np.mean(dt / policy['translation_m'] + dr / policy['rotation_deg'], axis=1)
    index = min(range(len(samples)), key=lambda i: (-counts[i], costs[i]))
    ids = np.flatnonzero(neighbors[index])
    if len(ids) < policy['min_samples'] or len(ids) / len(samples) < policy['min_fraction']:
        raise ValueError('multiple_pose_clusters_or_too_many_outliers')
    if stamps[ids[-1]] - stamps[ids[0]] < policy['min_span_s']:
        raise ValueError('inlier_span_too_short')
    if dt[np.ix_(ids,ids)].max() > policy['translation_m'] or dr[np.ix_(ids,ids)].max() > policy['rotation_deg']:
        raise ValueError('pose_spread_or_drift_too_large')
    if len(samples)-1 not in ids:
        raise ValueError('latest_measurement_disagrees_with_cluster')
    return samples[index], dict(total_samples=len(samples), inlier_samples=len(ids),
        rejected_samples=len(samples)-len(ids), span_s=float(stamps[-1]-stamps[0]),
        inlier_span_s=float(stamps[ids[-1]]-stamps[ids[0]]),
        translation_spread_mm=float(dt[np.ix_(ids,ids)].max()*1000),
        rotation_spread_deg=float(dr[np.ix_(ids,ids)].max()),
        selected_measurement_time=float(stamps[index]), latest_measurement_time=float(stamps[-1]),
        method='measured_pose_medoid', policy=policy)


def collect(reader, stage, report_path):
    policy = POLICIES[stage].copy()
    started = time.time(); deadline = time.monotonic()+policy['timeout_s']
    samples = []; seen = set(); last_error = 'waiting_for_measurements'; last_seen = None
    print(f"[稳定取值] {stage}: 等待至少{policy['min_samples']}个不同测量，跨度至少{policy['min_span_s']}秒", flush=True)
    while time.monotonic() < deadline:
        try:
            sample = reader()
            stamp = sample['stamp']
            if isinstance(stamp, bool) or not isinstance(stamp, (int,float)) or not math.isfinite(stamp):
                raise ValueError('invalid_measurement_time')
            now = time.time()
            if stamp < started or not 0 <= now-stamp <= 1.0:
                raise ValueError('waiting_for_new_fresh_measurement')
            matrix(sample['matrix'])
            if stamp not in seen:
                if last_seen is not None and stamp <= last_seen:
                    raise ValueError('measurement_clock_reversed')
                seen.add(stamp); last_seen=stamp; samples.append(sample)
            samples = [s for s in samples if stamp-s['stamp'] <= policy['window_s']]
            chosen, stats = select(samples, policy)
            atomic_json(report_path, dict(status='accepted', stage=stage, selected_at=now,
                statistics=stats, selected=chosen, samples=samples))
            print(f"[稳定取值通过] {stats['inlier_samples']}/{stats['total_samples']}帧，位置极差{stats['translation_spread_mm']:.3f}mm，方向极差{stats['rotation_spread_deg']:.3f}度", flush=True)
            return chosen, stats
        except (OSError, ValueError, KeyError, TypeError) as error:
            last_error = str(error)
            # Invalid or unavailable observations break the window; insufficient
            # sample count/spread simply waits for subsequent fresh measurements.
            if last_error in ('invalid_measurement_time','measurement_clock_reversed','Invalid rigid transform') or isinstance(error,(OSError,KeyError,TypeError)) or last_error.startswith(('invalid_visual:', 'invalid_request:')):
                samples=[]
        time.sleep(0.03)
    atomic_json(report_path, dict(status='refused', stage=stage, reason=last_error,
        policy=policy, samples=samples, requested_at=started))
    raise RuntimeError('稳定取值失败：'+last_error+'；保持静止并等待视觉有效后重新规划')
