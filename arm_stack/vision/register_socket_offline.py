#!/usr/bin/env python3
"""Offline RGB-D point-cloud registration for a GB/T 20234.2 AC vehicle inlet.

No robot connection and no motion commands. The source is in socket coordinates;
the target is a RealSense color-optical point cloud in metres. The output is
T_camera_from_socket such that P_camera = T @ P_socket.
"""
from __future__ import annotations
import argparse, json, time
from pathlib import Path
import numpy as np
import cv2

try:
    import open3d as o3d
except ImportError as exc:
    raise SystemExit("open3d is required; install it in the active Python environment") from exc


def read_matrix(path: Path) -> np.ndarray:
    a = np.asarray(json.loads(path.read_text()), dtype=float)
    if a.shape != (4, 4):
        raise ValueError(f"expected 4x4 matrix in {path}, got {a.shape}")
    return a


def parse_roi(text: str | None, width: int, height: int) -> tuple[int, int, int, int] | None:
    if not text:
        return None
    vals = [int(v) for v in text.split(",")]
    if len(vals) != 4:
        raise ValueError("ROI must be x,y,w,h")
    x, y, w, h = vals
    x, y = max(0, x), max(0, y)
    w, h = min(w, width - x), min(h, height - y)
    if w < 10 or h < 10:
        raise ValueError(f"ROI is too small or outside image: {vals}")
    return x, y, w, h


def crop_mask(mask: np.ndarray, roi: tuple[int, int, int, int] | None) -> np.ndarray:
    if roi is None:
        return mask
    x, y, w, h = roi
    out = np.zeros_like(mask)
    out[y:y+h, x:x+w] = mask[y:y+h, x:x+w]
    return out


def make_target(color_path: Path, depth_path: Path, meta: dict, roi, min_depth, max_depth):
    color = cv2.imread(str(color_path), cv2.IMREAD_COLOR)
    depth_raw = cv2.imread(str(depth_path), cv2.IMREAD_UNCHANGED)
    if color is None or depth_raw is None:
        raise FileNotFoundError("could not read color/depth image")
    if depth_raw.ndim != 2:
        raise ValueError("depth image must be single-channel uint16")
    h, w = depth_raw.shape
    if color.shape[:2] != (h, w):
        raise ValueError("color and depth must already be aligned to the same pixel grid")
    fx, fy = float(meta["fx"]), float(meta["fy"])
    cx, cy = float(meta["cx"]), float(meta["cy"])
    scale = float(meta["depth_scale"])
    z = depth_raw.astype(np.float32) * scale
    valid = np.isfinite(z) & (z >= min_depth) & (z <= max_depth) & (depth_raw > 0)
    valid = crop_mask(valid, roi)
    # Preserve the original aligned depth; mask only invalid/out-of-ROI pixels.
    masked = depth_raw.copy()
    masked[~valid] = 0
    intrinsic = o3d.camera.PinholeCameraIntrinsic(w, h, fx, fy, cx, cy)
    depth = o3d.geometry.Image(masked)
    rgb = o3d.geometry.Image(cv2.cvtColor(color, cv2.COLOR_BGR2RGB))
    rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
        rgb, depth, depth_scale=1.0 / scale, depth_trunc=max_depth,
        convert_rgb_to_intensity=False)
    cloud = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, intrinsic)
    # Open3D uses the pinhole optical convention: +X right, +Y down, +Z forward.
    return cloud, color, depth_raw, valid, {"width": w, "height": h, "fx": fx, "fy": fy, "cx": cx, "cy": cy, "depth_scale": scale}


def clean_cloud(cloud: o3d.geometry.PointCloud, voxel: float, nb: int, std: float):
    c = cloud.voxel_down_sample(voxel) if voxel > 0 else cloud
    if len(c.points) >= nb + 1:
        c, _ = c.remove_statistical_outlier(nb_neighbors=nb, std_ratio=std)
    c.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=max(voxel * 3.0, 0.006), max_nn=40))
    return c


def preprocess(pcd: o3d.geometry.PointCloud, voxel: float):
    p = pcd.voxel_down_sample(voxel)
    p.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=voxel * 2.5, max_nn=50))
    f = o3d.pipelines.registration.compute_fpfh_feature(
        p, o3d.geometry.KDTreeSearchParamHybrid(radius=voxel * 5.0, max_nn=100))
    return p, f


def transform_cloud(pcd, T, color):
    out = o3d.geometry.PointCloud(pcd)
    out.transform(T)
    out.paint_uniform_color(color)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", type=Path, required=True, help="socket template PLY, metres/socket frame")
    ap.add_argument("--color", type=Path, required=True, help="aligned color image")
    ap.add_argument("--depth", type=Path, required=True, help="aligned uint16 depth image")
    ap.add_argument("--meta", type=Path, required=True, help="capture JSON with intrinsics/depth_scale")
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--roi", help="manual color/depth ROI x,y,w,h; omit only for pre-segmented depth")
    ap.add_argument("--base-from-camera", type=Path, help="optional JSON 4x4 T_base_from_camera; never guessed")
    ap.add_argument("--init", type=Path, help="optional JSON 4x4 initial T_camera_from_socket")
    ap.add_argument("--voxel", type=float, default=0.003, help="metres; tune to measured point spacing")
    ap.add_argument("--min-depth", type=float, default=0.20)
    ap.add_argument("--max-depth", type=float, default=1.50)
    ap.add_argument("--max-correspondence", type=float, default=0.012)
    ap.add_argument("--no-global", action="store_true", help="skip FPFH/RANSAC; requires --init")
    args = ap.parse_args(); t0 = time.perf_counter(); args.out.mkdir(parents=True, exist_ok=True)
    meta = json.loads(args.meta.read_text())
    # Accept both the legacy flat capture metadata and the evidence-capture schema.
    stream = meta.get("stream", {})
    ci = meta.get("color_intrinsics", {})
    meta = {**meta, "width": meta.get("width", stream.get("width", ci.get("width"))),
            "height": meta.get("height", stream.get("height", ci.get("height"))),
            "fx": meta.get("fx", ci.get("fx")), "fy": meta.get("fy", ci.get("fy")),
            "cx": meta.get("cx", ci.get("cx")), "cy": meta.get("cy", ci.get("cy")),
            "depth_scale": meta.get("depth_scale", meta.get("depth_scale_m_per_unit"))}
    required = ("width", "height", "fx", "fy", "cx", "cy", "depth_scale")
    missing = [k for k in required if meta.get(k) is None]
    if missing:
        raise ValueError(f"capture metadata missing fields: {missing}")
    roi = parse_roi(args.roi, int(meta["width"]), int(meta["height"])) if args.roi else None
    model = o3d.io.read_point_cloud(str(args.model))
    if model.is_empty(): raise RuntimeError("empty model point cloud")
    target_raw, color, depth_raw, valid, camera = make_target(args.color, args.depth, meta, roi, args.min_depth, args.max_depth)
    o3d.io.write_point_cloud(str(args.out / "target_raw_aligned_m.ply"), target_raw)
    target = clean_cloud(target_raw, args.voxel, 20, 2.0)
    source = clean_cloud(model, args.voxel, 20, 2.0)
    o3d.io.write_point_cloud(str(args.out / "target_filtered_m.ply"), target)
    o3d.io.write_point_cloud(str(args.out / "model_filtered_m.ply"), source)
    src_ds, src_f = preprocess(source, args.voxel)
    tgt_ds, tgt_f = preprocess(target, args.voxel)
    if args.init:
        T = read_matrix(args.init)
        init_method = "provided_manual_initial_pose"
    elif args.no_global:
        raise SystemExit("--no-global requires --init")
    else:
        result = o3d.pipelines.registration.registration_fgr_based_on_feature_matching(
            src_ds, tgt_ds, src_f, tgt_f,
            o3d.pipelines.registration.FastGlobalRegistrationOption(
                maximum_correspondence_distance=args.max_correspondence * 1.5))
        T = result.transformation
        init_method = "fgr_feature_matching"
        if not result.transformation.shape == (4, 4):
            raise RuntimeError("global registration did not return a 4x4 transform")
    # Multi-scale point-to-plane ICP, source=model, target=measured camera cloud.
    icp = None
    for scale in (4.0, 2.0, 1.0):
        voxel = args.voxel * scale
        s = source.voxel_down_sample(voxel); d = target.voxel_down_sample(voxel)
        s.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=voxel*2.5, max_nn=50))
        d.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=voxel*2.5, max_nn=50))
        icp = o3d.pipelines.registration.registration_icp(
            s, d, max(args.max_correspondence * scale, voxel * 2.0), T,
            o3d.pipelines.registration.TransformationEstimationPointToPlane(),
            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=80))
        T = icp.transformation
    socket_in_camera = T[:3, 3].copy()
    insertion_camera = -T[:3, 2].copy()
    aligned_model = transform_cloud(model, T, [1.0, 0.2, 0.0])
    target_display = transform_cloud(target, np.eye(4), [0.1, 0.8, 0.2])
    o3d.io.write_point_cloud(str(args.out / "registration_overlay_colored_m.ply"), target_display + aligned_model)
    report = {
        "status": "candidate_only",
        "read_only": True,
        "convention": "P_camera = T_camera_from_socket @ P_socket",
        "model": str(args.model), "color": str(args.color), "depth": str(args.depth), "meta": str(args.meta),
        "model_units": "m", "target_units": "m", "camera_frame": "RealSense color optical frame after depth aligned to color",
        "roi_xywh": list(roi) if roi else None, "valid_depth_pixels": int(valid.sum()),
        "point_counts": {"model_raw": len(model.points), "target_raw": len(target_raw.points), "model_filtered": len(source.points), "target_filtered": len(target.points)},
        "initialization": init_method,
        "T_camera_from_socket": T.tolist(),
        "socket_origin_camera_m": socket_in_camera.tolist(),
        "socket_insertion_direction_camera": insertion_camera.tolist(),
        "icp_fitness": float(icp.fitness) if icp else None,
        "icp_inlier_rmse_m": float(icp.inlier_rmse) if icp else None,
        "base_pose_status": "not_computed",
        "elapsed_s": time.perf_counter() - t0,
        "warnings": ["ICP/low RMSE is not proof of correct socket pose; inspect holes, upper edge, overlap and independent ground truth."]
    }
    if args.base_from_camera:
        B = read_matrix(args.base_from_camera); TB = B @ T
        report["T_base_from_camera"] = B.tolist(); report["T_base_from_socket"] = TB.tolist(); report["base_pose_status"] = "computed_from_explicit_static_calibration"
    (args.out / "registration_report.json").write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8")
    print(json.dumps(report, indent=2, ensure_ascii=False))

if __name__ == "__main__": main()
