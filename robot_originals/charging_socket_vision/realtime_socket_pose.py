#!/usr/bin/env python3
"""Read-only real-time socket pose overlay for D435i.

Uses full-frame RGB contour with frame-edge separation (legacy adaptive cut optional). No robot connection or motion commands.
Pose convention: P_camera = T_camera_from_socket @ P_socket.
"""
from __future__ import annotations
import argparse, hashlib, itertools, json, time
from pathlib import Path
import cv2, numpy as np, open3d as o3d
import pyrealsense2 as rs
from scipy.spatial.transform import Rotation
from scipy.spatial import cKDTree

ROOT=Path(__import__('os').environ.get('CHARGING_SOCKET_VISION_ROOT','/home/nvidia/charging_socket_vision'))
MODEL=ROOT/'model/socket_frame_and_rim_2mm_m.ply'
INIT_REPORT=ROOT/'results/v013_gray80_cover_clip_filtered/metadata/report.json'
CUT_SLOPE=0.10  # horizontal shift per vertical pixel; positive slopes right downward
CONFIG=ROOT/'config/realtime_socket_pose.json'

DEFAULT_CONFIG={
    'segmentation_method':'frame_edges_merged',
    'expected_image_up':[0.0, -1.0],
    'expected_image_right':[1.0, 0.0],
    'orientation_min_axis_cosine':0.25,
    'observed_plane_trials':96,
    'observed_plane_distance_m':0.0025,
    'observed_plane_min_fraction':0.2,
    'observed_plane_min_minor_std_m':0.01,
    'observed_plane_max_angle_deg':25.0,
    'voxel_m':0.002,
    'tracking_correspondence_m':0.006,
    'recovery_correspondence_m':[0.012,0.008,0.005],
    'acceptance_distance_m':0.006,
    'min_points_per_layer':25,
    'min_layer_coverage':0.18,
    'far_pose_range_m':0.35,
    'far_min_layer_coverage':0.15,
    'min_overall_coverage':0.28,
    'max_layer_rmse_m':0.006,
    'min_target_points':80,
    'surface_xy_tolerance_m':0.0035,
    'surface_z_tolerance_m':0.006,
    'plane_max_residual_m':0.006,
    'min_reverse_coverage':0.15,
    'min_contour_support':0.20,
    'min_visible_fraction':0.08,
    'failures_before_recovery':3,
    'max_measurement_age_s':1.0,
    'tracking_max_translation_m':0.020,
    'tracking_max_rotation_deg':20.0,
    'recovery_translation_m':[0.0,0.012,-0.012,0.024,-0.024],
    'recovery_yaw_step_deg':30,
    'recovery_iterations':[18,22,30],
    'recovery_voxel_m':[0.006,0.003,0.002],
    'min_view_cosine':0.10,
    'expected_template_points':1098,
    'frame_corner_half_extent_m':0.038,
    'expected_template_sha256':'272b2ea7106a8da5930201d7ec0a590081a2b041de1bff2938be4ad1496f2e25',
    'min_layer_spread_m':0.008,
    'recovery_confirm_frames':2,
    'recovery_confirm_translation_m':0.008,
    'recovery_confirm_rotation_deg':8.0,
    'plane_min_points':80,
    'plane_inlier_distance_m':0.003,
    'plane_max_normal_angle_deg':15.0,
    'quad_corner_support_px':4.0,
    'recovery_seed_yaw_step_deg':30,
    'minimum_pose_range_m':0.12,
    'maximum_pose_range_m':0.50,
    'recovery_max_candidates':12,
    'recovery_timeout_s':0.45,
    'recovery_confirm_window_s':0.6,
    'degraded_hold_s':0.35,
    'degraded_max_translation_m':0.003,
    'degraded_max_rotation_deg':3.0,
    'degraded_anchor_translation_m':0.006,
    'degraded_anchor_rotation_deg':6.0,
    'ambiguity_loss_margin':0.03,
    'ambiguity_translation_m':0.008,
    'ambiguity_rotation_deg':8.0,

}
CFG=DEFAULT_CONFIG.copy()
TEMPLATE_CACHE={}
QUAD_INFO={}
CUT_INFO={'active':False,'ratio':None,'line':None,'removed_pixels':0,'intersections':[],'reason':'not_evaluated'}
MASK_INFO={'contour_found':False,'raw_contour_area_px':0,'bbox_xywh':None,'cut':{}}

def load_config():
    global CFG
    if CONFIG.exists():
        data=json.loads(CONFIG.read_text())
        unknown=set(data)-set(DEFAULT_CONFIG)
        if unknown: raise ValueError(f'unknown ICP settings: {sorted(unknown)}')
        CFG={**DEFAULT_CONFIG,**data}
    for key in ('voxel_m','tracking_correspondence_m','acceptance_distance_m','surface_xy_tolerance_m','surface_z_tolerance_m','far_pose_range_m','far_min_layer_coverage'):
        if not np.isfinite(CFG[key]) or CFG[key]<=0: raise ValueError(f'invalid {key}')
    if CFG['voxel_m']<.002: raise ValueError('final voxel must not imply detail below 2 mm')
    if max(CFG['tracking_correspondence_m'],CFG['acceptance_distance_m'],CFG['surface_z_tolerance_m'])>=.0075:
        raise ValueError('final distances must stay below half the 15 mm layer separation')
    if not (len(CFG['recovery_correspondence_m'])==len(CFG['recovery_voxel_m'])==len(CFG['recovery_iterations'])):
        raise ValueError('recovery stage settings differ in length')
    if CFG['recovery_correspondence_m'][-1]>CFG['acceptance_distance_m']: raise ValueError('last optimization stage exceeds acceptance distance')
    if any(v<CFG['voxel_m'] for v in CFG['recovery_voxel_m']): raise ValueError('recovery voxel cannot be finer than template resolution')
    if any(v<=0 for v in CFG['recovery_correspondence_m']+CFG['recovery_voxel_m']) or any(v<1 for v in CFG['recovery_iterations']): raise ValueError('invalid recovery stage values')
    for key in ('degraded_hold_s','degraded_max_translation_m','degraded_max_rotation_deg',
                'degraded_anchor_translation_m','degraded_anchor_rotation_deg','recovery_timeout_s',
                'recovery_confirm_window_s','max_measurement_age_s','ambiguity_loss_margin'):
        if not np.isfinite(CFG[key]) or CFG[key]<=0: raise ValueError('invalid '+key)
    for key in ('recovery_max_candidates','recovery_confirm_frames','failures_before_recovery'):
        if not isinstance(CFG[key],int) or isinstance(CFG[key],bool) or CFG[key]<1: raise ValueError('invalid '+key)
    if CFG['recovery_seed_yaw_step_deg']<=0 or CFG['recovery_seed_yaw_step_deg']>=360:
        raise ValueError('invalid recovery_seed_yaw_step_deg')
    for key in ('expected_image_up','expected_image_right'):
        axis=np.asarray(CFG[key],float)
        if axis.shape!=(2,) or not np.isfinite(axis).all() or np.linalg.norm(axis)<1e-9:
            raise ValueError('invalid '+key)
    if not 0<CFG['orientation_min_axis_cosine']<1: raise ValueError('invalid orientation_min_axis_cosine')
    if not 0<CFG['observed_plane_min_fraction']<=1: raise ValueError('invalid observed_plane_min_fraction')
    if not 0<CFG['observed_plane_max_angle_deg']<90: raise ValueError('invalid observed_plane_max_angle_deg')
    for key in ('observed_plane_distance_m','observed_plane_min_minor_std_m'):
        if not np.isfinite(CFG[key]) or CFG[key]<=0: raise ValueError('invalid '+key)
    if not isinstance(CFG['observed_plane_trials'],int) or isinstance(CFG['observed_plane_trials'],bool) or not 1<=CFG['observed_plane_trials']<=512:
        raise ValueError('invalid observed_plane_trials')
    if CFG['segmentation_method'] not in ('frame_edges_merged','frame_edges','adaptive_cut'):
        raise ValueError('invalid segmentation_method')
    return CFG

def _layer_cache(points,z):
    sel=np.abs(points[:,2]-z)<=0.0005
    layer=points[sel]
    if len(np.unique(layer,axis=0))<20 or np.min(np.ptp(layer[:,:2],axis=0))<.04:
        raise ValueError(f'template layer z={z} has inadequate unique/spatial support')
    return {'z':float(z),'points':layer,'xy':layer[:,:2], 'tree':cKDTree(layer[:,:2]),
            'min':layer.min(0).tolist(),'max':layer.max(0).tolist()}

def model_features():
    load_config()
    if not MODEL.is_file(): raise FileNotFoundError(MODEL)
    m=o3d.io.read_point_cloud(str(MODEL))
    if len(m.points)==0:
        raise RuntimeError(f'Empty socket CAD template: {MODEL}')
    points=np.asarray(m.points)
    digest=hashlib.sha256(MODEL.read_bytes()).hexdigest()
    if digest!=CFG['expected_template_sha256']: raise ValueError('template SHA256 differs from approved asset')
    if not np.isfinite(points).all(): raise ValueError('template contains non-finite coordinates')
    span=np.ptp(points,axis=0)
    if not (.05<span[0]<.10 and .05<span[1]<.10 and .014<span[2]<.016):
        raise ValueError('template dimensions/units are inconsistent with this socket')
    if len(points)!=CFG['expected_template_points']: raise ValueError('unexpected template count; inspect asset before use')
    if not np.all((np.abs(points[:,2]+.015)<=.0005)|(np.abs(points[:,2])<=.0005)):
        raise ValueError('template contains points outside the two declared surface layers')
    if not np.any(np.abs(points[:,2]+.015)<=.0005) or not np.any(np.abs(points[:,2])<=.0005):
        raise ValueError('template does not contain both z=-0.015 and z=0 layers')
    global TEMPLATE_CACHE
    TEMPLATE_CACHE={'path':str(MODEL.resolve()),'sha256':digest,
                    'points':points,'layers':[_layer_cache(points,-.015),_layer_cache(points,0.0)]}
    print(f"ICP_TEMPLATE path={MODEL} sha256={TEMPLATE_CACHE['sha256']} points={len(points)} "
          f"layers={[len(x['points']) for x in TEMPLATE_CACHE['layers']]} "
          f"min={points.min(axis=0).tolist()} max={points.max(axis=0).tolist()}")
    return m


def adaptive_cut_ratio(rawmask,bx,by,bw,bh):
    """Find cover-to-socket transition only from the RGB contour profile."""
    heights=[]
    for x in range(bx,bx+bw):
        ys=np.where(rawmask[:,x]>0)[0]
        heights.append(float(ys[-1]-ys[0]+1) if len(ys)>=3 else 0.0)
    h=np.asarray(heights,dtype=np.float32)
    if len(h)<40:
        return 0.50,False
    k=9; kernel=np.ones(k,np.float32)/k
    hs=np.convolve(h,kernel,mode='same')
    right=float(np.median(hs[int(.70*bw):int(.95*bw)]))
    if right<=10:
        return float(np.clip(np.argmin(hs)/max(1,len(hs)),0.05,0.90)),False
    # Search only in the left/middle transition zone. Require a local valley
    # and a sustained recovery to the rectangular frame on the right.
    lo=max(8,int(.08*bw)); hi=min(int(.55*bw),len(hs)-12)
    best_i=lo+int(np.argmin(hs[lo:hi]))
    for i in range(lo,hi):
        valley=float(np.min(hs[max(0,i-5):min(len(hs),i+6)]))
        after=float(np.median(hs[min(len(hs)-8,i+12):min(len(hs),i+30)]))
        before=float(np.median(hs[max(0,i-25):i]))
        if valley < .78*right and after > .82*right and before < .95*right:
            return float(i/bw),True
    # No fixed 0.24 fallback: use the geometric minimum of the RGB height
    # profile in the transition zone, while marking confidence as low.
    return float(best_i/bw),False

def apply_slanted_cut(rawmask,bx,by,bw,bh,ratio=None,slope=CUT_SLOPE):
    """Keep contour to right of the adaptive, slightly slanted cut line."""
    if ratio is None:
        ratio,_=adaptive_cut_ratio(rawmask,bx,by,bw,bh)
    h,w=rawmask.shape
    cut=int(round(ratio*bw)); xc=bx+cut; yc=by+0.5*bh
    yy=np.arange(h,dtype=np.float32)[:,None]
    boundary=xc+slope*(yy-yc)
    xx=np.arange(w,dtype=np.float32)[None,:]
    keep=(xx>=boundary).astype(np.uint8)*255
    out=cv2.bitwise_and(rawmask,keep)
    removed=int(np.count_nonzero((rawmask>0)&(out==0)))
    p1=(int(round(xc-slope*0.5*bh)),int(by))
    p2=(int(round(xc+slope*0.5*bh)),int(by+bh))
    global CUT_INFO
    intersections=[]
    # The cut is artificial only where its line enters/exits the original
    # contour. A normal left rectangle corner away from those intersections
    # must not be rejected merely for being the leftmost corner.
    inside=[]; ys_probe=[]
    for y in range(max(0,by),min(h,by+bh)):
        x=int(round(xc+slope*(y-yc)))
        xp=min(w-1,max(0,x+4))
        inside.append(bool(rawmask[y,xp]>0)); ys_probe.append(y)
    for i in range(1,len(inside)):
        if inside[i]!=inside[i-1]:
            y=ys_probe[i]; intersections.append((float(xc+slope*(y-yc)),float(y)))
    CUT_INFO={'active':removed>0,'ratio':float(ratio),'line':(float(xc),float(yc),float(slope)),
              'removed_pixels':removed,'reason':'contour_clipped' if removed else 'line_did_not_clip_contour',
              'bbox':(int(bx),int(by),int(bw),int(bh)),'intersections':intersections}
    return out,p1,p2

def order_quad(pts):
    """Order four image points as top-left, top-right, bottom-right, bottom-left."""
    p=np.asarray(pts,dtype=np.float32)
    c=p.mean(axis=0); ang=np.arctan2(p[:,1]-c[1],p[:,0]-c[0])
    q=p[np.argsort(ang)]
    # rotate so first is top-left; then ensure clockwise image order
    start=int(np.argmin(q[:,0]+q[:,1])); q=np.roll(q,-start,axis=0)
    if np.cross(q[1]-q[0],q[2]-q[1]) < 0: q=q[[0,3,2,1]]
    return q

def dynamic_quad(color, processed_mask=None):
    """Find perspective-distorted frame corners on the post-cut ICP mask.

    The raw image may contain the cover/door silhouette. Never select corners
    from that raw silhouette; use the exact mask that is sent to ICP after
    contour extraction and adaptive slanted cutting.
    """
    global QUAD_INFO
    QUAD_INFO={'verified':False,'source':'none','edge_support':[]}
    if processed_mask is None:
        processed_mask=contour_mask(color)
    if processed_mask is None: return None,None,0.0
    mask=(processed_mask>0).astype(np.uint8)*255
    cs,_=cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    cs=[c for c in cs if cv2.contourArea(c)>300]
    if not cs: return None,None,0.0
    c=max(cs,key=cv2.contourArea); hull=cv2.convexHull(c)
    peri=cv2.arcLength(hull,True); approx=cv2.approxPolyDP(hull,.025*peri,True).reshape(-1,2)
    candidates=[]
    if 4<=len(approx)<=8:
        from itertools import combinations
        for comb in combinations(approx,4):
            q=order_quad(np.asarray(comb,float)); area=abs(cv2.contourArea(q.astype(np.float32)))
            if area>0: candidates.append((area,q))
    exact_quad=len(approx)==4
    if not candidates:
        rect=cv2.boxPoints(cv2.minAreaRect(hull.astype(np.float32)))
        q=order_quad(rect); candidates=[(abs(cv2.contourArea(q.astype(np.float32))),q)]
    area,q=max(candidates,key=lambda x:x[0])
    x,y,w,h=cv2.boundingRect(c); bbox_area=max(1,w*h); fill=float(area/bbox_area)
    side=np.linalg.norm(np.roll(q,-1,axis=0)-q,axis=1)
    regular=float(np.min(side)/max(np.max(side),1e-6))
    conf=float(np.clip(.55*fill+.45*regular,0,1))
    edges=cv2.Canny(cv2.cvtColor(color,cv2.COLOR_BGR2GRAY),50,120)
    distance=cv2.distanceTransform((edges==0).astype(np.uint8),cv2.DIST_L2,3)
    supports=[]
    for a,b in zip(q,np.roll(q,-1,axis=0)):
        samples=np.rint(a[None,:]+np.linspace(.05,.95,40)[:,None]*(b-a)).astype(int)
        samples[:,0]=np.clip(samples[:,0],0,color.shape[1]-1)
        samples[:,1]=np.clip(samples[:,1],0,color.shape[0]-1)
        supports.append(float(np.mean(distance[samples[:,1],samples[:,0]]<=CFG['quad_corner_support_px'])))
    corners=cv2.goodFeaturesToTrack(cv2.cvtColor(color,cv2.COLOR_BGR2GRAY),200,.02,5)
    corner_ok=False
    if corners is not None:
        cd,_=cKDTree(corners.reshape(-1,2)).query(q)
        corner_ok=bool(np.all(cd<=CFG['quad_corner_support_px']))
    cut_corner=False
    endpoints=CUT_INFO.get('intersections',[]) if CUT_INFO.get('active') else []
    for px,py in np.rint(q).astype(float):
        if any(np.hypot(px-ex,py-ey)<=5.0 for ex,ey in endpoints):
            cut_corner=True
    verified=bool(exact_quad and corner_ok and min(supports)>=.75 and not cut_corner)
    QUAD_INFO={'verified':verified,'source':'contour_four' if exact_quad else 'inferred_quad',
               'edge_support':supports,'corners_supported':corner_ok,'cut_corner':cut_corner,
               'cut_endpoints':endpoints}
    if not verified: conf=min(conf,.30)
    return q.astype(np.float32),hull,conf

def quad_center_seed(quad, depth, K, scale, R_hint):
    """Use verified frame support only; never substitute center/hole depth."""
    if quad is None or R_hint is None or not QUAD_INFO.get('verified'): return None
    quad=np.asarray(quad,dtype=np.float32)
    # Project the actual CAD frame layer through the validated quadrilateral.
    a=float(CFG['frame_corner_half_extent_m'])
    src=np.array([[-a,a],[a,a],[a,-a],[-a,-a]],np.float32)
    H=cv2.getPerspectiveTransform(src,quad)
    frame=TEMPLATE_CACHE['layers'][0]
    xy=frame['xy'].astype(np.float32)
    uv=cv2.perspectiveTransform(xy[None,:,:],H)[0]
    raster=np.zeros(depth.shape,np.uint8)
    for x,y in np.rint(uv).astype(int):
        if 0<=x<depth.shape[1] and 0<=y<depth.shape[0]: cv2.circle(raster,(x,y),1,255,-1)
    ys,xs=np.where(raster>0)
    z=depth[ys,xs].astype(np.float64)*scale
    valid=(z>.12)&(z<.50); xs=xs[valid]; ys=ys[valid]; z=z[valid]
    if len(z)<CFG['plane_min_points']: return None
    ci=K['_intrinsics']
    pts=np.asarray([rs.rs2_deproject_pixel_to_point(ci,[int(x),int(y)],float(d)) for x,y,d in zip(xs,ys,z)])
    cloud=o3d.geometry.PointCloud(); cloud.points=o3d.utility.Vector3dVector(pts)
    coeff,inliers=cloud.segment_plane(CFG['plane_inlier_distance_m'],3,150)
    if len(inliers)<CFG['plane_min_points'] or len(inliers)<.6*len(pts): return None
    support=pts[inliers]; center=support.mean(0)
    _,_,vt=np.linalg.svd(support-center,full_matrices=False); normal=vt[-1]
    if abs(normal@R_hint[:,2])<np.cos(np.deg2rad(CFG['plane_max_normal_angle_deg'])): return None
    residual=np.abs((support-center)@normal)
    if np.sqrt(np.mean(residual**2))>CFG['plane_max_residual_m']: return None
    # Require support on at least three frame sides, not a tiny planar patch.
    pixel_support=np.c_[xs[inliers],ys[inliers]].astype(np.float32)
    invH=np.linalg.inv(H)
    socket_xy=cv2.perspectiveTransform(pixel_support[None,:,:],invH)[0]
    if np.min(np.ptp(socket_xy,axis=0))<.045: return None
    sides=[np.count_nonzero(socket_xy[:,0]<-.027),np.count_nonzero(socket_xy[:,0]>.027),
           np.count_nonzero(socket_xy[:,1]<-.027),np.count_nonzero(socket_xy[:,1]>.027)]
    if sum(n>=10 for n in sides)<3: return None
    center_uv=cv2.perspectiveTransform(np.array([[[0.,0.]]],np.float32),H)[0,0]
    ray=np.asarray(rs.rs2_deproject_pixel_to_point(ci,center_uv.tolist(),1.0))
    denom=float(normal@ray)
    if abs(denom)<1e-5: return None
    distance=float(normal@center)/denom
    if not .12<distance<.50: return None
    pcam=ray*distance
    T=np.eye(4); T[:3,:3]=R_hint; T[:3,3]=pcam-R_hint@np.array([0.,0.,-.015])
    return T,pcam,tuple(np.rint(center_uv).astype(int))

def pnp_from_dynamic_quad(quad,K):
    """Estimate socket pose from dynamic front-frame corners.

    CAD frame convention: +X right, +Y up, +Z outward. The visible frame is
    at z=-15 mm; image order is TL,TR,BR,BL.
    """
    if quad is None: return None
    quad=np.asarray(quad,dtype=np.float32)
    if quad.shape!=(4,2) or not np.isfinite(quad).all() or abs(cv2.contourArea(quad))<8: return None
    a=float(CFG['frame_corner_half_extent_m']); z=-.015
    obj=np.array([[-a,a,z],[a,a,z],[a,-a,z],[-a,-a,z]],np.float32)
    # A planar rectangle has two IPPE mirror solutions.  At longer range
    # IPPE can return the solution whose normal points away from the camera,
    # which makes it unusable as an ICP recovery seed.  Try the iterative
    # pose first and rank all candidates with the same installation prior
    # used by recovery (forward view, +X right, +Y up).
    image_points=quad.astype(np.float32)
    candidates=[]
    for flag in (cv2.SOLVEPNP_ITERATIVE, cv2.SOLVEPNP_IPPE):
        try:
            if flag == cv2.SOLVEPNP_IPPE:
                ok, rvecs, tvecs, _ = cv2.solvePnPGeneric(obj,image_points,K,np.zeros(5),flags=flag)
                if not ok: continue
                pairs=zip(rvecs,tvecs)
            else:
                ok,rvec,tvec=cv2.solvePnP(obj,image_points,K,np.zeros(5),flags=flag)
                if not ok: continue
                pairs=((rvec,tvec),)
            for rvec,tvec in pairs:
                R,_=cv2.Rodrigues(rvec); T=np.eye(4); T[:3,:3]=R; T[:3,3]=np.asarray(tvec).reshape(3)
                projected,_=cv2.projectPoints(obj,rvec,tvec,K,np.zeros(5))
                reproj=float(np.sqrt(np.mean((projected.reshape(-1,2)-image_points)**2)))
                sane=pose_is_sane(T)
                orient=image_orientation_check(T,{'K':K})['valid']
                forward=float(np.dot(T[:3,2],-T[:3,3]/max(np.linalg.norm(T[:3,3]),1e-9)))
                candidates.append((0 if sane and orient else 1, -forward, reproj, T))
        except cv2.error:
            continue
    if not candidates: return None
    candidates.sort(key=lambda x:(x[0],x[1],x[2]))
    return candidates[0][3]

def draw_dynamic_quad(img,quad,conf,locked):
    if quad is None: return
    q=quad.astype(int).reshape(-1,1,2); cv2.polylines(img,[q],True,(0,255,255),3,cv2.LINE_AA)
    for i,(x,y) in enumerate(quad.astype(int)):
        cv2.circle(img,(int(x),int(y)),6,(0,0,0),-1); cv2.circle(img,(int(x),int(y)),4,(0,255,255),-1)
        cv2.putText(img,['TL','TR','BR','BL'][i],(int(x)+7,int(y)-7),cv2.FONT_HERSHEY_SIMPLEX,.5,(0,255,255),2)
    cv2.putText(img,f'QUAD verified={bool(QUAD_INFO.get("verified"))} conf={conf:.2f}',(10,118),cv2.FONT_HERSHEY_SIMPLEX,.5,(0,255,255),2)

FRAME_EDGE_MERGE_PARAMS = {'min_segment_fraction': 0.1, 'angle_deg': 7.0, 'perpendicular_px_min': 2.0, 'perpendicular_fraction': 0.015, 'max_gap_fraction': 0.22, 'min_coverage': 0.5}

def _fit_frame_line_group(group):
    points=np.vstack([v[:2] for v in group]+[v[2:] for v in group]).astype(float)
    lengths=np.array([np.linalg.norm(v[2:]-v[:2]) for v in group]);weights=np.r_[lengths,lengths]
    center=np.average(points,axis=0,weights=weights);q=points-center
    _,vec=np.linalg.eigh((q*weights[:,None]).T@q);u=vec[:,-1];proj=q@u
    lo,hi=float(proj.min()),float(proj.max());intervals=[]
    for v in group:
        p=(np.array([v[:2],v[2:]])-center)@u;intervals.append(sorted(p))
    intervals.sort();covered=0.;start,end=intervals[0]
    for a,b in intervals[1:]:
        if a>end:covered+=end-start;start,end=a,b
        else:end=max(end,b)
    covered+=end-start
    return center+lo*u,center+hi*u,hi-lo,covered/max(hi-lo,1e-6)

def _merged_frame_lines(detected,bbox):
    x,y,w,h=bbox;scale=min(w,h);tol=max(FRAME_EDGE_MERGE_PARAMS['perpendicular_px_min'],FRAME_EDGE_MERGE_PARAMS['perpendicular_fraction']*scale)
    segments=[]
    for v in detected[:,0,:]:
        mid=(v[:2]+v[2:])/2
        if x-4<=mid[0]<=x+w+4 and y-4<=mid[1]<=y+h+4 and np.linalg.norm(v[2:]-v[:2])>=FRAME_EDGE_MERGE_PARAMS['min_segment_fraction']*scale:segments.append(v)
    segments.sort(key=lambda v:-np.linalg.norm(v[2:]-v[:2]));groups=[]
    for v in segments:
        best=None
        for i,g in enumerate(groups):
            a,b,L,_=_fit_frame_line_group(g);u=(b-a)/L;n=np.array([-u[1],u[0]])
            vv=(v[2:]-v[:2]);vv=vv/np.linalg.norm(vv)
            if abs(u@vv)<np.cos(np.deg2rad(FRAME_EDGE_MERGE_PARAMS['angle_deg'])):continue
            if max(abs((v[:2]-a)@n),abs((v[2:]-a)@n))>tol:continue
            ends=np.sort((np.array([v[:2],v[2:]])-a)@u)
            gap=max(0.,float(ends[0]-L),float(-ends[1]))
            if gap>FRAME_EDGE_MERGE_PARAMS['max_gap_fraction']*scale:continue
            aa,bb,LL,cov=_fit_frame_line_group(g+[v]);nn=np.array([-(bb-aa)[1],(bb-aa)[0]])/LL
            if max(abs((p-aa)@nn) for vv0 in g+[v] for p in [vv0[:2],vv0[2:]])>tol*1.5:continue
            best=i;break
        if best is None:groups.append([v])
        else:groups[best].append(v)
    lines=[];details=[]
    for g in groups:
        a,b,L,cov=_fit_frame_line_group(g)
        if L>=.5*scale and cov>=FRAME_EDGE_MERGE_PARAMS['min_coverage']:
            lines.append((a.astype(np.float32),b.astype(np.float32),float(L)))
            details.append({'fragments':len(g),'span_px':L,'measured_coverage':cov,'endpoints':[a.tolist(),b.tolist()]})
    _merged_frame_lines.details=details
    return lines

def frame_edge_mask(image, raw, bbox, merge_fragments=False):
    """Fit a frame hypothesis from measured opposite edges, not a cut percentage.

    Scope: approximately front-facing, near-square mounting frame. A 3D/perspective
    rectangle is NOT inferred here. Weak sides are reported rather than invented
    as observed edges. No capture IDs or annotated pixel coordinates are used.
    """
    (x, y, w, h) = bbox
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(gray, 50, 120)
    distance = cv2.distanceTransform((edges == 0).astype(np.uint8), cv2.DIST_L2, 3)
    # Detect only lines near the selected contour; keep the original image coordinates.
    # Five-pixel alignment preserves the default LSD 0.8-scale sampling phase.
    x0=max(0,((x-25)//5)*5); y0=max(0,((y-25)//5)*5)
    x1=min(gray.shape[1],((x+w+29)//5)*5); y1=min(gray.shape[0],((y+h+29)//5)*5)
    detected = cv2.createLineSegmentDetector().detect(gray[y0:y1,x0:x1])[0]
    if detected is not None:
        detected=detected.copy(); detected[:,:,0::2]+=x0; detected[:,:,1::2]+=y0
    if detected is None:
        raise ValueError('No visible line segments')
    if merge_fragments:
        lines = _merged_frame_lines(detected, bbox)
    else:
        lines = []
        for v in detected[:, 0, :]:
            (a, b) = (v[:2], v[2:])
            mid = (a + b) / 2
            length = np.linalg.norm(b - a)
            if x - 4 <= mid[0] <= x + w + 4 and y - 4 <= mid[1] <= y + h + 4 and (length >= 0.5 * min(w, h)):
                lines.append((a, b, length))
    candidates = []
    for (one, two) in itertools.combinations(lines, 2):
        (a, b, length) = one
        (c, d, other_length) = two
        u = (b - a) / length
        v = (d - c) / other_length
        if abs(u @ v) < np.cos(np.deg2rad(12)):
            continue
        if u @ v < 0:
            v = -v
        u = (u + v) / np.linalg.norm(u + v)
        normal = np.array([-u[1], u[0]])
        n1 = float((a + b) / 2 @ normal)
        n2 = float((c + d) / 2 @ normal)
        gap = abs(n2 - n1)
        if not 0.45 * min(w, h) < gap < 1.1 * max(w, h):
            continue
        if min(length, other_length) < 0.5 * gap:
            continue
        center_u = float(np.mean(np.array([a, b, c, d]) @ u))
        center_n = (n1 + n2) / 2
        aspects = np.repeat(np.linspace(0.8, 1.25, 19), 25)
        shifts = np.tile(np.linspace(-0.12, 0.12, 25), 19)
        centers = (u[None, :] * (center_u + shifts * gap)[:, None]).astype(u.dtype) + normal * center_n
        half_widths = (u[None, :] * (gap * aspects)[:, None]).astype(u.dtype) / 2
        half_height = normal * gap / 2
        quads = np.stack([centers-half_widths-half_height, centers+half_widths-half_height,
                          centers+half_widths+half_height, centers-half_widths+half_height], axis=1)
        xy = quads[:, :, None, :] + np.linspace(0.12, 0.88, 50)[None, None, :, None] * (np.roll(quads, -1, axis=1) - quads)[:, :, None, :]
        xx = np.clip(np.rint(xy[..., 0]).astype(int), 0, gray.shape[1] - 1)
        yy = np.clip(np.rint(xy[..., 1]).astype(int), 0, gray.shape[0] - 1)
        ds = distance[yy, xx]
        supports = np.mean(ds <= 2.0, axis=2)
        distances = np.mean(np.minimum(ds, 5.0), axis=2).astype(float) / 5.0
        scores = supports.mean(axis=1) - 0.25 * distances.mean(axis=1) - 0.08 * np.abs(np.log(aspects))
        best = int(np.argmax(scores))
        candidates.append((float(scores[best]), quads[best], supports[best].tolist(), float(aspects[best])))
    if not candidates:
        raise ValueError('No opposite frame-edge hypotheses')
    (score, quad, supports, aspect) = max(candidates, key=lambda x: x[0])
    keep = np.zeros_like(raw)
    cv2.fillConvexPoly(keep, np.rint(quad).astype(np.int32), 255)
    selected = cv2.bitwise_and(raw, keep)
    return (selected, {'quad': quad.tolist(), 'edge_support': supports, 'score': score, 'aspect': aspect, 'long_segments': len(lines), 'hypotheses': len(candidates) * 19 * 25, 'limitation': 'Near-square 2D frame prior; weak sides are inferred, not verified.'})

def contour_mask(color):
    global MASK_INFO,CUT_INFO
    CUT_INFO={'active':False,'line':None,'removed_pixels':0,'intersections':[],'reason':'not_evaluated'}
    MASK_INFO={'contour_found':False,'raw_contour_area_px':0,'bbox_xywh':None,'cut':{},
        'segmentation_method':CFG['segmentation_method'],'segmentation_reason':'no_contour'}
    gray=cv2.cvtColor(color,cv2.COLOR_BGR2GRAY)
    x0,y0,x1,y1=0,0,640,480
    roi=gray[y0:y1,x0:x1]
    raw=(roi<80).astype(np.uint8)*255
    raw=cv2.morphologyEx(raw,cv2.MORPH_CLOSE,cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(7,7)),iterations=1)
    raw=cv2.morphologyEx(raw,cv2.MORPH_OPEN,cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(5,5)),iterations=1)
    cs,_=cv2.findContours(raw,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    cs=[c for c in cs if cv2.contourArea(c)>500]
    if not cs: return None
    c=max(cs,key=cv2.contourArea); c=c.copy(); c[:,:,0]+=x0; c[:,:,1]+=y0
    rawmask=np.zeros(gray.shape,np.uint8); cv2.drawContours(rawmask,[c],-1,255,-1)
    bx,by,bw,bh=cv2.boundingRect(c)
    MASK_INFO.update({'contour_found':True,'raw_contour_area_px':float(cv2.contourArea(c)),
        'bbox_xywh':[int(bx),int(by),int(bw),int(bh)],
        'raw_mask_area_px':int(np.count_nonzero(rawmask)),'selected_mask_area_px':0})
    if CFG['segmentation_method'] in ('frame_edges_merged','frame_edges'):
        try:
            out,frame=frame_edge_mask(color,rawmask,(bx,by,bw,bh),merge_fragments=CFG['segmentation_method']=='frame_edges_merged')
        except ValueError as error:
            MASK_INFO['segmentation_reason']=str(error)
            return None
        CUT_INFO.update(active=bool(np.any((rawmask>0)&(out==0))),
            removed_pixels=int(np.count_nonzero((rawmask>0)&(out==0))),reason='frame_edge_mask')
        MASK_INFO['frame_edges']=frame
    else:
        ratio,adaptive=adaptive_cut_ratio(rawmask,bx,by,bw,bh)
        out,_,_=apply_slanted_cut(rawmask,bx,by,bw,bh,ratio=ratio)
        MASK_INFO.update(adaptive_cut_confident=bool(adaptive),cut_ratio=float(ratio))
    MASK_INFO.update(cut=dict(CUT_INFO),selected_mask_area_px=int(np.count_nonzero(out)),
        segmentation_reason='selected' if np.any(out) else 'empty_frame_mask')
    return out

def cloud_from_depth(depth, mask, K, scale, color=None, diagnostics=None):
    h,w=depth.shape; yy,xx=np.mgrid[:h,:w]; z=depth.astype(np.float32)*scale
    valid_depth=(z>.12)&(z<.50)
    mask_valid=np.zeros((h,w),dtype=bool) if mask is None else mask>0
    valid=valid_depth&mask_valid
    if diagnostics is not None:
        diagnostics['contour_found']=bool(np.any(mask_valid))
        diagnostics['valid_depth_pixels_frame']=int(np.count_nonzero(valid_depth))
        diagnostics['mask_area_px']=int(np.count_nonzero(mask_valid))
        diagnostics['valid_depth_in_mask_pixels']=int(np.count_nonzero(valid))
        diagnostics['depth_range_m']=[0.12,0.50]
    p=np.stack([(xx-K['cx'])/K['fx']*z,(yy-K['cy'])/K['fy']*z,z],-1)[valid]
    q=o3d.geometry.PointCloud(); q.points=o3d.utility.Vector3dVector(p)
    if color is not None and len(p):
        rgb=np.asarray(color)[...,::-1][valid].astype(np.float64)/255.0
        q.colors=o3d.utility.Vector3dVector(rgb)
    if len(p): q=q.voxel_down_sample(CFG['voxel_m'])
    if diagnostics is not None:
        diagnostics['target_raw_points_before_voxel']=int(len(p))
        diagnostics['target_raw_points_after_voxel']=len(q.points)
        diagnostics['voxel_m']=CFG['voxel_m']
        diagnostics['raw_cloud_reason']=None if len(q.points)>=CFG['min_target_points'] else 'raw_roi_points_below_minimum'
    return q

def replay_target_from_capture(color_path,depth_path,intrinsics,scale):
    color=cv2.imread(str(color_path),cv2.IMREAD_COLOR)
    depth=cv2.imread(str(depth_path),cv2.IMREAD_UNCHANGED)
    if color is None or depth is None: raise FileNotFoundError('missing replay color/depth')
    if color.shape[:2]!=depth.shape[:2]: raise ValueError('color/depth replay grids differ')
    mask=contour_mask(color)
    if mask is None: return color,depth,mask,None
    k={'fx':intrinsics['fx'],'fy':intrinsics['fy'],'cx':intrinsics['cx'],'cy':intrinsics['cy']}
    return color,depth,mask,cloud_from_depth(depth,mask,k,scale,color)

def surface_geometry(target, T, max_distance, context=None):
    cached=None if context is None else context.get('_surface_geometry')
    if cached is not None and cached['target'] is target and cached['max_distance']>=max_distance and np.array_equal(cached['T'],T):
        return cached
    p=np.asarray(target.points)
    socket=(p-T[:3,3])@T[:3,:3]
    # Only the <= tolerance classification is consumed. Prune farther branches
    # without changing any included point, including the equality boundary.
    bound=np.nextafter(float(max_distance),np.inf)
    distances=[layer['tree'].query(socket[:,:2],k=1,distance_upper_bound=bound)[0] for layer in TEMPLATE_CACHE['layers']]
    cached={'target':target,'T':T.copy(),'socket':socket,'xy_distances':distances,'max_distance':max_distance}
    if context is not None: context['_surface_geometry']=cached
    return cached

def filter_target_surfaces(target, model, T_camera_from_socket, z_tolerance=None, xy_tolerance=None, require_both=True, context=None):
    """Select each measured surface against its own cached XY/Z geometry."""
    info={'ok':False,'reason':'not evaluated','total_points':0,'layers':[],'coverage':0.0}
    if target is None or len(target.points)<CFG['min_target_points']:
        info['reason']=f'target_points<{CFG["min_target_points"]}'
        return None,info
    p=np.asarray(target.points); T=np.asarray(T_camera_from_socket,dtype=np.float64)
    if not np.isfinite(T).all(): info['reason']='nonfinite_pose'; return None,info
    # Reject nonfinite measurements before querying the KD trees.
    if not np.isfinite(p).all(): info['reason']='nonfinite_target'; return None,info
    geometry=surface_geometry(target,T,float(xy_tolerance or CFG['surface_xy_tolerance_m']),context)
    socket=geometry['socket']
    if not np.isfinite(socket).all(): info['reason']='nonfinite_target'; return None,info
    keep=np.zeros(len(p),dtype=bool); layer_masks=[]
    ztol=float(z_tolerance or CFG['surface_z_tolerance_m'])
    xytol=float(xy_tolerance or CFG['surface_xy_tolerance_m'])
    zdistances=np.stack([np.abs(socket[:,2]-layer['z']) for layer in TEMPLATE_CACHE['layers']],axis=1)
    nearest_layer=np.argmin(zdistances,axis=1)
    # Equal-distance points between layers are ambiguous and excluded.
    ambiguous=np.abs(zdistances[:,0]-zdistances[:,1])<1e-6
    for idx,layer in enumerate(TEMPLATE_CACHE['layers']):
        dist=geometry['xy_distances'][idx]
        xy_ok=dist<=xytol
        z_ok=zdistances[:,idx]<=ztol
        assigned=(nearest_layer==idx) & ~ambiguous
        mask=xy_ok & z_ok & assigned
        layer_masks.append(mask); keep|=mask
        support=socket[mask,:2]
        spread=np.ptp(support,axis=0).tolist() if len(support)>1 else [0.,0.]
        layer_count=int(mask.sum())
        reject_reason=('below_minimum_points' if layer_count<CFG['min_points_per_layer'] else 'ok')
        info['layers'].append({'z':layer['z'],'target_points':layer_count,'spread_xy_m':spread,
            'xy_candidates':int(xy_ok.sum()),'z_candidates':int(z_ok.sum()),
            'assigned_candidates':int(assigned.sum()),'reason':reject_reason,
            'xy_radius_rejected':int(np.count_nonzero(dist>xytol)),
            'depth_rejected':int(np.count_nonzero(zdistances[:,idx]>ztol)),
            'assigned_to_other_layer':int(np.count_nonzero((nearest_layer!=idx)&~ambiguous)),
            'ambiguous_between_layers':int(np.count_nonzero(ambiguous)),'rmse_m':None})
    info['total_points']=int(keep.sum())
    supported=[x['target_points']>=CFG['min_points_per_layer'] for x in info['layers']]
    if (require_both and not all(supported)) or (not require_both and not any(supported)):
        info['reason']='one_or_more_layers_below_minimum_points'
        return None,info
    filtered=o3d.geometry.PointCloud(); filtered.points=o3d.utility.Vector3dVector(p[keep])
    if target.has_colors(): filtered.colors=o3d.utility.Vector3dVector(np.asarray(target.colors)[keep])
    info['ok']=True; info['reason']='ok'
    return filtered,info

def pose_is_sane(T):
    T=np.asarray(T)
    return bool(T.shape==(4,4) and np.isfinite(T).all() and np.allclose(T[3],[0,0,0,1],atol=1e-6)
                and np.allclose(T[:3,:3].T@T[:3,:3],np.eye(3),atol=1e-4)
                and abs(np.linalg.det(T[:3,:3])-1)<1e-4 and CFG['minimum_pose_range_m']<T[2,3]<CFG['maximum_pose_range_m']
                and np.dot(T[:3,2],-T[:3,3]/np.linalg.norm(T[:3,3]))>=CFG['min_view_cosine'])

def pose_delta(a,b):
    return (float(np.linalg.norm(a[:3,3]-b[:3,3])),
            float(np.rad2deg(Rotation.from_matrix(a[:3,:3].T@b[:3,:3]).magnitude())))

def visible_model_points(points,T,context):
    camera=points@T[:3,:3].T+T[:3,3]
    positive=camera[:,2]>.01
    if not context or context.get('mask') is None:
        return positive,1.0
    K=context['K']; mask=context['mask']; h,w=mask.shape
    uv=np.rint((camera@K.T)[:,:2]/np.maximum(camera[:,2:3],1e-9)).astype(int)
    inside=positive&(uv[:,0]>=0)&(uv[:,0]<w)&(uv[:,1]>=0)&(uv[:,1]<h)
    support=np.zeros(len(points),bool); support[inside]=mask[uv[inside,1],uv[inside,0]]>0
    # Only independent occluder labels can remove model points from scoring.
    # Sensor depth disagreement with this same candidate must remain a penalty.
    visible=inside.copy()
    occluders=context.get('verified_occluder_mask')
    if occluders is not None:
        if np.shape(occluders)!=mask.shape: raise ValueError('occluder mask shape mismatch')
        visible[inside] &= ~np.asarray(occluders,dtype=bool)[uv[inside,1],uv[inside,0]]
    return visible,float(np.count_nonzero(support)/max(np.count_nonzero(inside),1))

def image_orientation_check(T, context=None):
    """Installation prior: socket +Y is up and +X is right in this camera view.

    This is not visual recognition of the D flat. Calibrate the reference axes
    if the camera installation rolls; never infer them from a candidate pose.
    """
    K=np.eye(3) if not context or context.get('K') is None else np.asarray(context['K'])
    t=T[:3,3]; directions=[]
    for column,reference in ((0,CFG['expected_image_right']),(1,CFG['expected_image_up'])):
        a=T[:3,column]
        tangent=K[:2,:2]@(a[:2]*t[2]-t[:2]*a[2])
        reference=np.asarray(reference,float)
        denominator=np.linalg.norm(tangent)*np.linalg.norm(reference)
        directions.append(float(np.dot(tangent,reference)/denominator) if denominator>1e-12 else -1.)
    valid=min(directions)>=CFG['orientation_min_axis_cosine']
    return {'valid':bool(valid),'x_right_cosine':directions[0],'y_up_cosine':directions[1],
            'reason':'ok' if valid else 'installation_orientation_mismatch'}


def estimate_observed_plane(target):
    """Candidate-independent dominant plane in raw ROI, never in gated points.

    A weak or spatially narrow fit is unavailable, not positive evidence.
    Fixed RNG and bounded point/proposal counts make the work reproducible.
    """
    points=np.asarray(target.points)
    points=points[np.isfinite(points).all(1)]
    if len(points)<CFG['plane_min_points']:
        return {'valid':False,'reason':'insufficient_raw_plane_points'}
    rng=np.random.default_rng(3201)
    sample=points if len(points)<=1800 else points[rng.choice(len(points),1800,replace=False)]
    best=None; best_count=0
    for _ in range(CFG['observed_plane_trials']):
        triple=sample[rng.choice(len(sample),3,replace=False)]
        n=np.cross(triple[1]-triple[0],triple[2]-triple[0]); length=np.linalg.norm(n)
        if length<1e-10: continue
        n/=length
        selected=np.abs((sample-triple[0])@n)<=CFG['observed_plane_distance_m']
        count=int(selected.sum())
        if count>best_count: best=selected; best_count=count
    if best is None: return {'valid':False,'reason':'no_independent_plane'}
    selected=sample[best]
    for _ in range(2):
        center=selected.mean(0)
        _,_,V=np.linalg.svd(selected-center,full_matrices=False); n=V[-1]
        selected=sample[np.abs((sample-center)@n)<=CFG['observed_plane_distance_m']]
        if len(selected)<3: return {'valid':False,'reason':'unstable_independent_plane'}
    center=selected.mean(0); _,singular,V=np.linalg.svd(selected-center,full_matrices=False); n=V[-1]
    if n[2]>0: n=-n
    fraction=len(selected)/len(sample)
    minor_std=float(singular[1]/np.sqrt(len(selected)))
    valid=(len(selected)>=CFG['plane_min_points'] and fraction>=CFG['observed_plane_min_fraction']
           and minor_std>=CFG['observed_plane_min_minor_std_m'])
    return {'valid':bool(valid),'normal_camera':n.tolist(),'center_camera_m':center.tolist(),
            'support_fraction':float(fraction),'inlier_points':len(selected),
            'minor_std_m':minor_std,'rmse_m':float(np.sqrt(np.mean(((selected-center)@n)**2))),
            'reason':'ok' if valid else 'weak_independent_plane'}


def independent_geometry_check(target,T,context=None,require_both=True):
    orientation=image_orientation_check(T,context)
    if context is not None and context.get('_plane_target') is target:
        plane=context['_observed_plane']
    else:
        plane=estimate_observed_plane(target)
        if context is not None:
            context['_plane_target']=target; context['_observed_plane']=plane
    angle=None
    if plane['valid']:
        cosine=abs(float(np.dot(T[:3,2],plane['normal_camera'])))
        angle=float(np.degrees(np.arccos(np.clip(cosine,0.,1.))))
    reason='ok'
    if not orientation['valid']: reason=orientation['reason']
    elif angle is not None and angle>CFG['observed_plane_max_angle_deg']: reason='observed_plane_normal_mismatch'
    elif require_both and not plane['valid']: reason='independent_plane_unavailable'
    return {'valid':reason=='ok','reason':reason,'orientation':orientation,
            'observed_plane':plane,'normal_error_deg':angle}


def evaluate_registration(model,target,T,acceptance_distance=None,context=None,require_both=True):
    """Same raw sensor ROI, 2 mm resolution and acceptance distance for all poses."""
    d=float(acceptance_distance or CFG['acceptance_distance_m'])
    out={'valid':False,'fitness':0.0,'rmse_m':None,'overall_coverage':0.0,'layers':[],'reason':'empty_or_invalid_pose'}
    if target is None or len(target.points)<CFG['min_target_points'] or not pose_is_sane(T): return out
    independent=independent_geometry_check(target,T,context,require_both)
    out['independent_geometry']=independent
    if not independent['valid']:
        out['reason']=independent['reason']; return out
    gated,gate=filter_target_surfaces(target,model,T,require_both=require_both,context=context)
    if gated is None: out['reason']=gate['reason']; return out
    p=np.asarray(target.points); geometry=surface_geometry(target,T,CFG['surface_xy_tolerance_m'],context); socket=geometry['socket']
    total_in=0; total_sq=0.0; total_visible=0; reverse_in=0; reverse_n=0; contour=[]
    loss_sum=0.0; visible_n=0
    for layer_index,layer in enumerate(TEMPLATE_CACHE['layers']):
        visible,support=visible_model_points(layer['points'],T,context); contour.append(support)
        lp=layer['points'][visible]@T[:3,:3].T+T[:3,3]
        xy_dist=geometry['xy_distances'][layer_index]
        selected=(xy_dist<=CFG['surface_xy_tolerance_m'])&(np.abs(socket[:,2]-layer['z'])<=CFG['surface_z_tolerance_m'])
        measured=p[selected]; measured_socket=socket[selected]
        dt=np.full(len(lp),np.inf); back=np.full(len(measured),np.inf); match_ids=np.array([],int)
        if len(measured) and len(lp):
            dt,match_ids=cKDTree(measured).query(lp); back,_=cKDTree(lp).query(measured)
        good=dt<=d; count=int(good.sum()); total_in+=count; total_visible+=len(lp)
        total_sq+=float(np.sum(dt[good]**2)); loss_sum+=float(np.sum(np.minimum(dt,d)**2)); visible_n+=len(lp)
        reverse_in+=int(np.count_nonzero(back<=d)); reverse_n+=len(measured)
        unique_support=len(np.unique(match_ids[good])) if count else 0
        spread=np.ptp(measured_socket[back<=d,:2],axis=0) if np.count_nonzero(back<=d)>1 else np.zeros(2)
        out['layers'].append({'z':layer['z'],'model_points':len(layer['points']),'visible_points':len(lp),
             'inlier_points':count,'distinct_measured_points':unique_support,'target_points':len(measured),
             'coverage':float(count/max(len(lp),1)),'visible_fraction':float(len(lp)/len(layer['points'])),
             'reverse_coverage':float(np.mean(back<=d)) if len(back) else 0.0,
             'spread_xy_m':spread.tolist(),'rmse_m':float(np.sqrt(np.mean(dt[good]**2))) if count else None})
    out['overall_coverage']=float(total_in/max(total_visible,1)); out['fitness']=out['overall_coverage']
    out['reverse_coverage']=float(reverse_in/max(reverse_n,1)); out['contour_support']=float(min(contour))
    out['rmse_m']=float(np.sqrt(total_sq/total_in)) if total_in else None
    out['geometric_loss']=float(loss_sum/max(visible_n,1)/d**2)
    # At longer range the rear 15 mm layer is sampled by fewer depth pixels.
    # Keep the normal 18% gate nearby, and use a small, explicit relaxation
    # only after the measured pose is beyond the far-range boundary.
    layer_coverage_gate = (CFG['far_min_layer_coverage']
        if float(T[2,3]) >= CFG['far_pose_range_m'] else CFG['min_layer_coverage'])
    out['acceptance_layer_coverage_min']=float(layer_coverage_gate)
    layer_pass=[x['distinct_measured_points']>=CFG['min_points_per_layer'] and x['coverage']>=layer_coverage_gate
         and x['visible_fraction']>=CFG['min_visible_fraction'] and min(x['spread_xy_m'])>=CFG['min_layer_spread_m']
         and x['rmse_m'] is not None and x['rmse_m']<=CFG['max_layer_rmse_m'] for x in out['layers']]
    out['supported_layers']=sum(layer_pass)
    good_layers=all(layer_pass) if require_both else any(layer_pass)
    out['valid']=bool(total_in and good_layers and out['overall_coverage']>=CFG['min_overall_coverage']
         and out['reverse_coverage']>=CFG['min_reverse_coverage'] and out['contour_support']>=CFG['min_contour_support'])
    out['reason']='ok' if out['valid'] else 'layer_or_overall_acceptance_failed'
    return out

def full_rgbd_cloud(depth, color, K, scale):
    h,w=depth.shape; yy,xx=np.mgrid[:h,:w]; z=depth.astype(np.float32)*scale
    valid=(z>.12)&(z<.80)
    p=np.stack([(xx-K['cx'])/K['fx']*z,(yy-K['cy'])/K['fy']*z,z],-1)[valid]
    if len(p)==0: return None
    q=o3d.geometry.PointCloud(); q.points=o3d.utility.Vector3dVector(p)
    rgb=np.asarray(color)[...,::-1][valid].astype(np.float64)/255.0
    q.colors=o3d.utility.Vector3dVector(rgb)
    return q.voxel_down_sample(.006)

def run_icp(model,target,seed,max_iteration=15,threshold=None):
    threshold=float(threshold or CFG['tracking_correspondence_m'])
    return o3d.pipelines.registration.registration_icp(
        model,target,threshold,seed,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=max_iteration))

def run_icp_multiscale(model,target,seed,deadline=None,context=None,poll_gui=None):
    """First stage uses raw ROI; later stages gate per candidate, not history."""
    current=seed.copy(); result=None
    for stage,(voxel,threshold,iterations) in enumerate(zip(CFG['recovery_voxel_m'],CFG['recovery_correspondence_m'],CFG['recovery_iterations'])):
        if poll_gui is not None: poll_gui()
        if deadline is not None and time.monotonic()>=deadline: return None
        if stage==0:
            selected=target
        else:
            selected,_=filter_target_surfaces(target,model,current,
                z_tolerance=max(CFG['surface_z_tolerance_m'],threshold),
                xy_tolerance=max(CFG['surface_xy_tolerance_m'],threshold),require_both=False,context=context)
        if selected is None: return result
        m=model.voxel_down_sample(voxel) if voxel>CFG['voxel_m'] else model
        t=selected.voxel_down_sample(voxel) if voxel>CFG['voxel_m'] else selected
        if min(len(m.points),len(t.points))<40: return result
        result=run_icp(m,t,current,max_iteration=iterations,threshold=threshold)
        if len(result.correspondence_set)==0 or not pose_is_sane(result.transformation): return None
        current=result.transformation
    return result

def smooth_axis_rotation(T_axis, T_raw, alpha=0.22):
    """Smooth only axis orientation; keep the raw pose and raw origin intact."""
    out=T_axis.copy()
    r0=Rotation.from_matrix(T_axis[:3,:3])
    rel=r0.inv()*Rotation.from_matrix(T_raw[:3,:3])
    out[:3,:3]=(r0*Rotation.from_rotvec(alpha*rel.as_rotvec())).as_matrix()
    out[:3,3]=T_raw[:3,3]
    return out

def direction_position_score(T, K):
    """Score approximate requested image-space axis positions, 0..1.

    Socket +X should project right, +Y should project up, and +Z should point
    outward toward the camera. This is a ranking prior, not a hard rejection.
    """
    origin=np.asarray(T[:3,3],dtype=np.float64)
    pts=np.array([origin, origin+T[:3,0]*.05, origin+T[:3,1]*.05],dtype=np.float64)
    uv,_=cv2.projectPoints(pts,np.zeros(3),np.zeros(3),K,np.zeros(5)) if False else (None,None)
    rvec,_=cv2.Rodrigues(T[:3,:3])
    uv,_=cv2.projectPoints(np.array([[0,0,0],[.05,0,0],[0,.05,0]],np.float32),rvec,T[:3,3],K,np.zeros(5))
    uv=uv.reshape(-1,2); dx=uv[1]-uv[0]; dy=uv[2]-uv[0]
    x_right=float(np.clip(dx[0]/(np.linalg.norm(dx)+1e-9),-1,1)*.5+.5)
    y_up=float(np.clip(-dy[1]/(np.linalg.norm(dy)+1e-9),-1,1)*.5+.5)
    z_out=float(np.clip(-T[2,2],-1,1)*.5+.5)
    return float((x_right+y_up+z_out)/3.0), (x_right,y_up,z_out)

def multistart_icp(model, target_raw, current_T, initial_T, K, center_T=None,context=None,
                   max_candidates=None, timeout_s=None, initial_poses=None, search_offset=0, poll_gui=None):
    """Try every source seed first, then interleave bounded perturbation families."""
    sources=list(initial_poses or [])
    if not sources:
        sources=[('trusted',current_T),('initial',initial_T),('plane_center',center_T)]
    unique=[]; seen_seed=set()
    for label,pose in sources:
        if pose is None or not pose_is_sane(pose): continue
        key=tuple(np.round(pose.ravel(),6))
        if key in seen_seed: continue
        seen_seed.add(key); unique.append((str(label),np.asarray(pose).copy()))
    # The first budget slots are one unmodified seed per source.
    seeds=[(label,pose,'raw') for label,pose in unique]
    # A rejected inverted source may propose an upright half-turn, which must
    # still pass ICP and every geometry/confirmation check before adoption.
    for label,pose in unique:
        if not image_orientation_check(pose,context)['valid']:
            D=np.diag([-1.,-1.,1.,1.])
            seeds.append((label,pose@D,'yaw'))
    variants=[]
    for label,base in unique:
        for deg in range(int(CFG['recovery_seed_yaw_step_deg']),360,int(CFG['recovery_seed_yaw_step_deg'])):
            D=np.eye(4); D[:3,:3]=Rotation.from_euler('z',deg,degrees=True).as_matrix()
            variants.append((label,base@D,'yaw'))
        for offset in CFG['recovery_translation_m']:
            if abs(offset)<1e-9: continue
            for axis in range(3):
                shifted=base.copy(); shifted[:3,3]+=base[:3,axis]*offset
                variants.append((label,shifted,'translation'))
        for rx,ry in ((10,0),(-10,0),(0,10),(0,-10),(10,10),(-10,-10)):
            D=np.eye(4); D[:3,:3]=Rotation.from_euler('xyz',[rx,ry,0],degrees=True).as_matrix()
            variants.append((label,base@D,'tilt'))
    # Round-robin candidate family lists across each source so later seeds are
    # represented even when the configured cap is reached.
    families=[]
    for family in ('translation','yaw','tilt'):
        per_source=[[x for x in variants if x[0]==label and x[2]==family] for label,_ in unique]
        families.extend(per_source)
    interleaved=[]
    while any(families):
        for family in families:
            if family: interleaved.append(family.pop(0))
    if interleaved:
        offset=int(search_offset)%len(interleaved)
        interleaved=interleaved[offset:]+interleaved[:offset]
    seeds.extend(interleaved)
    candidates=[]; rejected=[]; seen=set(); source_tested={label:{'raw':0,'translation':0,'yaw':0,'tilt':0} for label,_ in unique}; started=time.monotonic()
    max_candidates=int(max_candidates or CFG['recovery_max_candidates'])
    deadline=started+float(timeout_s or CFG['recovery_timeout_s'])
    for label,seed,family in seeds:
        if poll_gui is not None: poll_gui()
        if len(seen)>=max_candidates or time.monotonic()>=deadline: break
        key=tuple(np.round(seed[:3,3],4).tolist()+Rotation.from_matrix(seed[:3,:3]).as_rotvec().round(4).tolist())
        if key in seen or not pose_is_sane(seed): continue
        seen.add(key)
        source_tested[label][family]+=1
        result=run_icp_multiscale(model,target_raw,seed,deadline=deadline,context=context,poll_gui=poll_gui)
        if result is None or not np.isfinite(result.transformation).all(): continue
        quality=evaluate_registration(model,target_raw,result.transformation,context=context)
        if not quality['valid']:
            relaxed=evaluate_registration(model,target_raw,result.transformation,context=context,require_both=False)
            rejected.append((float(relaxed.get('geometric_loss',1.0)),result.transformation.copy(),quality,label,family))
            continue
        fitness=float(quality['fitness']); rmse=float(quality['rmse_m'])
        dscore,components=direction_position_score(result.transformation,K)
        continuity=pose_delta(current_T,result.transformation)[0] if current_T is not None else 0.0
        candidates.append((quality['geometric_loss'],continuity,-dscore,result.transformation,quality,dscore,components,label,family))
    if not candidates:
        rejected.sort(key=lambda x:x[0])
        best_rejected=rejected[0] if rejected else None
        return {'failed':True,'seed_count':len(seen),'source_tested':source_tested,
            'elapsed_s':time.monotonic()-started,'timed_out':time.monotonic()>=deadline,
            'reason':'no_accepted_candidates' if seen else 'no_usable_initial_pose',
            'best_candidate_T':None if best_rejected is None else best_rejected[1],
            'best_candidate_quality':None if best_rejected is None else best_rejected[2]}
    # Geometry dominates; continuity/direction only break near-identical loss.
    candidates.sort(key=lambda x:(x[0],x[1],x[2]))
    best=candidates[0]
    # Similar geometric losses for distinctly different poses are ambiguous.
    alternative=next((x for x in candidates[1:] if
        pose_delta(best[3],x[3])[0]>CFG['ambiguity_translation_m'] or
        pose_delta(best[3],x[3])[1]>CFG['ambiguity_rotation_deg']),None)
    if alternative is not None and alternative[0]-best[0]<=CFG['ambiguity_loss_margin']:
        return {'failed':True,'reason':'ambiguous_geometric_solutions','seed_count':len(seen),
            'source_tested':source_tested,'elapsed_s':time.monotonic()-started,
            'timed_out':time.monotonic()>=deadline,'best_candidate_T':best[3],
            'best_candidate_quality':best[4],'alternative_T':alternative[3],
            'loss_gap':float(alternative[0]-best[0])}
    winning,gate=filter_target_surfaces(target_raw,model,best[3],context=context)
    if winning is None: return None
    return {'T':best[3],'quality':best[4],'gate':gate,'target':winning,
            'direction_score':best[5],'direction_components':best[6],'seed_count':len(seen),
            'source_tested':source_tested,'selected_source':best[7],'selected_family':best[8],
            'elapsed_s':time.monotonic()-started,'timed_out':time.monotonic()>=deadline}

def draw_axes(img,T,K,size=.060,origin_camera=None):
    # Socket 坐标系：红 +X 向右，绿 +Y 向上，蓝 +Z 向外，黄 -Z 插入。
    obj=np.array([[0,0,0],[size,0,0],[0,size,0],[0,0,size],[0,0,-size]],np.float32)
    rvec,_=cv2.Rodrigues(T[:3,:3]); tvec=(T[:3,3] if origin_camera is None else np.asarray(origin_camera)).astype(np.float64)
    uv,_=cv2.projectPoints(obj,rvec,tvec,K,np.zeros(5)); uv=uv.reshape(-1,2).astype(int)
    o,x,y,z,minus_z=uv
    # Thick high-contrast lines, with black underlay for visibility on the image.
    for end,col in ((x,(0,0,255)),(y,(0,255,0)),(z,(255,0,0)),(minus_z,(255,255,0))):
        cv2.line(img,tuple(o),tuple(end),(0,0,0),8,cv2.LINE_AA)
        cv2.line(img,tuple(o),tuple(end),col,5,cv2.LINE_AA)
        cv2.circle(img,tuple(end),6,(0,0,0),-1,cv2.LINE_AA)
        cv2.circle(img,tuple(end),4,col,-1,cv2.LINE_AA)
    cv2.circle(img,tuple(o),7,(0,0,0),-1,cv2.LINE_AA)
    cv2.circle(img,tuple(o),4,(255,255,255),-1,cv2.LINE_AA)
    labels=[('Socket +X (right)',x,(0,0,255)),('Socket +Y (up)',y,(0,255,0)),('Socket +Z (out)',z,(255,0,0)),('Socket -Z (insert)',minus_z,(255,255,0))]
    for text,p,col in labels:
        org=(int(p[0]+8),int(p[1]-8))
        cv2.putText(img,text,org,cv2.FONT_HERSHEY_SIMPLEX,.60,(0,0,0),4,cv2.LINE_AA)
        cv2.putText(img,text,org,cv2.FONT_HERSHEY_SIMPLEX,.60,col,2,cv2.LINE_AA)

def project_points(points, T, K, shape):
    """Project camera-frame or socket-frame points into the RGB image."""
    p=np.asarray(points, dtype=np.float32)
    if len(p)==0: return np.empty((0,2), dtype=np.int32), np.empty((0,), dtype=bool)
    rvec,_=cv2.Rodrigues(T[:3,:3])
    uv,_=cv2.projectPoints(p,rvec,T[:3,3],K,np.zeros(5))
    uv=np.rint(uv.reshape(-1,2)).astype(np.int32)
    h,w=shape[:2]; valid=(uv[:,0]>=0)&(uv[:,0]<w)&(uv[:,1]>=0)&(uv[:,1]<h)
    return uv,valid

def draw_registration_view(target_raw, target_filtered, model, T, K, shape, fit, rmse, status, diagnostics):
    """Show this-frame raw ROI and gated ICP target independently."""
    view=np.zeros(shape,dtype=np.uint8)
    if target_raw is not None and len(target_raw.points):
        uv,valid=project_points(np.asarray(target_raw.points),np.eye(4),K,shape)
        for u,v in uv[valid][::2]: cv2.circle(view,(int(u),int(v)),2,(255,120,20),-1)
    if target_filtered is not None and len(target_filtered.points):
        uv,valid=project_points(np.asarray(target_filtered.points),np.eye(4),K,shape)
        for u,v in uv[valid][::2]: cv2.circle(view,(int(u),int(v)),2,(255,220,0),-1)
    if model is not None and len(model.points):
        uv,valid=project_points(np.asarray(model.points),T,K,shape)
        for u,v in uv[valid][::2]: cv2.circle(view,(int(u),int(v)),2,(0,0,255),-1)
    raw_n=0 if target_raw is None else len(target_raw.points)
    filtered_n=0 if target_filtered is None else len(target_filtered.points)
    cv2.putText(view,f'RAW ROI blue={raw_n} | FILTERED cyan={filtered_n} | CAD red',(8,24),cv2.FONT_HERSHEY_SIMPLEX,.48,(255,255,255),1)
    rmse_text='n/a' if rmse is None else f'{rmse*1000:.1f}mm'
    cv2.putText(view,f'{status} fit={fit:.3f} rmse={rmse_text} contour={diagnostics.get("contour_found")} area={diagnostics.get("mask_area_px",0)} depth={diagnostics.get("valid_depth_in_mask_pixels",0)}',(8,48),cv2.FONT_HERSHEY_SIMPLEX,.40,(0,255,255),1)
    display_gate=diagnostics.get('gate_diagnostic',diagnostics.get('gate',{}))
    layer_info=display_gate.get('layers',[])
    for i,z in enumerate((-.015,0.0)):
        item=next((x for x in layer_info if abs(x.get('z',9)-z)<1e-4),{})
        count=item.get('target_points',0)
        layer_reason=item.get('reason',display_gate.get('reason','not_run'))
        cv2.putText(view,f'layer z={z:+.3f}: n={count} gate={layer_reason}',(8,70+i*20),cv2.FONT_HERSHEY_SIMPLEX,.43,(255,255,255),1)
    return view

def draw_contour_view(color,depth=None,diagnostics=None,depth_scale=0.001,selected_mask=None):
    """Show the full-frame image-processing stages, not an ML detector."""
    gray=cv2.cvtColor(color,cv2.COLOR_BGR2GRAY)
    x0,y0,x1,y1=0,0,640,480
    roi=gray[y0:y1,x0:x1]
    raw=(roi<80).astype(np.uint8)*255
    raw=cv2.morphologyEx(raw,cv2.MORPH_CLOSE,cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(7,7)),iterations=1)
    raw=cv2.morphologyEx(raw,cv2.MORPH_OPEN,cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(5,5)),iterations=1)
    raw_full=np.zeros_like(gray); raw_full[y0:y1,x0:x1]=raw
    cs,_=cv2.findContours(raw,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    cs=[c for c in cs if cv2.contourArea(c)>500]
    raw_canvas=np.zeros_like(gray)
    selected=np.zeros_like(gray)
    overlay=color.copy()
    if cs:
        c=max(cs,key=cv2.contourArea).copy(); c[:,:,0]+=x0; c[:,:,1]+=y0
        cv2.drawContours(raw_canvas,[c],-1,255,-1)
    # Render the exact mask used by ICP; do not recompute or mutate CUT_INFO.
    if selected_mask is not None: selected[:]=selected_mask
    overlay[selected>0]=(0,140,255)
    diagnostics=diagnostics or {}
    frame=diagnostics.get('frame_edges',{})
    if frame.get('quad') is not None:
        cv2.polylines(overlay,[np.rint(frame['quad']).astype(np.int32)],True,(0,255,255),2)
    line=diagnostics.get('cut',{}).get('line')
    if line is not None:
        xc,yc,slope=line
        bx,by,bw,bh=diagnostics['bbox_xywh']
        p1=(int(round(xc+slope*(by-yc))),by)
        p2=(int(round(xc+slope*(by+bh-yc))),by+bh)
        cv2.line(overlay,p1,p2,(0,0,255),3,cv2.LINE_AA)
    label=diagnostics.get('segmentation_method','')+': '+diagnostics.get('segmentation_reason','')
    cv2.putText(overlay,label,(12,465),cv2.FONT_HERSHEY_SIMPLEX,.46,(0,255,255),2)
    def panel(title, im):
        p=cv2.cvtColor(im,cv2.COLOR_GRAY2BGR) if im.ndim==2 else im.copy()
        cv2.putText(p,title,(10,28),cv2.FONT_HERSHEY_SIMPLEX,.65,(0,255,255),2)
        return p
    top=np.hstack([panel('FULL FRAME GRAY < 80',raw_full),panel('RAW CONTOUR',raw_canvas)])
    bot=np.hstack([panel('SELECTED ORANGE MASK',selected),panel('OVERLAY: ORANGE USED FOR ICP',overlay)])
    view=np.vstack([top,bot])
    if depth is not None:
        valid=((depth.astype(np.float32)*depth_scale)>.12)&((depth.astype(np.float32)*depth_scale)<.50)
        depth_vis=np.zeros((*depth.shape,3),np.uint8); depth_vis[valid]=(0,180,0)
        depth_panel=panel('DEPTH VALID 0.12-0.50m',depth_vis)
        detail=np.zeros_like(depth_panel)
        diagnostics=diagnostics or {}
        lines=[f"contour={diagnostics.get('contour_found')} area={diagnostics.get('mask_area_px',0)}",
               f"valid depth frame={diagnostics.get('valid_depth_pixels_frame',0)} in ROI={diagnostics.get('valid_depth_in_mask_pixels',0)}",
               f"raw before/after voxel={diagnostics.get('target_raw_points_before_voxel',0)}/{diagnostics.get('target_raw_points_after_voxel',0)}",
               f"gate={diagnostics.get('gate',{}).get('reason','not run')}"]
        for i,line in enumerate(lines): cv2.putText(detail,line,(8,45+i*38),cv2.FONT_HERSHEY_SIMPLEX,.60,(255,255,255),2)
        view=np.vstack([view,np.hstack([depth_panel,detail])])
    return view

def mask_center_camera(mask, depth, ci, scale):
    ys, xs = np.where(mask > 0)
    if len(xs) == 0:
        return None
    # Use the detected contour bounding-box center, not raw RGB centroid, so
    # asymmetric dark regions do not pull the display origin toward a hole/clip.
    u = int(round((xs.min() + xs.max()) * 0.5)); v = int(round((ys.min() + ys.max()) * 0.5))
    r = 6; y0=max(0,v-r); y1=min(depth.shape[0],v+r+1); x0=max(0,u-r); x1=min(depth.shape[1],u+r+1)
    z = depth[y0:y1,x0:x1].astype(np.float32)*scale
    vals = z[(z > .12) & (z < .50)]
    if len(vals) == 0:
        return None
    d = float(np.median(vals))
    return np.asarray(rs.rs2_deproject_pixel_to_point(ci, [u,v], d), dtype=float), (u,v), d

def write_pose_output(path,T_raw,T_axis,status,fitness,rmse,score,direction_components,write_time,success_time,valid,quality=None,gate=None,acquisition_time=None,failure_reason=None):
    R=T_raw[:3,:3]
    age=None if success_time is None else max(0.0,float(write_time-success_time))
    quality=quality or {}
    gate=gate or {}
    payload={
        'read_only':True,
        'frame_convention':'P_camera = T_camera_from_socket @ P_socket',
        'camera_frame':'realsense_color_optical_frame',
        'target_frame':'socket_model_frame',
        'base_pose_enabled':False,
        'T_camera_from_socket_raw':np.asarray(T_raw).tolist(),
        'T_camera_from_socket_axis_display':np.asarray(T_axis).tolist(),
        'socket_origin_camera_m':np.asarray(T_raw[:3,3]).tolist(),
        'socket_axes_camera':{'x_plus':R[:,0].tolist(),'y_plus':R[:,1].tolist(),'z_plus':R[:,2].tolist(),'z_minus':(-R[:,2]).tolist()},
        'status':status,'fitness':float(fitness) if np.isfinite(fitness) else None,
        'rmse_m':float(rmse) if np.isfinite(rmse) else None,'score':float(score) if np.isfinite(score) else None,
        'direction_score_xyz':list(map(float,direction_components)),
        'timestamp':float(write_time),
        'write_time':float(write_time),
        'measurement_time':None if success_time is None else float(success_time),
        'frame_acquisition_time':None if acquisition_time is None else float(acquisition_time),
        'last_success_time':None if success_time is None else float(success_time),
        'measurement_age_s':age,
        'pose_valid':bool(valid),
        'pose_fresh':bool(valid and age is not None and age<=CFG['max_measurement_age_s']),
        'failure_reason':None if valid else str(failure_reason or status),
        'quality':quality,
        'surface_gate':gate,
    }
    tmp=path.with_suffix(path.suffix+'.tmp'); tmp.write_text(json.dumps(payload,indent=2)); tmp.replace(path)


class PoseState:
    """Only complete geometry updates trusted_T; degraded updates never renew it."""
    def __init__(self):
        self.trusted_T=None; self.tracking_T=None; self.strong_time=None
        self.trusted_quality=None
        self.pending=None; self.pending_time=None; self.pending_count=0
        self.level='invalid'; self.reason='not_initialized'; self.current_time=None

    def clear_pending(self):
        self.pending=None; self.pending_time=None; self.pending_count=0

    def observe(self,T,quality,stamp,now,recovery=False):
        self.level='invalid'; self.current_time=None
        if now<stamp or now-stamp>CFG['max_measurement_age_s']:
            self.reason='frame_expired_during_processing'; self.clear_pending(); return False
        if T is None or not pose_is_sane(T):
            self.reason='invalid_candidate'; self.clear_pending(); return False
        if quality.get('valid'):
            if recovery:
                consistent=False
                if self.pending is not None and self.pending_time is not None and 0<=stamp-self.pending_time<=CFG['recovery_confirm_window_s']:
                    dt,dr=pose_delta(self.pending,T)
                    consistent=stamp>self.pending_time and dt<=CFG['recovery_confirm_translation_m'] and dr<=CFG['recovery_confirm_rotation_deg']
                self.pending_count=self.pending_count+1 if consistent else 1
                self.pending=T.copy(); self.pending_time=stamp
                if self.pending_count<CFG['recovery_confirm_frames']:
                    self.level='candidate'; self.reason='pending_confirmation'; return False
            self.trusted_T=T.copy(); self.tracking_T=T.copy(); self.strong_time=stamp
            self.trusted_quality=dict(quality)
            self.current_time=stamp; self.level='full_geometry'; self.reason='ok'
            self.clear_pending(); return True
        self.reason=quality.get('reason','geometry_rejected'); self.clear_pending()
        return False

    def degrade(self,T,quality,stamp,now):
        self.level='invalid'; self.current_time=None; self.clear_pending()
        if self.trusted_T is None or self.tracking_T is None or self.strong_time is None:
            self.reason='no_recent_complete_pose'; return False
        if not quality.get('valid') or T is None or not pose_is_sane(T):
            self.reason='partial_geometry_rejected'; return False
        if stamp<=self.strong_time or now<stamp or now-stamp>CFG['max_measurement_age_s'] or now-self.strong_time>CFG['degraded_hold_s']:
            self.reason='degraded_hold_expired'; return False
        step=pose_delta(self.tracking_T,T); anchor=pose_delta(self.trusted_T,T)
        if step[0]>CFG['degraded_max_translation_m'] or step[1]>CFG['degraded_max_rotation_deg'] or anchor[0]>CFG['degraded_anchor_translation_m'] or anchor[1]>CFG['degraded_anchor_rotation_deg']:
            self.reason='degraded_motion_limit'; return False
        self.tracking_T=T.copy(); self.current_time=stamp
        self.level='degraded_tracking'; self.reason='partial_geometry_only'; return True


def json_safe(value):
    if isinstance(value,np.ndarray): return json_safe(value.tolist())
    if isinstance(value,(np.floating,float)): return float(value) if np.isfinite(value) else None
    if isinstance(value,np.integer): return int(value)
    if isinstance(value,np.bool_): return bool(value)
    if isinstance(value,dict): return {str(k):json_safe(v) for k,v in value.items()}
    if isinstance(value,(tuple,list)): return [json_safe(v) for v in value]
    return value


def recovery_diagnostic(result):
    """Do not put an Open3D PointCloud into the frame JSON log."""
    if result is None: return None
    return json_safe({k:v for k,v in result.items() if k!='target'})


def write_state_output(path,state,status,quality,gate,stamp,now,best_candidate=None):
    strong=state.trusted_T
    track=state.tracking_T
    age=None if state.strong_time is None else max(0.0,now-state.strong_time)
    valid=bool(state.level=='full_geometry' and age is not None and age<=CFG['max_measurement_age_s'])
    payload={'schema_version':3,'read_only':True,'base_pose_enabled':False,
        'frame_convention':'P_camera = T_camera_from_socket @ P_socket',
        'camera_frame':'realsense_color_optical_frame','target_frame':'socket_model_frame',
        'T_camera_from_socket_raw':strong,'T_camera_from_socket_tracking':track,
        'T_camera_from_socket_axis_display':track,
        'socket_origin_camera_m':None if strong is None else strong[:3,3],
        'socket_axes_camera':None if strong is None else {'x_plus':strong[:3,0],'y_plus':strong[:3,1],'z_plus':strong[:3,2],'z_minus':-strong[:3,2]},
        'timestamp':now,'write_time':now,'measurement_time':state.strong_time,
        'last_success_time':state.strong_time,'measurement_age_s':age,
        'frame_acquisition_time':stamp,'tracking_measurement_time':state.current_time,
        'pose_valid':valid,'pose_fresh':valid,'quality_level':state.level,
        'tracking_valid':state.level in ('full_geometry','degraded_tracking'),
        'planning_ready':False,
        'planning_block_reason':'geometric_match_is_not_independent_pose_or_docking_validation',
        'status':status,'failure_reason':None if valid else state.reason,
        'fitness':quality.get('fitness',0.),'rmse_m':quality.get('rmse_m'),
        'score':None if 'geometric_loss' not in quality else 1.-quality['geometric_loss'],
        'quality':quality,'trusted_pose_quality':state.trusted_quality,
        'surface_gate':gate,'best_candidate':best_candidate}
    tmp=path.with_suffix(path.suffix+'.tmp'); tmp.parent.mkdir(parents=True,exist_ok=True)
    tmp.write_text(json.dumps(json_safe(payload),indent=2,allow_nan=False),encoding='utf-8'); tmp.replace(path)



def cad_outline_curves():
    """Exact source-CAD nominal outlines; all XYZ in socket coordinates, meters."""
    def d_profile(radius,top,fillet,z):
        cy=top-fillet; cx=np.sqrt((radius-fillet)**2-cy*cy)
        alpha=np.arctan2(cy,cx)
        right=np.linspace(np.pi/2,alpha,30)
        bottom=np.linspace(alpha,-np.pi-alpha,120)
        left=np.linspace(np.pi-alpha,np.pi/2,30)
        xy=np.vstack([[-cx,top],[cx,top],
            np.c_[cx+fillet*np.cos(right),cy+fillet*np.sin(right)],
            np.c_[radius*np.cos(bottom),radius*np.sin(bottom)],
            np.c_[-cx+fillet*np.cos(left),cy+fillet*np.sin(left)],[-cx,top]])
        return np.c_[xy,np.full(len(xy),z)]
    rectangle=[]
    for cx,cy,angle in ((.032,.032,0),(-.032,.032,90),(-.032,-.032,180),(.032,-.032,270)):
        t=np.deg2rad(np.linspace(angle,angle+90,25))
        rectangle.extend(np.c_[cx+.006*np.cos(t),cy+.006*np.sin(t),np.full(len(t),-.015)])
    rectangle=np.vstack([rectangle,rectangle[0]])
    return {'frame_front':rectangle,'d_outer':d_profile(.0315,.02455,.01605,0.),
            'd_inner':d_profile(.02575,.0188,.0103,0.)}


def project_curve(points,T,K):
    camera=np.asarray(points)@T[:3,:3].T+T[:3,3]
    if not np.isfinite(camera).all() or np.any(camera[:,2]<=.01): return None
    return (camera@K.T)[:,:2]/camera[:,2:3]


def draw_cad_outlines(image,T,K,source):
    colors={'frame_front':(255,255,0),'d_outer':(0,255,0),'d_inner':(255,0,255)}
    for name,curve in cad_outline_curves().items():
        uv=project_curve(curve,T,K)
        if uv is None: continue
        pixels=np.clip(np.rint(uv),-100000,100000).astype(np.int32).reshape(-1,1,2)
        cv2.polylines(image,[pixels],True,(0,0,0),5,cv2.LINE_AA)
        cv2.polylines(image,[pixels],True,colors[name],2,cv2.LINE_AA)
    cv2.putText(image,'CAD outlines: frame cyan / outer green / inner magenta',(8,image.shape[0]-33),cv2.FONT_HERSHEY_SIMPLEX,.40,(0,0,0),3)
    cv2.putText(image,'CAD outlines: frame cyan / outer green / inner magenta',(8,image.shape[0]-33),cv2.FONT_HERSHEY_SIMPLEX,.40,(255,255,255),1)
    cv2.putText(image,'DISPLAY: '+source+' | planning blocked',(8,image.shape[0]-12),cv2.FONT_HERSHEY_SIMPLEX,.43,(0,0,0),3)
    cv2.putText(image,'DISPLAY: '+source+' | planning blocked',(8,image.shape[0]-12),cv2.FONT_HERSHEY_SIMPLEX,.43,(0,255,255),1)
    return image


def save_frame_capture(directory,frame_id,rgb,depth,camera,diagnostic,pose_payload,overlay):
    prefix=Path(directory)/(time.strftime('%Y%m%d_%H%M%S')+f'_f{frame_id:06d}')
    cv2.imwrite(str(prefix)+'_rgb.png',rgb)
    np.save(str(prefix)+'_depth.npy',depth)
    cv2.imwrite(str(prefix)+'_cad_overlay.png',overlay)
    Path(str(prefix)+'_camera.json').write_text(json.dumps(json_safe(camera),indent=2,allow_nan=False),encoding='utf-8')
    Path(str(prefix)+'_diagnostics.json').write_text(json.dumps(json_safe(diagnostic),indent=2,allow_nan=False),encoding='utf-8')
    Path(str(prefix)+'_pose.json').write_text(json.dumps(json_safe(pose_payload),indent=2,allow_nan=False),encoding='utf-8')
    print('CAPTURED',prefix,flush=True)
    return str(prefix)


class PreviewClosed(Exception):
    pass


def main():
    global CONFIG,MODEL,INIT_REPORT
    ap=argparse.ArgumentParser()
    ap.add_argument('--fps',type=int,default=15)
    ap.add_argument('--show-candidates',action='store_true',
        help='Diagnostic overlay of unconfirmed candidates; never a valid target')
    ap.add_argument('--segmentation',choices=('frame_edges_merged','frame_edges','adaptive_cut'),
        help='frame_edges_merged: merged fragments; frame_edges: previous version; adaptive_cut: legacy cut')
    ap.add_argument('--save-dir',type=Path,default=ROOT/'results/v3_diagnostics')
    ap.add_argument('--pose-output',type=Path,default=ROOT/'results/v3_diagnostics/latest_pose_readonly.json')
    ap.add_argument('--config',type=Path,default=CONFIG)
    ap.add_argument('--model',type=Path,default=MODEL)
    ap.add_argument('--init-report',type=Path,default=INIT_REPORT)
    ap.add_argument('--no-display',action='store_true')
    ap.add_argument('--max-frames',type=int,default=0)
    ap.add_argument('--capture-first-full',action='store_true',help='Save first complete-geometry frame for offline review')
    args=ap.parse_args(); CONFIG=args.config; MODEL=args.model; INIT_REPORT=args.init_report
    args.save_dir.mkdir(parents=True,exist_ok=True)
    model=model_features(); state=PoseState(); initial_T=None
    if args.segmentation: CFG['segmentation_method']=args.segmentation
    print('ICP_SEGMENTATION method='+CFG['segmentation_method'],flush=True)
    if INIT_REPORT.exists():
        try:
            proposed=np.asarray(json.loads(INIT_REPORT.read_text())['T_camera_from_socket_candidate'],float)
            if pose_is_sane(proposed): initial_T=proposed
        except (KeyError,ValueError,TypeError,json.JSONDecodeError) as error:
            print('INIT_REPORT ignored:',str(error))
    pipe=rs.pipeline(); cfg=rs.config()
    cfg.enable_stream(rs.stream.color,640,480,rs.format.bgr8,args.fps)
    cfg.enable_stream(rs.stream.depth,640,480,rs.format.z16,args.fps)
    if not args.no_display:
        cv2.namedWindow('Contour Image Processing',cv2.WINDOW_NORMAL)
        cv2.resizeWindow('Contour Image Processing',768,864)
    capture_requested=False
    last_gui_poll=0.0
    def poll_gui():
        nonlocal capture_requested,last_gui_poll
        if args.no_display: return
        now=time.monotonic()
        if now-last_gui_poll<0.03: return
        last_gui_poll=now
        key=cv2.waitKey(1)&0xff
        if key in (27,ord('q')): raise PreviewClosed
        if key==ord('s'): capture_requested=True
    frame_id=0; fail_count=0; recovery_cycle=0; first_full_captured=False
    try:
        profile=pipe.start(cfg); align=rs.align(rs.stream.color)
        scale=profile.get_device().first_depth_sensor().get_depth_scale()
        ci=profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
        K=np.array([[ci.fx,0,ci.ppx],[0,ci.fy,ci.ppy],[0,0,1]],float)
        for _ in range(20): pipe.wait_for_frames(timeout_ms=5000)
        while True:
            fs=align.process(pipe.wait_for_frames(timeout_ms=5000))
            cf=fs.get_color_frame(); df=fs.get_depth_frame()
            if not cf or not df: continue
            stamp=time.time(); raw_color=np.asanyarray(cf.get_data()).copy(); depth=np.asanyarray(df.get_data())
            poll_gui()
            mask=contour_mask(raw_color); diag=dict(MASK_INFO)
            poll_gui()
            target_raw=cloud_from_depth(depth,mask,{'fx':ci.fx,'fy':ci.fy,'cx':ci.ppx,'cy':ci.ppy},scale,raw_color,diag)
            context={'K':K,'mask':mask,'depth_m':depth.astype(np.float32)*scale}
            quad,_,conf=dynamic_quad(raw_color,mask)
            # An unverified quad can propose a seed, never grant acceptance.
            pnp_seed=pnp_from_dynamic_quad(quad,K) if quad is not None and conf>=.20 else None
            center_seed=None
            if pnp_seed is not None and QUAD_INFO.get('verified'):
                center=quad_center_seed(quad,depth,{'_intrinsics':ci},scale,pnp_seed[:3,:3])
                if center is not None: center_seed=center[0]
            target=None; quality={}; gate={}; best=None; search=None
            state.level='invalid'; state.reason='no_current_measurement'; state.current_time=None
            has_roi=mask is not None and target_raw is not None and len(target_raw.points)>=CFG['min_target_points']
            full=False; degraded=False
            # Pending recovery is cheaply checked on the next fresh frame.
            pending=state.pending
            if pending is not None and (state.pending_time is None or stamp-state.pending_time>CFG['recovery_confirm_window_s']):
                state.clear_pending(); pending=None
            seed=pending if pending is not None else state.tracking_T
            if has_roi and seed is not None:
                target,gate=filter_target_surfaces(target_raw,model,seed,require_both=False,context=context)
                if target is not None:
                    fit=run_icp(model,target,seed,max_iteration=12)
                    if len(fit.correspondence_set) and pose_is_sane(fit.transformation):
                        proposal=fit.transformation
                        quality=evaluate_registration(model,target_raw,proposal,context=context)
                        step=pose_delta(seed,proposal)
                        within_step=step[0]<=CFG['tracking_max_translation_m'] and step[1]<=CFG['tracking_max_rotation_deg']
                        if quality['valid'] and within_step:
                            full=state.observe(proposal,quality,stamp,time.time(),recovery=pending is not None)
                            if not full: best={'T':proposal,'quality':quality,'reason':state.reason}
                        elif within_step:
                            partial=evaluate_registration(model,target_raw,proposal,context=context,require_both=False)
                            degraded=state.degrade(proposal,partial,stamp,time.time())
                            if degraded: quality=partial
                            best={'T':proposal,'quality':quality,'reason':state.reason}
            if not full:
                fail_count+=1
                needs_search=has_roi and not degraded and (state.trusted_T is None or fail_count>=CFG['failures_before_recovery'])
                if needs_search and state.pending is None:
                    sources=[('verified_pnp' if QUAD_INFO.get('verified') else 'unverified_quad_candidate',pnp_seed),
                        ('frame_plane',center_seed),('history_report',initial_T),('last_complete_pose',state.trusted_T)]
                    search=multistart_icp(model,target_raw,state.trusted_T,initial_T,K,center_T=center_seed,context=context,initial_poses=sources,
                        search_offset=recovery_cycle*max(1,CFG['recovery_max_candidates']-len(sources)),
                        poll_gui=None if args.no_display else poll_gui)
                    recovery_cycle+=1
                    if search and not search.get('failed'):
                        proposal=search['T']; quality=search['quality']; target=search['target']; gate=search['gate']
                        full=state.observe(proposal,quality,stamp,time.time(),recovery=True)
                        best={'T':proposal,'quality':quality,'reason':state.reason}
                    else:
                        state.clear_pending(); state.level='candidate' if search and search.get('best_candidate_T') is not None else 'invalid'
                        state.reason=(search or {}).get('reason','recovery_failed')
                        if search and search.get('best_candidate_T') is not None:
                            best={'T':search['best_candidate_T'],'quality':search.get('best_candidate_quality'),'reason':state.reason}
                elif not has_roi:
                    state.clear_pending(); state.reason='no_contour_or_insufficient_raw_depth'
                elif state.pending is None and not degraded:
                    state.reason=quality.get('reason','tracking_failed')
            if full: fail_count=0; recovery_cycle=0
            # Finish a fresh confirmation before optional diagnostic work.
            confirmation_pending=state.pending is not None
            current_confirmed=(state.level=='full_geometry' and state.strong_time is not None
                and 0<=time.time()-state.strong_time<=CFG['max_measurement_age_s'])
            display_candidate=best
            display_T=state.trusted_T if current_confirmed else None
            display_source='current_confirmed' if current_confirmed else 'no_valid_pose'
            if display_T is None and args.show_candidates and best is not None:
                display_T=best['T']; display_source='UNCONFIRMED_candidate'
            # Candidate diagnostics are distinct from the confirmed target axes.
            registration_T=state.trusted_T if current_confirmed else (best or {}).get('T')
            registration_source='current_confirmed' if current_confirmed else ('UNCONFIRMED_candidate' if registration_T is not None else 'no_candidate')
            display_quality={}; display_gate={}; target=None
            if registration_T is not None and target_raw is not None and not confirmation_pending:
                target,display_gate=filter_target_surfaces(target_raw,model,registration_T,require_both=False,context=context)
                display_quality=(state.trusted_quality if current_confirmed else (best or {}).get('quality')) or {}
            status=state.level+': '+state.reason
            diag.update({'frame':frame_id,'acquisition_time':stamp,'status':status,'quality':quality,'gate':gate,
                'raw_cloud_points':0 if target_raw is None else len(target_raw.points),
                'filtered_cloud_points':0 if target is None else len(target.points),
                'recovery':recovery_diagnostic(search),'best_candidate':display_candidate,'quad':dict(QUAD_INFO),
                'display_pose_source':display_source,'display_T':display_T,
                'display_quality':display_quality,'display_gate':display_gate,
                'filtered_cloud_pose_source':registration_source,'registration_T':registration_T,'build':'v3.3-confirmation-priority',
                'diagnostics_deferred_for_confirmation':confirmation_pending,
                'pnp_seed_available':pnp_seed is not None,'plane_seed_available':center_seed is not None})
            # PLY files are diagnostic; pending confirmation takes priority.
            if not confirmation_pending:
                # Save diagnostic clouds independently, including on failed ICP frames.
                for name,cloud in [('latest_target_raw_camera.ply',target_raw),('latest_target_camera.ply',target)]:
                    path=args.save_dir/name; tmp=path.with_name(path.stem+'_tmp.ply')
                    if cloud is not None and len(cloud.points):
                        if not o3d.io.write_point_cloud(str(tmp),cloud): raise IOError(str(tmp))
                        tmp.replace(path)
                    else:
                        path.unlink(missing_ok=True); tmp.unlink(missing_ok=True)
            now=time.time()
            write_state_output(args.pose_output,state,status,quality,gate,stamp,now,display_candidate)
            with (args.save_dir/'icp_frame_diagnostics.jsonl').open('a',encoding='utf-8') as log:
                log.write(json.dumps(json_safe(diag),allow_nan=False)+'\n')
            capture_overlay=raw_color.copy()
            if display_T is not None: draw_cad_outlines(capture_overlay,display_T,K,display_source)
            if args.capture_first_full and full and not first_full_captured:
                save_frame_capture(args.save_dir,frame_id,raw_color,depth,
                    {'fx':ci.fx,'fy':ci.fy,'cx':ci.ppx,'cy':ci.ppy,'scale':scale,'width':640,'height':480,'aligned_to_color':True},
                    diag,json.loads(args.pose_output.read_text()),capture_overlay)
                first_full_captured=True
            frame_id+=1
            if not args.no_display:
                view=capture_overlay.copy()
                if display_T is not None: draw_axes(view,display_T,K)
                draw_dynamic_quad(view,quad,conf,center_seed is not None)
                cv2.putText(view,f'PnP candidate={pnp_seed is not None} | plane candidate={center_seed is not None}',
                    (10,45),cv2.FONT_HERSHEY_SIMPLEX,.43,(0,255,255),1)
                cv2.putText(view,status,(10,25),cv2.FONT_HERSHEY_SIMPLEX,.5,(0,255,255),1)
                cv2.putText(view,f'ICP input: {len(target_raw.points)} points | confirmed={current_confirmed}',(10,65),cv2.FONT_HERSHEY_SIMPLEX,.43,(0,255,255),1)
                cv2.imshow('D435i Socket Pose - V3.2 diagnostics',view)
                if not confirmation_pending:
                    # If no pose exists, show raw only; do not project identity CAD at Z=0.
                    shown_model=model if registration_T is not None else None
                    reg=draw_registration_view(target_raw,target,shown_model,registration_T if registration_T is not None else np.eye(4),K,raw_color.shape,
                        float(display_quality.get('fitness',0.)),display_quality.get('rmse_m'),registration_source+': '+status,
                        {**diag,'gate':display_gate})
                    cv2.imshow('Point Cloud Registration',reg)
                    cv2.imshow('Contour Image Processing',draw_contour_view(raw_color,depth,diag,scale,mask))
                else:
                    notice=np.zeros_like(raw_color)
                    cv2.putText(notice,f'CONFIRMING: ICP input {len(target_raw.points)} points',(12,35),cv2.FONT_HERSHEY_SIMPLEX,.6,(0,255,255),1)
                    cv2.putText(notice,'Diagnostic rendering deferred for one confirmation frame',(12,65),cv2.FONT_HERSHEY_SIMPLEX,.45,(255,255,255),1)
                    cv2.imshow('Point Cloud Registration',notice)
                poll_gui()
                if capture_requested:
                    capture_requested=False
                    save_frame_capture(args.save_dir,frame_id-1,raw_color,depth,
                        {'fx':ci.fx,'fy':ci.fy,'cx':ci.ppx,'cy':ci.ppy,'scale':scale,'width':640,'height':480,'aligned_to_color':True},
                        diag,json.loads(args.pose_output.read_text()),capture_overlay)
            if args.max_frames and frame_id>=args.max_frames: break
    except (PreviewClosed,KeyboardInterrupt):
        pass
    finally:
        pipe.stop(); cv2.destroyAllWindows()
if __name__=='__main__': main()
