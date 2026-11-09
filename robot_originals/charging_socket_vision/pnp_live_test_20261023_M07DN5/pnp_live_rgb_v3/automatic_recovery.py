"""Bounded local-ring recovery, offline experiment only."""
import numpy as np
import cv2
import socket_pnp as p
from ring_refinement import refine
from ring_baseline import refine as original_ring_refine


def dark_rois(image):
    gray=cv2.cvtColor(image,cv2.COLOR_BGR2GRAY);regions=[]
    for threshold in (45,75,100):
        mask=np.uint8(gray<threshold)*255
        mask=cv2.morphologyEx(mask,cv2.MORPH_CLOSE,np.ones((7,7),np.uint8))
        contours,_=cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            x,y,w,h=cv2.boundingRect(contour)
            if cv2.contourArea(contour)<3000 or not 70<w<600 or not 70<h<470 or not .45<w/h<2.5:continue
            roi=(max(0,x-5),max(0,y-5),min(640,x+w+5)-max(0,x-5),min(480,y+h+5)-max(0,y-5))
            if all(np.linalg.norm(np.asarray(roi)-old)>20 for _,old in regions):regions.append((cv2.contourArea(contour),roi))
    return [r for _,r in sorted(regions,reverse=True)[:3]]


def estimate(image,camera,cfg,roi=None):
    uv,baseline=p.automatic_points(image,camera,cfg,roi)
    if baseline.get('geometry_valid'):
        # Preserve previously validated corrections before trying the new fallback.
        for method in (original_ring_refine,refine):
            q=method(image,uv,baseline,camera,cfg)
            if q['adopted']:
                return np.asarray(q['selected_points_px']),q['result'],{'source':'ring_refinement','proposal':q}
        return uv,baseline,{'source':'baseline_retained'}
    seeds=[]
    for attempt in baseline.get('attempts',[]):
        for candidate in attempt['candidates']:
            # Do not recover candidates that violated facing/orientation/range gates.
            if candidate['reason']=='reprojection_error' and candidate['rmse_px']<4:
                seeds.append((candidate['rmse_px'],attempt['points_px'],candidate))
    seeds.sort(key=lambda x:x[0]);found=[];attempted=[]
    for _,points,candidate in seeds[:4]:
        q=refine(image,points,{'best':candidate},camera,cfg)
        attempted.append(q)
        if q['adopted']:
            found.append(q)
    if not found:
        if roi is None:
            recovered=[]
            for region in dark_rois(image):
                points,result,diagnostic=estimate(image,camera,cfg,region)
                if result.get('geometry_valid'):recovered.append((points,result,diagnostic,region))
            recovered.sort(key=lambda item:item[1]['best']['rmse_px'])
            if recovered:
                first=recovered[0]
                for other in recovered[1:]:
                    dt,dr=p.pose_delta(np.asarray(first[1]['best']['T_camera_from_socket']),np.asarray(other[1]['best']['T_camera_from_socket']))
                    if (dt>cfg['ambiguity_translation_m'] or dr>cfg['ambiguity_rotation_deg']) and other[1]['best']['rmse_px']-first[1]['best']['rmse_px']<cfg['ambiguity_rmse_gap_px']:
                        return None,{'geometry_valid':False,'reason':'ambiguous_region_recovery'},{'source':'region_ambiguous'}
                return first[0],first[1],{'source':'dark_region_recovery','roi':first[3],'inner':first[2]}
        return None,baseline,{'source':'recovery_failed','attempts':attempted}
    found.sort(key=lambda q:q['result']['best']['rmse_px'])
    first=found[0]
    for other in found[1:]:
        dt,dr=p.pose_delta(np.asarray(first['result']['best']['T_camera_from_socket']),np.asarray(other['result']['best']['T_camera_from_socket']))
        if (dt>cfg['ambiguity_translation_m'] or dr>cfg['ambiguity_rotation_deg']) and other['result']['best']['rmse_px']-first['result']['best']['rmse_px']<cfg['ambiguity_rmse_gap_px']:
            return None,{'geometry_valid':False,'reason':'ambiguous_recovery'}, {'source':'recovery_ambiguous','attempts':attempted}
    return np.asarray(first['selected_points_px']),first['result'],{'source':'local_ring_recovery','attempts':attempted}
