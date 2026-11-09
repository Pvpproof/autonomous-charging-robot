"""Natural-feature socket PnP. No camera/robot side effects at import time."""
from pathlib import Path
import json
import cv2
import numpy as np

NAMES=('CP','CC','N','PE','L1','L3','L2')


def load_config(path):
    c=json.loads(Path(path).read_text(encoding='utf-8-sig'))
    p=np.asarray(c['feature_points_socket_m'],float)
    if p.shape!=(7,3) or not np.isfinite(p).all() or not np.allclose(p[:,2],p[0,2]):
        raise ValueError('Seven finite coplanar feature points in CP,CC,N,PE,L1,L3,L2 order required')
    if not .025<np.linalg.norm(p[4]-p[2])<.040: raise ValueError('Check feature units/layout')
    if not 0<c['working_z_m'][0]<c['working_z_m'][1]: raise ValueError('Invalid working range')
    return c


def camera_parameters(camera):
    if (camera['width'],camera['height'])!=(640,480): raise ValueError('Native 640x480 only')
    K=np.array([[camera['fx'],0,camera['cx']],[0,camera['fy'],camera['cy']],[0,0,1]],float)
    D=np.asarray(camera.get('coeffs',[0]*5),float)
    model=camera.get('distortion_model','assumed_none').lower().replace('distortion.','')
    if model not in ('none','assumed_none','brown_conrady','inverse_brown_conrady'):
        raise ValueError('Unsupported distortion model: '+model+'; provide a verified OpenCV calibration')
    if not np.isfinite(K).all() or K[0,0]<=0 or K[1,1]<=0 or not np.isfinite(D).all():
        raise ValueError('Invalid camera calibration')
    return K,D,model


def undistort_pixels(uv,camera):
    K,D,model=camera_parameters(camera); uv=np.asarray(uv,float).reshape(-1,2)
    if model in ('none','assumed_none'): return uv.copy()
    if model=='brown_conrady': return cv2.undistortPoints(uv[:,None,:],K,D,P=K).reshape(-1,2)
    # Follow librealsense's INVERSE_BROWN_CONRADY iterative ray conversion,
    # not OpenCV Brown with a sign change. See upstream rs.cpp v2.55.1.
    original=(uv-K[:2,2])/np.diag(K)[:2];xy=original.copy()
    for _ in range(10):
        r2=np.sum(xy**2,axis=1)
        radial=1+D[0]*r2+D[1]*r2**2+D[4]*r2**3
        scaled=xy*radial[:,None];x,y=scaled.T
        tangent=np.column_stack((2*D[2]*x*y+D[3]*(r2+2*x*x),
                                 2*D[3]*x*y+D[2]*(r2+2*y*y)))
        xy=(original-tangent)/radial[:,None]
    return xy*np.diag(K)[:2]+K[:2,2]


def project(points,T,camera):
    K,D,model=camera_parameters(camera); camera_pts=np.asarray(points)@T[:3,:3].T+T[:3,3]
    if np.any(camera_pts[:,2]<=0): raise ValueError('Projection behind camera')
    if model=='inverse_brown_conrady':
        ideal=camera_pts[:,:2]/camera_pts[:,2:3]
        r2=np.sum(ideal**2,axis=1);radial=1+D[0]*r2+D[1]*r2**2+D[4]*r2**3
        x,y=(ideal*radial[:,None]).T
        warped=np.column_stack((x+2*D[2]*x*y+D[3]*(r2+2*x*x),
                                y+2*D[3]*x*y+D[2]*(r2+2*y*y)))
        return warped*np.diag(K)[:2]+K[:2,2]
    return cv2.projectPoints(camera_pts,np.zeros(3),np.zeros(3),K,
                            np.zeros(5) if model in ('none','assumed_none') else D)[0].reshape(-1,2)


def pose_delta(a,b):
    return float(np.linalg.norm(a[:3,3]-b[:3,3])),float(np.degrees(np.linalg.norm(cv2.Rodrigues(a[:3,:3].T@b[:3,:3])[0])))


def solve_pose(uv,camera,cfg):
    uv=np.asarray(uv,float)
    result={'geometry_valid':False,'reason':'invalid_features','candidates':[]}
    if uv.shape!=(7,2) or not np.isfinite(uv).all(): return result
    if np.min(np.linalg.norm(uv[:,None]-uv[None,:]+np.eye(7)[:,:,None]*1e6,axis=2))<cfg['min_feature_separation_px']: return result
    if np.any(uv[:,0]<0) or np.any(uv[:,0]>=640) or np.any(uv[:,1]<0) or np.any(uv[:,1]>=480): return result
    obj=np.asarray(cfg['feature_points_socket_m'],float); plane=obj.copy();z0=obj[0,2];plane[:,2]=0.
    K,_,_=camera_parameters(camera); ideal=undistort_pixels(uv,camera)
    answer=cv2.solvePnPGeneric(plane,ideal,K,np.zeros(5),flags=cv2.SOLVEPNP_IPPE)
    if not answer[0]:result['reason']='pnp_failed';return result
    for rv,tv in zip(answer[1],answer[2]):
        if not np.isfinite(rv).all() or not np.isfinite(tv).all():continue
        rv,tv=cv2.solvePnPRefineLM(plane,ideal,K,np.zeros(5),rv.copy(),tv.copy())
        R=cv2.Rodrigues(rv)[0];T=np.eye(4);T[:3,:3]=R;T[:3,3]=tv.ravel()-R[:,2]*z0
        if np.any((obj@R.T+T[:3,3])[:,2]<=0):continue
        projected=project(obj,T,camera);errors=np.linalg.norm(projected-uv,axis=1)
        axes=project(np.array([[0,0,0],[.012,0,0],[0,.012,0]]),T,camera)
        dx=axes[1]-axes[0];dy=axes[2]-axes[0]
        xcos=float(dx[0]/max(np.linalg.norm(dx),1e-9));ycos=float(-dy[1]/max(np.linalg.norm(dy),1e-9))
        front=float(np.dot(R[:,2],-T[:3,3]/np.linalg.norm(T[:3,3])))
        reason='ok'
        if front<cfg['minimum_facing_cosine']:reason='back_facing_or_grazing'
        elif min(xcos,ycos)<cfg['minimum_upright_cosine']:reason='upside_down_or_wrong_feature_identity'
        elif not cfg['initialization_z_m'][0]<=T[2,3]<=cfg['initialization_z_m'][1]:reason='outside_initialization_range'
        elif np.sqrt(np.mean(errors**2))>cfg['max_reprojection_rmse_px'] or errors.max()>cfg['max_reprojection_error_px']:reason='reprojection_error'
        item={'T_camera_from_socket':T.tolist(),'rmse_px':float(np.sqrt(np.mean(errors**2))),
              'max_error_px':float(errors.max()),'errors_px':errors.tolist(),
              'x_right_cosine':xcos,'y_up_cosine':ycos,'facing_cosine':front,'reason':reason}
        if not any(pose_delta(np.asarray(q['T_camera_from_socket']),T)[0]<1e-5 and
                   pose_delta(np.asarray(q['T_camera_from_socket']),T)[1]<.05 for q in result['candidates']):
            result['candidates'].append(item)
    good=sorted([x for x in result['candidates'] if x['reason']=='ok'],key=lambda x:x['rmse_px'])
    if not good:
        result['reason']=result['candidates'][0]['reason'] if result['candidates'] else 'no_finite_pose';return result
    best=good[0];result['best']=best
    if len(good)>1:
        dt,dr=pose_delta(np.asarray(best['T_camera_from_socket']),np.asarray(good[1]['T_camera_from_socket']))
        if (dr>cfg['ambiguity_rotation_deg'] or dt>cfg['ambiguity_translation_m']) and good[1]['rmse_px']-best['rmse_px']<cfg['ambiguity_rmse_gap_px']:
            result['reason']='ambiguous_planar_pose';return result
    result['geometry_valid']=True;result['reason']='ok'
    result['in_working_range']=bool(cfg['working_z_m'][0]<=best['T_camera_from_socket'][2][3]<=cfg['working_z_m'][1])
    return result


def detect_circles(image,roi=None,preserve_radii=False):
    h,w=image.shape[:2]
    x0,y0,rw,rh=(0,0,w,h) if roi is None else [int(x) for x in roi]
    if rw<=0 or rh<=0 or x0<0 or y0<0 or x0+rw>w or y0+rh>h:raise ValueError('Invalid ROI')
    gray=cv2.cvtColor(image[y0:y0+rh,x0:x0+rw],cv2.COLOR_BGR2GRAY)
    gray=cv2.createCLAHE(2.,(8,8)).apply(gray)
    contours=[]
    # Multiple contrast levels recover dark plastic ring boundaries.
    for low,high in ((12,35),(25,65),(45,110)):
        edges=cv2.Canny(gray,low,high)
        found,_=cv2.findContours(edges,cv2.RETR_LIST,cv2.CHAIN_APPROX_NONE)
        contours.extend(found)
    for level in range(15,151,10):
        _,binary=cv2.threshold(gray,level,255,cv2.THRESH_BINARY)
        found,_=cv2.findContours(binary,cv2.RETR_LIST,cv2.CHAIN_APPROX_NONE)
        contours.extend(found)
    candidates=[]
    for contour in contours:
        if len(contour)<18:continue
        (x,y),(a,b),angle=cv2.fitEllipse(contour)
        if not 4<=min(a,b)/2<=35 or max(a,b)/min(a,b)>1.7:continue
        ellipse_area=np.pi*a*b/4
        if abs(cv2.contourArea(contour))/max(ellipse_area,1)<.65:continue
        points=contour[:,0,:].astype(float);theta=np.deg2rad(angle)
        rotation=np.array([[np.cos(theta),np.sin(theta)],[-np.sin(theta),np.cos(theta)]])
        local=(points-[x,y])@rotation.T
        radial=np.sqrt((local[:,0]/(a/2))**2+(local[:,1]/(b/2))**2)
        error=float(np.mean(abs(radial-1)))
        if error>.10:continue
        candidates.append([x+x0,y+y0,(a+b)/4,error])
    # Dark bores can have broken contours; Hough provides additional proposals.
    hough=cv2.HoughCircles(cv2.GaussianBlur(gray,(5,5),1.0),cv2.HOUGH_GRADIENT,
                           1.0,20,param1=75,param2=18,minRadius=7,maxRadius=34)
    if hough is not None:
        for x,y,r in hough[0]:candidates.append([float(x+x0),float(y+y0),float(r),.11])
    candidates.sort(key=lambda q:q[3]);unique=[]
    for q in candidates:
        if all(np.linalg.norm(np.asarray(q[:2])-p[:2])>4 or (preserve_radii and abs(q[2]-p[2])/max(q[2],p[2])>.15) for p in unique):unique.append(np.array(q))
    return np.asarray(unique[:500 if preserve_radii else 100]).reshape(-1,4)


def match_layout(circles,camera,cfg):
    """Bounded upright similarity proposals; PnP does the final verification."""
    if len(circles)<7:return []
    centers=circles[:,:2]; obj=np.asarray(cfg['feature_points_socket_m'])[:,:2]
    fx=float(camera['fx']);lo,hi=cfg['initialization_z_m'];patterns=[]
    for i,left in enumerate(centers):
        for j,right in enumerate(centers):
            delta=right-left;length=np.linalg.norm(delta)
            if delta[0]<=0 or not fx*.032/(hi+.008)<=length<=fx*.032/lo:continue
            direction=delta/length
            if direction[0]<.85:continue
            up=np.array([direction[1],-direction[0]]);center=(left+right)/2
            predicted=center+length/.032*(obj[:,0,None]*direction+obj[:,1,None]*up)
            distance=np.linalg.norm(predicted[:,None,:]-centers[None,:,:],axis=2)
            ids=distance.argmin(1);err=distance[np.arange(7),ids]
            if len(set(ids))<7 or err.max()>cfg['layout_match_radius_px']:continue
            patterns.append((float(np.mean(err**2)),tuple(int(n) for n in ids)))
    patterns.sort();seen=set();out=[]
    for score,ids in patterns:
        if ids in seen:continue
        seen.add(ids);out.append(centers[list(ids)].copy())
        if len(out)>=cfg['max_layout_candidates']:break
    return out


def automatic_points(image,camera,cfg,roi=None):
    circles=detect_circles(image,roi);found=[];attempts=[]
    layouts=match_layout(circles,camera,cfg)
    for uv in layouts:
        pose=solve_pose(uv,camera,cfg)
        attempts.append({'reason':pose['reason'], 'points_px':uv.tolist(),
                         'candidates':pose.get('candidates',[])})
        if pose.get('geometry_valid'):found.append((pose['best']['rmse_px'],uv,pose))
    diagnostic={'circle_count':len(circles),'layout_count':len(layouts),'attempts':attempts}
    found.sort(key=lambda x:x[0])
    if not found:return None,{'geometry_valid':False,'reason':'no_verified_seven_hole_layout',**diagnostic}
    if len(found)>1 and np.max(np.linalg.norm(found[0][1]-found[1][1],axis=1))>5 and found[1][0]-found[0][0]<cfg['ambiguity_rmse_gap_px']:
        return None,{'geometry_valid':False,'reason':'ambiguous_feature_layout',**diagnostic}
    found[0][2].update(diagnostic)
    return found[0][1],found[0][2]


def d_inner_curve():
    # Nominal source-CAD outline, diagnostic only: not fitted or an acceptance gate.
    radius,top,fillet=.02575,.0188,.0103
    cy=top-fillet;cx=np.sqrt((radius-fillet)**2-cy*cy);alpha=np.arctan2(cy,cx)
    a=np.linspace(np.pi/2,alpha,30);b=np.linspace(alpha,-np.pi-alpha,120);c=np.linspace(np.pi-alpha,np.pi/2,30)
    xy=np.vstack([[-cx,top],[cx,top],np.c_[cx+fillet*np.cos(a),cy+fillet*np.sin(a)],
                  np.c_[radius*np.cos(b),radius*np.sin(b)],np.c_[-cx+fillet*np.cos(c),cy+fillet*np.sin(c)],[-cx,top]])
    return np.c_[xy,np.zeros(len(xy))]


class Confirmation:
    def __init__(self):self.reset()
    def reset(self):self.T=None;self.stamp=None;self.count=0
    def update(self,result,stamp,cfg):
        if not result.get('geometry_valid') or not result.get('in_working_range'):
            self.reset();return False
        T=np.asarray(result['best']['T_camera_from_socket'])
        same=False
        if self.T is not None:
            dt,dr=pose_delta(self.T,T)
            same=(0<stamp-self.stamp<=cfg['confirmation_window_s'] and dt<=cfg['confirmation_translation_m'] and dr<=cfg['confirmation_rotation_deg'])
        self.count=self.count+1 if same else 1;self.T=T.copy();self.stamp=stamp
        return self.count>=cfg['confirm_frames']


def draw_result(image,uv,result,camera):
    out=image.copy()
    if uv is not None:
        for name,point in zip(NAMES,uv):
            point=tuple(np.rint(point).astype(int));cv2.circle(out,point,4,(0,255,255),1)
            cv2.putText(out,name,(point[0]+5,point[1]-5),0,.45,(0,255,255),1)
    if result.get('best'):
        T=np.asarray(result['best']['T_camera_from_socket']);axes=project(np.array([[0,0,0],[.02,0,0],[0,.02,0],[0,0,.02]]),T,camera)
        if np.isfinite(axes).all() and np.max(abs(axes))<1e5:
            axes=np.rint(axes).astype(int)
            for end,color in zip(axes[1:],[(0,0,255),(0,255,0),(255,0,0)]):cv2.line(out,tuple(axes[0]),tuple(end),color,2)
        outline=project(d_inner_curve(),T,camera)
        if np.isfinite(outline).all() and np.max(abs(outline))<1e5:
            cv2.polylines(out,[np.rint(outline).astype(np.int32)],True,(255,255,0),1)
        text=f"z={T[2,3]*1000:.1f}mm rmse={result['best']['rmse_px']:.2f}px"
        cv2.putText(out,text,(10,45),0,.55,(0,255,255),1)
    cv2.putText(out,result.get('reason','unknown'),(10,22),0,.5,(0,255,255),1)
    return out
