"""Experimental outer-ring identity selection using nominal CAD radii."""
import numpy as np
import socket_pnp as p


def refine(image,uv,result,camera,cfg):
    circles=p.detect_circles(image,preserve_radii=True)
    T=np.asarray(result['best']['T_camera_from_socket']);uv=np.asarray(uv)
    theta=np.linspace(0,2*np.pi,72,endpoint=False)
    selected=[];details=[]
    for i,center in enumerate(np.asarray(cfg['feature_points_socket_m'])):
        radius=.0045 if i<2 else .0068
        ring=center+np.column_stack([radius*np.cos(theta),radius*np.sin(theta),np.zeros(72)])
        projected=p.project(ring,T,camera)
        r=float(np.mean(np.linalg.norm(projected-projected.mean(axis=0),axis=1)))
        # Local contrast normalization can recover dark outer rings missed globally.
        extent=int(np.ceil(r*1.65))
        x0=max(0,int(uv[i,0])-extent);y0=max(0,int(uv[i,1])-extent)
        x1=min(image.shape[1],int(uv[i,0])+extent+1);y1=min(image.shape[0],int(uv[i,1])+extent+1)
        local=p.detect_circles(image,roi=(x0,y0,x1-x0,y1-y0),preserve_radii=True)
        candidates=np.vstack([circles,local])
        displacement=np.linalg.norm(candidates[:,:2]-uv[i],axis=1)
        ratio=abs(np.log(candidates[:,2]/r))
        valid=(displacement<8)&(ratio<.22)&(candidates[:,3]<.105)
        ids=np.flatnonzero(valid)
        if not len(ids):return {'adopted':False,'reason':'no_outer_ring_for_'+p.NAMES[i]}
        score=4*ratio[ids]+.06*displacement[ids]+candidates[ids,3]
        j=ids[np.argmin(score)];selected.append(candidates[j,:2])
        details.append({'feature':p.NAMES[i],'expected_radius_px':r,'selected_radius_px':float(candidates[j,2]),'shift_px':float(displacement[j])})
    q=p.solve_pose(selected,camera,cfg)
    adopted=bool(q.get('geometry_valid') and q['best']['rmse_px']<result['best']['rmse_px'])
    return {'adopted':adopted,'reason':'research_candidate' if adopted else 'no_verified_improvement',
            'result':q,'selected_points_px':np.asarray(selected).tolist(),'details':details,
            'pose_valid':False,'planning_ready':False}
