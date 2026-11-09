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
        displacement=np.linalg.norm(circles[:,:2]-uv[i],axis=1)
        ratio=abs(np.log(circles[:,2]/r))
        valid=(displacement<8)&(ratio<.22)&(circles[:,3]<.105)
        ids=np.flatnonzero(valid)
        if not len(ids):return {'adopted':False,'reason':'no_outer_ring_for_'+p.NAMES[i]}
        score=4*ratio[ids]+.06*displacement[ids]+circles[ids,3]
        j=ids[np.argmin(score)];selected.append(circles[j,:2])
        details.append({'feature':p.NAMES[i],'expected_radius_px':r,'selected_radius_px':float(circles[j,2]),'shift_px':float(displacement[j])})
    q=p.solve_pose(selected,camera,cfg)
    adopted=bool(q.get('geometry_valid') and q['best']['rmse_px']<result['best']['rmse_px'])
    return {'adopted':adopted,'reason':'research_candidate' if adopted else 'no_verified_improvement',
            'result':q,'selected_points_px':np.asarray(selected).tolist(),'details':details,
            'pose_valid':False,'planning_ready':False}
