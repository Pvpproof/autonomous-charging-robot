"""Bounded real-template selector; all source clouds are in socket frame."""
import json
from pathlib import Path
import numpy as np
import open3d as o3d
from scipy.spatial.transform import Rotation

class MultiViewBackend:
    def __init__(self, root, cad):
        self.models={'cad':cad}; self.views={}; self.active='cad'; self.tick=0
        self.last_info={'template':'cad','accepted':False}
        for p in sorted((Path(root)/'template_library').glob('*/metadata.json')):
            try:
                d=json.loads(p.read_text()); T=np.asarray(d['T_camera_from_socket'],float)
                q=o3d.io.read_point_cloud(str(p.parent/'target_socket.ply')).voxel_down_sample(.002)
                if T.shape!=(4,4) or not np.isfinite(T).all() or len(q.points)<80: continue
                self.models[p.parent.name]=q; self.views[p.parent.name]=T[:3,:3]
            except (ValueError,KeyError,OSError): continue
        self.pyramids={name:[q.voxel_down_sample(v) for v in (.006,.003,.002)] for name,q in self.models.items()}
        print('MULTIVIEW loaded:',', '.join(self.models),flush=True)

    @staticmethod
    def direction(T):
        R=T[:3,:3]
        return R[0,0]>.25 and R[1,1]<-.25 and R[2,2]<-.25

    def select(self,target,seed):
        self.tick+=1
        # Evaluate the closest two recorded orientations, CAD, and active model.
        names=sorted(self.views,key=lambda n:Rotation.from_matrix(self.views[n].T@seed[:3,:3]).magnitude())[:2]
        names=list(dict.fromkeys(['cad',self.active]+names))
        targets=[target.voxel_down_sample(v) for v in (.006,.003,.002)]
        results=[]
        for name in names:
            T=seed.copy()
            for source,dest,gate,iters in zip(self.pyramids[name],targets,(.025,.014,.008),(10,12,16)):
                r=o3d.pipelines.registration.registration_icp(source,dest,gate,T,
                    o3d.pipelines.registration.TransformationEstimationPointToPoint(),
                    o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=iters))
                T=r.transformation
            if not np.isfinite(T).all() or not self.direction(T): continue
            delta=float(np.linalg.norm(T[:3,3]-seed[:3,3]))
            angle=float(Rotation.from_matrix(seed[:3,:3].T@T[:3,:3]).magnitude())
            if delta>.015 or angle>np.deg2rad(10): continue
            source=self.pyramids[name][-1]; dest=targets[-1]
            f=o3d.pipelines.registration.evaluate_registration(source,dest,.008,T)
            rev=o3d.pipelines.registration.evaluate_registration(dest,source,.008,np.linalg.inv(T))
            coverage=min(f.fitness,rev.fitness)
            if coverage<.35 or f.inlier_rmse>.006 or f.fitness<.55: continue
            score=coverage-f.inlier_rmse/.02
            results.append((score,name,T,f,coverage))
        if not results:
            self.last_info={'template':self.active,'accepted':False}; return None
        results.sort(key=lambda x:x[0],reverse=True); best=results[0]
        active=next((x for x in results if x[1]==self.active),None)
        if active is not None and best[0]<active[0]+.025: best=active
        score,name,T,f,coverage=best; self.active=name
        self.last_info={'template':name,'accepted':True,'score':float(score),'bidirectional_coverage':float(coverage),'rmse_m':float(f.inlier_rmse)}
        return T,float(f.fitness),float(f.inlier_rmse)
