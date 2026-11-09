"""Latest-frame display only. No camera, ROS, action or pose-output interfaces."""
import ctypes,json,multiprocessing as mp,struct,time,os,signal,sys,threading
from functools import lru_cache
import numpy as np
META_BYTES=8192
HEADER_BYTES=16
IMAGE_OFFSET=HEADER_BYTES+META_BYTES

def read_latest(arena,lock,last_sequence=0):
    if not lock.acquire(False):return None
    try:
        sequence,length=struct.unpack_from('<QI',arena,0)
        if sequence==last_sequence or not sequence or not 0<length<=META_BYTES:return None
        info=json.loads(bytes(arena[HEADER_BYTES:HEADER_BYTES+length]))
        shape=tuple(info['image_shape']);size=int(np.prod(shape))
        if len(shape)!=3 or shape[2]!=3 or size<=0 or size>len(arena)-IMAGE_OFFSET:return None
        frame=np.frombuffer(arena,dtype=np.uint8,count=size,offset=IMAGE_OFFSET).copy().reshape(shape)
        return sequence,frame,info
    finally:lock.release()


@lru_cache(maxsize=1)
def preview_cad_curves():
    """Exact source-CAD nominal outlines; all XYZ in socket coordinates, meters."""

    def d_profile(radius, top, fillet, z):
        cy = top - fillet
        cx = np.sqrt((radius - fillet) ** 2 - cy * cy)
        alpha = np.arctan2(cy, cx)
        right = np.linspace(np.pi / 2, alpha, 30)
        bottom = np.linspace(alpha, -np.pi - alpha, 120)
        left = np.linspace(np.pi - alpha, np.pi / 2, 30)
        xy = np.vstack([[-cx, top], [cx, top], np.c_[cx + fillet * np.cos(right), cy + fillet * np.sin(right)], np.c_[radius * np.cos(bottom), radius * np.sin(bottom)], np.c_[-cx + fillet * np.cos(left), cy + fillet * np.sin(left)], [-cx, top]])
        return np.c_[xy, np.full(len(xy), z)]
    rectangle = []
    for (cx, cy, angle) in ((0.032, 0.032, 0), (-0.032, 0.032, 90), (-0.032, -0.032, 180), (0.032, -0.032, 270)):
        t = np.deg2rad(np.linspace(angle, angle + 90, 25))
        rectangle.extend(np.c_[cx + 0.006 * np.cos(t), cy + 0.006 * np.sin(t), np.full(len(t), -0.015)])
    rectangle = np.vstack([rectangle, rectangle[0]])
    return {'frame_front': rectangle, 'd_outer': d_profile(0.0315, 0.02455, 0.01605, 0.0), 'd_inner': d_profile(0.02575, 0.0188, 0.0103, 0.0)}

def overlay(frame,metadata,now=None):
    import cv2
    result=frame.copy();now=time.time() if now is None else now
    confirmed=metadata.get('confirmed') is True
    age=now-metadata['capture_host_s'] if type(metadata.get('capture_host_s')) in (int,float) else None
    stale=age is None or age<0 or age>1.
    color=(0,190,0) if confirmed and not stale else (0,190,255)
    lines=[str(metadata.get('source','Preview'))+' | '+str(metadata.get('status','waiting')),
           'VISUAL CONFIRMED - DISPLAY ONLY' if confirmed and not stale else ('STALE - DISPLAY ONLY' if stale else 'UNCONFIRMED / TRACKING - DISPLAY ONLY')]
    if age is not None:lines.append('frame age: %.3f s'%age)
    # Display metadata is never written back to the producer's pose state.
    if not stale:
        xyz=metadata.get('xyz_m');text=metadata.get('coordinates_text')
        matrix=metadata.get('T_camera_from_socket') if confirmed else metadata.get('candidate_T')
        if xyz is None and matrix is not None:
            try:xyz=np.asarray(matrix,float)[:3,3].tolist()
            except (TypeError,ValueError,IndexError):xyz=None
        prefix='XYZ' if confirmed else 'DISPLAY ONLY XYZ'
        if isinstance(xyz,list) and len(xyz)==3:lines.append(prefix+': %.2f %.2f %.2f mm'%tuple(float(x)*1000 for x in xyz))
        elif text:lines.append(('' if confirmed else 'DISPLAY ONLY ')+str(text))
        lines.append('frame: '+str(metadata.get('coordinate_frame','unspecified')))
    if not stale:
        T=metadata.get('T_camera_from_socket') if confirmed else metadata.get('candidate_T')
        K=metadata.get('K')
        if T is not None and K is not None:
            try:
                T=np.asarray(T,float);K=np.asarray(K,float)
                if T.shape==(4,4) and K.shape==(3,3) and np.isfinite(T).all() and np.isfinite(K).all():
                    points=T[:3,3,None]+T[:3,:3]@np.array([[0,.035,0,0],[0,0,.035,0],[0,0,0,.035]])
                    if np.all(points[2]>.01):
                        uv=K@points;uv=np.round(uv[:2]/uv[2]).astype(int).T
                        for i,c in zip(range(1,4),[(0,0,255),(0,255,0),(255,0,0)]):cv2.line(result,tuple(uv[0]),tuple(uv[i]),c if confirmed else (0,190,255),2)
                        if metadata.get('source')=='ICP':
                            colors={'frame_front':(255,255,0),'d_outer':(0,255,0),'d_inner':(255,0,255)}
                            for name,curve in preview_cad_curves().items():
                                camera=curve@T[:3,:3].T+T[:3,3]
                                if np.any(camera[:,2]<=.01):continue
                                projected=camera@K.T;pixels=np.clip(np.rint(projected[:,:2]/projected[:,2:3]),-100000,100000).astype(np.int32).reshape(-1,1,2)
                                cv2.polylines(result,[pixels],True,colors[name] if confirmed else (0,190,255),2,cv2.LINE_AA)
            except (ValueError,TypeError):pass
    for i,line in enumerate(lines):
        cv2.putText(result,line[:110],(8,20+i*21),cv2.FONT_HERSHEY_SIMPLEX,.46,(0,0,0),3,cv2.LINE_AA)
        cv2.putText(result,line[:110],(8,20+i*21),cv2.FONT_HERSHEY_SIMPLEX,.46,color,1,cv2.LINE_AA)
    return result

def bind_parent_lifetime(arena):
    expected=struct.unpack_from('<I',arena,12)[0]
    if os.getppid()!=expected:return False
    if sys.platform.startswith('linux'):
        # GUI child only: a killed camera process must not leave an orphan UI.
        ctypes.CDLL(None).prctl(1,signal.SIGTERM,0,0,0)
    return os.getppid()==expected

def preview_worker(arena,lock,stop,source):
    if not bind_parent_lifetime(arena):stop.set();return
    try:os.nice(10)
    except OSError:pass
    import cv2
    cv2.setNumThreads(1)
    window='Charging robot | '+source+' | preview only';sequence=0;latest=None;metadata=None;last_paint=0.
    try:
        cv2.namedWindow(window,cv2.WINDOW_NORMAL)
        while not stop.is_set() and os.getppid()==struct.unpack_from('<I',arena,12)[0]:
            sample=read_latest(arena,lock,sequence)
            if sample is not None:sequence,latest,metadata=sample
            if latest is not None:
                if time.monotonic()-last_paint>=.1:
                    cv2.imshow(window,overlay(latest,metadata));last_paint=time.monotonic()
                if cv2.waitKey(20)&0xff in (27,ord('q')):break
                # GTK3 does not implement WND_PROP_VISIBLE (-1 even for live windows).
                # AUTOSIZE is supported: 0 for this normal window, -1 after close.
                visibility=cv2.getWindowProperty(window,cv2.WND_PROP_AUTOSIZE)
                if visibility<0:
                    print('[PREVIEW] closing: visibility=%r'%visibility,file=sys.stderr,flush=True);break
            else:stop.wait(.02)
    except Exception:
        # Display failure is isolated from camera, recognition and control.
        import traceback
        print('[PREVIEW] display failed',file=sys.stderr,flush=True);traceback.print_exc()
    finally:
        stop.set()
        try:cv2.destroyWindow(window)
        except Exception:pass

class PreviewPublisher:
    def __init__(self,source='Preview',enabled=True,max_fps=10,worker=None,max_shape=(720,1280,3)):
        self.source=source;self.enabled=enabled;self.interval=1./max_fps;self.last_copy=-float('inf');self.last_frame_id=None;self.last_shape=None;self.sequence=0;self.child=None;self.dropped=0;self.closed=False
        if not enabled:return
        context=mp.get_context('spawn');self.arena=context.RawArray(ctypes.c_ubyte,IMAGE_OFFSET+int(np.prod(max_shape)));self.lock=context.Lock();self.stop=context.Event();struct.pack_into('<I',self.arena,12,os.getpid())
        self.child=context.Process(target=worker or preview_worker,args=(self.arena,self.lock,self.stop,source),daemon=True)
        try:self.child.start()
        except Exception:self.enabled=False
    def submit(self,frame,metadata):
        if not self.enabled or self.stop.is_set() or not self.child.is_alive():return False
        if not isinstance(frame,np.ndarray) or frame.dtype!=np.uint8 or frame.ndim!=3 or frame.shape[2]!=3:return False
        now=time.monotonic();frame_id=metadata.get('frame_id');same=frame_id is not None and frame_id==self.last_frame_id and frame.shape==self.last_shape
        if not same and now-self.last_copy<self.interval:return False
        if frame.nbytes>len(self.arena)-IMAGE_OFFSET:return False
        try:
            info=dict(metadata,source=self.source,image_shape=list(frame.shape))
            encoded=json.dumps(info,ensure_ascii=False,allow_nan=False).encode()
        except (TypeError,ValueError):return False
        if len(encoded)>META_BYTES:return False
        if not self.lock.acquire(False):self.dropped+=1;return False
        try:
            if not same:np.copyto(np.frombuffer(self.arena,dtype=np.uint8,count=frame.size,offset=IMAGE_OFFSET).reshape(frame.shape),frame);self.last_copy=now;self.last_frame_id=frame_id;self.last_shape=frame.shape
            self.arena[HEADER_BYTES:HEADER_BYTES+len(encoded)]=encoded
            self.sequence+=1;struct.pack_into('<QI',self.arena,0,self.sequence,len(encoded));return True
        finally:self.lock.release()
    def close(self):
        if self.closed:return
        self.closed=True;self.enabled=False
        if self.child is not None and self.child.pid is not None:
            self.stop.set();self.child.join(timeout=.1)
            if self.child.is_alive():
                # Keep shared semaphores alive while a spawn child finishes
                # bootstrapping; do not block the recognition shutdown path.
                threading.Thread(target=_finish_child,args=(self.child,(self.arena,self.lock,self.stop)),daemon=True).start()

def _finish_child(child,resources):
    from multiprocessing.connection import wait
    wait([child.sentinel])
