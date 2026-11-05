"""Map geometry and annotations; independent of ROS and the desktop UI."""
import hashlib
import json
import math
import os
import shutil
import time
from pathlib import Path
import numpy as np
import yaml
from PIL import Image


def yaw_of(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


def write_json(path, data):
    path = Path(path)
    temp = path.with_suffix(path.suffix + '.tmp')
    temp.write_text(json.dumps(data, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
    os.replace(temp, path)


class MapDocument:
    def __init__(self, rgb, resolution, origin, yaml_path=None, fingerprint=None):
        if not math.isfinite(resolution) or resolution <= 0:
            raise ValueError('地图分辨率必须大于零')
        self.rgb = np.ascontiguousarray(rgb, dtype=np.uint8)
        self.resolution, self.origin = resolution, tuple(origin)
        self.yaml_path, self.fingerprint = yaml_path, fingerprint
        self.height, self.width = self.rgb.shape[:2]

    @classmethod
    def load(cls, path):
        path = Path(path).expanduser().resolve()
        content = path.read_bytes()
        meta = yaml.safe_load(content)
        image_path = Path(meta['image']).expanduser()
        if not image_path.is_absolute(): image_path = path.parent / image_path
        image_bytes = image_path.read_bytes()
        digest = hashlib.sha256(content + b'\0' + image_bytes).hexdigest()
        with Image.open(image_path) as image: rgb = np.array(image.convert('RGB'))
        return cls(rgb, float(meta['resolution']), meta['origin'], str(path), digest)

    @classmethod
    def from_grid(cls, msg):
        values = np.array(msg.data, dtype=np.int16).reshape(msg.info.height, msg.info.width)
        shades = np.where(values < 0, 110, 255-values*255/100).clip(0,255).astype(np.uint8)
        rgb = np.repeat(np.flipud(shades)[:, :, None], 3, axis=2)
        o = msg.info.origin
        return cls(rgb, msg.info.resolution, (o.position.x, o.position.y, yaw_of(o.orientation)))

    def pixel_to_world(self, u, v):
        x, y = u*self.resolution, (self.height-v)*self.resolution
        ox, oy, angle = self.origin
        c, s = math.cos(angle), math.sin(angle)
        return ox+c*x-s*y, oy+s*x+c*y

    def world_to_pixel(self, x, y):
        ox, oy, angle = self.origin
        c, s = math.cos(angle), math.sin(angle)
        dx, dy = x-ox, y-oy
        return (c*dx+s*dy)/self.resolution, self.height-(-s*dx+c*dy)/self.resolution

    def contains(self, x, y):
        u,v=self.world_to_pixel(x,y)
        return 0 <= u < self.width and 0 <= v < self.height


class Stations:
    def __init__(self, doc):
        self.document = doc
        self.path = Path(doc.yaml_path).with_suffix('.stations.'+doc.fingerprint[:16]+'.json')
        self.points, self.dirty = [], False
        if self.path.exists():
            data = json.loads(self.path.read_text(encoding='utf-8'))
            if data['map_sha256'] != doc.fingerprint or data['schema_version'] != 1:
                raise ValueError('点位与地图版本不匹配')
            self.points = data['stations']

    def add(self, name):
        name=name.strip()
        if not name or any(p['name']==name for p in self.points):
            raise ValueError('名称不能为空或重复')
        self.points.append({'name':name, 'socket':None, 'pre_dock':None})
        self.dirty=True
        return len(self.points)-1

    def update(self,index,name,socket,pre_dock):
        name=name.strip()
        if not name or any(i!=index and p['name']==name for i,p in enumerate(self.points)):
            raise ValueError('名称不能为空或重复')
        for pose in (socket,pre_dock):
            if pose is not None and (not all(math.isfinite(v) for v in pose.values()) or
                                    not self.document.contains(pose['x_m'],pose['y_m'])):
                raise ValueError('点位必须位于当前地图范围内，数值必须有限')
        self.points[index]={'name':name,'socket':socket,'pre_dock':pre_dock}
        self.dirty=True

    def delete(self,index):
        self.points.pop(index); self.dirty=True

    def save(self):
        write_json(self.path, {'schema_version':1,'frame_id':'map',
                   'map_yaml':self.document.yaml_path,'map_sha256':self.document.fingerprint,
                   'stations':self.points})
        self.dirty=False


def import_map_copy(source, directory):
    source=Path(source).expanduser().resolve()
    MapDocument.load(source)
    metadata=yaml.safe_load(source.read_text())
    image=Path(metadata['image']).expanduser()
    if not image.is_absolute():image=source.parent/image
    directory=Path(directory)
    target=directory/(source.stem+'.yaml')
    count=1
    while target.exists() or target.with_suffix(image.suffix).exists():
        target=directory/(source.stem+'_import_'+str(count)+'.yaml');count+=1
    copied_image=target.with_suffix(image.suffix)
    shutil.copy2(image,copied_image)
    metadata['image']=copied_image.name
    target.write_text(yaml.safe_dump(metadata,allow_unicode=True,sort_keys=False),encoding='utf-8')
    return target
