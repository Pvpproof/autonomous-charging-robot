"""Compare the actual ROS OccupancyGrid with a saved trinary map."""
import hashlib
import json
import math
from pathlib import Path
import numpy as np
from PIL import Image
import yaml


def digest(width, height, resolution, origin, values):
    canonical = [width, height, round(float(resolution), 6),
                 [round(float(x), 6) for x in origin], list(map(int, values))]
    return hashlib.sha256(json.dumps(canonical, separators=(',', ':')).encode()).hexdigest()


def grid_digest(msg):
    q = msg.info.origin.orientation
    yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
    if msg.header.frame_id != 'map' or len(msg.data) != msg.info.width*msg.info.height:
        raise ValueError('地图消息坐标或尺寸不匹配')
    return digest(msg.info.width, msg.info.height, msg.info.resolution,
                  (msg.info.origin.position.x, msg.info.origin.position.y, yaw), msg.data)


def saved_grid_digest(path):
    path = Path(path)
    meta = yaml.safe_load(path.read_bytes())
    if meta.get('mode', 'trinary') != 'trinary':
        raise ValueError('当前返程仅验收 trinary 地图')
    image = Path(meta['image'])
    if not image.is_absolute(): image = path.parent/image
    with Image.open(image) as im:
        if 'A' in im.getbands(): raise ValueError('当前地图验收不支持透明图像')
        pixels = np.array(im.convert('RGB'), dtype=float).mean(axis=2)/255.
    occ = pixels if meta.get('negate', 0) else 1.-pixels
    data = np.where(occ > meta['occupied_thresh'], 100,
                    np.where(occ < meta['free_thresh'], 0, -1))
    h, w = data.shape
    return digest(w, h, meta['resolution'], meta['origin'], np.flipud(data).flat)
