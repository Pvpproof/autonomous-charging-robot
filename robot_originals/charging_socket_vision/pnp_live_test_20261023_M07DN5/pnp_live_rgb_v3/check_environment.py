"""Import/version checks only; does not start a camera."""
import sys
import numpy as np
import cv2
import pyrealsense2 as rs

print('Python:',sys.executable,sys.version.split()[0])
print('numpy:',np.__version__)
print('OpenCV:',cv2.__version__)
print('RealSense SDK imported:',getattr(rs,'__version__','version not exposed'))
for name in ('solvePnPGeneric','solvePnPRefineLM','SOLVEPNP_IPPE','calcOpticalFlowPyrLK','imshow'):
    if not hasattr(cv2,name):raise RuntimeError('OpenCV missing '+name)
for line in cv2.getBuildInformation().splitlines():
    if 'GUI:' in line:
        print(line.strip())
        if 'NONE' in line.upper():raise RuntimeError('This OpenCV build has no GUI; use the existing ICP GUI environment')
print('Imports and required symbols OK. No camera started; display/USB still require live testing.')
