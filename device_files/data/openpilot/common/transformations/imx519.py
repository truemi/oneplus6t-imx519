"""Measured pinhole profile for already rectified IMX519 frames only."""
import json
from pathlib import Path
import time
import numpy as np
from openpilot.common.transformations.camera import CameraConfig, DeviceCameraConfig

PROFILE='imx519_720p_20261029'
K=np.array([[1181.1035921673238,0.,663.8852284771285],
            [0.,1180.88327553333,307.6199600164883],[0.,0.,1.]])

class MeasuredCameraConfig(CameraConfig):
  @property
  def intrinsics(self):
    return K.copy()

camera=MeasuredCameraConfig(1280,720,float((K[0,0]+K[1,1])/2))
# The single-camera extra model receives the same image and the same K.
# This does not create the field of view of a physical wide camera.
config=DeviceCameraConfig(camera,camera,camera)

def camera_config(width,height):
  if (width,height)!=(1280,720):return None
  try:
    h=json.loads(Path('/dev/shm/imx519_native_health.json').read_text())
    age=time.clock_gettime_ns(time.CLOCK_BOOTTIME)-int(h['boottime_ns'])
    if 0<=age<3_000_000_000 and h.get('geometry')==PROFILE and h.get('width')==width and h.get('height')==height:
      return config
  except (OSError,ValueError,KeyError,TypeError):pass
  return None
