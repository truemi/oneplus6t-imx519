"""Private SSH preview/capture protocol. Reads native VisionIPC; never changes the backend."""
import collections
import argparse
import hashlib
import json
import os
from pathlib import Path
import select
import subprocess
import atexit
import sys
import time
import numpy as np

os.chdir('/data/openpilot');sys.path.insert(0,'.')
from msgq.visionipc import VisionIpcClient,VisionStreamType

assert Path('/data/imx519_native_enabled').read_bytes()==b'1'
p=argparse.ArgumentParser();p.add_argument('--endpoint',choices=['auto','camerad','imx519_probe'],default='auto');args=p.parse_args()
v=None;owned_camera=None
def cleanup():
  if owned_camera and owned_camera.poll() is None:
    owned_camera.terminate()
    try:owned_camera.wait(timeout=5)
    except subprocess.TimeoutExpired:owned_camera.kill()
atexit.register(cleanup)
for endpoint in (['camerad','imx519_probe'] if args.endpoint=='auto' else [args.endpoint]):
  trial=VisionIpcClient(endpoint,VisionStreamType.VISION_STREAM_ROAD,True)
  until=time.monotonic()+1
  while time.monotonic()<until:
    if trial.connect(False):v=trial;break
    time.sleep(.1)
  if v is not None:break
if v is None:
  endpoint='imx519_probe'
  owned_camera=subprocess.Popen(['/usr/local/venv/bin/python3','/data/imx519_calibration_camera.py'],
    stdin=subprocess.DEVNULL,stdout=open('/data/imx519_calibration_camera.log','ab'),stderr=subprocess.STDOUT)
  trial=VisionIpcClient('imx519_probe',VisionStreamType.VISION_STREAM_ROAD,True)
  until=time.monotonic()+10
  while time.monotonic()<until:
    if trial.connect(False):v=trial;break
    if owned_camera.poll() is not None:break
    time.sleep(.1)
assert v is not None,'Camera connection timeout; inspect /data/imx519_calibration_camera.log'
driver=hashlib.sha256(Path('/data/openpilot/system/camerad/imx519_port/camerad_imx519_probe').read_bytes()).hexdigest()
cache=collections.OrderedDict()
def send(header,payload=b''):
  header['bytes']=len(payload)
  sys.stdout.buffer.write((json.dumps(header)+'\n').encode())
  if payload:sys.stdout.buffer.write(payload)
  sys.stdout.buffer.flush()

next_frame=0
last_received=time.monotonic()
while True:
  if endpoint=='camerad':
    try:
      health=json.loads(Path('/dev/shm/imx519_native_health.json').read_text())
    except (OSError,ValueError):health={}
    assert health.get('geometry','raw')=='raw','Already rectified: use private raw calibration camera instead'
  if owned_camera and owned_camera.poll() is not None:
    raise RuntimeError('Calibration camera exited; inspect /data/imx519_calibration_camera.log')
  if time.monotonic()-last_received>12:
    raise RuntimeError('No camera frames for 12 seconds; reopen the assistant')
  assert Path('/data/params/d/IsOnroad').read_bytes()==b'0','Parked preview only'
  assert Path('/data/params/d/IsEngaged').read_bytes()==b'0'
  assert Path('/data/imx519_native_enabled').read_bytes()==b'1'
  if select.select([sys.stdin],[],[],0)[0]:
    line=sys.stdin.readline()
    if not line:break
    cmd=json.loads(line)
    if cmd.get('command')=='quit':break
    if cmd.get('command')=='save':
      fid=int(cmd['frame_id'])
      if fid in cache:
        meta,raw=cache[fid]
        send({'kind':'capture','metadata':meta},raw)
      else:send({'kind':'expired','frame_id':fid})
  b=v.recv(200)
  if b is None or not v.valid:continue
  last_received=time.monotonic()
  if time.monotonic()<next_frame:continue
  heartbeat=Path('/dev/shm/imx519_calibration_viewer')
  heartbeat.with_suffix('.tmp').write_text(str(time.clock_gettime_ns(time.CLOCK_BOOTTIME)))
  heartbeat.with_suffix('.tmp').replace(heartbeat)
  start=time.clock_gettime_ns(time.CLOCK_BOOTTIME)
  age=(start-v.timestamp_eof)/1e9
  if not 0<=age<.015:continue
  w,h,s,u=v.width,v.height,v.stride,v.uv_offset
  fid=v.frame_id;sof=v.timestamp_sof;eof=v.timestamp_eof
  raw=b.data[:u+s*(h//2)].copy()
  elapsed=(time.clock_gettime_ns(time.CLOCK_BOOTTIME)-start)/1e9
  if elapsed>.1 or age+elapsed>.12:continue
  meta={'width':w,'height':h,'stride':s,'uv_offset':u,'sensor':'IMX519','focus_dac':1366,'rectified':False,
        'frame_valid':True,'driver_sha256':driver,'frame_id':fid,'sof_ns':sof,'eof_ns':eof,'copy_ms':elapsed*1000}
  cache[fid]=(meta,raw.tobytes())
  while len(cache)>8:cache.popitem(last=False)
  tw=480;th=h*tw//w
  iy=np.arange(th)*h//th;ix=np.arange(tw)*w//tw
  y=raw[:s*h].reshape(h,s)[iy[:,None],ix[None,:]].astype(np.float32)
  uv=raw[u:u+s*h//2].reshape(h//2,s)
  ui=(ix//2)*2
  cb=uv[(iy//2)[:,None],ui[None,:]].astype(np.float32)-128
  cr=uv[(iy//2)[:,None],(ui+1)[None,:]].astype(np.float32)-128
  c=(y-16)*1.164
  rgb=np.stack([c+1.596*cr,c-.392*cb-.813*cr,c+2.017*cb],axis=2).clip(0,255).astype(np.uint8)
  send({'kind':'preview','frame_id':fid,'width':tw,'height':th,'full_width':w,'full_height':h},rgb.tobytes())
  next_frame=time.monotonic()+.35
