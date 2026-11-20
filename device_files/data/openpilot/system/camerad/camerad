#!/usr/bin/python3
# IMX519_NATIVE_ONLY_V2
"""Native camera supervisor. Restart failed capture; never switch backends."""
import fcntl
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

ROOT=Path('/data/openpilot')
NATIVE=ROOT/'system/camerad/imx519_port/camerad_imx519_probe'
HEALTH=Path('/dev/shm/imx519_native_health.json')
PROFILE='imx519_720p_20261029'
lock=open('/data/imx519_native_camera.lock','w')
try:fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
except BlockingIOError:sys.exit('Native camera is already owned by another process')
p=json.loads(Path('/data/imx519_native_profile.json').read_text())
assert p['geometry']==PROFILE and p['intrinsics_calibrated'] and p['rectified_output']
assert p['vision_output']==[1280,720] and p['fixed_dac']==1366 and not p['continuous_af']
assert hashlib.sha256(NATIVE.read_bytes()).hexdigest()==p['native_sha256']
assert Path('/data/imx519_native_enabled').read_bytes()==b'1'
assert Path('/data/imx519_geometry_enabled').read_bytes()==b'1'
env=os.environ.copy()
for key in ('IMX519_DIRECT_BENCH','IMX519_SNAPSHOT','IMX519_FOCUS_DAC','IMX519_DEBUG','DEBUG_FRAMES','LD_PRELOAD'):
  env.pop(key,None)
env.update(IMX519_NATIVE_ACTIVE='1',IMX519_OUTPUT_720='1',IMX519_RECTIFY='1',
           LD_LIBRARY_PATH='/data/imx519_native_runtime/lib:/usr/local/lib:/system/vendor/lib64:/usr/lib/aarch64-linux-gnu')
stopping=False;child=None;failures=0
def stop(sig,frame):
  global stopping
  stopping=True
  if child and child.poll() is None:child.terminate()
signal.signal(signal.SIGTERM,stop);signal.signal(signal.SIGINT,stop)
while not stopping:
  started=time.clock_gettime_ns(time.CLOCK_BOOTTIME);reason='';child=None
  try:
    child=subprocess.Popen([str(NATIVE)],cwd=ROOT,env=env)
    while not stopping and child.poll() is None:
      now=time.clock_gettime_ns(time.CLOCK_BOOTTIME)
      try:
        h=json.loads(HEALTH.read_text());stamp=int(h['boottime_ns'])
        healthy=started<=stamp<=now and now-stamp<4_000_000_000 and h.get('geometry')==PROFILE
      except (OSError,ValueError,KeyError,TypeError):healthy=False
      if now-started>10_000_000_000 and not healthy:
        reason='frame heartbeat timeout';break
      if healthy and now-started>30_000_000_000:failures=0
      time.sleep(.2)
    if not reason:reason=f'capture exited: {child.poll()}'
  except OSError as e:reason=f'capture launch failed: {e}'
  finally:
    if child and child.poll() is None:
      child.terminate()
      try:child.wait(timeout=3)
      except subprocess.TimeoutExpired:child.kill();child.wait(timeout=3)
  if not stopping:
    failures+=1;delay=min(8.,.5*2**min(failures-1,4))
    print(f'[imx519] {reason}; restarting native camera in {delay:g}s',flush=True)
    until=time.monotonic()+delay
    while not stopping and time.monotonic()<until:time.sleep(.1)
