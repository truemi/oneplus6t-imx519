"""Temporary camera-only supervisor for calibration when openpilot manager is stopped."""
import os
import fcntl
from pathlib import Path
import signal
import subprocess
import time

def parked():
  return all(Path('/data/params/d',k).read_bytes()==b'0' for k in ('IsOnroad','IsEngaged'))
def application_running():
  rows=subprocess.check_output(['ps','-eo','args'],text=True).splitlines()
  return any(any(x in ('./manager.py','/data/openpilot/system/manager/manager.py') for x in row.split()) for row in rows)

assert parked() and not application_running(),'Use the application camera when manager is running'
lock=open('/data/imx519_native_camera.lock','w')
fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
env=os.environ.copy();env.pop('IMX519_NATIVE_ACTIVE',None);env.pop('LD_PRELOAD',None)
env.pop('IMX519_RECTIFY',None)  # Calibration always observes the original lens image.
env['IMX519_DIRECT_BENCH']='1';env['IMX519_OUTPUT_720']='1'
env['LD_LIBRARY_PATH']='/data/imx519_native_runtime/lib:/usr/local/lib:/system/vendor/lib64:/usr/lib/aarch64-linux-gnu'
proc=None;stop=False
def signal_stop(sig,frame):
  global stop
  stop=True
signal.signal(signal.SIGTERM,signal_stop);signal.signal(signal.SIGINT,signal_stop)
started=time.monotonic();last_check=0;app=False
last=time.clock_gettime(time.CLOCK_BOOTTIME)
try:
  proc=subprocess.Popen(['/data/openpilot/system/camerad/imx519_port/camerad_imx519_probe'],cwd='/data/openpilot',env=env)
  while proc.poll() is None and not stop and parked() and Path('/data/imx519_native_enabled').read_bytes()==b'1':
    now=time.monotonic()
    if now-last_check>1:
      app=application_running();last_check=now
    if app:break
    try:last=int(Path('/dev/shm/imx519_calibration_viewer').read_text())/1e9
    except (OSError,ValueError):pass
    if now-started>60 and time.clock_gettime(time.CLOCK_BOOTTIME)-last>60:break
    time.sleep(.2)
finally:
  if proc and proc.poll() is None:
    proc.terminate()
    try:proc.wait(timeout=4)
    except subprocess.TimeoutExpired:proc.kill();proc.wait(timeout=3)
