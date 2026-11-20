"""Check the installed native-only deployment, without starting camera/control."""
import hashlib
import json
from pathlib import Path
import subprocess
import sys
base=Path(sys.argv[1]) if len(sys.argv)>1 else Path(__file__).resolve().parent
m=json.loads((base/'installed_manifest.json').read_text())
for name,digest in m.items():
  assert hashlib.sha256(Path(name).read_bytes()).hexdigest()==digest,name
for name in ('/data/hal3_direct','/data/openpilot/system/camerad/android_hal_manager.py',
             '/data/openpilot/system/camerad/camerad_hal_v2','/data/android_root/tmp/hal_camera'):
  assert not Path(name).exists(),name
rows=subprocess.check_output(['ps','-eo','comm,args'],text=True).splitlines()
assert not any(r.split()[0]=='hal3_direct' or r.strip().endswith('system.camerad.android_hal_manager') for r in rows)
assert 'android_hal_manager' not in Path('/data/openpilot/system/manager/process_config.py').read_text()
assert Path('/data/imx519_native_enabled').read_bytes()==b'1'
assert Path('/data/imx519_geometry_enabled').read_bytes()==b'1'
print('Native-only files, startup configuration, flags and HAL absence verified')
