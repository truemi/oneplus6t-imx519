"""Restore this native-only snapshot onto the same already-cleaned device setup."""
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
base=Path(__file__).resolve().parent
if '--check' in sys.argv:
  subprocess.run([sys.executable,str(base/'check_imx519_native_only.py'),str(base)],check=True);sys.exit(0)
assert '--install' in sys.argv,'Use --check or --install'
assert os.geteuid()==0
assert all(Path('/data/params/d',k).read_bytes()==b'0' for k in ('IsOnroad','IsEngaged'))
for r in subprocess.check_output(['ps','-eo','args'],text=True).splitlines():
  assert './manager.py' not in r.split() and not r.strip().endswith('/camerad_imx519_probe')
assert not Path('/data/openpilot/system/camerad/android_hal_manager.py').exists(),'Retire the old forwarding stack first'
m=json.loads((base/'installed_manifest.json').read_text())
for name,digest in m.items():
  assert name.startswith('/data/') and '..' not in Path(name).parts
  src=base/'device_files'/name.lstrip('/')
  assert hashlib.sha256(src.read_bytes()).hexdigest()==digest,src
for name in m:
  dst=Path(name);src=base/'device_files'/name.lstrip('/');dst.parent.mkdir(parents=True,exist_ok=True)
  tmp=dst.with_name(dst.name+'.native_only_new');tmp.write_bytes(src.read_bytes())
  tmp.chmod(0o755 if name.endswith(('camerad','camerad_imx519_probe','/ui','.so','.py')) else 0o644)
  os.chown(tmp,1000,1000);tmp.replace(dst)
subprocess.run([sys.executable,str(base/'check_imx519_native_only.py'),str(base)],check=True)
