"""Deterministic steering demands; no claim of mechanical/vehicle validation."""
from pathlib import Path
import os,subprocess,shutil
p=Path(__file__).resolve().parent
zig=os.environ.get('ZIG') or shutil.which('zig')
exe=p/'s-bend.exe'
subprocess.run([zig,'cc','-Wall','-Wextra','-Werror','-I',str(p/'vision-stubs'),
 '-I',str(p.parent/'demo/user/inc'),str(p.parent/'demo/user/src/steering_control.c'),
 str(p/'test-s-bend.c'),'-o',str(exe)],check=True)
failed=False
for case in ['linear','preview','limits','reverse','spike']:
 r=subprocess.run([str(exe),case],stdout=subprocess.PIPE,stderr=subprocess.STDOUT,encoding='utf-8',errors='replace')
 print(r.stdout,end='');failed |= bool(r.returncode)
raise SystemExit(1 if failed else 0)
