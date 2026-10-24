"""Host-only checks. Never opens serial, DAP, or sends motion commands."""
from pathlib import Path
import os,shutil,subprocess,sys
base=Path(__file__).resolve().parents[1]
v=base/'project/verification'
zig=os.environ.get('ZIG') or shutil.which('zig') or r'D:\major-car\firmware\camera_car\.tools\ziglang\zig.exe'
if not Path(zig).is_file():raise SystemExit('Set ZIG to the zig executable path, or add zig to PATH.')
os.environ['ZIG']=zig
inc=base/'project/demo/user/inc';src=base/'project/demo/user/src'
exe=v/('test-steering.exe' if os.name=='nt' else 'test-steering')
commands=[[zig,'cc','-I',str(inc),str(src/'steering_control.c'),str(v/'test-steering.c'),'-o',str(exe)],
          [str(exe)],[sys.executable,str(v/'check-auto-integration.py')],
          [sys.executable,str(v/'check-hood-replay.py')]]
with (v/'host-checks-LAP21.log').open('w',encoding='utf-8') as log:
 for command in commands:
  log.write('COMMAND '+repr(command)+'\n');log.flush()
  r=subprocess.run(command,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,encoding='utf-8',errors='replace')
  print(r.stdout,end='');log.write(r.stdout);log.flush()
  if r.returncode:raise SystemExit(r.returncode)
 print('PASS: all LAP21 host-only checks')
 log.write('PASS: all LAP21 host-only checks\n')
