"""Host-only checks. Never opens serial, DAP, or sends motion commands."""
from pathlib import Path
import os,shutil,subprocess,sys,argparse
parser=argparse.ArgumentParser()
parser.add_argument("--current",action="store_true",help="Run current production tests; report historical R11 fixture regression separately")
args=parser.parse_args()
base=Path(__file__).resolve().parents[1]
v=base/'project/verification'
zig=os.environ.get('ZIG') or shutil.which('zig') or r'D:\major-car\firmware\camera_car\.tools\ziglang\zig.exe'
if not Path(zig).is_file():raise SystemExit('Set ZIG to the zig executable path, or add zig to PATH.')
os.environ['ZIG']=zig
inc=base/'project/demo/user/inc';src=base/'project/demo/user/src'
exe=v/('test-steering.exe' if os.name=='nt' else 'test-steering')
commands=[[zig,'cc','-I',str(inc),'-I',str(v/'vision-stubs'),str(src/'steering_control.c'),str(v/'test-steering.c'),'-o',str(exe)],
          [str(exe)],
          [zig,'cc','-O2','-Wall','-Wextra','-Werror','-I',str(inc),'-I',str(v/'vision-stubs'),str(v/'test-path-turn.c'),'-o',str(v/'test-path-turn.exe')],
          [str(v/'test-path-turn.exe')],[sys.executable,str(v/'check-auto-integration.py')],
          [sys.executable,str(v/'check-s-bend.py')],
          [sys.executable,str(v/'check-stripes.py')],
          [sys.executable,str(v/'check-hood-replay.py')],
          [zig,'cc','-Wall','-Wextra','-Werror','-I',str(inc),str(v/'test-roundabout.c'),'-o',str(v/'test-roundabout.exe')],
          [str(v/'test-roundabout.exe')],
          [sys.executable,str(v/'check-roundabout.py')],
          [sys.executable,str(v/'check-ring-recovery.py')],
          [sys.executable,'-X','utf8',str(base.parents[1]/'records/ring-research-20261107/entry-r11/entry-regression.py')]]
commands[0:0]=[[zig,'cc','-Wall','-Wextra','-Werror','-I',str(inc),str(v/'test-track-geometry.c'),'-o',str(v/'test-track-geometry.exe')],[str(v/'test-track-geometry.exe')]]
if args.current:
 commands=commands[:-1]
 print('SCOPE: current production tests; historical R11 regression excluded explicitly (--current)')
with (v/'host-checks-LAP30.log').open('w',encoding='utf-8') as log:
 for command in commands:
  log.write('COMMAND '+repr(command)+'\n');log.flush()
  r=subprocess.run(command,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,encoding='utf-8',errors='replace')
  print(r.stdout,end='');log.write(r.stdout);log.flush()
  if r.returncode:raise SystemExit(r.returncode)
 print('PASS: scoped LAP30 host-only checks')
 log.write('PASS: scoped LAP30 host-only checks\n')
