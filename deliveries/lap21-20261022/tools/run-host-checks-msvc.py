"""Run in a Visual Studio x64 developer shell. No hardware access."""
from pathlib import Path
import os,subprocess,sys
base=Path(__file__).resolve().parents[1]
v=base/'project/verification';src=base/'project/demo/user/src';inc=base/'project/demo/user/inc'
os.environ['CC']='cl'
cmds=[['cl','/nologo','/TC','/W4','/WX','/I'+str(inc),str(src/'steering_control.c'),str(v/'test-steering.c'),'/Fe'+str(v/'test-steering.exe')],
      [str(v/'test-steering.exe')],
      [sys.executable,str(v/'check-auto-integration.py')],
      [sys.executable,str(v/'check-hood-replay.py')]]
with (v/'host-checks-LAP21.log').open('w',encoding='utf-8') as log:
    for command in cmds:
        result=subprocess.run(command,cwd=v,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,encoding='utf-8',errors='replace')
        log.write('COMMAND '+repr(command)+'\n'+result.stdout);log.flush()
        print(result.stdout,end='')
        if result.returncode:raise SystemExit(result.returncode)
    log.write('PASS: all LAP21 host checks\n')
print('PASS: all LAP21 host checks')
