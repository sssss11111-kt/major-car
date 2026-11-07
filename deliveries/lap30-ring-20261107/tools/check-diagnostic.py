"""Host-only diagnostic and baseline regressions. No hardware access."""
from pathlib import Path
import os,subprocess,sys
b=Path(__file__).resolve().parents[1];v=b/'project/verification';inc=b/'project/demo/user/inc'
os.environ['CC']='cl';os.environ['ZIG']=str(b/'tools/cc-msvc.cmd')
# Only the new request flag needs a mock for the unchanged supervisor tests.
p=v/'check-auto-integration.py';s=p.read_text(encoding='utf-8')
if 'snapshot_requested=0,rd29_requested=0;' not in s:
 s=s.replace('snapshot_requested=0;','snapshot_requested=0,rd29_requested=0;',1)
 p.write_text(s,encoding='utf-8')
commands=[['cl','/nologo','/TC','/std:c17','/W4','/WX','/utf-8','/I'+str(inc),str(v/'test-ring-diag.c'),'/Fe'+str(v/'test-ring-diag.exe')],
 [str(v/'test-ring-diag.exe')],
 [sys.executable,str(b/'tools/run-host-checks.py')]]
with (v/'diagnostic-host-checks.log').open('w',encoding='utf-8') as log:
 for c in commands:
  p=subprocess.run(c,cwd=v,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,encoding='utf-8',errors='replace')
  print(p.stdout,end='');log.write(str(c)+'\n'+p.stdout);log.flush()
  if p.returncode:raise SystemExit(p.returncode)
 log.write('PASS: all diagnostic and original regressions\n')
