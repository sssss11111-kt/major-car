"""Translate this delivery's small Zig cc argument subset for installed MSVC.
Baseline conversion/codepage warnings remain visible; this is not Zig validation.
"""
from pathlib import Path
import subprocess,sys
a=sys.argv[1:]
assert a.pop(0)=='cc'
cmd=['cl','/nologo','/TC','/std:c17','/W3','/utf-8'];i=0
while i<len(a):
    v=a[i];i+=1
    if v=='-shared':cmd+=['/LD']
    elif v=='-O2':cmd+=['/O2']
    elif v=='-I':cmd+=['/I'+a[i]];i+=1
    elif v=='-o':cmd+=['/Fe'+a[i]];i+=1
    elif v in ['-Wall','-Wextra','-Werror']:pass
    elif v.startswith('-'):raise ValueError(v)
    else:cmd+=[v]
raise SystemExit(subprocess.call(cmd,cwd=Path(__file__).resolve().parents[1]/'project/verification'))
