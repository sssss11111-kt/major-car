"""Compile and execute the same portable C module used by RT1064 firmware."""
from pathlib import Path
import subprocess
import sys
root = Path(__file__).resolve().parents[1]
zig = root / '.tools/ziglang/zig.exe'
out = root / 'tests/out'
out.mkdir(exist_ok=True)
exe = out / 'test_core.exe'
subprocess.run([str(zig), 'cc', '-std=c99', '-Wall', '-Wextra', '-Werror',
                '-O1', '-UNDEBUG', '-I', str(root/'demo/user/inc'),
                str(root/'tests/test_core.c'), str(root/'demo/user/src/car_core.c'),
                '-o', str(exe)], check=True)
subprocess.run([str(exe)], check=True)
app_exe = out / 'test_app.exe'
subprocess.run([str(zig), 'cc', '-std=c99', '-Wall', '-Wextra', '-Werror',
               '-O1', '-UNDEBUG', '-I', str(root/'tests/sdk'),
               '-I', str(root/'demo/user/inc'), str(root/'tests/test_app.c'),
               str(root/'demo/user/src/car_core.c'), str(root/'demo/user/src/car_app.c'),
               '-o', str(app_exe)], check=True)
subprocess.run([str(app_exe)], check=True)
sys.exit(subprocess.run([sys.executable,str(root/'tests/test_capture.py')]).returncode)
