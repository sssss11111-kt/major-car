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
               str(root/'demo/user/src/car_servo.c'),
               '-o', str(app_exe)], check=True)
subprocess.run([str(app_exe)], check=True)
servo_exe = out / 'test_servo.exe'
subprocess.run([str(zig), 'cc', '-std=c99', '-Wall', '-Wextra', '-Werror',
               '-O1', '-UNDEBUG', '-I', str(root/'tests/sdk'),
               '-I', str(root/'demo/user/inc'), str(root/'tests/test_servo.c'),
               str(root/'demo/user/src/car_servo.c'), '-o', str(servo_exe)], check=True)
subprocess.run([str(servo_exe)], check=True)
for diagnostic, bad_ack, auto_baud in ((0, 0, 1), (1, 0, 1), (1, 1, 1), (0, 0, 0)):
    uart_exe = out / ('test_uart8_%d_%d_%d.exe' % (diagnostic, bad_ack, auto_baud))
    extra = ['-DTEST_BAD_BAUD_ACK=1'] if bad_ack else []
    subprocess.run([str(zig), 'cc', '-std=c99', '-Wall', '-Wextra', '-Werror',
                   '-O1', '-UNDEBUG', '-DCAR_COMM_DIAGNOSTIC='+str(diagnostic),
                   '-DCAR_WIRELESS_AUTO_BAUD='+str(auto_baud), *extra,
                   '-I', str(root/'tests/sdk'), '-I', str(root/'demo/user/inc'),
                   str(root/'tests/test_uart8.c'), str(root/'demo/user/src/car_core.c'),
                   str(root/'demo/user/src/car_app.c'), str(root/'demo/user/src/car_servo.c'),
                   '-o', str(uart_exe)], check=True)
    subprocess.run([str(uart_exe)], check=True)
scene_exe = out / 'test_vision_scenarios.exe'
scene_out = out / 'vision'
scene_out.mkdir(exist_ok=True)
subprocess.run([str(zig), 'cc', '-std=c99', '-Wall', '-Wextra', '-Werror',
               '-O1', '-UNDEBUG', '-I', str(root/'demo/user/inc'),
               str(root/'tests/test_vision_scenarios.c'), str(root/'demo/user/src/car_core.c'),
               '-o', str(scene_exe)], check=True)
subprocess.run([str(scene_exe), str(scene_out)], check=True)
subprocess.run([sys.executable,str(root/'tests/render_vision_report.py')], check=True)
sys.exit(subprocess.run([sys.executable,str(root/'tests/test_capture.py')]).returncode)
