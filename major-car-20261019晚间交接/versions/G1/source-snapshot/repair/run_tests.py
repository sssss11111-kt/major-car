"""Run UART8 regression and existing suites with installed MSVC, not Keil."""
from pathlib import Path
import subprocess
import sys
import os
import tempfile
os.environ['PYTHONDONTWRITEBYTECODE']='1'
BASE=Path(__file__).resolve().parents[1]
root=BASE/'competition/firmware/camera_car'
out=root/'tests/out'
out.mkdir(exist_ok=True)
object_root=Path(tempfile.mkdtemp(prefix='msvc-',dir=out))
evidence=BASE/'repair/evidence'
evidence.mkdir(exist_ok=True)
with (evidence/'host-tests.log').open('w',encoding='utf-8') as log:
    def run(args):
        log.write(subprocess.list2cmdline([str(a) for a in args])+'\n');log.flush()
        result=subprocess.run(args,cwd=out,stdout=subprocess.PIPE,stderr=subprocess.STDOUT)
        try: text=result.stdout.decode('utf-8')
        except UnicodeDecodeError: text=result.stdout.decode('mbcs',errors='replace')
        log.write(text);log.flush()
        if result.returncode: raise SystemExit(result.returncode)
    def build(label,files,includes,defines=()):
        exe=object_root/(label+'.exe')
        obj_dir=object_root/label;obj_dir.mkdir()
        args=['cl','/nologo','/std:c11','/W3','/O1','/UNDEBUG','/D_CRT_SECURE_NO_WARNINGS','/Fe:'+str(exe),'/Fo:'+str(obj_dir)+os.sep]
        args.extend('/D'+d for d in defines)
        for folder in includes: args.extend(['/I',str(folder)])
        args.extend(str(p) for p in files)
        run(args)
        return exe
    inc=root/'demo/user/inc';src=root/'demo/user/src';tests=root/'tests'
    run([build('test_tracking_low',[tests/'test_tracking_low.c',src/'car_core.c'],[inc])])
    run([build('test_low_speed',[tests/'test_low_speed.c',src/'car_core.c'],[inc])])
    run([build('test_sustain',[tests/'test_sustain.c',src/'car_core.c'],[inc])])
    run([build('test_calibration',[tests/'test_calibration.c',src/'car_core.c'],[inc])])
    run([build('test_core',[tests/'test_core.c',src/'car_core.c'],[inc])])
    run([build('test_tracking',[tests/'test_tracking.c',src/'car_core.c'],[inc])])
    app=[tests/'test_app.c',src/'car_core.c',src/'car_app.c',src/'car_servo.c']
    telemetry_fixture=out/'telemetry-fixture.txt'
    run([build('test_app',app,[tests/'sdk',inc]),telemetry_fixture,out/'hold-fixture.txt',out/'low-fixture.txt'])
    run(['powershell','-NoProfile','-ExecutionPolicy','Bypass','-File',BASE/'repair/test_hold_protocol.ps1','-Fixture',out/'hold-fixture.txt'])
    run(['powershell','-NoProfile','-ExecutionPolicy','Bypass','-File',BASE/'repair/test_low_protocol.ps1','-Fixture',out/'low-fixture.txt'])
    run([build('test_servo',[tests/'test_servo.c',src/'car_servo.c'],[tests/'sdk',inc])])
    if (tests/'test_uart8.c').exists():
        files=[tests/'test_uart8.c',src/'car_core.c',src/'car_app.c',src/'car_servo.c']
        run([build('test_uart8',files,[tests/'sdk',inc])])
        run([build('test_uart8_diag',files,[tests/'sdk',inc],['CAR_COMM_DIAGNOSTIC=1'])])
        run([build('test_uart8_bad_ack',files,[tests/'sdk',inc],['CAR_COMM_DIAGNOSTIC=1','TEST_BAD_BAUD_ACK=1'])])
        run([build('test_uart8_fixed_baud',files,[tests/'sdk',inc],['CAR_WIRELESS_AUTO_BAUD=0'])])
    vision=out/'vision';vision.mkdir(exist_ok=True)
    run([build('test_vision_scenarios',[tests/'test_vision_scenarios.c',src/'car_core.c'],[inc]),vision])
    run([sys.executable,tests/'render_vision_report.py'])
    run([sys.executable,tests/'test_capture.py'])
    run([sys.executable,tests/'test_csi_buffers.py'])
    os.environ['TRACK_TEST_TELEMETRY']=str(telemetry_fixture)
    run([sys.executable,BASE/'repair/test_tracking_tool.py'])
    log.write('ALL TESTS PASSED\n')
print('ALL TESTS PASSED; see repair/evidence/host-tests.log')
