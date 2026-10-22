import os, shutil
from pathlib import Path
import ctypes as C
import json
import subprocess

p = Path(__file__).resolve().parent
stubs = p/'vision-stubs'
subprocess.run([(os.environ.get('ZIG') or shutil.which('zig') or r'D:\major-car\firmware\camera_car\.tools\ziglang\zig.exe'),'cc','-shared','-O2',
               '-I',str(stubs),'-I',str(p.parent/'demo/user/inc'),
               str(p.parent/'demo/user/src/img_process.c'),str(p.parent/'demo/user/src/steering_control.c'),str(p/'vision-replay-wrapper.c'),
               '-o',str(p/'vision-replay.dll')],check=True)
d = C.CDLL(str(p/'vision-replay.dll'))
d.replay_frame.argtypes = [C.POINTER(C.c_uint8),C.POINTER(C.c_int)]
captures = p/'fixtures'
reports = []
for name in ['snapshot-20261021-180609','snapshot-20261021-180619']:
    raw = (captures/name/'payload.bin').read_bytes()[:22560]
    data = (C.c_uint8*len(raw))(*raw)
    d.replay_reset()
    results = []
    for frame in range(60):
        out = (C.c_int*4)()
        ready = d.replay_frame(data,out)
        results.append(list(out))
        assert ready, (name,frame,'Straight must remain start-ready',list(out))
        assert abs(out[1]) <= 10, (name,frame,'Straight error contaminated by vehicle nose',list(out))
        turn=(C.c_float*5)()
        d.replay_turn(turn)
        assert turn[2] == 0 and abs(turn[1]-0.5)<0.001, (name,'Straight must not trigger curve boost',list(turn))
    reports.append({'source':name,'frames':60,'first':results[0],'last':results[-1]})
    # Masked car-body brightness must not alter thresholds or steering.
    for shade in [0,255]:
        changed = raw[:106*188] + bytes([shade])*(14*188)
        pixels = (C.c_uint8*len(changed))(*changed)
        d.replay_reset()
        out = (C.c_int*4)()
        d.replay_frame(pixels,out)
        assert list(out) == results[0], (name,'masked body still influences detector',shade,list(out),results[0])
for road,body in [(0,255),(255,0)]:
    raw = bytes([road])*(106*188)+bytes([body])*(14*188)
    pixels = (C.c_uint8*len(raw))(*raw)
    d.replay_reset()
    out = (C.c_int*4)()
    d.replay_frame(pixels,out)
    assert out[0] == 0, ('Car body must not create road contrast',road,body,list(out))
print(json.dumps(reports))
(p/'hood-replay-result.json').write_text(json.dumps(reports,indent=2),encoding='utf-8')
