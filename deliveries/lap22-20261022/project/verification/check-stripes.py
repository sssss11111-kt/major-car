"""Replay deterministic flat-stripe scenes through the production vision owner.

Synthetic fixtures exercise logic; they do not calibrate real camera thresholds.
No serial/DAP access or motion commands.
"""
from pathlib import Path
import ctypes as C
import os
import shutil
import subprocess

p = Path(__file__).resolve().parent
zig = os.environ.get('ZIG') or shutil.which('zig')
subprocess.run([zig, 'cc', '-shared', '-O2', '-Wall', '-Wextra', '-Werror',
                '-I', str(p/'vision-stubs'), '-I', str(p.parent/'demo/user/inc'),
                str(p.parent/'demo/user/src/img_process.c'),
                str(p.parent/'demo/user/src/steering_control.c'),
                str(p/'vision-replay-wrapper.c'), '-o', str(p/'stripe-replay.dll')], check=True)
d = C.CDLL(str(p/'stripe-replay.dll'))
d.replay_frame.argtypes = [C.POINTER(C.c_uint8), C.POINTER(C.c_int)]


def scene(offset=0, stripe=False, y0=55, y1=105, bars=7, edges=True):
    raw = bytearray([20] * (188*120))
    for y in range(106):
        half = int(71 + 32*(y-90)/119 + 0.5)
        left, right = 94+offset-half, 94+offset+half-1
        if not edges:
            left, right = 0, 187
        for x in range(max(0,left), min(187,right)+1):
            raw[y*188+x] = 220
        if stripe and y0 <= y <= y1:
            for bar in range(bars):
                start = 94+offset-(bars*12-6)//2 + bar*12
                for x in range(start,start+6):
                    if 0 <= x < 188:
                        raw[y*188+x] = 20
    return raw


def frame(raw):
    out = (C.c_int*4)()
    d.replay_frame((C.c_uint8*len(raw))(*raw), out)
    return list(out)


def turn():
    out = (C.c_float*5)()
    d.replay_turn(out)
    return list(out)


# Baseline entry -> long stripe region -> ordinary road recovery, left/right.
for offset in [0, -12, 12]:
    d.replay_reset()
    for _ in range(8):
        baseline = frame(scene(offset))
        assert baseline[0] and abs(baseline[1]-offset) <= 2, ('baseline',baseline)
    for n in range(60):
        result = frame(scene(offset, True))
        assert result[0] and abs(result[1]-baseline[1]) <= 2, ('stripe steering',offset,n,result)
        assert turn()[4] == 0, ('stripe preview must be suppressed',turn())
    for _ in range(8):
        assert frame(scene(offset))[:2] == baseline[:2]
    assert turn()[4] == 1, 'normal preview must recover'
print('PASS: measured stripe corridor preserves centered/left/right steering and reacquires')

# A current stripe image without visible outer boundaries is not a lane lease.
d.replay_reset()
for _ in range(8): frame(scene())
for n in range(60):
    result = frame(scene(stripe=True, y0=18, edges=False))
    assert not result[0], ('unbounded stripe image must fail closed',n,result)
print('PASS: no blind continuation from historical centers')

# Enough distant rows must not hide loss of the near control corridor.
d.replay_reset()
for _ in range(8): frame(scene())
raw = scene(stripe=True,y0=80,edges=False)
raw[:80*188] = scene()[:80*188]
result = frame(raw)
assert not result[0] and result[2] >= 30, ('distant rows cannot authorize driving',result)
print('PASS: clean distant track cannot mask a missing near corridor')

# Start-ready requires actual normal track. Stripe-only startup is inhibited.
d.replay_reset()
for _ in range(10):
    raw = scene(stripe=True)
    out = (C.c_int*4)()
    ready = d.replay_frame((C.c_uint8*len(raw))(*raw), out)
    assert not ready, ('stripe must not qualify for start',list(out))
print('PASS: stripe-only startup rejected')

# One bar, short marks, and vehicle-body stripes do not enter crossing mode.
for raw in [scene(stripe=True, bars=1), scene(stripe=True,y0=90,y1=91)]:
    d.replay_reset()
    for _ in range(8): frame(scene())
    frame(raw)
    assert d.replay_stripe() == 0, 'isolated marks must not be classified as a stripe region'
for shade in [0,255]:
    raw = scene()
    for y in range(107,120):
        for x in range(188):
            raw[y*188+x] = shade if (x//6)%2 else 255-shade
    d.replay_reset()
    expected = frame(scene())
    d.replay_reset()
    assert frame(raw) == expected and not d.replay_stripe(), 'body bars must not affect vision'
for shade in [0,127,255]:
    d.replay_reset()
    assert frame(bytearray([shade]*(188*120)))[0] == 0
print('PASS: isolated marks, body mask, and uniform scenes')

for direction in [-1,1]:
    raw = scene()
    for y in range(106):
        half = int(71+32*(y-90)/119+0.5)
        center = 94+direction*(10+(105-y)//8)
        for x in range(188):
            raw[y*188+x] = 220 if center-half <= x < center+half else 20
    d.replay_reset()
    for _ in range(8): result=frame(raw)
    assert result[0] and result[1]*direction > 5 and not d.replay_stripe(), ('normal curve',result)
    assert turn()[4] == 1, 'normal curves retain preview'
print('PASS: mirrored normal curves do not trigger stripes')

# Exercise current vision and LAP21's stateful steering across entry and exit.
d.replay_control_step.argtypes = [C.c_uint32, C.POINTER(C.c_float)]
for offset in [-12,12]:
    d.replay_reset()
    previous=92.5
    for n in range(60):
        result=frame(scene(offset,stripe=10 <= n < 40))
        state=(C.c_float*5)()
        d.replay_control_step((n+1)*2,state)
        assert result[0] and state[4] == 1, ('steering history lost',n,list(state))
        assert 66 <= state[0] <= 119, ('configured inward limits',list(state))
        assert abs(state[0]-previous) <= 4.401, ('LAP21 slew bound',n,list(state))
        if 10 <= n < 40:
            assert state[3] == 0, ('stripe preview must clear history',n,list(state))
            assert abs(state[0]-92.5) > 1, ('stripe entry must not reset steering to center',n,list(state))
        previous=state[0]
    assert state[3] >= 2, 'fresh preview must recover after stripes'
print('PASS: combined vision/stateful steering retains history, preview gating and 66..119 limits')
