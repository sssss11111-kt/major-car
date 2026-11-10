"""Replay original frames 120..247 through the production R17-4 entry point."""
from pathlib import Path
import argparse
import base64
import csv
import ctypes as C
import hashlib
import json
import math
import os
import subprocess
import zipfile
import zlib
from io import BytesIO
from PIL import Image

WORK = Path(__file__).resolve().parent
ROOT = WORK.parents[2]
PROJECT = ROOT / 'deliveries/lap30-ring-20261107/project'
baseline = json.loads((WORK/'baseline-r17.json').read_text(encoding='utf-8'))
parser = argparse.ArgumentParser()
parser.add_argument('--capture-zip', type=Path, default=Path(baseline['capture_zip']))
capture = parser.parse_args().capture_zip
assert hashlib.sha256(capture.read_bytes()).hexdigest() == baseline['capture_sha256']
compiler = os.environ.get('ZIG', r'D:\桌面\智能车\.tmp\ring-research-20261109\toolchain\ziglang\zig.exe')
command = [compiler, 'cc', '-shared', '-O2', '-Wall', '-Wextra', '-Werror', '-DREPLAY_CURRENT']
for path in ['verification/vision-stubs', 'demo/user/inc', 'demo/user/src', 'verification']:
    command += ['-I', str(PROJECT / path)]
command += [str(WORK / 'ordered-replay.c'), '-o', str(WORK / 'r174.dll')]
subprocess.run(command, check=True)
native = C.CDLL(str(WORK / 'r174.dll'))
native.replay_ring_scenario.argtypes = [C.c_int, C.c_float]


def png(gray):
    stream = BytesIO()
    Image.frombytes('L', (188, 120), gray).save(stream, format='PNG')
    return 'data:image/png;base64,' + base64.b64encode(stream.getvalue()).decode()


def turn_angle(path):
    maximum = 0
    for a, b, c in zip(path, path[1:], path[2:]):
        ax, ay, bx, by = b[0]-a[0], b[1]-a[1], c[0]-b[0], c[1]-b[1]
        maximum = max(maximum, math.degrees(math.atan2(abs(ax*by-ay*bx), ax*bx+ay*by)))
    return maximum


with zipfile.ZipFile(capture) as archive:
    assert archive.testzip() is None
    metadata = [json.loads(line) for line in archive.read('manual-02/frames.jsonl').decode().splitlines()]
    pixels = {}
    for index in range(111, 248):
        meta = metadata[index-1]
        pgm = archive.read('manual-02/' + meta['file'].replace('\\', '/'))
        assert pgm[:15] == b'P5\n188 120\n255\n' and len(pgm) == 22575
        gray = pgm[15:]
        assert hashlib.sha256(gray).hexdigest() == meta['sha256']
        assert f'{zlib.crc32(gray)&0xffffffff:08x}' == meta['crc32']
        pixels[index] = gray


def replay(exit_scenario=False):
    native.replay_ring_reset()
    frames = []
    for index, gray in pixels.items():
        if exit_scenario and index == 180:
            native.replay_ring_scenario(2, 240.0)
        if exit_scenario and index == 203:
            native.replay_ring_scenario(3, 270.0)
        out, debug, state = (C.c_float*5)(), (C.c_int*9)(), (C.c_int*6)()
        thresholds, lines, buffer = (C.c_uint8*120)(), (C.c_uint8*360)(), (C.c_uint8*1120)()
        native.replay_ring_step((C.c_uint8*22560).from_buffer_copy(gray), metadata[index-1]['board_frame_tick'], out)
        count = native.replay_entry_path(buffer)
        path = [list(buffer[i*7:i*7+7]) for i in range(count)]
        native.replay_local_debug(debug)
        native.replay_planner_state(state)
        native.replay_thresholds(thresholds)
        native.replay_lines(lines)
        angle = turn_angle(path)
        assert angle <= 40.0001, (index, angle)
        for point in path:
            assert gray[point[1]*188+point[0]] > thresholds[point[1]], index
        for a, b in zip(path, path[1:]):
            steps = max(abs(b[0]-a[0]), abs(b[1]-a[1]))
            for k in range(steps+1):
                x, y = a[0]+int((b[0]-a[0])*k/steps), a[1]+int((b[1]-a[1])*k/steps)
                assert gray[y*188+x] > thresholds[y], (index, x, y)
        local = bool(native.replay_ring_local_active())
        if local:
            local_rows = [(y, lines[120+y]) for y in range(18, 106) if lines[120+y]]
            assert all(gray[y*188+x] > thresholds[y] for y, x in local_rows)
            assert all(b[0]-a[0] <= 3 for a, b in zip(local_rows, local_rows[1:]) if a[0]<=75), index
        if index < 120:
            continue
        gates = (C.c_int*8)()
        native.replay_gate_facts(gates)
        frames.append(dict(index=index, command=float(out[0]), phase=int(out[1]), error=float(out[2]),
            valid=bool(out[3]), local=local, preview=state[3],
            preview_valid=bool(native.replay_preview_valid()), status=native.replay_entry_status(),
            path=path, lines=list(lines), thresholds=list(thresholds), debug=list(debug), gates=list(gates),
            angle=angle, image=png(gray), r17=baseline['frames'][index-120]))
    return frames


frames = replay()
scenario = replay(True)
summary = dict(version='C11LAP30R17-4', baseline='d62a1b2608b931298ec4013092ef18f050cabba2',
    capture_sha256=baseline['capture_sha256'], first_frame=120, last_frame=247, count=len(frames),
    warmup='111..119, no warmup images embedded in playback',
    valid=sum(f['valid'] for f in frames), rejected=[f['index'] for f in frames if not f['valid']],
    first_entry=next((f['index'] for f in frames if f['phase']==1), None),
    first_local=next((f['index'] for f in frames if f['local']), None),
    first_path=next((f['index'] for f in frames if f['path']), None),
    exit_scenario=dict(injected_frame=180, injected_phase=2, injected_yaw=240,
        injected_exit_frame=203, injected_exit_yaw=270,
        first_exit=next((f['index'] for f in scenario if f['phase']==3), None)),
    maximum_path_segment_turn=max(f['angle'] for f in frames),
    hardware_accessed=False, flashed=False, real_track_tested=False)
summary['source_sha256'] = {str(p.relative_to(PROJECT)): hashlib.sha256(p.read_bytes()).hexdigest()
    for p in sorted((PROJECT/'demo/user').rglob('*')) if p.suffix in ('.c', '.h')}
(WORK/'results.json').write_text(json.dumps(dict(summary=summary, frames=frames, exit_scenario=scenario), ensure_ascii=False), encoding='utf-8')
with (WORK/'逐帧检查.csv').open('w', encoding='utf-8-sig', newline='') as stream:
    writer=csv.writer(stream)
    writer.writerow(['原帧号','有效','阶段','普通弯道接管','近端误差','预瞄误差','模拟舵机命令','路径点数','最大段夹角','近端行数'])
    for f in frames:
        writer.writerow([f['index'], f['valid'], f['phase'], f['local'], f['error'], f['preview'], round(f['command'],3), len(f['path']), round(f['angle'],3), f['debug'][0]])
print(json.dumps(summary, ensure_ascii=False, indent=2))
