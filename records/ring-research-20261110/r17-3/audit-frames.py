"""Re-run all captured frames and audit current R17-3 evidence and controls."""
from pathlib import Path
import csv
import ctypes as C
import hashlib
import json
import os
import subprocess
import zipfile
import zlib

work = Path(__file__).resolve().parent
root = work.parents[2]
project = root / 'deliveries/lap30-ring-20261107/project'
prior = json.loads((work / 'results.json').read_text(encoding='utf-8'))
assert hashlib.sha256(Path(prior['capture_zip']).read_bytes()).hexdigest() == prior['zip_sha256']
status = json.loads((project.parent / 'LAP30-status.json').read_text(encoding='utf-8'))
for path, expected in status['source_hashes'].items():
    assert hashlib.sha256((project.parent / path).read_bytes()).hexdigest() == expected, path

library = work / 'audit-replay.dll'
command = [os.environ['ZIG'], 'cc', '-shared', '-O2', '-Wall', '-Wextra', '-Werror', '-DREPLAY_CURRENT']
for directory in [project/'verification/vision-stubs', project/'demo/user/inc',
                  project/'demo/user/src', project/'verification']:
    command += ['-I', str(directory)]
command += [str(work/'audit-replay.c'), '-o', str(library)]
subprocess.run(command, check=True)
native = C.CDLL(str(library))
native.replay_ring_reset()
rows = []

with zipfile.ZipFile(prior['capture_zip']) as archive:
    assert archive.testzip() is None
    metadata = [json.loads(line) for line in archive.read('manual-02/frames.jsonl').decode().splitlines()]
    for index, meta in enumerate(metadata, 1):
        if index == 111:
            native.replay_ring_reset()
        pgm = archive.read('manual-02/' + meta['file'].replace('\\', '/'))
        assert pgm[:15] == b'P5\n188 120\n255\n' and len(pgm) == 22575
        gray = pgm[15:]
        assert hashlib.sha256(gray).hexdigest() == meta['sha256']
        assert f'{zlib.crc32(gray)&0xffffffff:08x}' == meta['crc32']
        output = (C.c_float * 5)()
        debug = (C.c_int * 9)()
        state = (C.c_float * 20)()
        thresholds = (C.c_uint8 * 120)()
        lines = (C.c_uint8 * 360)()
        native.replay_ring_step((C.c_uint8 * 22560).from_buffer_copy(gray), meta['board_frame_tick'], output)
        native.replay_local_debug(debug)
        native.replay_audit_state(state)
        native.replay_thresholds(thresholds)
        native.replay_local_rows(lines)
        previous = prior['frames'][index-1]['r173']
        assert int(output[3]) == previous['valid']
        assert abs(output[0]-previous['command']) < 0.0001
        assert output[2] == previous['error']
        near_rows = [y for y in range(80, 106) if lines[120+y]]
        near_gaps = [y for y in range(min(near_rows), max(near_rows)+1)
                     if not lines[120+y]] if near_rows else []
        flags = []
        local = bool(native.replay_ring_local_active())
        included = not (86 <= index <= 110 or index >= 265)
        if not included:
            flags.append('此前确认的搬车排除段')
        elif not output[3]:
            flags.append(f'近端有效观测{debug[0]}行，少于6行，路径拒绝')
        elif local:
            if debug[0] < 12:
                flags.append('近端边界支持较少')
            if previous['status'] == 2:
                flags.append('主要依靠单边与半路宽推算')
            if not state[15]:
                flags.append('无有效远端预瞄')
            elif output[2] * state[16] < 0:
                flags.append('近端纠偏与远端弯向相反')
            elif abs(output[2]) <= 3 and abs(state[16]) >= 15:
                flags.append('近端误差小而远端偏差大')
        raw_near = float(state[12]) if local else None
        row = dict(frame=index, included=included, tick=meta['board_frame_tick'],
                   valid=bool(output[3]), phase=int(output[1]), local=local,
                   status=previous['status'], failure=previous['failure'],
                   near_rows=debug[0], near_bottom=debug[2], near_gaps=near_gaps,
                   single_near=int(state[8]), paired_total=int(state[9]),
                   preview_rows=int(state[10]), preview_valid=bool(state[15]),
                   error=float(output[2]), raw_local_error=raw_near,
                   preview=float(state[16]), near_filtered=float(state[0]),
                   preview_filtered=float(state[1]), effective_error=float(state[2]),
                   command=float(output[0]), severity=float(state[3]),
                   speed_scale=float(state[4]), limited=bool(state[5]),
                   slew_limited=bool(state[6]), entrance_seen=bool(state[13]),
                   entrance_visible_proxy=bool(state[14]), island_top=debug[3],
                   island_bottom=debug[4], island_hold_age=debug[5],
                   confirm=debug[8], thresholds=list(thresholds),
                   local_rows=list(lines), flags=flags)
        rows.append(row)

scoped = [row for row in rows if row['included']]
local = [row for row in scoped if row['local']]
summary = dict(total_raw=len(rows), included=len(scoped), valid=sum(row['valid'] for row in scoped),
               invalid=[row['frame'] for row in scoped if not row['valid']],
               local_frames=len(local), local_valid=sum(row['valid'] for row in local),
               local_single=sum(row['status']==2 for row in local),
               local_dual=sum(row['status']==6 for row in local),
               limited_frames=[row['frame'] for row in scoped if row['limited']],
               slew_limited_frames=[row['frame'] for row in scoped if row['slew_limited']],
               command_min=min(row['command'] for row in scoped),
               command_max=max(row['command'] for row in scoped),
               local_without_preview=[row['frame'] for row in local if row['valid'] and not row['preview_valid']],
               local_opposite_errors=[row['frame'] for row in local if row['valid'] and
                                      row['preview_valid'] and row['error']*row['preview']<0])
report = dict(version=status['version'], algorithm_commit='4582012',
              source_hashes_verified=True, frame_crc_sha_verified=True,
              synchronized_wheel_odometry_available=False, summary=summary, frames=rows)
(work/'frame-audit.json').write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
with (work/'逐帧检查.csv').open('w', encoding='utf-8-sig', newline='') as handle:
    writer = csv.writer(handle)
    writer.writerow(['帧号','纳入','有效','软件阶段','局部接管','状态','失败码','局部近端观测行（接管后适用）',
                     '单边近端行','预瞄行','近端原始误差','最终近端误差','预瞄误差',
                     '模拟舵机','命令限幅','变化速率限制','入口下端行','岛体暂存年龄','确认计数','问题与观察'])
    for row in rows:
        writer.writerow([row['frame'],int(row['included']),int(row['valid']),row['phase'],int(row['local']),
                         row['status'],row['failure'],row['near_rows'],row['single_near'],row['preview_rows'],
                         row['raw_local_error'],row['error'],row['preview'],round(row['command'],3),
                         int(row['limited']),int(row['slew_limited']),row['island_bottom'],row['island_hold_age'],
                         row['confirm'],'；'.join(row['flags'])])
print(json.dumps(summary, ensure_ascii=False, indent=2))
