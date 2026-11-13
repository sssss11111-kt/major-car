"""Regression tests for the wide-throat / single-bank fixes.

These pin the behaviours that failed on the captured field data. Wherever a
capture exists it is used as the input; synthetic fixtures are labelled as such
and only cover topology that no capture contains.

Inputs are found relative to the repository or from CAR_FIELD_DIR:
    CAR_FIELD_DIR=/path/to/交接包/field python -X utf8 check-wide-throat.py

Run from this directory with ZIG set to the zig executable.
"""
import ctypes as C
import os
import subprocess
import sys
from pathlib import Path

p = Path(__file__).resolve().parent
src = p.parent / 'demo/user/src'
shared = p.parents[3] / 'firmware/camera_car/demo/user'
zig = os.environ['ZIG']

PGM_HDR = b'P5\n188 120\n255\n'


def field_root():
    env = os.environ.get('CAR_FIELD_DIR')
    if env:
        root = Path(env)
        assert root.is_dir(), f'CAR_FIELD_DIR is not a directory: {root}'
        return root
    # handoff package layout: <package>/field, repo sits beside it
    for base in (p, *p.parents):
        cand = base / 'field'
        if (cand / 'run02-clear').is_dir():
            return cand
    raise SystemExit('Set CAR_FIELD_DIR to the handoff package "field" directory.')


FIELD = field_root()


def build():
    out = p / 'wide-throat-replay.so'
    modules = ('img_process.c', 'steering_control.c', 'roundabout_scan.c', 'cross_drive.c')
    shared_mods = ('special_track.c', 'special_track_scan.c', 'special_route.c',
                   'cross_island.c', 'roundabout.c')
    subprocess.run([zig, 'cc', '-shared', '-O2', '-std=c99', '-Wall', '-Wextra', '-Werror',
                    '-I', str(p / 'vision-stubs'), '-I', str(src.parent / 'inc'),
                    '-I', str(shared / 'inc'), '-I', str(shared / 'src'),
                    *(str(src / m) for m in modules),
                    *(str(shared / 'src' / m) for m in shared_mods),
                    str(p / 'wide-throat-wrapper.c'), '-o', str(out)], check=True)
    d = C.CDLL(str(out))
    d.replay_ring_step.argtypes = [C.POINTER(C.c_uint8), C.c_uint32, C.POINTER(C.c_float)]
    return d


def pixels(path):
    data = Path(path).read_bytes()
    if data[:len(PGM_HDR)] != PGM_HDR:
        raise SystemExit(f'{path}: unexpected PGM header')
    return data[len(PGM_HDR):]


def run(d, files, tick_step=13):
    """Replay in capture order; state carries across frames as on the car."""
    d.replay_ring_reset()
    out = (C.c_float * 24)()
    tick = 0
    rows = []
    for f in files:
        tick += tick_step
        raw = pixels(f)
        d.replay_ring_step((C.c_uint8 * len(raw)).from_buffer_copy(raw), tick, out)
        o = list(out)
        rows.append(dict(file=Path(f).name, valid=int(o[3]), usable=int(o[21]),
                         single=int(o[22]), near_run=int(o[16]),
                         ring_phase=int(o[1]), path_count=int(o[19]),
                         command=o[0], error=o[4]))
    return rows


def capture(name):
    return sorted((FIELD / name).glob('frame-*.pgm'))


def check_saturated_throat(d):
    """run02-clear 125: the road fills the lower image, only one bank visible.

    Before the fix this frame reported 26 usable rows against
    IP_VALID_ROWS_MIN=30, so main.c latched fault X4 with reason 2 ("no valid
    vision") and stopped the car while steering data was still available.
    """
    files = capture('run02-clear')
    rows = run(d, files)
    idx = {int(Path(f).stem.split('-')[1]): i for i, f in enumerate(files)}
    f125 = rows[idx[125]]
    f126 = rows[idx[126]]
    assert f125['usable'] == 26, f125
    assert f125['near_run'] >= 10, ('frame 125 must keep its measured near corridor', f125)
    assert f125['valid'] == 1, ('frame 125 must not report "no vision"', f125)
    assert f126['valid'] == 0, ('frame 126 must still be rejected', f126)
    print('PASS: saturated throat keeps the near corridor '
          f'(125 usable={f125["usable"]} near_run={f125["near_run"]} valid; '
          f'126 near_run={f126["near_run"]} rejected)')


def check_stops_are_genuine(d):
    """Both captures must still stop once the near corridor really is gone.

    Validity is granted either by the whole-image row total or by a continuous
    measured near run together with a minimum row count. A frame claiming
    validity with neither is the defect this guards against.
    """
    for name in ('run02-clear', 'run01-occluded'):
        rows = run(d, capture(name))
        bad = [r for r in rows
               if r['valid'] and r['usable'] < 30
               and not (r['near_run'] >= 8 and r['usable'] >= 15)]
        assert not bad, ('a frame claimed validity with neither route', bad[0])
        last = rows[-1]
        assert last['valid'] == 0, ('capture must end stopped', last)
        print(f'PASS: {name} never claims vision without a near corridor and '
              f'ends stopped (last: valid={last["valid"]} usable={last["usable"]} '
              f'near_run={last["near_run"]})')


def check_entry_commit(d):
    """Captured ring segment: entry commits only with a real measured path."""
    files = sorted((FIELD / 'historical-manual02' / 'frames').glob('*.pgm'))
    sel = [f for f in files if 599 + 118 <= int(Path(f).name[:6]) <= 599 + 250]
    rows = run(d, sel)
    enters = [i for i, r in enumerate(rows) if r['ring_phase'] == 1]
    assert enters, 'roundabout entry must commit somewhere on the captured segment'
    first = enters[0]
    assert rows[first]['path_count'] >= 12, (
        'entry must commit with a usable measured path', rows[first])
    print(f'PASS: ring entry commits at original index {118 + first} '
          f'with a {rows[first]["path_count"]}-point measured path')


def check_no_blind_entry(d):
    """The ring capture must not commit entry with an empty path.

    Guards the earlier failure mode where commit was forced by corner progress
    alone, so the car turned into the island without a validated path.
    """
    files = sorted((FIELD / 'historical-manual02' / 'frames').glob('*.pgm'))
    sel = [f for f in files if 599 + 118 <= int(Path(f).name[:6]) <= 599 + 250]
    rows = run(d, sel)
    for i, r in enumerate(rows):
        if r['ring_phase'] == 1 and (i == 0 or rows[i - 1]['ring_phase'] == 5):
            assert r['path_count'] >= 12, (
                'entry committed without a validated path', r)
    print('PASS: no entry commit occurs without a validated path')


if __name__ == '__main__':
    d = build()
    check_saturated_throat(d)
    check_stops_are_genuine(d)
    check_entry_commit(d)
    check_no_blind_entry(d)
    print('PASS: wide-throat regression set')
