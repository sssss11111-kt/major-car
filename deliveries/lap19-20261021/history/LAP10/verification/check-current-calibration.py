from pathlib import Path
import ctypes as C
import json
import sys

p = Path(__file__).resolve().parent
class Result(C.Structure):
    _fields_ = [('mode', C.c_uint8), ('center', C.c_uint8), ('degraded', C.c_uint8),
                ('model_matches', C.c_uint8), ('since', C.c_uint32)]
dll = C.CDLL(str(p / sys.argv[1]))
f = dll.evaluate
f.restype = Result
f.argtypes = [C.POINTER(C.c_uint8)] * 4 + [C.c_uint32, C.c_uint8]

def call(data, tick=0, allow=0):
    arrays = [(C.c_uint8 * len(a))(*a) for a in data]
    return f(*arrays, tick, allow)

for name in ['snapshot-20261021-180609', 'snapshot-20261021-180619']:
    d = json.loads((p/name/'capture.json').read_text())
    assert d['crc_ok']
    raw = (p/name/'payload.bin').read_bytes()[:22560]
    data = [[255 if v > d['threshold'] else 0 for v in raw]] + [d['lines'][k] for k in ['left','mid','right']]
    r = call(data)
    print(name, 'mode', r.mode, 'center', r.center, 'model', r.model_matches, flush=True)
    assert r.mode == 1 and r.model_matches == 1, 'Current straight frame must satisfy the width model'

# Exercise the actual updated C implementation at the five control rows.
widths = json.loads((p/'width-reference-current.json').read_text())['width_px']
for side in ['left', 'right']:
    call(data)  # stopped frame resets the episode
    b = [255] * 22560
    l, m, r = [0]*120, [94]*120, [187]*120
    for y in range(88,93):
        if side == 'left': l[y] = 20
        else: r[y] = 168
    single = [b,l,m,r]
    mode = 2 if side == 'left' else 3
    expected = 20 + widths[90]//2 if side == 'left' else 168 - widths[90]//2
    out = call(single,100,1)
    assert (out.mode,out.center,out.degraded) == (mode,expected,1)
    assert call(single,149,1).mode == mode
    assert call(single,150,1).mode == 0
    assert call(single,151,0).mode == 0
    print(side, 'center', expected, 'timeout and stopped inhibition PASS')
print('PASS: two current straight frames, both single-edge directions, timeout and stopped inhibition')
