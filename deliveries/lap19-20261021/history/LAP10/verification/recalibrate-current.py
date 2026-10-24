from pathlib import Path
import json
import re
import hashlib

p = Path(__file__).resolve().parent
names = ['snapshot-20261021-180609', 'snapshot-20261021-180619']
captures = []
for name in names:
    d = json.loads((p/name/'capture.json').read_text())
    assert d['crc_ok']
    raw = (p/name/'payload.bin').read_bytes()[:22560]
    captures.append((d, raw))

def valid(d, raw, y):
    l,m,r = [d['lines'][k][y] for k in ['left','mid','right']]
    return 0 < l < r < 187 and r-l > 10 and raw[y*188+m] > d['threshold']

widths = [0]*120
for y in range(30,111):
    if not all(all(valid(d,raw,j) for j in range(y-1,y+2)) and
               all(abs(d['lines'][edge][j]-d['lines'][edge][j-1]) <= 4
                   for edge in ['left','right'] for j in range(y,y+2))
               for d,raw in captures):
        continue
    ws = [d['lines']['right'][y]-d['lines']['left'][y] for d,_ in captures]
    if abs(ws[0]-ws[1]) <= 4:
        widths[y] = (sum(ws)+1)//2
# Keep the continuous track segment containing the control row. Isolated
# detections below the first gap can belong to the vehicle nose.
start = end = 90
while start > 30 and widths[start-1]: start -= 1
while end < 110 and widths[end+1]: end += 1
widths = [v if start <= y <= end else 0 for y,v in enumerate(widths)]
assert all(widths[y] for y in range(88,93))
assert all(abs(widths[y] - (d['lines']['right'][y]-d['lines']['left'][y])) <= 8
           for d,_ in captures for y in range(88,93))
source = p.parent/'demo/user/src/single_edge.c'
old = source.read_bytes()
if not (p/'single_edge-before-calibration.c').exists():
    (p/'single_edge-before-calibration.c').write_bytes(old)
text = old.decode('utf-8')
text,n = re.subn(r'static const uint8_t width_reference\[SE_H\] = \{[^}]+\};',
                'static const uint8_t width_reference[SE_H] = { '+','.join(map(str,widths))+' };', text)
assert n == 1
text = text.replace('Reference from two CRC-verified same-pose straight snapshots, 131631/131850.',
                    'Reference from two CRC-verified fixed-pose straight snapshots, 180609/180619.')
source.write_bytes(text.encode('utf-8'))
report = {'sources':names, 'camera_pose_unchanged_user_confirmed':True,
          'width_px':widths, 'control_rows':{str(y):widths[y] for y in range(88,93)},
          'rule':'Continuous valid segment containing row 90 within rows 30..110: two CRC frames; dual white-center neighbors; edge continuity <=4 px; inter-frame width difference <=4 px; rounded mean.',
          'source_before_sha256':hashlib.sha256(old).hexdigest(),
          'source_after_sha256':hashlib.sha256(source.read_bytes()).hexdigest()}
(p/'width-reference-current.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
print(json.dumps({'control_rows':report['control_rows'], 'calibrated_rows':sum(v>0 for v in widths)}))
