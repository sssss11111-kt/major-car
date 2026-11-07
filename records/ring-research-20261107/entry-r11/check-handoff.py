"""Native regression for measured-width continuation and entrance handoff."""
from pathlib import Path
import ctypes as C
import json
HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[2]
INPUT=ROOT/'records/ring-entry-current-20261107/图像采集-manual-02-20261107'
report=json.loads((INPUT/'R11中线计算结果_40-60帧.json').read_text(encoding='utf-8'))
meta=[json.loads(line) for line in (INPUT/'manual-02/frames.jsonl').read_text().splitlines()]
raws=[(INPUT/'manual-02'/m['file']).read_bytes()[15:] for m in meta]
d=C.CDLL(str(HERE/'manual02-native.dll'))
d.replay_set_odometry.argtypes=[C.c_float,C.c_float,C.c_uint32,C.c_int]
checks=[]
def check(ok,label):
    assert ok,label
    checks.append(label)
def frame(raw,tick=0):
    d.replay_set_odometry(0,0,tick,0)
    result=(C.c_int*4)()
    d.replay_frame((C.c_uint8*22560).from_buffer_copy(raw),result)
    sides=(C.c_uint8*120)();edges=(C.c_uint8*120)();lines=(C.c_uint8*360)();thr=(C.c_uint8*120)();model=(C.c_float*5)()
    d.replay_approach(sides,edges);d.replay_lines(lines);d.replay_thresholds(thr);d.replay_approach_model(model)
    return dict(sides=list(sides),edges=list(edges),lines=list(lines),thresholds=list(thr),model=list(model),valid=int(result[0]))
def train():
    d.replay_reset()
    a=frame(raws[45],100)
    check(a['model'][0]==2 and any(a['sides']),'learn from frame46 measured pair')
    return a
rows=report['frame_results']
for f in rows[39:52]:
    n=f['modes'][0]
    check(n['valid'] and any(n['bridge_side']) and not n['path'],'continuous approach frame'+str(f['index']))
    if f['index']>=47:
        check(abs(n['error'])<=3,'near center preserved frame'+str(f['index']))
        check(abs(n['command']-92.5)<=2.5,'straight command frame'+str(f['index']))
for f in rows[52:60]:
    n,o=f['modes']
    check(n['valid'] and n['path'] and not any(n['bridge_side']),'ordered curve owns frame'+str(f['index']))
    check(n['path']==o['path'],'paired entrance geometry preserved frame'+str(f['index']))
check(rows[51]['modes'][0]['command']>92,'frame52 keeps approach instead of early turn')
check(rows[52]['modes'][0]['command']<rows[51]['modes'][0]['command']-3,'frame53 starts steering at path takeover')
check(not rows[73]['modes'][0]['valid'] and not any(rows[73]['modes'][0]['bridge_side']),'loss after takeover cannot reactivate straight bridge')
for f in rows:
    n,o=f['modes']
    check((n['valid'],n['phase'])==(o['valid'],o['phase']),'full capture validity and phase '+str(f['index']))
    if o['path']:check(n['path']==o['path'],'full capture path preserved '+str(f['index']))
# Mirror ordinary approach, so the configured clockwise route does not force direction.
normal=[]
for mirrored in [False,True]:
    d.replay_reset()
    for i in range(52):
        raw=raws[i]
        if mirrored:raw=b''.join(raw[y*188:(y+1)*188][::-1] for y in range(120))
        a=frame(raw,meta[i]['board_frame_tick'])
        if 46<=i<=51:
            check(a['valid'] and any(a['sides']),f'current side support {mirrored} frame{i+1}')
            for y,side in enumerate(a['sides']):
                if not side:continue
                check(raw[y*188+a['lines'][120+y]]>a['thresholds'][y],f'white center {mirrored} frame{i+1} row{y}')
                check(a['edges'][y]==a['lines'][240+y if side==2 else y],f'control uses repair {mirrored} frame{i+1} row{y}')
            if not mirrored:normal.append(a)
            else:
                other=normal[i-46]
                check([bool(v) for v in a['sides']]==[bool(v) for v in other['sides']],f'mirrored repaired rows frame{i+1}')
                check(all(abs(a['lines'][120+y]+other['lines'][120+y]-187)<=1 for y in range(120) if a['sides'][y]),f'mirrored centers frame{i+1}')
base=train();a=frame(raws[49],127)
check(any(a['sides']) and a['model'][2:4]==base['model'][2:4] and a['model'][1]>0,'estimated edge never refreshes learned width')
d.replay_reset();check(not any(frame(raws[49],127)['sides']),'no model means no extrapolated continuation')
train();check(not any(frame(raws[49],351)['sides']),'250 tick lease expires')
train()
for _ in range(31):a=frame(raws[49],100)
check(not any(a['sides']),'30 observation lease expires when clock is absent')
train();d.replay_guide(1);d.replay_entry_mode(1)
check(not any(frame(raws[49],127)['sides']),'opposite route cannot reuse right-side width')
for case in ['no opposite','no far anchor','blocked corridor','no contrast']:
    train();raw=bytearray(raws[49])
    if case=='no opposite':
        for y in range(18,106):raw[y*188:y*188+94]=bytes([255])*94
    elif case=='no far anchor':
        for y in range(18,80):raw[y*188+94:(y+1)*188]=bytes([255])*94
    elif case=='blocked corridor':
        for y in range(88,94):raw[y*188+99:y*188+104]=bytes([0])*5
    else:raw=bytearray([255]*22560)
    check(not any(frame(raw,127)['sides']),case+' rejects continuation')
summary={'checks':len(checks),'passed':True,'continuous_bridge_frames':[40,52],'first_ordered_path_frame':53,'frame52_command_before':rows[51]['modes'][1]['command'],'frame52_command_after':rows[51]['modes'][0]['command'],'frame53_command_after':rows[52]['modes'][0]['command'],'all_312_validity_phase_and_entry_paths_preserved':True,'first_invalid_after_entry':74,'limits':{'max_age_calls':30,'max_age_10ms_ticks':250}}
(HERE/'handoff-checks.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2),encoding='utf-8')
print(json.dumps(summary,ensure_ascii=False))
