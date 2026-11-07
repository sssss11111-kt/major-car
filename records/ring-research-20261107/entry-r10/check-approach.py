from pathlib import Path
import ctypes as C
import json
HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[2]
INPUT=ROOT/'records/ring-entry-current-20261107/图像采集-manual-02-20261107'
report=json.loads((INPUT/'R10中线计算结果.json').read_text(encoding='utf-8'))
d=C.CDLL(str(HERE/'manual02-native.dll'))
meta=[json.loads(line) for line in (INPUT/'manual-02/frames.jsonl').read_text().splitlines()]
gray=(INPUT/'manual-02'/meta[20]['file']).read_bytes()[15:]
checks=[]
def check(ok,label):
    assert ok,label
    checks.append(label)
def process(raw,guide=0,entry=0):
    d.replay_reset();d.replay_guide(guide);d.replay_entry_mode(entry)
    o=(C.c_int*4)();d.replay_frame((C.c_uint8*22560).from_buffer_copy(raw),o)
    sides=(C.c_uint8*120)();edges=(C.c_uint8*120)();lines=(C.c_uint8*360)()
    d.replay_approach(sides,edges);d.replay_lines(lines)
    return list(sides),list(edges),list(lines)
sides,edges,lines=process(gray)
check(sum(bool(x) for x in sides)>=30,'frame21 bridges observed opening')
mirror=b''.join(gray[y*188:(y+1)*188][::-1] for y in range(120))
ms,me,ml=process(mirror)
check([x==2 for x in sides]==[x==1 for x in ms],'left/right mirror activation')
check(all(edges[y]+me[y]==187 and abs(lines[y+120]+ml[y+120]-187)<=1 for y in range(120) if sides[y]),'mirrored boundaries and centers')
for guide,entry in [(-1,1),(1,1),(-1,2),(1,2)]:
    check(not any(process(gray,guide,entry)[0]),'disabled during active guidance '+str((guide,entry)))
for name,start,end in [('missing upper endpoint',18,64),('missing lower endpoint',64,106)]:
    raw=bytearray(gray)
    for y in range(start,end):raw[y*188+94:(y+1)*188]=bytes([255])*94
    check(not any(process(raw)[0]),name+' rejects bridge')
raw=bytearray(gray)
for y in range(44,49):raw[y*188+52:y*188+84]=bytes([0])*32
check(not any(process(raw)[0]),'opposite boundary discontinuity rejects bridge')
for name,raw in [('all white',bytes([255])*22560),('all black',bytes(22560))]:
    check(not any(process(raw)[0]),name+' rejects bridge')
raw=bytearray([20]*22560)
for y in range(106):
    half=int(71+32*(y-90)/119+.5)
    raw[y*188+94-half:y*188+94+half+1]=bytes([220])*(2*half+1)
check(not any(process(raw)[0]),'ordinary continuous straight has no bridge')
rows=report['frame_results']
bridged=[]
for f in rows:
    new,old=f['modes']
    check(new['valid']==old['valid'] and new['phase']==old['phase'],'validity and phase frame '+str(f['index']))
    if new['phase']:
        check(not any(new['bridge_side']) and new['path']==old['path'],'entry path preserved frame '+str(f['index']))
    if any(new['bridge_side']):
        bridged.append(f['index'])
        data=(INPUT/'manual-02'/meta[f['index']-1]['file']).read_bytes()[15:]
        for y,side in enumerate(new['bridge_side']):
            if not side:continue
            x=new['bridge_edge'][y]
            check(new['lines'][240+y if side==2 else y]==x,'bridge reaches production row '+str((f['index'],y)))
            check(abs(new['lines'][120+y]-(new['lines'][y]+new['lines'][240+y])/2)<=1,'center follows repaired pair '+str((f['index'],y)))
f=rows[20];new,old=f['modes']
# Compare the opening and its two joins, using rows modified by the new algorithm.
ys=[y for y,v in enumerate(new['bridge_side']) if v]
def max_step(m):
    return max(abs(m['lines'][120+y]-m['lines'][119+y]) for y in range(min(ys),max(ys)+2))
check(max_step(new)<=2 and max_step(old)>=10,'frame21 removes spatial center jump')
summary={'checks':len(checks),'passed':True,'bridge_frames':bridged,'frame21':{'rows':len(ys),'old_max_adjacent_midline_step_px':max_step(old),'new_max_adjacent_midline_step_px':max_step(new),'old_error':old['error'],'new_error':new['error'],'old_command':old['command'],'new_command':new['command']},'focus_valid':sum(f['modes'][0]['valid'] for f in rows[9:50]),'focus_frames':41,'missing_anchor_frames':[f['index'] for f in rows[9:50] if not any(f['modes'][0]['bridge_side'])]}
(HERE/'approach-checks.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2),encoding='utf-8')
print(json.dumps(summary,ensure_ascii=False))
