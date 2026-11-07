"""Reproduce R10 native entrance planning, checks and offline comparison."""
from pathlib import Path
import base64
import ctypes as C
import hashlib
import json
import math
import os
import re
import subprocess
import sys
import time

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
PROJECT = ROOT / 'deliveries/lap30-ring-20261107/project'
OLD = Path(r'D:\桌面\智能车\major-car-LAP29-20261028')
sys.path.insert(0, str(OLD / 'deliveries/pc-replay-20261030'))
from replay import read_pgm, png

ZIG = os.environ.get('ZIG', r'C:\Users\liyuy\AppData\Local\uv\cache\archive-v0\lr7GmYIgFrzqabj5\ziglang\zig.exe')
command = [ZIG, 'cc', '-shared', '-O2', '-Wall', '-Wextra', '-Werror']
for directory in [PROJECT/'verification/vision-stubs', PROJECT/'demo/user/inc', PROJECT/'demo/user/src', PROJECT/'verification']:
    command += ['-I', str(directory)]
command += [str(HERE/'ordered-replay.c'), '-o', str(HERE/'ordered-replay.dll')]
subprocess.run(command, check=True)
d = C.CDLL(str(HERE/'ordered-replay.dll'))
d.replay_estimated_width.restype=C.c_float
checks = []

def check(condition, message):
    assert condition, message
    checks.append(message)

def points():
    buffer = (C.c_uint8 * 1120)()
    count = d.replay_entry_path(buffer)
    return [list(buffer[i*7:i*7+7]) for i in range(count)]

def thresholds():
    output = (C.c_uint8 * 120)()
    d.replay_thresholds(output)
    return bytes(output)

def segment(x0, y0, x1, y1):
    steps = max(abs(x1-x0), abs(y1-y0))
    for i in range(steps+1):
        yield round(x0+(x1-x0)*i/max(1, steps)), round(y0+(y1-y0)*i/max(1, steps))

def verify_path(raw, path, thr, label):
    check(12 <= len(path) <= 160, label+' ordered path capacity')
    check(path[0][:2] == [94,100], label+' begins at current vehicle axis')
    for i, point in enumerate(path):
        x,y = point[:2]
        assert raw[y*188+x] > thr[y], (label, 'point outside current white',i,point)
        if i:
            previous = path[i-1]
            assert max(abs(x-previous[0]), abs(y-previous[1])) <= 3, (label,'disconnected',i)
            for px,py in segment(previous[0],previous[1],x,y):
                assert raw[py*188+px] > thr[py], (label,'segment crosses black',i,px,py)
    check(True, label+' all consecutive target segments remain on current white')
    controls=(C.c_float*8)()
    d.replay_curve(controls)
    c=list(controls)
    check(abs(c[0]-94)<0.01 and abs(c[1]-100)<0.01 and abs(c[2]-c[0])<0.01 and c[3]<c[1],
          label+' curve enters with the vehicle forward tangent')
    headings=[]
    for sample in range(129):
        t=sample/128
        u=1-t
        dx=3*(u*u*(c[2]-c[0])+2*u*t*(c[4]-c[2])+t*t*(c[6]-c[4]))
        dy=3*(u*u*(c[3]-c[1])+2*u*t*(c[5]-c[3])+t*t*(c[7]-c[5]))
        headings.append(math.degrees(math.atan2(dy,dx)))
    changes=[abs((b-a+180)%360-180) for a,b in zip(headings,headings[1:])]
    check(max(changes)<8, label+' continuous curve has no abrupt heading step')
    check(sum(v>0.1 for v in changes)>20,label+' turning spreads over the curve rather than one corner')


capture = OLD / 'records/ring-entry-20261106/manual-01'
frames=[]
for line in (capture/'frames.jsonl').read_text(encoding='utf-8').splitlines():
    meta=json.loads(line)
    raw=read_pgm(capture/meta['file'])
    assert hashlib.sha256(raw).hexdigest()==meta['sha256']
    frames.append((meta, raw))

old_page=(HERE.parent/'entry-r8/comparison.html').read_text(encoding='utf-8')
old_overlays={}
old_binary={}
for article in re.findall(r'<article[^>]*>(.*?)</article>', old_page, re.S):
    match=re.search(r'第(\d+)帧',article)
    if match:
        images=re.findall(r'<img src="([^"]+)"',article)
        old_overlays[int(match.group(1))]=images[-1]
        old_binary[int(match.group(1))]=images[1]

def data_uri(pixels, channels=1):
    return 'data:image/png;base64,'+base64.b64encode(png(pixels,188,120,channels)).decode()

def render(raw, path, lines):
    image=bytearray(v for pixel in raw for v in (pixel,pixel,pixel))
    def draw(x,y,color):
        if 0<=x<188 and 0<=y<120:
            image[(y*188+x)*3:(y*188+x)*3+3]=bytes(color)
    if not path:
        if d.replay_entry_status()==0:
            for y in range(18,106):
                if lines[120+y]: draw(lines[120+y],y,(255,135,0))
        return bytes(image)
    for boundary, color, flag in [(2,(0,120,255),1), (4,(255,0,180),2)]:
        previous=None
        for point in path:
            if not point[6]&flag:
                previous=None
                continue
            xy=point[boundary:boundary+2]
            draw(*xy,color)
            if previous and max(abs(previous[0]-xy[0]),abs(previous[1]-xy[1]))<=8:
                for x,y in segment(*previous,*xy):draw(x,y,color)
            previous=xy
    for a,b in zip(path,path[1:]):
        for x,y in segment(*a[:2],*b[:2]):draw(x,y,(255,135,0))
    return bytes(image)

d.replay_ring_reset()
rows=[]
sequence_states=[]
sections=[]
visuals={}
first_invalid=None
for meta, raw in frames:
    index=meta['index']+1
    output=(C.c_float*5)()
    started=time.perf_counter()
    d.replay_ring_step((C.c_uint8*22560).from_buffer_copy(raw),meta['board_frame_tick'],output)
    elapsed=(time.perf_counter()-started)*1000
    current_status=d.replay_entry_status()
    sequence_meta=(C.c_int*10)();d.replay_entry_meta(sequence_meta)
    sequence_states.append(dict(frame=index,phase=int(output[1]),valid=int(output[3]),status=current_status,
                                reason=int(sequence_meta[4]),points=int(sequence_meta[0]),
                                boundary_source=d.replay_boundary_source(),goal=list(sequence_meta[1:3])))
    if current_status==3 and first_invalid is None:first_invalid=index
    if not 105<=index<=165:
        if 166<=index<=204:verify_path(raw,points(),thresholds(),f'frame{index}')
        continue
    path=points()
    values=(C.c_int*10)()
    d.replay_entry_meta(values)
    thr=thresholds()
    lines=(C.c_uint8*360)()
    d.replay_lines(lines)
    status=int(values[5])
    if path:
        verify_path(raw,path,thr,f'frame{index}')
        check(path[-1][0]>=180,f'frame{index} selects the right side branch')
    if 113<=index<=126:
        check(output[3]==1 and status==1,f'frame{index} current paired branch')
        paired=[p for p in path[-12:] if p[6]==3]
        check(len(paired)>=6,f'frame{index} at least six paired turn cross sections')
        check(all(p[3]<p[1]<p[5] for p in paired),f'frame{index} blue outer bank above and pink inner bank below')
    controls=(C.c_float*8)();d.replay_curve(controls)
    row=dict(band_thresholds=list(thr)[::15],boundary_source=d.replay_boundary_source(),boundary_support=d.replay_boundary_support(),estimated_width=round(d.replay_estimated_width(),2),curve=list(controls),frame=index, phase=int(output[1]), valid=int(output[3]), candidate=int(d.replay_island()),
             status=status, reason=int(values[4]), inferred=bool(values[3]),
             entry_error=round(output[2],2), command=round(output[0],2),
             goal=[int(values[1]),int(values[2])], points=path,
             recorded_tick=meta['board_frame_tick'], board_frame=meta['board_frame'],
             host_ms=round(elapsed,3), diagnostic_after_first_invalid=first_invalid is not None and index>=first_invalid)
    row['half_width_error_px']=None
    if path and 113<=index<=126:
        c=row['curve']
        normal_scale=abs(c[6]-c[4])/math.hypot(c[6]-c[4],c[7]-c[5])
        model=lambda y:71+32*(y-90)/119
        errors=[]
        for i,point in enumerate(path):
            if i<len(path)*0.75 or not point[6]&2: continue
            measured=math.hypot(point[4]-point[0],point[5]-point[1])
            target=row['estimated_width']*0.5*normal_scale*model(point[1])/model(c[7])
            errors.append(abs(measured-target))
        check(bool(errors) and sum(errors)/len(errors)<4,
              f'frame{index} observed inner-edge offset approaches half estimated width')
        row['half_width_error_px']=round(sum(errors)/len(errors),2)
    rows.append(row)
    binary=bytes(255 if raw[y*188+x]>thr[y] else 0 for y in range(120) for x in range(188))
    overlay=render(raw,path,lines)
    visuals[index]=(raw,binary,overlay)
    captions=['原始灰度','R8二值图（156帧起仅灰度）','R10当前二值图','R8旧路径（156帧起仅灰度）','R10实际控制曲线']
    images=[data_uri(raw),old_binary.get(index,data_uri(raw)),data_uri(binary),old_overlays.get(index,data_uri(raw)),data_uri(overlay,3)]
    state={0:'准备靠近',1:'当前双边支路',2:'当前外侧单边循迹',3:'路径无效',4:'当前内侧单边循迹',5:'编码器短时曲率保持'}[status]
    note=''
    if first_invalid is not None and index>=first_invalid:
        note=f'<p class="bad">在这组图像输入下，第{first_invalid}帧会触发原有视觉停车。后续帧仅继续作诊断回放，不能表示停车后自动恢复行驶。</p>'
    sections.append(f'<article id="f{index}"><h2>第{index}帧 · {state}</h2><p>phase={row["phase"]} candidate={row["candidate"]} valid={row["valid"]} · 路径{len(path)}点 · 开口估计宽度{row["estimated_width"]}px · 当前边界来源{row["boundary_source"]}／支持{row["boundary_support"]} · 目标{row["goal"]} · 转向输入{row["entry_error"]}px · 命令{row["command"]} · 原车端tick={row["recorded_tick"]}</p>'+note+'<div class="grid">'+''.join(f'<figure><figcaption>{title}</figcaption><img src="{source}" alt="第{index}帧{title}"></figure>' for title,source in zip(captions,images))+'</div></article>')
check(first_invalid==205,'complete recorded sequence first becomes invalid at frame205')
check(all(r['valid'] and r['points'] for r in sequence_states if 113<=r['frame']<=204),
      'recorded frames113 through204 retain current image paths without odometry injection')
check(all(r['valid'] and r['points'] for r in rows if 140<=r['frame']<=160),
      'every frame140 through160 has a current connected smooth path')


for index in [113,117,120,122,126]:
    raw=frames[index-1][1]
    for side in [-1,1]:
        image=raw if side<0 else bytes(v for y in range(120) for v in reversed(raw[y*188:(y+1)*188]))
        d.replay_ring_reset()
        buffer=(C.c_uint8*1120)();meta=(C.c_int*10)()
        count=d.direct_path((C.c_uint8*22560).from_buffer_copy(image),side,buffer,meta)
        path=[list(buffer[i*7:i*7+7]) for i in range(count)]
        verify_path(image,path,thresholds(),f'frame{index} mirror{side}')
        check((path[-1][0]>180 if side<0 else path[-1][0]<8),f'frame{index} mirror{side} chooses matching branch')
        check((path[count//2][0]-94)*(-side)>0,f'frame{index} mirror{side} lookahead sign')

for kind in ['uniform','near_loss','disconnected','ambiguous']:
    raw=bytearray(frames[116][1])
    d.replay_ring_reset();d.replay_guide(-1);d.replay_entry_mode(1)
    result=(C.c_int*4)()
    d.replay_frame((C.c_uint8*22560).from_buffer_copy(raw),result)
    check(bool(points()),kind+' primes current branch')
    if kind=='uniform':raw[:]=bytes([255])*22560
    elif kind=='near_loss':raw[80*188:106*188]=bytes([255])*(26*188)
    elif kind=='disconnected':raw[65*188:69*188]=bytes([20])*(4*188)
    else:
        for y in range(18,106):
            for x in range(170,188):
                raw[y*188+x]=255 if 26<=y<=38 or 54<=y<=69 else 20
    d.replay_frame((C.c_uint8*22560).from_buffer_copy(raw),result)
    if kind=='ambiguous':
        check(result[0]==1 and bool(points()) and d.replay_entry_status()==1,
              'a distant unrelated opening cannot override the current associated paired corridor')
    elif kind=='near_loss':
        check(result[0]==1 and bool(points()) and d.replay_entry_status() in [1,2],'white near field with a current measured corridor uses its continuous white path')
    else:
        check(result[0]==0 and not points() and d.replay_entry_status()==3,kind+' rejects locked branch without blind continuation')

# A current single contour refreshes geometry beyond the old three-frame cutoff.
raw=bytearray(frames[125][1])
d.replay_ring_reset();d.replay_guide(-1);d.replay_entry_mode(1)
result=(C.c_int*4)()
d.replay_frame((C.c_uint8*22560).from_buffer_copy(raw),result)
for y in range(24,106):
    for x in range(170,188): raw[y*188+x]=20 if y<36 else 255
for age in range(1,13):
    d.replay_frame((C.c_uint8*22560).from_buffer_copy(raw),result)
    check(result[0]==1 and d.replay_entry_status()==2 and bool(points()),f'outer-only frame{age} is refreshed by current contour evidence')
d.replay_entry_mode(0)
d.replay_frame((C.c_uint8*22560).from_buffer_copy(frames[116][1]),result)
check(d.replay_entry_status()==0 and not points(),'leaving guided phases removes ordered entrance path')
main=(PROJECT/'demo/user/src/main.c').read_text(encoding='utf-8')
wrapper=(PROJECT/'verification/vision-replay-wrapper.c').read_text(encoding='utf-8')
check('img_set_roundabout_entry(rb_guiding(&roundabout_state) ? roundabout_state.phase : 0)' in main and
      'img_set_roundabout_entry(rb_guiding(&integrated_ring) ? integrated_ring.phase : 0)' in wrapper,
      'production main and replay select the same entrance mode')
check('image[point->y][point->x] = 0' in main,'LCD renders the actual ordered control path')

# A narrow right-angle L is grid-connected, but it cannot accept a smooth
# entrance curve from the car's forward tangent. It must be rejected explicitly.
raw=bytearray([20]*22560)
for y in range(45,106):
    for x in range(88,101):raw[y*188+x]=220
for y in range(45,58):
    for x in range(94,188):raw[y*188+x]=220
d.replay_ring_reset()
buffer=(C.c_uint8*1120)();meta=(C.c_int*10)()
count=d.direct_path((C.c_uint8*22560).from_buffer_copy(raw),-1,buffer,meta)
check(count==0 and meta[4]==6,'grid-connected sharp corner rejects an infeasible smooth control curve')

# ENTER to ORBIT must retain the selected path and learned width.
d.replay_ring_reset()
for meta,raw in frames[:126]:
    output=(C.c_float*5)()
    d.replay_ring_step((C.c_uint8*22560).from_buffer_copy(raw),meta['board_frame_tick'],output)
d.replay_ring_odometry.argtypes=[C.c_float,C.c_float]
d.replay_ring_odometry(0.2,45.0);d.replay_ring_tick(30,30)
check(d.replay_ring_phase()==2,'encoder yaw transition enters ORBIT')
meta,raw=frames[126]
d.replay_ring_step((C.c_uint8*22560).from_buffer_copy(raw),meta['board_frame_tick'],output)
check(output[3]==1 and d.replay_entry_status()==2 and bool(points()),
      'ORBIT retains the selected current outer contour and learned width')

# Encoder-assisted blind bridge. These are synthetic motion samples with real
# camera frames; no encoder sequence exists for the hand-pushed recording.
d.replay_set_odometry.argtypes=[C.c_float,C.c_float,C.c_uint32,C.c_int]
real=frames[116][1]
blank=bytes([255])*22560

def direct_frame(raw,distance,yaw,tick,valid=1):
    d.replay_set_odometry(distance,yaw,tick,valid)
    output=(C.c_int*4)()
    d.replay_frame((C.c_uint8*22560).from_buffer_copy(raw),output)
    return list(output)

def prime_bridge():
    d.replay_ring_reset();d.replay_guide(-1);d.replay_entry_mode(1)
    for i in range(5):
        distance=i*0.005
        output=direct_frame(real,distance,distance*2*180/math.pi,i*2)
        check(output[0]==1 and bool(points()),f'bridge prime{i} uses real current visual path')

prime_bridge()
for tick in [10,12,14,16]:
    distance=tick*0.0025
    output=direct_frame(blank,distance,distance*2*180/math.pi,tick)
    check(output[0]==1 and d.replay_entry_status()==5 and d.replay_bridge_active()==1 and not points(),
          f'blind tick{tick} holds stable encoder curvature with explicit predicted status')
output=direct_frame(blank,0.045,0.045*2*180/math.pi,18)
check(output[0]==0 and d.replay_entry_status()==3,'blind bridge expires100ms after last measured path')
prime_bridge()
output=direct_frame(blank,0.065,0.065*2*180/math.pi,10)
check(output[0]==0,'encoder bridge stops at40mm prediction distance')
prime_bridge()
output=direct_frame(blank,0.025,0.025*2*180/math.pi+4,10)
check(output[0]==0,'encoder bridge stops at more than3deg heading prediction error')
prime_bridge()
output=direct_frame(blank,0.025,0.025*2*180/math.pi,10,valid=0)
check(output[0]==0,'missing encoder validity cannot authorize blind motion')
prime_bridge()
blocked=bytearray(blank);blocked[96*188:103*188]=bytes([20])*(7*188)
output=direct_frame(blocked,0.025,0.025*2*180/math.pi,10)
check(output[0]==0,'current near black obstruction blocks an odometry bridge')
prime_bridge()
ambiguous=bytearray(real)
for y in range(18,106):
    for x in range(170,188):ambiguous[y*188+x]=255 if 26<=y<=38 or 54<=y<=69 else 20
output=direct_frame(ambiguous,0.025,0.025*2*180/math.pi,10)
check(output[0]==1 and not d.replay_bridge_active() and bool(points()),
      'an unrelated opening is resolved by current visual association without encoder hold')
d.replay_ring_reset();d.replay_guide(-1);d.replay_entry_mode(1)
for i in range(5):direct_frame(real,0,0,i*2)
output=direct_frame(blank,0,0,10)
check(output[0]==0,'stationary views cannot establish a motion curvature model')

prime_bridge()
output=direct_frame(blank,0.025,0.025*2*180/math.pi,10)
check(output[0]==1 and d.replay_bridge_active(),'short whiteout starts the bounded bridge')
output=direct_frame(real,0.030,0.030*2*180/math.pi,12)
check(output[0]==1 and d.replay_entry_status()==1 and not d.replay_bridge_active() and bool(points()),
      'current paired contour takes control back from encoder hold')
output=direct_frame(blank,0.035,0.035*2*180/math.pi,14)
check(output[0]==1 and d.replay_bridge_active(),'only a newly measured visual path renews the reference')

prime_bridge()
d.replay_guide(1)
output=direct_frame(blank,0.025,0.025*2*180/math.pi,10)
check(output[0]==0 and not d.replay_bridge_active(),
      'changing selected side clears the previous branch motion model')

# Regression coverage for isolated threshold collapse, candidate selection and
# transactional geometry. The planner is exercised on native current pixels.
def cache_state():
    result=(C.c_float*11)();d.replay_cache(result)
    return list(result)

def prime_until(index):
    d.replay_ring_reset()
    for meta,raw in frames[:index]:
        result=(C.c_float*5)()
        d.replay_ring_step((C.c_uint8*22560).from_buffer_copy(raw),meta['board_frame_tick'],result)
    check(result[3]==1,f'frame{index} primes verified current geometry')

def native_plan(raw,thr=None,reset=False,side=-1):
    if thr is None:thr=bytes([100])*120
    return d.replay_binary_plan((C.c_uint8*22560).from_buffer_copy(raw),
                               (C.c_uint8*120).from_buffer_copy(thr),side,int(reset))

for index in range(147,156):
    prime_until(index)
    thr=thresholds()
    check(thr[30]>=min(thr[15],thr[45])-8,
          f'frame{index} local bright population or agreeing neighbors prevents threshold collapse')
    check(all(visuals[index][1][y*188+182]==0 for y in range(30,38)),
          f'frame{index} gray background no longer forms a white horizontal stripe')

# Preserve local thresholds for an actual shaded road with a supported bright
# road population, even when its neighbors have higher brightness.
shadow=bytearray([20]*22560)
for y in range(106):
    for x in range(40,149):shadow[y*188+x]=150 if 30<=y<=44 else 220
for y in range(30,45):
    for x in list(range(40))+list(range(149,188)):shadow[y*188+x]=14
d.replay_ring_reset()
output=(C.c_int*4)()
d.replay_frame((C.c_uint8*22560).from_buffer_copy(shadow),output)
thr=thresholds()
check(thr[37]<150 and thr[37]<thr[22]-24 and thr[37]<thr[52]-24,
      'a genuinely shaded road keeps its own lower supported threshold')
check(shadow[37*188+94]>thr[37] and shadow[37*188+20]<=thr[37],
      'shadowed road stays white and its dark background stays black')

for width in [10,24]:
    narrow=bytearray([20]*22560)
    for y in range(106):
        left=94-width//2 if 30<=y<=44 else 40
        right=left+width if 30<=y<=44 else 149
        for x in range(left,right):narrow[y*188+x]=150 if 30<=y<=44 else 220
    d.replay_ring_reset();d.replay_frame((C.c_uint8*22560).from_buffer_copy(narrow),output)
    thr=thresholds()
    check(thr[37]<150 and narrow[37*188+94]>thr[37],
          f'a shaded road only{width}px wide retains a local bright-population threshold')

for gain,offset in [(0.5,0),(0.7,10),(0.85,0)]:
    raw=bytes(round(v*gain+offset) for v in frames[146][1])
    d.replay_ring_reset();d.replay_frame((C.c_uint8*22560).from_buffer_copy(raw),output)
    thr=thresholds()
    check(thr[30]==min(thr[15],thr[45]) and raw[37*188+182]<=thr[37] and raw[60*188+182]>thr[60],
          f'threshold correction remains exposure-relative gain{gain} offset{offset}')

for side in [-1,1]:
    paired=visuals[117][1]
    inner_only=bytearray(paired)
    for y in range(18,31):
        for x in range(135,188):inner_only[y*188+x]=255
    if side>0:
        paired=bytes(v for y in range(120) for v in reversed(paired[y*188:(y+1)*188]))
        inner_only=bytearray(v for y in range(120) for v in reversed(inner_only[y*188:(y+1)*188]))
    check(native_plan(paired,reset=True,side=side)==1,f'mirror{side} paired corridor seeds accepted history')
    check(native_plan(inner_only,side=side)==1 and d.replay_boundary_source()==3,
          f'mirror{side} currently visible inner contour supports a complete smooth path')
    check(native_plan(paired,side=side)==1 and d.replay_boundary_source()==1,
          f'mirror{side} current paired observation resumes after inner-only tracking')

prime_until(146)
before=cache_state()
wrong_thresholds=bytearray(thresholds());wrong_thresholds[30:45]=bytes([178])*15
check(native_plan(frames[146][1],wrong_thresholds)==1 and d.replay_boundary_source()==2,
      'current outer corridor beats a disconnected paired gray stripe even with the old bad threshold')
check(points()[-1][1]>=70 and cache_state()[0]==before[0],
      'the false paired stripe cannot replace the learned track width')

prime_until(125)
before=cache_state()
blocked=bytearray(visuals[126][1]);blocked[65*188:70*188]=bytes(5*188)
check(native_plan(blocked)==0,'candidate with changed width and disconnected near corridor fails')
check(cache_state()==before,'failed connectivity changes no cached width reference edges goal or age')
check(native_plan(visuals[126][1])==1 and cache_state()[0]>before[0],
      'the same wider candidate updates width only after a complete path succeeds')

# Two paired candidates with equal prior association. Only current feasibility
# can distinguish the first scene. The second has two feasible routes and must
# remain ambiguous.
for side in [-1,1]:
    for both_connected in [False,True]:
        raw=bytearray([20]*22560)
        for y in range(25 if both_connected else 45,106):
            for x in range(70,170 if both_connected else 161):raw[y*188+x]=220
        for y in list(range(30,46))+list(range(60,76)):
            for x in range(170 if y<46 else 161,188):raw[y*188+x]=220
        if side>0:raw=bytearray(v for y in range(120) for v in reversed(raw[y*188:(y+1)*188]))
        valid=native_plan(raw,reset=True,side=side)
        if both_connected:
            meta=(C.c_int*10)();d.replay_entry_meta(meta)
            check(valid==0 and meta[4]==7 and cache_state()[0]==0,
                  f'mirror{side} equally associated feasible openings reject without width mutation')
        else:
            check(valid==1 and points()[-1][1]>=60 and cache_state()[2]>=60,
                  f'mirror{side} reachable candidate wins over a disconnected paired opening')

# A grid-connected sharp corner fails the curve check and cannot seed history.
raw=bytearray([20]*22560)
for y in range(45,106):
    for x in range(88,101):raw[y*188+x]=220
for y in range(45,58):
    for x in range(94,188):raw[y*188+x]=220
check(native_plan(raw,reset=True)==0 and cache_state()[0]==0,
      'failed smooth-curve validation cannot seed learned width')

report={'version':'C11LAP30R10','checks':checks,'first_invalid':first_invalid,
        'hardware_accessed':False,'flashed':False,'real_track_tested':False,
        'capture':'ring-entry-20261106/manual-01','time_source':'recorded board_frame_tick',
        'sequence_states':sequence_states,'frames':rows}
(HERE/'results.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
page='''<!doctype html><html lang="zh-CN"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>R10入口边界切换回放</title><style>body{font:16px/1.6 Arial,"Microsoft YaHei",sans-serif;margin:24px auto;padding:0 16px;max-width:1600px;background:#eef2f5;color:#17212b}article{padding:18px;margin:18px 0;border:1px solid #cbd5df;border-radius:9px;background:white}.grid{display:grid;grid-template-columns:repeat(5,1fr);gap:12px}figure{margin:0}img{width:100%;image-rendering:pixelated}figcaption{font-weight:bold}.bad{color:#a32020}a{color:#175ca5}@media(max-width:950px){.grid{grid-template-columns:repeat(2,1fr)}}@media(max-width:550px){.grid{grid-template-columns:1fr}}</style><h1>R10 环岛入口曲线</h1><p>蓝色为沿行驶方向的左边界，粉色为右边界，橙色为控制器实际使用的曲线。曲线起点切线沿车辆前方，末端切线来自当前可见弯道边界方向。从可靠双边段估算宽度，按当前可见边界法线向道路内部偏移约半个赛道宽度，并检验每段路径的当前白色区域。第二列为R8二值图，第三列为R10二值图，第四列保留R8旧路径供对照。R8旧图只覆盖到第155帧，此后相应列显示原始灰度。</p><p>第113～126帧使用双边观测更新宽度。第140～160帧连续有效。第147帧的异常分带阈值由178修正为239，目标由伪横带的第37行移至下方道路的第75行。候选同时比较当前位置、宽度与通路可行性，完整平滑路径验证通过后才更新宽度等历史。整组348帧诊断回放首次无效在第205帧，本页展示第105～165帧。编码器曲率保持要求连续稳定的运动样本，最多100ms、40mm，航向偏差最多3°，近场黑色阻挡、路径不连通和多支路歧义都会拒绝续行。预测状态不绘制伪造的当前图像中线。本记录缺少同步编码器数据，续行只由独立合成运动测试检验。图像源为电机断电手推采集，车速和舵机几何仍需现场验证。回放使用记录中的车端帧时间，R8旧图沿用此前回放供比较。</p><nav>跳转 <a href="#f116">116帧</a> · <a href="#f117">117帧</a> · <a href="#f122">122帧</a> · <a href="#f126">126帧</a> · <a href="#f127">127帧</a> · <a href="#f130">130帧</a> · <a href="#f137">137帧</a> · <a href="#f145">145帧</a> · <a href="#f147">147帧</a> · <a href="#f148">148帧</a> · <a href="#f150">150帧</a> · <a href="#f160">160帧</a></nav>'''+''.join(sections)+'</html>'
(HERE/'comparison.html').write_text(page,encoding='utf-8')
assert page.count('<img ')==61*5
contact=[]
for index in [117,130,137,145,147,148,150,155,160]:
    gray,binary,overlay=visuals[index]
    panels=[bytes(v for p in gray for v in (p,p,p)),bytes(v for p in binary for v in (p,p,p)),overlay]
    for y in range(120):
        contact.append(b''.join(p[y*188*3:(y+1)*188*3] for p in panels))
(HERE/'contact.png').write_bytes(png(b''.join(contact),564,120*9,3))
for index in [116,117,126,130,137,145,147,148,150,155,160]:
    gray,binary,overlay=visuals[index]
    (HERE/f'frame-{index}.png').write_bytes(png(overlay,188,120,3))
# A focused, pixel-preserving triptych for the user's marked frame.
gray,binary,overlay=visuals[116]
panels=[bytes(v for p in gray for v in (p,p,p)),bytes(v for p in binary for v in (p,p,p)),overlay]
focus=[]
for y in range(120):
    row=b''.join(p[y*188*3:(y+1)*188*3] for p in panels)
    expanded=b''.join(row[x:x+3]*2 for x in range(0,len(row),3))
    focus.extend([expanded,expanded])
(HERE/'focus-116.png').write_bytes(png(b''.join(focus),1128,240,3))
# Frame147 focus shows raw, previous binary, current binary and actual curve.
gray,binary,overlay=visuals[147]
# The original R8 binary can also be reproduced from this frame's known band
# thresholds; all bands except30..44 match R10 for this frame.
d.replay_ring_reset()
for meta,raw in frames[:147]:
    output=(C.c_float*5)()
    d.replay_ring_step((C.c_uint8*22560).from_buffer_copy(raw),meta['board_frame_tick'],output)
old_thr=bytearray(thresholds());old_thr[30:45]=bytes([178])*15
old_pixels=bytes(255 if gray[y*188+x]>old_thr[y] else 0 for y in range(120) for x in range(188))
assert png(old_pixels,188,120)==base64.b64decode(old_binary[147].split(',',1)[1])
panels=[bytes(v for p in gray for v in (p,p,p)),
        bytes(v for p in old_pixels for v in (p,p,p)),
        bytes(v for p in binary for v in (p,p,p)),overlay]
focus=[]
for y in range(120):
    row=b''.join(p[y*188*3:(y+1)*188*3] for p in panels)
    expanded=b''.join(row[x:x+3]*2 for x in range(0,len(row),3))
    focus.extend([expanded,expanded])
(HERE/'focus-147.png').write_bytes(png(b''.join(focus),1504,240,3))

print('PASS',len(checks),'ordered-path checks. First invalid:',first_invalid)
print('Selected results:',[(r['frame'],r['status'],len(r['points']),r['entry_error'],r['command']) for r in rows if r['frame'] in [117,122,126,127,128,129,130]])
