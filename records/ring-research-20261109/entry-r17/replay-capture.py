"""Replay immutable captured pixels through R16 and current native production C."""
from pathlib import Path
import argparse
import base64
import ctypes as C
import hashlib
import json
import math
import os
import struct
import subprocess
import zipfile
import zlib

WORK = Path(__file__).resolve().parent
ROOT = WORK.parents[2]
PROJECT_REL = Path('deliveries/lap30-ring-20261107/project')
PROJECT = ROOT / PROJECT_REL
BASELINE_REF = '0f28f86'
parser = argparse.ArgumentParser()
parser.add_argument('capture_zip', type=Path)
parser.add_argument('--zig', default=os.environ.get('ZIG'))
args = parser.parse_args()
if not args.zig:
    parser.error('Provide --zig or set ZIG')


def sha(data):
    return hashlib.sha256(data).hexdigest()


def png(gray):
    def chunk(kind, data):
        return struct.pack('>I', len(data)) + kind + data + struct.pack('>I', zlib.crc32(kind + data) & 0xffffffff)
    rows = b''.join(b'\0' + gray[y*188:(y+1)*188] for y in range(120))
    payload = b'\x89PNG\r\n\x1a\n'
    payload += chunk(b'IHDR', struct.pack('>IIBBBBB', 188, 120, 8, 0, 0, 0, 0))
    payload += chunk(b'IDAT', zlib.compress(rows)) + chunk(b'IEND', b'')
    return 'data:image/png;base64,' + base64.b64encode(payload).decode()


zip_bytes = args.capture_zip.read_bytes()
with zipfile.ZipFile(args.capture_zip) as archive:
    assert archive.testzip() is None
    metadata = [json.loads(line) for line in archive.read('manual-02/frames.jsonl').decode('utf-8').splitlines() if line.strip()]
    pixels = []
    for index, meta in enumerate(metadata):
        assert meta['index'] == index
        data = archive.read('manual-02/' + meta['file'].replace('\\', '/'))
        assert data[:15] == b'P5\n188 120\n255\n' and len(data) == 22575
        gray = data[15:]
        assert sha(gray) == meta['sha256']
        assert f'{zlib.crc32(gray)&0xffffffff:08x}' == meta['crc32']
        pixels.append(gray)
assert len(pixels) == 312

# Materialize only the immutable R16 source files needed for this replay.
baseline = WORK / 'baseline'
paths = subprocess.check_output(['git', 'ls-tree', '-r', '--name-only', BASELINE_REF,
    str(PROJECT_REL / 'demo/user/inc').replace('\\', '/')], cwd=ROOT).decode().splitlines()
paths += [str(PROJECT_REL / name).replace('\\', '/') for name in
          ['demo/user/src/img_process.c', 'demo/user/src/steering_control.c',
           'verification/vision-replay-wrapper.c']]
for path in paths:
    target = baseline / Path(path).relative_to(PROJECT_REL)
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_bytes(subprocess.check_output(['git', 'show', f'{BASELINE_REF}:{path}'], cwd=ROOT))


def compile_native(project, name):
    library = WORK / f'{name}.dll'
    command = [args.zig, 'cc', '-shared', '-O2', '-Wall', '-Wextra', '-Werror']
    if project == PROJECT: command += ['-DREPLAY_CURRENT']
    for directory in [PROJECT/'verification/vision-stubs', project/'demo/user/inc',
                      project/'demo/user/src', project/'verification']:
        command += ['-I', str(directory)]
    command += [str(WORK/'ordered-replay.c'), '-o', str(library)]
    subprocess.run(command, check=True)
    return C.CDLL(str(library))


def path_max_angle(path):
    maximum = 0.0
    for a, b, c in zip(path, path[1:], path[2:]):
        ax, ay = b[0]-a[0], b[1]-a[1]
        bx, by = c[0]-b[0], c[1]-b[1]
        assert (ax or ay) and (bx or by)
        maximum = max(maximum, math.degrees(math.atan2(abs(ax*by-ay*bx), ax*bx+ay*by)))
    return maximum


def replay(native, version):
    native.replay_ring_reset()
    rows = []
    for index, (meta, gray) in enumerate(zip(metadata, pixels), 1):
        excluded = 86 <= index <= 110 or index >= 265
        if index == 111: native.replay_ring_reset()
        out = (C.c_float*5)()
        native.replay_ring_step((C.c_uint8*22560).from_buffer_copy(gray), meta['board_frame_tick'], out)
        buffer = (C.c_uint8*(160*7))()
        count = native.replay_entry_path(buffer)
        assert 0 <= count <= 160
        path = [list(buffer[i*7:i*7+7]) for i in range(count)]
        thresholds = (C.c_uint8*120)()
        lines = (C.c_uint8*360)()
        info = (C.c_int*3)()
        native.replay_thresholds(thresholds)
        native.replay_lines(lines)
        native.replay_path_meta(info)
        island = (C.c_uint8*361)()
        planner = (C.c_int*6)()
        if version == 'R17':
            native.replay_current_island(island)
            native.replay_planner_state(planner)
        curve = (C.c_float*8)()
        if version == 'R17': native.replay_curve_control(curve)
        maximum = path_max_angle(path)
        if version == 'R17' and path:
            assert maximum <= 40.0, (version, index, maximum)
            for point in path:
                assert gray[point[1]*188+point[0]] > thresholds[point[1]]
            for a, b in zip(path, path[1:]):
                dx, dy = b[0]-a[0], b[1]-a[1]
                steps = max(abs(dx), abs(dy))
                for k in range(steps+1):
                    x, y = a[0]+int(dx*k/steps), a[1]+int(dy*k/steps)
                    assert gray[y*188+x] > thresholds[y], (index, x, y)
        rows.append(dict(angle_checked=bool(path), curve=list(curve), island=list(island), planner=list(planner), excluded=excluded, index=index, valid=int(out[3]), phase=int(out[1]), error=float(out[2]),
                         command=float(out[0]), failure=info[0], near_valid=info[1],
                         candidate=native.replay_island(), status=native.replay_entry_status(), boundary_source=info[2],
                         path=path, lines=list(lines), max_angle_deg=maximum))
    rows_scoped = [r for r in rows if not r['excluded']]
    first_entry = next((r['index'] for r in rows if r['phase'] == 1), None)
    summary = dict(version=version, total=len(rows_scoped), valid=sum(r['valid'] for r in rows_scoped),
                   path_frames=sum(bool(r['path']) for r in rows_scoped), first_entry=first_entry,
                   first_invalid_after_entry=next((r['index'] for r in rows if first_entry and
                       r['index'] >= first_entry and not r['excluded'] and not r['valid']), None),
                   max_angle_deg=max(r['max_angle_deg'] for r in rows_scoped),
                   angle_violating_frames=[r['index'] for r in rows_scoped if r['path'] and r['max_angle_deg'] > 40.0])
    return rows, summary


old, old_summary = replay(compile_native(baseline, 'r16'), 'R16')
new, new_summary = replay(compile_native(PROJECT, 'r17'), 'R17')
source_names = ['demo/user/src/img_process.c', 'demo/user/inc/ring_entry_path.h',
                'demo/user/src/steering_control.c', 'demo/user/inc/steering_control.h',
                'demo/user/src/main.c', 'verification/vision-replay-wrapper.c']
report = dict(version='C11LAP30R17', capture_zip=str(args.capture_zip.resolve()),
              zip_sha256=sha(zip_bytes), frame_hashes_and_crc_verified=True,
              assumption='Replay assumes running, no synchronized wheel odometry; later frames after first invalid are diagnostic.',
              source_sha256={name:sha((PROJECT/name).read_bytes()) for name in source_names},
              planner_fields=['cached_width_px','island_hold_age','raw_island_top','preview_error','goal_x','goal_y'],
              island_array_layout={'left':[0,120],'right':[120,240],'mask':[240,360],'fresh':360},
              summaries=[old_summary,new_summary],
              included_ranges=[[1,85],[111,264]], excluded_ranges=[[86,110],[265,312]], history_reset_frames=[1,111],
              improved_valid_frames=[i+1 for i,(a,b) in enumerate(zip(old,new)) if not a['excluded'] and not a['valid'] and b['valid']],
              regressed_valid_frames=[i+1 for i,(a,b) in enumerate(zip(old,new)) if not a['excluded'] and a['valid'] and not b['valid']],
              frames=[dict(index=i+1, board_frame=m['board_frame'], tick=m['board_frame_tick'],
                           sha256=m['sha256'], r16=a, r17=b) for i,(m,a,b) in enumerate(zip(metadata,old,new))])
(WORK/'results.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
display = [dict(index=i+1, gray=png(gray), old=a, new=b) for i,(gray,a,b) in enumerate(zip(pixels,old,new))]
template = '''<!doctype html><html lang="zh-CN"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>R16与R17环岛全段对照</title><style>
body{font:16px/1.6 system-ui,"Microsoft YaHei";margin:24px;background:#f3f5f7;color:#223}
main{max-width:1400px;margin:auto}.panels{display:grid;grid-template-columns:repeat(3,1fr);gap:16px}
section{background:white;padding:14px;border:1px solid #ccd3da}canvas{width:100%;image-rendering:pixelated;background:#111}
button,input{font:inherit}#seek{width:70%}.warn{color:#a32}p{margin:8px 0}@media(max-width:800px){.panels{grid-template-columns:1fr}}
</style><main><h1>环岛全段路线 · 平滑性与相邻段夹角≤40°</h1>
<p>全312帧保留展示，验收范围1～85与111～264共239帧。排除86～110及265～312，111帧重置历史。灰度像素已核验SHA-256与CRC。</p>
<p>回放假定车辆运行。记录缺少同步轮速，回放无同步轮速，不能验证环内和出口运动阶段；图像中的出口场景与真实控制器阶段分别核验。首次无效帧之后的结果用于逐帧诊断。</p>
<p><button id="prev">上一帧</button> <button id="play">播放</button> <button id="next">下一帧</button> <input id="jump" type="number" min="1" max="312" value="1">帧</p>
<input id="seek" type="range" min="0" max="311" value="0"><strong id="counter"></strong>
<div class="panels"><section><h2>原始灰度</h2><canvas id="gray" width="188" height="120"></canvas></section>
<section><h2>R16路线</h2><canvas id="old" width="188" height="120"></canvas><p id="oldinfo"></p></section>
<section><h2>R17 · 全段平滑拟合与当前边界</h2><canvas id="new" width="188" height="120"></canvas><p id="newinfo"></p></section></div>
<p>橙线连接算法实际输出点。有序入口路线检查相邻段≤40°及逐像素白色连接，普通逐行中线单独显示。没有有序入口路径时，灰色点显示普通逐行中线。无效帧显示状态。</p>
<pre id="summary"></pre></main><script id="data" type="application/json">__DATA__</script><script>
const data=JSON.parse(document.getElementById('data').textContent);let index=0,timer=null;window.replayReady=false;
const el=id=>document.getElementById(id);let token=0;
function overlay(ctx,m){if(!m.valid)return;if(m.path.length){ctx.strokeStyle='#ff8c00';ctx.lineWidth=1;ctx.beginPath();m.path.forEach((p,i)=>{if(i)ctx.lineTo(p[0]+.5,p[1]+.5);else ctx.moveTo(p[0]+.5,p[1]+.5)});ctx.stroke();ctx.fillStyle='#ffcf60';m.path.forEach(p=>ctx.fillRect(p[0],p[1],1,1));}else{ctx.fillStyle='#aaa';for(let y=18;y<=105;y++){let x=m.lines[120+y];if(x)ctx.fillRect(x,y,1,1)}}}
async function render(){const turn=++token,f=data.frames[index];el('seek').value=index;el('jump').value=index+1;el('counter').textContent=` 第${index+1}/312帧${index>=214&&index<=234?" · 用户标注即将出环":""}`;
await new Promise((resolve,reject)=>{const im=new Image;im.onload=()=>{if(turn===token){for(const id of ['gray','old','new']){const ctx=el(id).getContext('2d');ctx.clearRect(0,0,188,120);ctx.drawImage(im,0,0);if(id!=='gray')overlay(ctx,f[id])}}resolve()};im.onerror=reject;im.src=f.gray});
if(turn!==token)return;for(const id of ['old','new']){const m=f[id];el(id+'info').textContent=`${m.valid?(m.path.length?'有序路线有效':'普通逐行识别有效，尚无有序路线'):'视觉证据不足，路径拒绝'} · ${m.excluded?'搬车排除':'赛道纳入'} · phase=${m.phase} · status=${m.status} · failure=${m.failure} · 路径${m.path.length}点 · 最大夹角${m.path.length?m.max_angle_deg.toFixed(3)+'°':'未计算'} · 模拟舵机${m.command.toFixed(2)}`;el(id+'info').className=m.valid?'':'warn'}window.replayReady=true;window.renderedFrame=index+1;}
function seek(n){index=Math.max(0,Math.min(311,n));return render()}
el('prev').onclick=()=>seek(index-1);el('next').onclick=()=>seek(index+1);el('seek').oninput=e=>seek(+e.target.value);el('jump').onchange=e=>seek(+e.target.value-1);
el('play').onclick=()=>{if(timer){clearInterval(timer);timer=null;el('play').textContent='播放'}else{el('play').textContent='暂停';timer=setInterval(()=>{if(index===311){clearInterval(timer);timer=null;el('play').textContent='播放'}else seek(index+1)},300)}};
el('summary').textContent=JSON.stringify(data.summaries,null,2);window.seekFrame=n=>seek(n-1);render();
</script></html>'''
payload=json.dumps(dict(frames=display,summaries=report['summaries']),ensure_ascii=False,separators=(',',':')).replace('</','<\\/')
(WORK/'comparison.html').write_text(template.replace('__DATA__',payload),encoding='utf-8')
print(json.dumps({k:report[k] for k in ['summaries','improved_valid_frames','regressed_valid_frames']},ensure_ascii=False,indent=2))
