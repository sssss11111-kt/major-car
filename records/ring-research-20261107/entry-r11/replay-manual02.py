from pathlib import Path
import argparse
import base64
import ctypes as C
import hashlib
import json
import os
import re
import shutil
import struct
import subprocess
import zlib

ROOT = Path(__file__).resolve().parents[3]
INPUT = ROOT / 'records/ring-entry-current-20261107/图像采集-manual-02-20261107'
CAPTURE = INPUT / 'manual-02'
WORK = Path(__file__).resolve().parent
PROJECT = ROOT / 'deliveries/lap30-ring-20261107/project'
parser = argparse.ArgumentParser(description='Replay a capture range with native R11, retaining preceding state.')
parser.add_argument('--start', type=int, default=40)
parser.add_argument('--end', type=int, default=60)
args = parser.parse_args()
if not 1 <= args.start <= args.end <= 312:
    parser.error('Frame range must satisfy 1 <= start <= end <= 312')
OUTPUT = INPUT / f'R11补右边线对比_{args.start}-{args.end}帧.html'
initial_frame = 47 if args.start <= 47 <= args.end else args.start
ZIG = r'C:\Users\liyuy\AppData\Local\uv\cache\archive-v0\lr7GmYIgFrzqabj5\ziglang\zig.exe'


def png(gray):
    def chunk(kind, data):
        return struct.pack('>I', len(data)) + kind + data + struct.pack('>I', zlib.crc32(kind + data) & 0xffffffff)
    rows = b''.join(b'\0' + gray[y*188:(y+1)*188] for y in range(120))
    data = b'\x89PNG\r\n\x1a\n' + chunk(b'IHDR', struct.pack('>IIBBBBB', 188, 120, 8, 0, 0, 0, 0))
    return 'data:image/png;base64,' + base64.b64encode(data + chunk(b'IDAT', zlib.compress(rows)) + chunk(b'IEND', b'')).decode()


source_files = ['demo/user/src/img_process.c', 'demo/user/inc/ring_entry_path.h', 'demo/user/src/main.c']
adapter = WORK / 'ordered-replay.c'
command = [ZIG, 'cc', '-shared', '-O2', '-Wall', '-Wextra', '-Werror']
for directory in ['verification/vision-stubs', 'demo/user/inc', 'demo/user/src', 'verification']:
    command += ['-I', str(PROJECT/directory)]
command += [str(adapter), '-o', str(WORK/'manual02-native.dll')]
subprocess.run(command, check=True)
d = C.CDLL(str(WORK/'manual02-native.dll'))

metadata = [json.loads(line) for line in (CAPTURE/'frames.jsonl').read_text(encoding='utf-8').splitlines() if line.strip()]
assert len(metadata) == 312
pixels = []
for index, meta in enumerate(metadata):
    assert meta['index'] == index
    relative = Path(meta['file'])
    assert not relative.is_absolute() and '..' not in relative.parts
    raw = (CAPTURE/relative).read_bytes()
    assert raw[:15] == b'P5\n188 120\n255\n' and len(raw) == 22575
    gray = raw[15:]
    assert hashlib.sha256(gray).hexdigest() == meta['sha256']
    assert f'{zlib.crc32(gray)&0xffffffff:08x}' == meta['crc32']
    pixels.append(gray)

mode_outputs = []
for mode in ['integrated']:
    d.replay_ring_reset()
    output_rows = []
    for meta, gray in zip(metadata, pixels):
        native = (C.c_uint8*22560).from_buffer_copy(gray)
        if mode == 'integrated':
            result = (C.c_float*5)()
            d.replay_ring_step(native, meta['board_frame_tick'], result)
            valid, phase, error, command_value = int(result[3]), int(result[1]), round(result[2], 2), round(result[0], 2)
        else:
            result = (C.c_int*4)()
            d.replay_frame(native, result)
            valid, phase, error, command_value = int(result[0]), 0, int(result[1]), None
        thresholds = (C.c_uint8*120)()
        lines = (C.c_uint8*360)()
        info = (C.c_int*10)()
        buffer = (C.c_uint8*1120)()
        d.replay_thresholds(thresholds)
        d.replay_lines(lines)
        d.replay_entry_meta(info)
        count = d.replay_entry_path(buffer)
        path = [list(buffer[i*7:i*7+7]) for i in range(count)]
        binary = bytes(255 if gray[y*188+x] > thresholds[y] else 0 for y in range(120) for x in range(188))
        if path and valid:
            assert path[0][:2] == [94, 100]
            for x,y,*_ in path:
                assert gray[y*188+x] > thresholds[y]
        side = (C.c_uint8*120)(); edge = (C.c_uint8*120)()
        d.replay_approach(side,edge)
        output_rows.append(dict(bridge_side=list(side), bridge_edge=list(edge), valid=valid, phase=phase, error=error, command=command_value,
            candidate=d.replay_island(), evidence=d.replay_evidence(), near_valid=int(info[8]),
            entry_status=int(info[5]), failure=int(info[4]), boundary_source=d.replay_boundary_source(),
            boundary_support=d.replay_boundary_support(), path=path, lines=list(lines), binary=png(binary)))
    mode_outputs.append(output_rows)

old_html=(INPUT/'R9中线播放.html').read_text(encoding='utf-8')
old_data=json.loads(re.search(r'<script id="data" type="application/json">(.*?)</script>',old_html,re.S).group(1))
baseline = json.loads((INPUT/'R10中线计算结果_1-150帧.json').read_text(encoding='utf-8'))
old_modes = []
for old_frame, result in zip(old_data['frames'], baseline['frame_results']):
    old_modes.append(dict(result['modes'][0], binary=old_frame['modes'][0]['binary']))
mode_outputs.append(old_modes)
frames = []
start = metadata[0]['host_monotonic']
for i, (meta, gray) in enumerate(zip(metadata, pixels)):
    frames.append(dict(index=i+1, board_frame=meta['board_frame'], board_tick=meta['board_frame_tick'],
        t=round(meta['host_monotonic']-start, 3), sha256=meta['sha256'], gray=png(gray),
        modes=[mode_outputs[0][i], mode_outputs[1][i]]))

summaries = []
for mode, values in zip(['R11连续补线', 'R10修改前'], mode_outputs):
    first_entry = next((i+1 for i,v in enumerate(values) if v['phase']==1), None)
    first_bad = next((i+1 for i,v in enumerate(values) if first_entry is not None and i+1>=first_entry and not v['valid']), None)
    summaries.append(dict(name=mode, total=len(values), valid=sum(v['valid'] for v in values),
        entry_path_frames=sum(bool(v['path']) and v['valid'] for v in values), first_entry=first_entry,
        first_invalid_after_entry=first_bad))
report = dict(version='C11LAP30R11', source='manual-02', frames=len(frames), width=188, height=120,
    duration_s=round(frames[-1]['t'],3), original_crc_sha256_verified=True,
    native_sources_sha256={rel:hashlib.sha256((PROJECT/rel).read_bytes()).hexdigest() for rel in source_files},
    integrated_replay_assumes_running=True, synchronized_vehicle_odometry_available=False,
    summaries=summaries,
    frame_results=[dict(index=f['index'], board_frame=f['board_frame'], t=f['t'],
        modes=[{k:v for k,v in m.items() if k not in ['binary']} | {'path_points':len(m['path'])} for m in f['modes']]) for f in frames])

TEMPLATE = r'''<!doctype html>
<html lang="zh-CN"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>第10～50帧 · R11补线对比</title>
<style>
:root{color-scheme:light;--ink:#162c38;--muted:#5c6d76;--line:#d5dfe5;--paper:#f3f6f8;--accent:#087f8c}
*{box-sizing:border-box}body{margin:0;background:var(--paper);color:var(--ink);font:15px/1.6 system-ui,"Microsoft YaHei",sans-serif}main{max-width:1580px;padding:28px 24px 44px;margin:auto}.eyebrow{color:var(--accent);font-size:13px;letter-spacing:.08em}h1{font-size:30px;line-height:1.3;margin:5px 0 10px}.sub{color:var(--muted);margin:4px 0}.notice{margin:20px 0;padding:12px 16px;border-left:4px solid var(--accent);background:#e5f1f3}.toolbar{display:flex;align-items:center;gap:10px;flex-wrap:wrap;background:#fff;border:1px solid var(--line);padding:14px;border-radius:10px}.toolbar label{display:flex;align-items:center;gap:6px}button,select,input[type=number]{font:inherit;border:1px solid #b9cbd3;background:#fff;color:var(--ink);padding:8px 12px;border-radius:6px}button{cursor:pointer}button:hover{border-color:var(--accent);background:#eef8fa}button:focus-visible,select:focus-visible,input:focus-visible{outline:3px solid #70bfc6;outline-offset:2px}button.primary{background:var(--accent);color:#fff;border-color:var(--accent);min-width:92px}button:disabled{opacity:.45;cursor:default}input[type=number]{width:78px;padding:6px}input[type=range]{width:100%;accent-color:var(--accent);cursor:pointer}.seek{display:grid;grid-template-columns:1fr auto;align-items:center;gap:12px;margin:15px 0}.counter{font-variant-numeric:tabular-nums;font-weight:650;white-space:nowrap}#timeline{width:100%;height:18px;border-radius:3px;cursor:pointer;display:block;margin:0 0 14px;background:#d8e3e8}.panels{display:grid;grid-template-columns:repeat(3,minmax(0,1fr));gap:16px}.panel{background:white;border:1px solid var(--line);border-radius:10px;overflow:hidden}.panel h2{font-size:16px;font-weight:650;margin:0;padding:12px 14px;border-bottom:1px solid var(--line)}canvas.image{display:block;width:100%;height:auto;aspect-ratio:188/120;image-rendering:pixelated;background:#111}.caption{font-size:13px;color:var(--muted);padding:9px 14px;margin:0;min-height:42px}.meta{display:flex;flex-wrap:wrap;gap:8px;margin:16px 0}.pill{padding:5px 10px;background:#e5ecef;border-radius:5px;font-variant-numeric:tabular-nums}.pill.good{background:#def2e8;color:#17613c}.pill.bad{background:#f8e0df;color:#9d302a}#failureNotice{padding:10px 14px;border:1px solid #e6b3ae;background:#fff2f0;border-radius:6px;color:#92352c;margin:12px 0}.legend{display:flex;gap:18px;flex-wrap:wrap;margin:12px 0;color:var(--muted);font-size:13px}.dot{display:inline-block;width:10px;height:10px;border-radius:50%;margin-right:6px}.facts{display:grid;grid-template-columns:repeat(4,1fr);gap:10px;margin-top:20px}.fact{background:white;border-top:2px solid var(--line);padding:12px}.fact strong{display:block;font-size:21px}.fact span{font-size:12px;color:var(--muted)}details{background:#fff;border:1px solid var(--line);padding:12px 16px;border-radius:8px;margin-top:18px}summary{cursor:pointer;font-weight:600}details p{margin:8px 0;color:var(--muted)}.footer{font-size:12px;color:var(--muted);margin-top:18px}#renderState{font-size:12px;color:var(--muted)}
@media(max-width:900px){main{padding:18px 14px}.panels{grid-template-columns:1fr}.facts{grid-template-columns:repeat(2,1fr)}h1{font-size:25px}.toolbar{gap:8px}.seek{grid-template-columns:1fr}.counter{text-align:right}}
</style></head><body><main>
<div class="eyebrow">MANUAL-02 / C11LAP30R11 / OFFLINE REPLAY</div>
<h1>环岛入口接管 · 连续补线对比</h1>
<p class="sub">本批第10～50帧，共41帧 · 188 × 120 · 2026年11月7日18:18:56—18:20:23 · 双击此文件即可离线播放</p>
<div class="notice" id="modeNote">第1帧起连续计算，再截取第10～50帧。默认显示第21帧。青色虚线为估计补边，橙色为实际算法中线。电脑复算假定行驶，缺少同步轮速。</div>
<div class="toolbar">
<button id="prev" aria-label="上一帧">← 上一帧</button><button id="play" class="primary">播放</button><button id="next" aria-label="下一帧">下一帧 →</button>
<label>算法视图 <select id="mode"><option value="0">R11连续补线</option><option value="1">R10修改前</option></select></label>
<label>速度 <select id="speed"><option value="0.25">0.25倍</option><option value="0.5">0.5倍</option><option value="1" selected>1倍</option><option value="2">2倍</option><option value="4">4倍</option></select></label>
<label><input id="boundaries" type="checkbox" checked>显示边界</label>
<label>跳至 <input id="jump" type="number" min="10" max="50" value="21" aria-label="跳转帧号">帧</label>
</div>
<div class="seek"><input id="slider" type="range" min="0" max="40" step="1" value="0" aria-label="帧进度"><div id="counter" class="counter">第1 / 312帧</div></div>
<canvas id="timeline" width="164" height="24" aria-label="识别有效性时间轴"></canvas>
<div class="panels">
<section class="panel"><h2>原始灰度图</h2><canvas class="image" id="gray" width="188" height="120"></canvas><p class="caption">原始像素保留，已核验CRC和SHA-256。</p></section>
<section class="panel"><h2>当前二值化结果</h2><canvas class="image" id="binary" width="188" height="120"></canvas><p class="caption">两版灰度阈值保持一致，可切换查看修改前后中线。</p></section>
<section class="panel"><h2>中线与道路边界</h2><canvas class="image" id="overlay" width="188" height="120"></canvas><p class="caption" id="pathCaption">正在加载图像</p></section>
</div>
<div class="legend"><span><i class="dot" style="background:#ff8500"></i>算法输出中线</span><span><i class="dot" style="background:#0078ff"></i>左边界</span><span><i class="dot" style="background:#ff00b4"></i>右边界</span><span><i class="dot" style="background:#00b8d9"></i>估计补边（虚线）</span><span>时间轴绿色为有效，红色为无效。边界及普通中线可能含补线估计。</span></div>
<div class="meta" id="meta"></div><div id="failureNotice" hidden></div><span id="renderState" role="status">正在加载</span>
<div class="facts" id="facts"></div>
<details><summary>计算方式与本次修改</summary><p>近端右边界移出画面后，使用此前从真实双边测得的透视路宽，结合当前可见左边界与上方岛边端点续补右边线。逐行检查对边连续和白色通路，宽度估计有使用期限。左右对称。</p><p>状态机进入ENTER后，仍由有效补线保持直道中线，直到当前图像生成可行入环曲线再接管。青色虚线为估计边界，橙色为算法控制中线。首次成功入环路径之后失效时，按原有失线处理。</p><p>两版从第1帧连续计算后截取本段。手推采集缺少同步轮速，页面是电脑诊断回放。支持逐帧、跳转、变速和修改前后切换。</p></details>
<p class="footer">帧号从本批第1帧开始，同时显示原车端帧号。原始图像有12处车端帧号缺口，共缺12帧，保存帧的像素完整性检查通过。</p>
</main>
<script id="data" type="application/json">__DATA__</script>
<script>
'use strict';
const DATA=JSON.parse(document.getElementById('data').textContent);
const frames=DATA.frames;
const el=id=>document.getElementById(id);
const phaseNames=['IDLE','ENTER','ORBIT','EXIT','COOLDOWN'];
const statusNames=['普通靠近','当前双边','外侧单边','入口无效','内侧单边','编码器续行'];
const reasons={0:'正常',1:'没有可接受的当前侧向开口',2:'起点或目标净空不足',3:'白色通路不连通',4:'路径长度或重建失败',5:'场景对比不足',6:'无法生成可行平滑曲线',7:'候选道路存在歧义'};
const state={index:11,mode:0,playing:false,rendered:-1};
let timer=null,renderToken=0;
const cache=new Map();
function loadImage(url){if(!cache.has(url)){cache.set(url,new Promise((resolve,reject)=>{const image=new Image();image.onload=()=>resolve(image);image.onerror=reject;image.src=url;}));}return cache.get(url);}
function stop(){state.playing=false;clearTimeout(timer);el('play').textContent='播放';}
function rowLine(ctx,lines,offset,color){ctx.strokeStyle=color;ctx.fillStyle=color;ctx.lineWidth=1;ctx.beginPath();let previous=null;for(let y=18;y<=105;y++){const x=lines[offset+y];if(!x||x>=187){previous=null;continue;}if(previous&&Math.abs(x-previous[0])<=22){ctx.moveTo(previous[0]+.5,previous[1]+.5);ctx.lineTo(x+.5,y+.5);}ctx.fillRect(x,y,1,1);previous=[x,y];}ctx.stroke();}
function pathLine(ctx,path,offset,color,flag){ctx.strokeStyle=color;ctx.fillStyle=color;ctx.lineWidth=1;ctx.beginPath();let previous=null;for(const point of path){if(flag&&!(point[6]&flag)){previous=null;continue;}const x=point[offset],y=point[offset+1];if(previous&&(!flag||Math.max(Math.abs(x-previous[0]),Math.abs(y-previous[1]))<=8)){ctx.moveTo(previous[0]+.5,previous[1]+.5);ctx.lineTo(x+.5,y+.5);}ctx.fillRect(x,y,1,1);previous=[x,y];}ctx.stroke();}
async function render(){const token=++renderToken;const f=frames[state.index],m=f.modes[state.mode];el('slider').value=state.index;el('jump').value=f.index;el('counter').textContent=`第${f.index}帧 / 本段${frames.length}帧 · ${f.t.toFixed(2)}秒`;el('prev').disabled=state.index===0;el('next').disabled=state.index===frames.length-1;
const firstBad=DATA.summaries[state.mode].first_invalid_after_entry;const extra=firstBad&&f.index>=firstBad?' · 首次失效后的诊断回放':'';
el('meta').innerHTML=`<span class="pill ${m.valid?'good':'bad'}">${m.valid?'识别有效':'识别无效'}</span><span class="pill">车端帧 ${f.board_frame}</span><span class="pill">${phaseNames[m.phase]}</span><span class="pill">环岛候选 ${m.candidate}</span><span class="pill">近场有效 ${m.near_valid}</span><span class="pill">${statusNames[m.entry_status]}</span><span class="pill">中线误差 ${m.error} px</span>${m.command===null?'':`<span class="pill">模拟舵机命令 ${m.command}</span>`}`;
el('pathCaption').textContent=m.valid?(m.path.length?`有序入口曲线 ${m.path.length} 点 · 当前轮廓支持 ${m.boundary_support}`:`普通视觉中线 · 补边 ${m.bridge_side?m.bridge_side.filter(Boolean).length:0} 行`):'当前帧没有可用于控制的中线';el('failureNotice').hidden=!!m.valid;el('failureNotice').textContent=(m.failure?`failure=${m.failure} · ${reasons[m.failure]||'未知原因'}`:'基础视觉有效性判断未通过')+extra;
el('renderState').textContent='正在加载图像';try{const [gray,binary]=await Promise.all([loadImage(f.gray),loadImage(m.binary)]);if(token!==renderToken)return;for(const [id,image] of [['gray',gray],['binary',binary],['overlay',gray]]){const ctx=el(id).getContext('2d');ctx.imageSmoothingEnabled=false;ctx.clearRect(0,0,188,120);ctx.drawImage(image,0,0,188,120);}const ctx=el('overlay').getContext('2d');if(m.valid){if(m.path.length){if(el('boundaries').checked){pathLine(ctx,m.path,2,'#0078ff',1);pathLine(ctx,m.path,4,'#ff00b4',2);}pathLine(ctx,m.path,0,'#ff8500',0);}else{if(el('boundaries').checked){rowLine(ctx,m.lines,0,'#0078ff');rowLine(ctx,m.lines,240,'#ff00b4');}rowLine(ctx,m.lines,120,'#ff8500');if(el('boundaries').checked&&m.bridge_side){ctx.strokeStyle='#00b8d9';ctx.lineWidth=1;ctx.setLineDash([2,2]);ctx.beginPath();let prev=null;for(let y=18;y<=105;y++){if(!m.bridge_side[y]){prev=null;continue;}const x=m.bridge_edge[y];if(prev){ctx.moveTo(prev[0]+.5,prev[1]+.5);ctx.lineTo(x+.5,y+.5);}prev=[x,y];}ctx.stroke();ctx.setLineDash([]);}}}state.rendered=state.index;el('renderState').textContent=`已显示第${f.index}帧 · ${extra?extra.substring(3):'原图与算法结果已加载'}`;}catch(error){el('renderState').textContent='图像加载失败，请重新打开此HTML文件';console.error(error);}}
function drawTimeline(){const c=el('timeline'),ctx=c.getContext('2d');ctx.clearRect(0,0,c.width,c.height);for(let i=0;i<frames.length;i++){ctx.fillStyle=frames[i].modes[state.mode].valid?'#29845d':'#cb5e53';ctx.fillRect(i*4,0,3,c.height);}const s=DATA.summaries[state.mode];el('facts').innerHTML=`<div class="fact"><strong>${s.valid} / ${s.total}</strong><span>整组312帧有效数</span></div><div class="fact"><strong>${s.entry_path_frames}</strong><span>有序入口曲线帧</span></div><div class="fact"><strong>${s.first_entry||'—'}</strong><span>首次确认ENTER的本批帧号</span></div><div class="fact"><strong>${s.first_invalid_after_entry||'—'}</strong><span>确认入口后首次无效帧</span></div>`;}
function seek(index){stop();state.index=Math.max(0,Math.min(frames.length-1,index));render();}
function advance(){if(!state.playing)return;if(state.index>=frames.length-1){stop();return;}const delta=Math.max(.01,frames[state.index+1].t-frames[state.index].t);timer=setTimeout(async()=>{if(!state.playing)return;state.index++;await render();advance();},Math.min(3000,delta*1000/Number(el('speed').value)));}
el('play').addEventListener('click',()=>{if(state.playing){stop();return;}if(state.index===frames.length-1){state.index=0;render();}state.playing=true;el('play').textContent='暂停';advance();});el('prev').addEventListener('click',()=>seek(state.index-1));el('next').addEventListener('click',()=>seek(state.index+1));el('slider').addEventListener('input',e=>seek(Number(e.target.value)));el('jump').addEventListener('change',e=>{const value=Number(e.target.value);if(Number.isFinite(value))seek(value-10);});el('boundaries').addEventListener('change',render);el('speed').addEventListener('change',()=>{if(state.playing){clearTimeout(timer);advance();}});el('mode').addEventListener('change',e=>{stop();state.mode=Number(e.target.value);el('modeNote').textContent=state.mode===0?'第1帧起连续计算，再截取第10～50帧。默认显示第21帧。青色虚线为估计补边，橙色为实际算法中线。电脑复算假定行驶，缺少同步轮速。':'R10修改前，使用同一批图像和相同入口状态机假设。';drawTimeline();render();});el('timeline').addEventListener('click',e=>{const bounds=e.currentTarget.getBoundingClientRect();seek(Math.floor((e.clientX-bounds.left)/bounds.width*frames.length));});document.addEventListener('keydown',e=>{if(['INPUT','SELECT','BUTTON'].includes(e.target.tagName))return;if(e.code==='Space'){e.preventDefault();el('play').click();}else if(e.code==='ArrowLeft'){e.preventDefault();seek(state.index-1);}else if(e.code==='ArrowRight'){e.preventDefault();seek(state.index+1);}else if(e.code==='Home'){e.preventDefault();seek(0);}else if(e.code==='End'){e.preventDefault();seek(frames.length-1);}});
window.replayState=state;window.replayReady=false;drawTimeline();render().then(()=>{window.replayReady=true;});
</script></body></html>'''
frame_count = args.end - args.start + 1
TEMPLATE = (TEMPLATE
    .replace('第10～50帧', f'第{args.start}～{args.end}帧')
    .replace('共41帧', f'共{frame_count}帧')
    .replace('默认显示第21帧', f'默认显示第{initial_frame}帧')
    .replace('min="10" max="50" value="21"', f'min="{args.start}" max="{args.end}" value="{initial_frame}"')
    .replace('max="40"', f'max="{frame_count - 1}"')
    .replace('width="164"', f'width="{frame_count * 4}"')
    .replace('const state={index:11,', f'const state={{index:{initial_frame - args.start},')
    .replace('seek(value-10)', f'seek(value-{args.start})'))
report['display_range'] = [args.start, args.end]
frames = frames[args.start - 1:args.end]
html = TEMPLATE.replace('__DATA__',json.dumps(dict(frames=frames,summaries=summaries),ensure_ascii=False,separators=(',',':')).replace('</',r'<\/'))
OUTPUT.write_text(html,encoding='utf-8')
report['html_sha256']=hashlib.sha256(OUTPUT.read_bytes()).hexdigest()
(INPUT/('R11中线计算结果.json' if (args.start, args.end) == (10, 50) else f'R11中线计算结果_{args.start}-{args.end}帧.json')).write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
(WORK/'build-summary.json').write_text(json.dumps(dict(output=str(OUTPUT),bytes=OUTPUT.stat().st_size,summaries=summaries),ensure_ascii=False,indent=2),encoding='utf-8')
print(json.dumps(dict(output=str(OUTPUT),bytes=OUTPUT.stat().st_size,summaries=summaries),ensure_ascii=False))
