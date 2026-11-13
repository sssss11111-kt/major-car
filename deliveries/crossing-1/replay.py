import base64, ctypes as C, io, json, os, re, sys
from pathlib import Path
from PIL import Image
base=Path(__file__).with_name('CROSS13-replay-input.html')
html=base.read_text(encoding='utf8')
m=re.search(r'id="payload"[^>]*>(.*?)</script>',html,re.S)
data=json.loads(m[1])
dll=C.CDLL(str(Path(os.environ['TEMP'])/'crossing-1.dll'))
dll.replay_thresholds.restype=C.POINTER(C.c_uint8)
dll.replay_ring_reset()
for f in data['frames']:
 r=f['result']
 old=r.copy()
 f['original']=old
 pixels=Image.open(io.BytesIO(base64.b64decode(f['image'].split(',')[1]))).convert('L').tobytes()
 out=(C.c_float*5)(); facts=(C.c_int*39)(); lines=(C.c_uint8*360)()
 banks=(C.c_uint8*192)(); path=(C.c_uint8*1120)(); scan=(C.c_int*5)(); gate=(C.c_int*8)()
 dll.replay_ring_step((C.c_uint8*len(pixels)).from_buffer_copy(pixels),f['tick'],out)
 dll.replay_lines(lines); dll.replay_cross_diagnostics(facts,banks); dll.replay_scan_status(scan)
 dll.replay_gate_facts(gate); count=dll.replay_control_path(path)
 names='straight_active straight_valid straight_opening straight_recovered straight_support straight_source bend_side bend_frames bend_released outer_count near_valid preview_valid preview candidate evidence primary_valid rows both_rows scan_state confidence full_scan_valid island_side ring_confirm ring_pending ring_fault single_rows near_run cross_active cross_valid cross_approach_only cross_near_error cross_preview_error cross_side cross_saw_split boundary_source bend_follow exit_frames exit_white_rows exit_white_seen'.split()
 r.update(zip(names,facts))
 r.update(zip('scan_type scan_frame_type scan_flags scan_allows_entry full_scan_attempted'.split(),scan))
 r.update(zip(('command','phase','control_error','valid','error'),[round(out[0],5),int(out[1]),int(out[2]),int(out[3]),int(out[4])]))
 r.update(cross_phase=dll.replay_cross_phase(),lines=list(lines),path=[list(path[i:i+7]) for i in range(0,count*7,7)],outer=[list(banks[i:i+2]) for i in range(0,facts[9]*2,2)],thresholds=list(dll.replay_thresholds()[:120]),corner_row=gate[1],entry_ready=gate[0],exit_ready=gate[4],road_recovered=dll.replay_road_recovered(),status=dll.replay_entry_status(),failure=gate[2],boundary_source=facts[34],stripe=dll.replay_stripe())
 if f['index'] in [136,*range(145,165),*range(235,248),280,300,320]:
  print(f['index'],tuple(r[k] for k in ('straight_active','bend_frames','bend_released','outer_count','valid','control_error','scan_type','candidate','ring_confirm')),len(r['path']),r['outer'][-1:] )
 if r['thresholds'] != old['thresholds']:
  im=Image.frombytes('L',(188,120),bytes(255 if px>r['thresholds'][i//188] else 0 for i,px in enumerate(pixels)))
  buf=io.BytesIO();im.save(buf,format='PNG')
  f['binary']='data:image/png;base64,'+base64.b64encode(buf.getvalue()).decode()
print('baseline/new',sum(f['original']['valid'] for f in data['frames']),sum(f['result']['valid'] for f in data['frames']))
print('key',sum(f['result']['valid'] for f in data['frames'] if 126<=f['index']<=155 or 275<=f['index']<=324))
Path(os.environ['TEMP'],'crossing-1-data.json').write_text(json.dumps(data,separators=(',',':'),ensure_ascii=False),encoding='utf8')

frames = data['frames']
summary = data['summary']
def ranges(numbers):
    result = []
    for n in numbers:
        if result and n == result[-1][1]+1:
            result[-1][1] = n
        else:
            result.append([n,n])
    return result

summary.update(
    baseline_valid=sum(f['original']['valid'] for f in frames),
    control_valid=sum(f['result']['valid'] for f in frames),
    regressions=[f['index'] for f in frames if f['original']['valid'] and not f['result']['valid']],
    improvements=[f['index'] for f in frames if not f['original']['valid'] and f['result']['valid']],
    recovered_frames=[f['index'] for f in frames if f['result']['straight_recovered']],
    cross_candidate_ranges=ranges(f['index'] for f in frames if f['result']['scan_frame_type']==1),
    source='analysis/CROSS14-manual-03/deliveries/lap30-ring-20261107/project/demo/user',
)
def result(n):
    return frames[n-1]['result']

assert all(result(n)['valid'] for n in [136,151,156,243])
assert result(136)['path'] == frames[135]['original']['path'] or [p[:2] for p in result(136)['path']] == [p[:2] for p in frames[135]['original']['path']]
assert result(151)['lines'][295] > frames[150]['original']['lines'][295]+10
assert result(156)['outer'][-1][0] >= 175
for frame in (156,160):
    current = result(frame)
    seam_x, seam_y = current['outer'][0]
    offset = 0 if current['bend_side'] > 0 else 240
    assert not any(current['lines'][offset:offset+seam_y])
    assert current['lines'][offset+seam_y] == seam_x
    assert any(current['lines'][offset+seam_y+1:offset+120])

assert [result(n)['bend_frames'] for n in (241,242,243)] == [1,2,3]
assert result(154)['bend_follow'] and result(154)['cross_active'] and not result(154)['bend_released']
assert result(243)['bend_follow'] and not result(243)['bend_released']
assert result(164)['cross_active'] and not result(164)['bend_released']
for frame in range(227,233):
    current = result(frame)
    assert current['valid'] and current['boundary_source'] == 7, frame
    assert all(current['lines'][25:106]) and all(current['lines'][265:346]), frame
assert result(232)['exit_white_rows'] >= 8
assert result(237)['exit_white_seen']
assert result(244)['cross_active'] and not result(244)['bend_released']
assert result(245)['bend_released'] and not result(245)['cross_active']
for frame in range(245,249):
    current = result(frame)
    assert current['valid'] and current['boundary_source'] == 6 and current['path'], frame
    assert not (current['cross_active'] or current['bend_follow'] or current['outer']), frame
assert all(result(n)['valid'] for n in [*range(126,156),*range(275,325)])
print('validity delta', summary['regressions'], summary['improvements'])

def change(old, new):
    global html
    assert html.count(old)==1,old
    html=html.replace(old,new)

html=html[:m.start(1)]+json.dumps(data,separators=(',',':'),ensure_ascii=False)+html[m.end(1):]
change('<title>CROSS13 十字岛回放播放器 · manual-03</title>', '<title>CROSS14 十字岛回放播放器 · manual-03</title>')
change('<h1>CROSS13 十字岛回放播放器</h1>', '<h1>CROSS14 十字岛回放播放器</h1>')
change('CROSS13候选', 'CROSS14 离线候选')
change('修改后 · CROSS13','修改后 · CROSS14')
change("'修改前 · SCAN8'","'修改前 · CROSS13'")
change('<h2 id="middleTitle">修改前 · SCAN8</h2>', '<h2 id="middleTitle">修改前 · CROSS13</h2>')
change('    <button type="button" data-checkpoint="275">', '''    <button type="button" data-checkpoint="151">151 · 岛缘弯线</button>
    <button type="button" data-checkpoint="156">156 · 外侧边界</button>
    <button type="button" data-checkpoint="160">160 · 单一外边界</button>
    <button type="button" data-checkpoint="227">227 · 双侧补线</button>
    <button type="button" data-checkpoint="232">232 · 双侧白色开口</button>
    <button type="button" data-checkpoint="243">243 · 弯道过渡</button>
    <button type="button" data-checkpoint="245">245 · 出十字进弯道</button>
    <button type="button" data-checkpoint="275">''')
change('  if (el(\'center\').checked) drawRows', '''  if (el('boundaries').checked && result.outer?.length) {
    ctx.setLineDash([]);
    ctx.strokeStyle = '#ff655a';
    ctx.lineWidth = 1.25;
    ctx.beginPath();
    result.outer.forEach(([x, y], i) => {
      if (i && Math.hypot(x-result.outer[i-1][0], y-result.outer[i-1][1]) < 10) ctx.lineTo(x, y);
      else ctx.moveTo(x, y);
    });
    ctx.stroke();
  }
  if (el('center').checked) drawRows''')
change("  const flags = [];","""  if (r.bend_follow && r.straight_active && r.boundary_source !== 7) el('overlayCaption').textContent = '沿弯道通过十字岛，等待近场出口确认。';
  if (r.outer?.length) el('overlayCaption').textContent += ' 红线为当前帧提取的外侧轮廓。';
  const flags = [];
  if (r.bend_frames) flags.push(['弯道证据 '+r.bend_frames+' / 3', false]);
  if (r.bend_follow && r.boundary_source !== 7) flags.push(['沿弯道过渡', false]);""")
change("    ['环岛候选 / 证据 / 允许入环'",'''    ['环岛同侧拓扑连续帧数 / 状态机确认帧数', r.ring_pending+' / '+r.ring_confirm],
    ['环岛候选 / 证据 / 允许入环\'''')
change('go(135);', 'go(150);')
html=re.sub(r"^el\('stats'\)\.innerHTML = .*?$", """el('stats').textContent = 'CROSS13 对照 CROSS14。红线为当前帧提取的外侧轮廓。全序列控制有效 '+summary.baseline_valid+' → '+summary.control_valid+' / '+frames.length+' 帧。';""",html,flags=re.M)
html=re.sub(r"summary\.assumptions\.forEach\(.*?\n\]\.forEach\(.*?\n", """for (const text of [
  '从第 1 帧按原始板端 tick 连续运行 CROSS14 C 代码。',
  '中间图使用原代码包的 CROSS13 计算结果。右图使用此次复算结果。',
  '手推采集缺少同步里程和偏航。当前结果来自电脑回放。'
]) { const p = document.createElement('p'); p.textContent = text; conditions.append(p); }
""",html,flags=re.S)


change('</style>', '''.imagebox{position:relative}
.cross-overlay{position:absolute;top:8px;right:8px;max-width:calc(100% - 16px);box-sizing:border-box;padding:6px 9px;border-radius:6px;background:rgba(15,28,44,.9);color:#fff;font:600 clamp(11px,1.05vw,14px)/1.5 system-ui,sans-serif;pointer-events:none}
.cross-overlay[data-state="detected"]{border-left:3px solid #ffcf67}
.cross-overlay[data-state="entered"]{border-left:3px solid #65e1ae}
.cross-overlay[data-state="fault"]{border-left:3px solid #ff8585}
</style>''')
change('aria-label="平滑中线及十字补线"></canvas></div>', '''aria-label="平滑中线及十字补线"></canvas><div id="crossOverlay" class="cross-overlay" aria-label="本帧十字岛识别与通行状态"></div></div>''')
change("  el('seek').value = current;", """  const overlay = el('crossOverlay');
  const phase = r.cross_phase;
  const candidate = r.scan_frame_type === 1 || r.straight_opening;
  const active = Boolean(r.cross_active || r.straight_active);
  const confirmed = active || r.scan_type === 1;
  const exited = Boolean(r.straight_recovered || r.bend_released);
  const detection = confirmed || exited ? '已确认' : candidate ? '候选，待确认' : '未识别';
  const passage = exited ? '已退出' : phase === 3 ? '通道丢失' :
    phase === 2 ? '退出确认' : r.bend_follow && active ? (r.boundary_source === 7 ? '直道补线，等待出口确认' : '沿弯道通过，等待出口确认') : phase === 1 ? '已进入' :
    active ? '接近确认' : '未进入';
  overlay.dataset.state = phase === 3 ? 'fault' : phase === 1 || phase === 2 ? 'entered' :
    confirmed || candidate ? 'detected' : 'idle';
  const title = document.createElement('div');
  title.textContent = '十字岛：' + detection + ' · ' + passage;
  const detail = document.createElement('div');
  const control = exited ? '普通弯道循迹' : r.bend_follow && !r.straight_support ? (active ? '弯道循迹' : '普通弯道循迹') : r.cross_active ? '十字岛循迹' : '普通循迹';
  detail.textContent = control + ' · ' + (r.valid ? '控制有效' : '控制拒绝') +
    (r.bend_frames ? ' · 弯道证据 ' + r.bend_frames + '/3' : '') + (r.exit_white_rows >= 8 ? ' · 双侧白区 ' + r.exit_white_rows + '行' : '') +
    (active && r.bend_follow ? ' · 出口 ' + (r.exit_white_seen ? r.exit_frames + '/3' : '等待双侧白区') : '');
  overlay.replaceChildren(title, detail);
  el('seek').value = current;""")
html = html.replace('CROSS14', 'crossing-1')
Path(__file__).with_name('crossing-1.html').write_text(html,encoding='utf8')

