import base64, ctypes as C, io, json, os, re, sys
from pathlib import Path
from PIL import Image
base=Path(__file__).resolve().parents[1] / 'crossing-1' / 'crossing-1.html'
html=base.read_text(encoding='utf8')
m=re.search(r'id="payload"[^>]*>(.*?)</script>',html,re.S)
data=json.loads(m[1])
dll=C.CDLL(str(Path(os.environ['TEMP'])/'crossing-2.dll'))
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
Path(os.environ['TEMP'],'crossing-2-data.json').write_text(json.dumps(data,separators=(',',':'),ensure_ascii=False),encoding='utf8')

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
    source='analysis/crossing-2/deliveries/lap30-ring-20261107/project/demo/user',
)
def result(n):
    return frames[n-1]['result']

assert all(result(n)['valid'] for n in [136,151,156,243])
assert result(136)['path'] == frames[135]['original']['path'] or [p[:2] for p in result(136)['path']] == [p[:2] for p in frames[135]['original']['path']]
assert result(151)['lines'] == frames[150]['original']['lines']
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

output = Path(__file__).with_name('crossing-2.html')
assert result(163)['boundary_source'] == 8
assert result(163)['path'][-1][0] > frames[162]['original']['path'][-1][0]+20
html=html[:m.start(1)]+json.dumps(data,separators=(',',':'),ensure_ascii=False)+html[m.end(1):]

html = html.replace('crossing-1', 'crossing-2').replace('CROSS13', 'crossing-1')
html = html.replace("const control = exited ? '普通弯道循迹' : exited ? '普通弯道循迹' :", "const control = exited ? '普通弯道循迹' :")
html = html.replace('    <button type="button" data-checkpoint="227">', '    <button type="button" data-checkpoint="163">163 · 二维中线</button>\n    <button type="button" data-checkpoint="227">', 1)
html = html.replace("  const flags = [];", "  if (r.boundary_source === 8) el('overlayCaption').textContent += ' 控制采用二维弯道中线。';\n  const flags = [];")
output.write_text(html, encoding='utf8')

