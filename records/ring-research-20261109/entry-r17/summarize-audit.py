from pathlib import Path
import json,math,csv,hashlib
p=Path('records/ring-research-20261109/entry-r17');r=json.loads((p/'results.json').read_text(encoding='utf-8'));frames=[f for f in r['frames'] if not f['r17']['excluded']]
assert len(frames)==239
wrong=[];forward=[]
for f in frames:
 m=f['r17'];a=m['island']
 if m['path']:assert m['valid'] and m['max_angle_deg']<=40
 if a[360]:
  for x,y,*_ in m['path']:
   if a[240+y] and x>a[y]:wrong.append([f['index'],x,y])
 if m['path'] and m['boundary_source']==5:
  c=m['curve'];assert c[0]==94 and c[1]==105 and c[2]==94
  previous=0;changes=0
  for k in range(129):
   t=k/128;u=1-t
   dx=3*(u*u*(c[2]-c[0])+2*u*t*(c[4]-c[2])+t*t*(c[6]-c[4]))
   dy=3*(u*u*(c[3]-c[1])+2*u*t*(c[5]-c[3])+t*t*(c[7]-c[5]))
   ddx=6*(u*(c[4]-2*c[2]+c[0])+t*(c[6]-2*c[4]+c[2]))
   ddy=6*(u*(c[5]-2*c[3]+c[1])+t*(c[7]-2*c[5]+c[3]))
   cross=dx*ddy-dy*ddx;sign=1 if cross>.01 else (-1 if cross<-.01 else 0)
   if sign and previous and sign!=previous:changes+=1
   if sign:previous=sign
  assert changes<=1,(f['index'],changes)
  forward.append(f['index'])
assert not wrong
invalid=[f['index'] for f in frames if not f['r17']['valid']]
reacquire=[f['index'] for f in frames if not f['r17']['valid'] and f['r17']['failure']==9]
summary=r['summaries'][1]
summary.update(invalid_frames=invalid,reacquisition_pending_frames=reacquire,forward_fit_frames=len(forward),max_forward_curve_inflections=1,observed_island_side_violations=wrong,command_min=min(f['r17']['command'] for f in frames),command_max=max(f['r17']['command'] for f in frames),segments=[])
for start,end in r['included_ranges']:
 fs=[f for f in frames if start<=f['index']<=end]
 summary['segments'].append(dict(start=start,end=end,total=len(fs),valid=sum(f['r17']['valid'] for f in fs),path_frames=sum(bool(f['r17']['path']) for f in fs),invalid_frames=[f['index'] for f in fs if not f['r17']['valid']]))
r['telemetry_available']=False;r['telemetry_absence_confirmed_by_user']=True
r['user_scene_annotations']=[dict(start=215,end=235,scene='exit_approach',use='Review annotation only; never supplied to production planner')]
(p/'results.json').write_text(json.dumps(r,ensure_ascii=False,indent=2),encoding='utf-8')
with (p/'frame-audit.csv').open('w',encoding='utf-8-sig',newline='') as file:
 w=csv.writer(file);w.writerow(['frame','scope','scene_annotation','r16_valid','r17_valid','phase','status','failure','method','points','max_turn_deg','command','finding'])
 for f in r['frames']:
  m=f['r17'];i=f['index'];finding=''
  if m['excluded']:finding='用户确认的搬车排除段'
  elif m['failure']==9:finding='当前可形成路线，等待丢失后的第二次有效观测'
  elif not m['valid']:finding='近场边界证据不足，保留拒绝'
  elif not m['path']:finding='普通逐行或入口对齐阶段，未交付有序转入路线'
  elif m['boundary_source']==5:finding='当前走廊整体三次曲线拟合，检查白色连接、角度及至多一次连续曲率变号'
  else:finding='当前岛体上方开口曲线，检查白色连接、角度和岛边侧别'
  w.writerow([i,'排除' if m['excluded'] else '纳入','用户标注即将出环' if 215<=i<=235 else '',f['r16']['valid'],m['valid'],m['phase'],m['status'],m['failure'],m['boundary_source'],len(m['path']),round(m['max_angle_deg'],5) if m['path'] else '',round(m['command'],5),finding])
print(json.dumps(summary,ensure_ascii=False,indent=2));print('improved',len(r['improved_valid_frames']),'regressed',len(r['regressed_valid_frames']))
