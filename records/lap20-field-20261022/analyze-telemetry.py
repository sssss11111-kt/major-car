"""Parse complete received status lines, separating resets; no hardware access."""
from pathlib import Path
import json,re
root=Path(__file__).resolve().parent
lines=(root/'serial-01/telemetry.txt').read_text(encoding='utf-8').splitlines()
pat=re.compile(r'^(\S+) C11LAP20 t=(\d+) run=(\d+) fault=(\d+) frame=(\d+) good=(\d+) age=(\d+) e1=(-?\d+) e2=(-?\d+) p1=(-?\d+) p2=(-?\d+)$')
sessions=[];session=None
for line in lines:
    if 'C11LAP20 AUTO BOOT' in line:
        session={'boot_line':line,'statuses':[]};sessions.append(session)
    m=pat.match(line)
    if not m:continue
    v=m.groups();row=dict(zip(['elapsed','tick','run','fault','frame','good','age','e1','e2','p1','p2'],[float(v[0])]+list(map(int,v[1:]))))
    if session is None or (session['statuses'] and row['tick']<session['statuses'][-1]['tick']):
        session={'boot_line':'Boot line not captured; tick reset detected','statuses':[]};sessions.append(session)
    session['statuses'].append(row)
result={'version':'C11LAP20','boots':[],'field_acceptance':False,
        'physical_scene_motor_power_and_reason_for_stopping':'User feedback pending',
        'fault6_meaning':'After first 30 startup ticks, either encoder below 2 for 20 consecutive 10ms ticks.',
        'limits':'Sampled telemetry cannot establish exact startup/stop times, full trajectory, motor supply state or mechanical cause.'}
for s in sessions:
    rows=s['statuses'];running=[r for r in rows if r['run']];fault=[r for r in rows if r['fault']]
    result['boots'].append({'boot_line':s['boot_line'],'complete_status_lines':len(rows),
        'first_status':rows[0] if rows else None,'first_running_sample':running[0] if running else None,
        'last_running_sample':running[-1] if running else None,'running_sample_count':len(running),
        'first_fault_sample':fault[0] if fault else None,'last_status':rows[-1] if rows else None,
        'running_encoder_range':{k:[min(r[k] for r in running),max(r[k] for r in running)] for k in ['e1','e2']} if running else {},
        'running_pwm_range':{k:[min(r[k] for r in running),max(r[k] for r in running)] for k in ['p1','p2']} if running else {},
        'all_fault_samples_pwm_zero':all(r['p1']==r['p2']==0 for r in fault) if fault else None})
(root/'telemetry-analysis.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
print(json.dumps(result,ensure_ascii=False,indent=2))
