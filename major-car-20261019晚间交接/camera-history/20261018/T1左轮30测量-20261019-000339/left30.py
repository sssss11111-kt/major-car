from pathlib import Path
import sys,time,json,importlib.util
sys.path.insert(0,r'D:\桌面\智能车\.tmp\tracking-trial-deps')
import serial
spec=importlib.util.spec_from_file_location('tracking',r'D:\桌面\智能车\major-car-GitHub更新-T1-4994cd5-20261018-源码包\major-car-T1-20261018\code\repair\tracking_trial.py')
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
out=Path(__file__).parent;raw=bytearray();rows=[];cfg=None;latest=None;armed=False;active=[]
result={'outcome':'error','physical_observation':'pending'}
p=serial.Serial('COM6',115200,timeout=.02,write_timeout=.3)
def receive():
 global cfg,latest
 line=p.readline();raw.extend(line)
 text=line.decode('ascii',errors='replace').strip()
 if text.startswith('TRACK-CFG '):cfg=m.parse_config(text)
 if 'TEST REFUSED' in text:raise RuntimeError(text)
 row=m.parse_status(text)
 if row:
  rows.append(row);latest=row
 return row
try:
 p.reset_input_buffer();p.write(b'sdp');end=time.monotonic()+2;query=0
 while time.monotonic()<end:
  if time.monotonic()>=query:p.write(b'p');query=time.monotonic()+.2
  receive()
  if cfg and latest and latest['mode']==0 and latest['fault']==0 and latest['pwm']==(0,0) and latest['us']==1500:break
 if not cfg or not latest or latest['mode']!=0 or latest['fault']!=0 or latest['pwm']!=(0,0):raise RuntimeError('Idle T1 preflight failed; no start')
 armed=True;p.write(b'1');print('SENT LEFT 30% SINGLE 2-SECOND TEST',flush=True)
 began=time.monotonic();query=0;stopped=False
 while time.monotonic()-began<3.3:
  if time.monotonic()>=query:p.write(b'p');query=time.monotonic()+.1
  row=receive()
  if row:
   if row['fault'] or row['mode'] not in (0,2) or row['pwm'][1]!=0 or row['pwm'][0] not in (0,3000):raise RuntimeError('Unexpected state or output')
   if row['mode']==2:active.append(row)
   elif active:
    if any(row['pwm']) or row['us']!=1500:raise RuntimeError('Stop output mismatch')
    stopped=True;break
 if not active or not stopped:raise RuntimeError('No observed test or natural stop')
 result={'outcome':'completed','active_samples':len(active),'left_pwm_values':sorted(set(r['pwm'][0] for r in active)),'left_encoder_values':sorted(set(r['enc'][0] for r in active)),'right_encoder_values':sorted(set(r['enc'][1] for r in active)),'final':latest,'physical_observation':'pending user measurement','elapsed_host_seconds':round(time.monotonic()-began,3)}
 print(json.dumps(result),flush=True)
except BaseException as exc:
 result.update(reason=str(exc),latest=latest);raise
finally:
 try:
  if armed:p.write(b's')
 finally:
  p.close();(out/'serial.bin').write_bytes(raw);(out/'serial.txt').write_text(raw.decode('ascii',errors='replace'),encoding='utf-8');(out/'status.json').write_text(json.dumps(rows,indent=2),encoding='utf-8');(out/'result.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
