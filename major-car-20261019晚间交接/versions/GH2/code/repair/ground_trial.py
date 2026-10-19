"""GH2 supervised ground hold. Default only queries; --run starts exactly one Y."""
import argparse,csv,json,re,time
from pathlib import Path

def fields(line):return dict(re.findall(r'(\w+)=([^\s]+)',line))
def validate_config(line):
 v=fields(line)
 expected={'min':900,'max':2000,'step':100,'settle':200,'kick':3000,'kick_min':200,'kick_max':300,'hold':1500,'total':2000,'lease':500,'guard':120,'us':1500,'simulated':0}
 try:
  if not line.startswith('GROUND-CFG ') or v['v']!='20261019GH2':raise ValueError('identity')
  c={k:int(v[k]) for k in expected};c['selected']=int(v['selected'])
  if any(c[k]!=x for k,x in expected.items()) or c['selected'] not in range(900,2001,100):raise ValueError('envelope')
  return c
 except (KeyError,ValueError) as e:raise RuntimeError('GH2 exact bounded configuration required') from e

def parse_state(line):
 if not line.startswith('GROUND-STATE '):return None
 try:
  v=fields(line);s={k:int(v[k]) for k in ['ms','mode','fault','id','phase','done','elapsed','duty','us']}
  for k in ['enc','pwm','n','sum','min','max']:
   s[k]=tuple(map(int,v[k].split(',')))
   if len(s[k])!=2:raise ValueError('pair')
  return s
 except (KeyError,ValueError):return None

def check_state(s):
 if s is None or s['fault'] or s['mode'] not in (0,10) or s['us']!=1500 or any(x<0 or x>3000 for x in s['pwm']) or any(x< -5 or x>120 for x in s['enc']):raise RuntimeError('Abnormal GH2 state; stop required')

def classify(s,trial_id,duty):
 check_state(s)
 if s['id']!=trial_id or s['duty']!=duty:raise RuntimeError('Trial identity/duty mismatch')
 if s['mode']==10:return 'pending'
 if not(s['mode']==0 and s['done']==1 and s['phase']==3 and 1900<=s['elapsed']<2000 and s['pwm']==(0,0) and all(x==50 for x in s['n']) and all(x>=2 for x in s['min'])):raise RuntimeError('No complete continuous-hold expiry evidence')
 return 'completed'

def session(port,out,run=False,duty=1000):
 out.mkdir(parents=True,exist_ok=False);raw=bytearray();pending=bytearray();states=[];commands=[];cfg=None;latest=None;armed=False
 result={'outcome':'error','motion_sent':False,'reason':'Incomplete session','physical_observation':'pending'}
 def send(c):port.write(c.encode('ascii'));commands.append({'monotonic':time.monotonic(),'command':c})
 def receive():
  nonlocal pending,cfg,latest
  chunk=port.read(max(1,min(port.in_waiting,4096)));raw.extend(chunk);pending.extend(chunk);new=[]
  if len(pending)>65536:raise RuntimeError('Unbounded serial fragment')
  while b'\n' in pending:
   line,_,rest=pending.partition(b'\n');pending=bytearray(rest);line=line.decode('ascii','replace').strip()
   if line.startswith('GROUND-CFG '):cfg=validate_config(line)
   if 'GROUND-START REFUSED' in line:raise RuntimeError('Start refused; no retry')
   s=parse_state(line)
   if s is not None:latest=s;states.append(s);new.append(s)
  return new
 try:
  if hasattr(port,'reset_input_buffer'):port.reset_input_buffer()
  deadline=time.monotonic()+5;nextq=0
  while time.monotonic()<deadline:
   if time.monotonic()>=nextq:send('W');nextq=time.monotonic()+.6
   receive()
   if latest:
    check_state(latest)
    if latest['mode']!=0 or latest['pwm']!=(0,0) or latest['enc']!=(0,0):raise RuntimeError('Idle zero required')
   if cfg and latest:break
  if not cfg or latest is None:raise RuntimeError('No complete identified preflight')
  if run:
   # Selection is idle-only and bounded; always reset to 9% before adding steps.
   send('<'*11+'>'*((duty-900)//100));cfg=None;deadline=time.monotonic()+3;nextq=0
   while time.monotonic()<deadline:
    if time.monotonic()>=nextq:send('W');nextq=time.monotonic()+.6
    receive()
    if cfg and cfg['selected']==duty:break
   if not cfg or cfg['selected']!=duty:raise RuntimeError('Selected duty unconfirmed')
   check_state(latest)
   if latest['mode']!=0 or latest['pwm']!=(0,0) or latest['enc']!=(0,0):raise RuntimeError('Idle zero required before start')
   expected=(latest['id']+1)&0xffffffff
   print(json.dumps({'preflight':latest,'configuration':cfg,'single_trial_duty':duty}),flush=True)
   armed=True;result['motion_sent']=True;send('Y');began=time.monotonic();fresh=began;nextq=began;last_ms=latest['ms'];seen_feedback=False
   while time.monotonic()-began<3.2:
    if time.monotonic()>=nextq:send('W');nextq=time.monotonic()+.1
    for s in receive():
     check_state(s)
     if s['ms']!=last_ms:fresh=time.monotonic();last_ms=s['ms']
     if s['id']==expected:
      if s['mode']==10 and s['phase']==2 and all(x>=2 for x in s['enc']):seen_feedback=True
      if classify(s,expected,duty)=='completed':
       if not seen_feedback:raise RuntimeError('No observed two-wheel hold feedback')
       result.update(outcome='completed',reason='Normal bounded hold and both-wheel continuous feedback',completion=s);return
     elif s['mode']==10:raise RuntimeError('Unexpected active identity')
    if time.monotonic()-fresh>.7:raise RuntimeError('Stale telemetry; stop required')
   raise RuntimeError('No identified normal expiry before deadline')
  result.update(outcome='preflight',reason='Queries only; no motion sent',configuration=cfg,status=latest)
 except BaseException as e:
  result.update(outcome='error',reason=str(e),status=latest);raise
 finally:
  if armed:
   try:
    send('s');deadline=time.monotonic()+1.2;nextq=0;stop_states=[]
    while time.monotonic()<deadline:
     if time.monotonic()>=nextq:send('W');nextq=time.monotonic()+.25
     stop_states.extend(receive())
    result['final']=stop_states[-1] if stop_states else None
    result['final_zero_verified']=bool(stop_states and stop_states[-1]['mode']==0 and stop_states[-1]['fault']==0 and stop_states[-1]['pwm']==(0,0) and stop_states[-1]['enc']==(0,0) and stop_states[-1]['us']==1500)
   except Exception as e:result.update(final_zero_verified=False,stop_query_error=str(e))
  (out/'serial.bin').write_bytes(raw);(out/'serial.txt').write_text(raw.decode('ascii','replace'),encoding='utf-8')
  for name,value in [('result',result),('states',states),('commands',commands)]:
   (out/(name+'.json')).write_text(json.dumps(value,ensure_ascii=False,indent=2),encoding='utf-8')
  if states:
   with (out/'status.csv').open('w',newline='',encoding='utf-8-sig') as f:
    w=csv.DictWriter(f,fieldnames=states[0]);w.writeheader();w.writerows(states)
  if armed and not result.get('final_zero_verified'):
   result.update(outcome='error',reason='Final zero output/feedback not independently confirmed')
   (out/'result.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
   print(json.dumps(result,ensure_ascii=False),flush=True)
   raise RuntimeError(result['reason'])
  print(json.dumps(result,ensure_ascii=False),flush=True)

def main():
 a=argparse.ArgumentParser(description=__doc__);a.add_argument('--port',required=True);a.add_argument('--run',action='store_true');a.add_argument('--duty',type=int,choices=range(9,21),default=20);a.add_argument('--out',type=Path,required=True);args=a.parse_args()
 import serial
 from serial.tools import list_ports
 devices=[p for p in list_ports.comports() if p.device.upper()==args.port.upper()]
 if len(devices)!=1 or (devices[0].vid,devices[0].pid)!=(0x2e3c,0x5740):raise RuntimeError('Wireless USB identity unconfirmed')
 with serial.Serial(args.port,115200,timeout=.02,write_timeout=.2) as port:session(port,args.out,args.run,args.duty*100)
if __name__=='__main__':main()
