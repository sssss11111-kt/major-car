"""R7 curved entrance path + actual integrated vision/FSM regression, host only."""
from pathlib import Path
from special_scanner_host import scanner_args
import os, subprocess, ctypes as C, json
p=Path(__file__).resolve().parent
src=p.parent/'demo/user/src'
subprocess.run([os.environ['ZIG'],'cc','-shared','-O2','-Wall','-Wextra','-Werror','-I',str(p/'vision-stubs'),'-I',str(src.parent/'inc'),str(src/'img_process.c'),str(src/'steering_control.c'),str(p/'vision-replay-wrapper.c'),
                '-DR17_WITH_SPECIAL_TRACK',*scanner_args(),'-o',str(p/'recovery-replay.dll')],check=True)
d=C.CDLL(str(p/'recovery-replay.dll'));checks=[]
def check(x,label):
 assert x,label
 checks.append(label)
def scene(shift=0,island=False):
 raw=bytearray([20]*22560)
 for y in range(106):
  hw=int(71+32*(y-90)/119+0.5)
  for x in range(max(0,94+shift-hw),min(188,94+shift+hw)):raw[y*188+x]=220
  if island and 48<=y<=96:
   h=int((1-((y-72)/25)**2)**0.5*26)
   for x in range(94+shift-h,95+shift+h):raw[y*188+x]=20
 return raw
frame_tick=0
d.replay_set_entry_odometry.argtypes=[C.c_float,C.c_float,C.c_uint32,C.c_uint8]
def frame(raw):
 global frame_tick
 frame_tick+=2;d.replay_set_entry_odometry(0,0,frame_tick,0)
 out=(C.c_int*4)();d.replay_frame((C.c_uint8*22560).from_buffer_copy(raw),out);return list(out)
for guide in [-1,0,1]:
 for shift in [-12,0,12]:
  d.replay_reset();d.replay_guide(guide)
  for _ in range(4):frame(scene(shift))
  check(d.replay_road_recovered()==1,f'measured corridor shift{shift} guide{guide}')
 for kind in ['island','white','black','near_loss','single_edge','wide','broken']:
  d.replay_reset();d.replay_guide(guide)
  for _ in range(4):frame(scene())
  raw=scene(island=kind=='island')
  if kind=='white':raw[:]=bytes([220])*22560
  if kind=='black':raw[:]=bytes([20])*22560
  if kind=='near_loss':raw[80*188:106*188]=bytes([220])*(26*188)
  if kind in ['single_edge','wide']:
   for y in range(55,106):
    for x in range(94 if kind=='single_edge' else 2,188 if kind=='single_edge' else 186):raw[y*188+x]=220
  if kind=='broken':raw[75*188:78*188]=bytes([20])*(3*188)
  frame(raw);check(not d.replay_road_recovered(),f'{kind} cannot reuse prior clean corridor guide{guide}')
# Full real vision -> state -> steering, simulated encoder input explicitly labeled.
def step(raw,tick):
 out=(C.c_float*5)();d.replay_ring_step((C.c_uint8*22560).from_buffer_copy(raw),tick,out)
 check(69<=out[0]<=116,f'servo bound tick{tick}')
 return list(out)
d.replay_ring_reset()
entry=(p/'fixtures/ring-entry-gray.bin').read_bytes()
for n in range(3):step(bytearray(entry),n*2)
check(d.replay_ring_phase()==5,'stationary captured corner starts alignment')
advanced=bytearray(entry)
for y in range(21,106):advanced[y*188:(y+1)*188]=entry[(y-3)*188:(y-2)*188]
for n in range(3,7):
 step(advanced,n*2)
 check(d.replay_ring_phase()==5,'close island without clearance stays aligned')
for y in range(21,106):
 row=bytes(advanced[y*188:(y+1)*188])
 advanced[y*188:(y+1)*188]=row[9:]+row[-1:]*9
for n in range(7,11):step(advanced,n*2)
check(d.replay_ring_phase()==5,'static throat without continuous handoff remains aligned')
for _ in range(410):d.replay_ring_tick(80,40)
check(d.replay_ring_phase()==5,'encoder rotation alone cannot advance visual phase')
step(bytearray([220]*22560),842)
check(d.replay_ring_phase()==5,'all white dropout cannot establish orbit')
report={'hardware_accessed':False,'flashed':False,'real_track_tested':False,'checks':checks}
d.replay_reset();d.replay_guide(-1);d.replay_entry_mode(1)
for _ in range(3):
 current=frame(scene())
 check(current[0] and d.replay_entry_status() in [1,2,4],'current measured path primes reacquisition test')
frame(bytearray([20]*22560));frame(bytearray([20]*22560))
first=frame(scene())
check(not first[0] and d.replay_entry_status()==3,'first current path after loss waits for reacquisition')
second=frame(scene())
check(second[0] and d.replay_entry_status() in [1,2,4],'second current path confirms reacquisition')

# Synthetic OUTER-bank convex tip; the production FSM must see it move.
# Injected ORBIT/armed state tests only exit logic, not acquisition or a real lap.
def tip_scene(row,side):
 raw=bytearray([20]*22560)
 for y in range(18,106):
  bank=max(20,100-2*abs(y-row))
  for x in range(bank+1,188):raw[y*188+(x if side>0 else 187-x)]=220
 return raw

d.replay_ring_scenario.argtypes=[C.c_int,C.c_float]
for side in (-1,1):
 for yaw in (0,1000):
  d.replay_ring_reset();d.replay_ring_scenario(2,yaw);d.replay_ring_direction(side)
  for n in range(6):
   step(tip_scene(55,side),n*2)
   check(d.replay_ring_phase()==2,'repeating a stationary tip cannot exit')
  for n,row in enumerate((59,62,65),6):step(tip_scene(row,side),n*2)
  check(d.replay_ring_phase()==3,'current advancing tip commits mirrored visual exit regardless of yaw')
  path=(C.c_uint8*(160*7))();count=d.replay_entry_path(path)
  check(count>=12,'EXIT publishes the current measured curve')
  current=tip_scene(65,side)
  check(all(current[path[i*7+1]*188+path[i*7]]>100 for i in range(count)), 'exit curve stays in current white road')
  step(bytearray([20]*22560),20)
  check(d.replay_ring_phase()==3 and d.replay_entry_status()==0,'exit loss retains phase and invalid current rows')
  phases=[]
  for n in range(25):
   result=step(scene(),22+n*2);phases.append(d.replay_ring_phase())
  check(4 in phases and phases[-1]==0,'current normal road recovers through cooldown')

report['checks']=checks
(p/'recovery-checks.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
print('PASS',len(checks),'recovery/integrated checks; visual exit, no yaw transition')
