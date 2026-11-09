"""R7 curved entrance path + actual integrated vision/FSM regression, host only."""
from pathlib import Path
import os, subprocess, ctypes as C, json, sys
p=Path(__file__).resolve().parent
sys.path.insert(0, str((p.parents[3] / "records/ring-fix-20261107").resolve()))
src=p.parent/'demo/user/src'
subprocess.run([os.environ['ZIG'],'cc','-shared','-O2','-Wall','-Wextra','-Werror','-I',str(p/'vision-stubs'),'-I',str(src.parent/'inc'),str(src/'img_process.c'),str(src/'steering_control.c'),str(p/'vision-replay-wrapper.c'),'-o',str(p/'recovery-replay.dll')],check=True)
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
def frame(raw):
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
for n in range(3,7):step(advanced,n*2)
check(d.replay_ring_phase()==1,'actual island frames enter')
for _ in range(410):d.replay_ring_tick(80,40)
check(d.replay_ring_phase()==2,'encoder threshold retains orbit')
for n in range(5):step(scene(island=True),830+n*2)
check(d.replay_ring_phase()==2,'visible island blocks exit beyond yaw gate')
step(bytearray([220]*22560),842)
check(d.replay_ring_phase()==2,'all white dropout does not exit')
# Let frame-to-frame jump gate recover, then require the visual three-frame gate.
phases=[]
for n in range(15):
 step(scene(),850+n*2);phases.append(d.replay_ring_phase())
check(3 in phases and 4 in phases,'measured normal road transitions EXIT then COOLDOWN')
check(phases.index(3)>=2,'no immediate exit on first/second normal frame')
report={'hardware_accessed':False,'flashed':False,'real_track_tested':False,'checks':checks,'phases':phases}
(p/'recovery-checks.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
# The user's captured entrance sequence is checked through the same production
# replay entry point. This checks entry path generation, not vehicle motion.
OLD = Path(__file__).resolve().parents[5] / 'major-car-LAP29-20261028'
sys.path.insert(0, str(OLD / 'deliveries/pc-replay-20261030'))
from replay import read_pgm
capture_dir=OLD/'records/ring-entry-20261106/manual-01'
captured=[]
for line in (capture_dir/'frames.jsonl').read_text(encoding='utf-8').splitlines():
    meta=json.loads(line)
    captured.append((meta,read_pgm(capture_dir/meta['file'])))
d.replay_ring_reset();entry_rows=[]
for meta,raw in captured[100:130]:
    idx=meta['index']+1;out=(C.c_float*5)()
    d.replay_ring_step((C.c_uint8*22560).from_buffer_copy(raw),idx*2,out)
    if 113<=idx<=116:
        check(out[3]==1 and out[1]==(5 if idx<116 else 1),
              f'captured entry frame{idx}: align before measured corner commitment')
        if idx==116:check(out[2]>=3.0,'committed current branch retains measured direction')
        entry_rows.append({'frame':idx,'phase':out[1],'entry_error':out[2],'valid':out[3]})
check(all(d.replay_ring_phase()==1 for _ in [0]),'captured sequence remains in entry phase during entrance')
report={'hardware_accessed':False,'flashed':False,'real_track_tested':False,'checks':checks,'phases':phases,'captured_entry':entry_rows}
d.replay_reset();d.replay_guide(-1);d.replay_entry_mode(1)
for _ in range(3):
 current=frame(scene())
 check(current[0] and d.replay_entry_status() in [1,2,4],'current measured path primes reacquisition test')
frame(bytearray([20]*22560));frame(bytearray([20]*22560))
first=frame(scene())
check(not first[0] and d.replay_entry_status()==3,'first current path after loss waits for reacquisition')
second=frame(scene())
check(second[0] and d.replay_entry_status() in [1,2,4],'second current path confirms reacquisition')

# Synthetic split: continued orbit is left of the island, exit corridor right.
# State/angle are injected assumptions, not reconstructed field telemetry.
import math
fork=scene()
for y in range(30,91):
 h=int(math.sqrt(max(0,1-((y-60)/31)**2))*26)
 for x in range(65-h,66+h):fork[y*188+x]=20
d.replay_ring_scenario.argtypes=[C.c_int,C.c_float]
for yaw,expected in [(224,2),(230,3)]:
 d.replay_ring_reset();d.replay_ring_scenario(2,yaw)
 for n in range(7):
  out=step(fork,2*n)
  if expected==2:check(d.replay_ring_phase()==2,'exit branch blocked before motion window')
 check(d.replay_island()==1,'synthetic island still visible during exit test')
 check(d.replay_ring_phase()==expected,'active exit commits only inside motion window')
 if expected==3:
  check(out[3]==1 and out[2]>0,'current exit path selects right corridor while island is visible')
  step(bytearray([20]*22560),16)
  check(d.replay_ring_phase()==3,'exit loss never falls back to orbit branch')
  check(d.replay_entry_status()==3,'missing exit stops vision instead of blind continuation')

report['checks']=checks
(p/'recovery-checks.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
print('PASS',len(checks),'R7 recovery/integrated checks',phases,'captured',entry_rows)
