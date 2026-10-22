"""Synthetic topology regression, not real camera calibration."""
from pathlib import Path
import ctypes as C, os, subprocess
p=Path(__file__).resolve().parent;src=p.parent/'demo/user/src'
subprocess.run([os.environ['ZIG'],'cc','-shared','-O2','-Wall','-Wextra','-Werror','-I',str(p/'vision-stubs'),'-I',str(src.parent/'inc'),os.environ.get('IP_TEST_VISION_SOURCE',str(src/'img_process.c')),str(src/'steering_control.c'),str(p/'vision-replay-wrapper.c'),'-o',str(p/'roundabout-replay.dll')],check=True)
d=C.CDLL(str(p/'roundabout-replay.dll'))
def scene(island=0,shift=0,stripe=False):
 raw=bytearray([20]*(188*120))
 for y in range(106):
  hw=int(71+32*(y-90)/119+0.5)
  for x in range(max(0,94+shift-hw),min(187,94+shift+hw-1)+1):raw[y*188+x]=220
  if island and 48<=y<=96:
   h=int((1-((y-72)/25)**2)**0.5*26) if island==1 else min(26,(y-47)*3)
   for x in range(94+shift-h,94+shift+h+1):raw[y*188+x]=20
  if stripe and 55<=y<=105:
   for b in range(7):
    for x in range(55+b*12,61+b*12):raw[y*188+x]=20
 return raw
def frame(raw):
 o=(C.c_int*4)();d.replay_frame((C.c_uint8*len(raw))(*raw),o);return list(o)
for kind in [1,2]:
 for side in [-1,1]:
  d.replay_reset();d.replay_guide(side)
  for _ in range(8):frame(scene())
  o=frame(scene(kind));assert d.replay_island(),(kind,side,o)
  for _ in range(8):o=frame(scene(kind))
  assert o[0] and o[1]*side>5,(kind,side,o)
  lines=(C.c_uint8*360)();d.replay_lines(lines);raw=scene(kind)
  for y in range(50,97):
   assert lines[120+y] and raw[y*188+lines[120+y]]>100,('center on island',kind,side,y)
  for tick in range(10,210,2):
   state=(C.c_float*5)();d.replay_control_step(tick,state)
   assert 69<=state[0]<=116,('ring servo bounds',list(state))
  # Near field erased: historical island must not authorize continued driving.
  raw=scene(kind);raw[80*188:106*188]=bytes([20])*(26*188)
  assert not frame(raw)[0]
  raw=scene(kind);raw[80*188:106*188]=bytes([220])*(26*188)
  assert not frame(raw)[0], 'unbounded near white cannot authorize ring guidance' 
for shift in [-15,0,15]:
 d.replay_reset()
 for _ in range(8):
  frame(scene(shift=shift));assert not d.replay_island()
d.replay_reset();frame(scene(stripe=True));assert not d.replay_island() and d.replay_stripe()
# Loss, reset, uniform frames, body mask and an isolated mark cannot latch an island.
for raw in [bytearray([220]*(188*120)),bytearray([20]*(188*120))]:
 frame(raw);assert not d.replay_island()
# A broad rectangular stripe or an open-ended fork is not a closed island.
for open_tip in [False,True]:
 d.replay_reset();raw=scene()
 for y in range(18 if open_tip else 48,97):
  h=min(26,max(6,(y-18)//2)) if open_tip else 9
  for x in range(94-h,95+h):raw[y*188+x]=20
 frame(raw);assert not d.replay_island(),('rectangle/open fork',open_tip)
# A meander lobe can have a white cap and two branches while remaining
# connected to the exterior below/to one side. Include one-pixel diagonal necks.
for mirrored in [False,True]:
 for thickness in [1,3,7]:
  d.replay_reset();raw=scene(1)
  for y in range(72,106):
   x=94+(y-72)
   for xx in range(x,x+thickness):
    if xx<188:raw[y*188+xx]=20
  if mirrored:raw=bytearray(v for y in range(120) for v in reversed(raw[y*188:(y+1)*188]))
  for _ in range(8):
   frame(raw);assert not d.replay_island(),('open wave lobe',mirrored,thickness)
# Rasterized continuous wave roads, spanning the left/right image edges.
import math
for amplitude in [6,12,18]:
 for width in [12,18,24]:
  for phase in [0,0.7,1.5]:
   raw=bytearray([20]*(188*120))
   for x in range(188):
    cy=60+amplitude*math.sin(x*2*math.pi/75+phase)
    for y in range(18,106):
     if abs(y-cy)<=width/2:raw[y*188+x]=220
   d.replay_reset()
   for _ in range(8):
    frame(raw);assert not d.replay_island(),('continuous wave',amplitude,width,phase)
d.replay_reset();assert not d.replay_island()
print('PASS: both islands and mirrored guidance; 6 open-lobe variants x8 frames; 27 continuous-wave variants x8 frames; original negative and near-field cases')

# Captured stationary LAP27 entry, not the historical fault-triggering frame.
real=(p/'fixtures/ring-entry-gray.bin').read_bytes()
for mirror in [False,True]:
 raw=real if not mirror else bytes(v for y in range(120) for v in reversed(real[y*188:(y+1)*188]))
 d.replay_reset()
 for _ in range(3):o=frame(raw)
 assert d.replay_island(), ('real clipped island missed',mirror,o)
 d.replay_guide(1 if mirror else -1)
 for _ in range(8):o=frame(raw)
 assert o[0] and o[1]*(1 if mirror else -1)>5, ('real branch invalid',mirror,o)
print('PASS: captured side-clipped island and mirror')

# Opening the captured component to either vertical ROI boundary disqualifies
# it, even through a one-pixel diagonal neck. Rectangles also stay negative.
for end in [18,105]:
 raw=bytearray(real)
 for y in range(min(end,60),max(end,60)+1):raw[y*188+140]=0
 d.replay_reset()
 for _ in range(8):frame(raw);assert not d.replay_island(),('open real component',end)
for side in [-1,1]:
 raw=scene()
 for y in range(30,94):
  for x in range(110,188):raw[y*188+x]=20
 if side>0:raw=bytearray(v for y in range(120) for v in reversed(raw[y*188:(y+1)*188]))
 d.replay_reset();frame(raw);assert not d.replay_island(),('side rectangle',side)
# Full camera -> 3-frame confirmation -> entry assistance -> stateful servo.
# A distant island leaves near error zero: the old near-only path went straight.
raw=scene()
for y in range(42,77):
 h=int((1-((y-59)/18)**2)**0.5*26)
 for x in range(94-h,95+h):raw[y*188+x]=20
d.replay_ring_reset();out=(C.c_float*5)()
for n in range(20):
 d.replay_ring_step((C.c_uint8*len(raw))(*raw),n*2,out)
 assert out[3] and 69<=out[0]<=116,list(out)
 if n<2:assert out[1]==0 and out[2]==out[4],list(out)
assert out[1]==1 and out[4]==0 and out[2]<-5 and out[0]>94,list(out)
# Loss must revoke guidance even after entry, and stripes suppress entry aid.
raw[80*188:106*188]=bytes([20])*(26*188)
d.replay_ring_step((C.c_uint8*len(raw))(*raw),42,out)
assert out[3]==0,list(out)
d.replay_ring_reset()
for n in range(10):
 raw=scene(stripe=True);d.replay_ring_step((C.c_uint8*len(raw))(*raw),n*2,out)
 assert out[1]==0,list(out)
# Guard actual production integration against omitting the measured-branch gate.
main=(src/'main.c').read_text(encoding='utf-8')
assert 'turn_error=rb_entry_error(roundabout_state.phase,applied_ring_guide,' in main
assert 'img_get_preview_error(),img_get_roundabout_preview_valid(),' in main
assert 'steering_update(&steering_state,turn_error,' in main
print('PASS: side-open/rectangular negatives; integrated distant entry, loss, stripe priority and production wiring')

d.replay_ring_reset();out=(C.c_float*5)()
for n in range(12):
 d.replay_ring_step((C.c_uint8*len(real))(*real),n*2,out)
 assert out[3] and 69<=out[0]<=116,('integrated captured frame',n,list(out))
 assert out[1]==(0 if n<2 else 1),('confirmation timing',n,list(out))
print('PASS: actual captured entry remains valid through confirmation and guide activation')
