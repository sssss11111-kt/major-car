"""Synthetic topology regression, not real camera calibration."""
from pathlib import Path
import ctypes as C, os, subprocess
p=Path(__file__).resolve().parent;src=p.parent/'demo/user/src'
subprocess.run([os.environ['ZIG'],'cc','-shared','-O2','-Wall','-Wextra','-Werror','-I',str(p/'vision-stubs'),'-I',str(src.parent/'inc'),str(src/'img_process.c'),str(src/'steering_control.c'),str(p/'vision-replay-wrapper.c'),'-o',str(p/'roundabout-replay.dll')],check=True)
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
d.replay_reset();assert not d.replay_island()
print('PASS: both island shapes; mirrored branch guidance; no blind near-field lease; straight/stripe/uniform/reset isolation')
