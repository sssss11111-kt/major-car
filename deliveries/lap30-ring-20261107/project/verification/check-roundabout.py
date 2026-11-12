"""Synthetic topology regression, not real camera calibration."""
from pathlib import Path
from special_scanner_host import scanner_args
import ctypes as C, os, subprocess
p=Path(__file__).resolve().parent;src=p.parent/'demo/user/src'
scanner_test=p/'test-rb-scan.exe'
subprocess.run([os.environ['ZIG'],'cc','-std=c99','-Wall','-Wextra','-Werror',
                '-I',str(p/'vision-stubs'),'-I',str(src.parent/'inc'),
                str(src/'img_process.c'),str(p/'test-rb-scan.c'),*scanner_args(),
                '-o',str(scanner_test)],check=True)
subprocess.run([str(scanner_test)],check=True)
cross_test=p/'test-cross-drive.exe'
subprocess.run([os.environ['ZIG'],'cc','-std=c99','-Wall','-Wextra','-Werror',
                '-I',str(src.parent/'inc'),'-I',str(p.parents[3]/'firmware/camera_car/demo/user/inc'),
                str(src/'cross_drive.c'),str(p/'test-cross-drive.c'),'-o',str(cross_test)],check=True)
subprocess.run([str(cross_test)],check=True)
subprocess.run([os.environ['ZIG'],'cc','-shared','-O2','-Wall','-Wextra','-Werror','-I',str(p/'vision-stubs'),'-I',str(src.parent/'inc'),os.environ.get('IP_TEST_VISION_SOURCE',str(src/'img_process.c')),str(src/'steering_control.c'),str(p/'vision-replay-wrapper.c'),
                '-DR17_WITH_SPECIAL_TRACK','-DRB_TURN_SIGN=1',*scanner_args(),'-o',str(p/'roundabout-replay.dll')],check=True)
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

# Before ring acquisition, a connected side opening must not move the stem.
# A real curve shifts both banks and should continue to produce steering.
for side in (-1, 1):
 for opening in (True, False):
  raw=bytearray([20]*(188*120))
  for y in range(18,106):
   half=int(71+32*(y-90)/119+0.5)
   delta=max(0,min(55,(101-y)*2,(y-48)*4))
   left,right=94-half,94+half
   if opening:
    left-=delta if side<0 else 0
    right+=delta if side>0 else 0
   else:
    left+=side*delta//2
    right+=side*delta//2
   left,right=max(0,left),min(188,right)
   raw[y*188+left:y*188+right]=bytes([220])*(right-left)
  d.replay_ring_reset()
  out=(C.c_float*5)()
  for n in range(2,42,2):
   image=scene() if n<18 else raw
   d.replay_ring_step((C.c_uint8*len(image)).from_buffer_copy(image),n,out)
   assert out[1]==0 and out[3]==1 and d.replay_cross_phase()==0,list(out)
  if opening:
   assert abs(out[2])<=1 and abs(out[0]-92.5)<0.75,('side opening pulls steering',side,list(out))
  else:
   assert out[2]*side>=4,('ordinary bend lost steering',side,list(out))
print('PASS: ordinary approach holds through either side opening and still follows bends')

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

# R12 entry fallback: the far endpoint is outside the current white corridor,
# so the complete opening search must fail while the measured corridor remains
# usable. Every accepted path point is checked against the same current frame.
raw=bytearray([20]*(188*120))
for y in range(18,106):
 center=94-(105-y)//3
 left=center-40
 right=center+40
 for x in range(left,right+1): raw[y*188+x]=220
d.replay_reset(); d.replay_guide(-1); d.replay_entry_mode(1)
frame(raw)
path=(C.c_uint8*(160*7))(); count=d.replay_entry_path(path)
assert count>=12, ('measured corridor fallback path',count)
for i in range(count):
 x=path[i*7]; y=path[i*7+1]
 assert raw[y*188+x]>100, ('fallback point outside current white corridor',i,x,y)
assert path[(count-1)*7] < 94, ('fallback did not move toward selected branch',count)
print('PASS: measured current-frame entry corridor fallback',count,'points')

# Distant recognition starts alignment without pulling the near controller.
raw=scene()
for y in range(42,77):
 h=int((1-((y-59)/18)**2)**0.5*26)
 for x in range(94-h,95+h):raw[y*188+x]=20
d.replay_ring_reset();out=(C.c_float*5)()
for n in range(12):
 d.replay_ring_step((C.c_uint8*len(raw))(*raw),n*2,out)
 assert out[3] and 69<=out[0]<=116,list(out)
 if n>=2:assert out[1]==0 and abs(out[2])<=1,('ambiguous centered island must not choose direction',list(out))
assert d.replay_entry_path((C.c_uint8*(160*7))())==0
# Move the island's corner toward the camera, retaining current path evidence.
d.replay_ring_reset()
for n in range(3):
 d.replay_ring_step((C.c_uint8*len(real))(*real),n*2,out)
raw=bytearray(real)
for y in range(21,106):raw[y*188:(y+1)*188]=real[(y-3)*188:(y-2)*188]
for n in range(3,7):
 d.replay_ring_step((C.c_uint8*len(raw))(*raw),n*2,out)
 assert out[3] and out[1]==5 and 69<=out[0]<=116,('close but unsafe throat',list(out))
# ALIGN now publishes a measured ordinary curve, without authorizing entry.
path=(C.c_uint8*(160*7))();count=d.replay_entry_path(path)
assert (count==0 or count>=12) and out[1]==5
for i in range(count):
 assert raw[path[i*7+1]*188+path[i*7]]>100, ('alignment curve outside current road',i)
# Move the whole measured corridor away from the image center. The more
# open branch must appear in two consecutive current frames before turning.
for y in range(21,106):
 row=bytes(raw[y*188:(y+1)*188])
 raw[y*188:(y+1)*188]=row[9:]+row[-1:]*9
for n in range(7,14):
 d.replay_ring_step((C.c_uint8*len(raw))(*raw),n*2,out)
 assert out[3] and 69<=out[0]<=116,list(out)
 if n==7: assert out[1]==5,list(out)
# This translated static throat has a safe distant entry curve but its
# near ordinary corridor cannot join it continuously. Repetition must not
# force entry; actual progressing entrances are checked by check-handoff.py.
assert out[1]==5, ('incompatible stationary handoff must stay aligned',list(out))
# ALIGN now publishes a measured ordinary curve, without authorizing entry.
path=(C.c_uint8*(160*7))();count=d.replay_entry_path(path)
assert (count==0 or count>=12) and out[1]==5
for i in range(count):
 assert raw[path[i*7+1]*188+path[i*7]]>100, ('alignment curve outside current road',i)
# Loss must revoke guidance even after entry, and stripes suppress entry aid.
raw[80*188:106*188]=bytes([20])*(26*188)
d.replay_ring_step((C.c_uint8*len(raw))(*raw),42,out)
assert out[3]==0,list(out)
d.replay_ring_reset()
for n in range(10):
 raw=scene(stripe=True);d.replay_ring_step((C.c_uint8*len(raw))(*raw),n*2,out)
 assert out[1]==0,list(out)
# Confirmed cross island supplies the current connected corridor to steering.
cross = scene()
for y in range(56, 88):
 cross[y*188+83:y*188+105]=bytes([20])*22
d.replay_ring_reset()
assert frame(cross)[0], 'primary vision alone would drive'
d.replay_ring_reset()
for n in range(5):
 d.replay_ring_step((C.c_uint8*len(cross))(*cross),n*2,out)
scan=(C.c_int*5)()
d.replay_scan_status(scan)
assert scan[0]==1 and scan[4] and out[1]==0 and out[3]==1,(list(scan),list(out))
assert d.replay_cross_phase()==1 and abs(out[2])>=8,(list(scan),list(out))
exit_view=scene()
offset=28 if out[2]<0 else -28
for y in range(86,106):
 exit_view[y*188+83+offset:y*188+105+offset]=bytes([20])*22
d.replay_ring_step((C.c_uint8*len(exit_view))(*exit_view),10,out)
assert d.replay_cross_phase()==2 and out[3]==1,list(out)
for n in range(6,16):
 raw=scene()
 d.replay_ring_step((C.c_uint8*len(raw))(*raw),n*2,out)
 assert out[3]==1,list(out)
assert d.replay_cross_phase()==0 and abs(out[2])<=1,list(out)
print('PASS: confirmed cross island drives the selected corridor, exits and restores ordinary steering')

# Same centered island, but its far end is above the scanner's first row.
d.replay_ring_reset()
clipped_cross=scene()
for y in range(28,88):
 clipped_cross[y*188+83:y*188+105]=bytes([20])*22
for n in range(3):
 raw=scene()
 d.replay_ring_step((C.c_uint8*len(raw))(*raw),n*2,out)
for n in range(3,9):
 d.replay_ring_step((C.c_uint8*len(clipped_cross))(*clipped_cross),n*2,out)
d.replay_scan_status(scan)
assert scan[0]==1 and d.replay_cross_phase()==1 and out[3]==1 and abs(out[2])>=8,(list(scan),list(out))
print('PASS: cross-island entry stays available when the far merge is outside the scan')

def cross_step(raw, n):
 d.replay_ring_step((C.c_uint8*len(raw))(*raw),n*2,out)
 d.replay_scan_status(scan)
 return scan[0],d.replay_cross_phase(),int(out[1]),int(out[3])

# Four sampled split rows and one bounded near merge remain visible.
# Full-width white rows are skipped in the scanner's coverage count.
limited_cross=scene()
limited_cross[:72*188]=bytes([220])*(72*188)
limited_cross[105*188:106*188]=bytes([220])*188
for y in range(72,94):
 limited_cross[y*188+83:y*188+105]=bytes([20])*22
d.replay_ring_reset()
for n in range(2):
 result=cross_step(limited_cross,n)
 assert result[1]==0,('reduced coverage still needs three votes',result)
result=cross_step(limited_cross,2)
assert result==(1,1,0,1),('five usable rows and measured corridor should enter',result)

blocked_cross=bytearray(limited_cross)
blocked_cross[96*188:97*188]=bytes([20])*188
d.replay_ring_reset()
for n in range(2): cross_step(limited_cross,n)
assert cross_step(blocked_cross,2)==(1,3,0,0), 'lower coverage cannot bypass a blocked near corridor'

limited_cross[99*188:100*188]=bytes([220])*188
d.replay_ring_reset()
for n in range(6): result=cross_step(limited_cross,n)
assert result[:2]==(3,0),('four usable rows must remain unconfirmed',result)

for offset in (-20,20):
 d.replay_ring_reset()
 shifted=scene(shift=offset)
 for y in range(56,88):
  shifted[y*188+83+offset:y*188+105+offset]=bytes([20])*22
 for n in range(6):
  result=cross_step(shifted,n)
 assert result==(1,1,0,1),('road-relative cross',offset,result)

d.replay_ring_reset()
for n in range(2): cross_step(cross,n)
interrupted=bytearray(cross)
interrupted[70*188:71*188]=bytes([20])*188
result=cross_step(interrupted,2)
assert result==(1,4,0,1),('short measured approach keeps entry pending',result)
assert cross_step(cross,3)==(1,1,0,1)

d.replay_ring_reset()
for n in range(2): cross_step(cross,n)
for n in (2,3): assert cross_step(interrupted,n)==(1,4,0,1)
assert cross_step(interrupted,4)==(1,3,0,0), 'persistent gap exhausts the approach window'
assert cross_step(cross,5)==(1,3,0,0), 'loss remains latched after the window expires'

d.replay_ring_reset()
for n in range(2): cross_step(cross,n)
interrupted=bytearray(cross)
interrupted[96*188:97*188]=bytes([20])*188
assert cross_step(interrupted,2)==(1,3,0,0), 'blocked near approach must stop'

d.replay_ring_reset()
for n in range(4): result=cross_step(real,n)
assert result[2]==5,('ring alignment fixture',result)
for n in range(4,10): result=cross_step(cross,n)
assert result==(1,1,0,1),('confirmed cross cancels uncommitted ring alignment',result)

d.replay_ring_reset()
d.replay_ring_scenario(2,C.c_float(0))
for n in range(6): result=cross_step(cross,n)
assert result[:3]==(1,0,2),('confirmed cross cannot take over during orbit',result)
print('PASS: road-relative entry, bounded measured retry, near loss and mode handoff')
# Guard actual production integration against omitting the measured-branch gate.
main=(src/'main.c').read_text(encoding='utf-8')
assert 'scan = rb_scan_frame(' in main
assert 'rb_scan_observe(&roundabout_state,island,scan_entry_allowed &&' in main
assert 'cross_drive_step(&cross, &scan,' in main
assert 'turn_error=rb_plan_entry_error(roundabout_state.phase,applied_ring_guide,' in main
assert 'img_get_preview_error(),img_get_roundabout_preview_valid(),' in main
assert 'steering_update_with_entry_bias(&steering_state,(int)turn_error,' in main
assert 'float entry_bias = cross_path.active || metric_control ? 0 : img_get_align_turn_bias();' in main
print('PASS: side-open/rectangular negatives; integrated distant entry, loss, stripe priority and production wiring')

d.replay_ring_reset();out=(C.c_float*5)()
for n in range(12):
 d.replay_ring_step((C.c_uint8*len(real))(*real),n*2,out)
 assert out[3] and 69<=out[0]<=116,('integrated captured frame',n,list(out))
 assert out[1]==(0 if n<2 else 5),('stationary corner must not commit',n,list(out))
print('PASS: actual captured entry remains valid through confirmation and guide activation')

# Current-frame evidence: missing banks stay unknown; every published target
# must lie on observed white road. Do not demand the removed historical ramp.
d.replay_reset(); d.replay_guide(-1); o=frame(real)
lines=(C.c_uint8*360)(); d.replay_lines(lines)
assert o[0], ('captured approach remains valid', o)
assert sum(lines[120+y]>0 for y in range(18,106))>=20
assert all(not lines[120+y] or real[y*188+lines[120+y]]>100 for y in range(18,106)), 'target outside observed road'
assert all(x==0 or 2<x<185 for x in list(lines[:120])+list(lines[240:])), 'crop edge is not a bank'
assert d.replay_evidence()==0x3e and d.replay_corner_row()==93
for curvature in [-35,-25,-18,18,25,35]:
 raw=bytearray([20]*(188*120))
 for y in range(18,106):
  hw=int(71+32*(y-90)/119+0.5); cx=int(94+curvature*((105-y)/87.0)**1.4)
  for x in range(max(0,cx-hw),min(187,cx+hw)+1):raw[y*188+x]=220
 d.replay_reset()
 for _ in range(8):frame(raw);assert not d.replay_island(),('single-side curve fp',curvature)
print('PASS: current white-road targets, unknown crop boundaries, auxiliary evidence and curve negatives')

# A nearly uniform bright road may contain texture. Splitting that bright
# population must not create near-field boundary evidence.
for road_level in [220,250]:
 d.replay_reset();d.replay_guide(-1);d.replay_entry_mode(1)
 base=bytearray(road_level if value==220 else value for value in scene())
 for _ in range(3):assert frame(base)[0],('measured road primes texture test',road_level)
 raw=bytearray(base)
 for y in range(65,106):
  for x in range(188):raw[y*188+x]=248+2*((7*x+3*y)%4)
 for _ in range(3):
  o=frame(raw)
  assert not o[0],('bright road texture cannot authorize a near corridor',road_level,o)
print('PASS: bright-only textured bands cannot manufacture current near boundaries')
