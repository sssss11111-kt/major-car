from pathlib import Path
import ctypes as C,json,copy,hashlib
p=Path(__file__).resolve().parent
src=Path(r'D:\桌面\智能车\镜头组例程-C11-20261021\verification')
class Result(C.Structure):_fields_=[('mode',C.c_uint8),('center',C.c_uint8),('degraded',C.c_uint8),('model_matches',C.c_uint8),('since',C.c_uint32)]
dll=C.CDLL(str(p/'single_edge_test.dll'));f=dll.evaluate;f.restype=Result;f.argtypes=[C.POINTER(C.c_uint8)]*4+[C.c_uint32,C.c_uint8]
def fixture(name):
 d=src/('snapshot-20261021-'+name);c=json.loads((d/'capture.json').read_text());raw=(d/'payload.bin').read_bytes()[:22560]
 return [[255 if b>c['threshold'] else 0 for b in raw],c['lines']['left'],c['lines']['mid'],c['lines']['right']]
def call(data,t=100,allow=1):
 arrays=[(C.c_uint8*len(a))(*a) for a in data];return f(*arrays,t,allow)
dual=fixture('131631');single=fixture('132210');passed=[]
def reset():call(dual,0,0)
def test(name,fn):
 reset();fn();passed.append(name)
def check(cond):assert cond
for name,center in [('131411',95),('131631',90),('131850',90)]:
 test('dual preserved '+name,lambda name=name,center=center:check((call(fixture(name)).mode,call(fixture(name)).center)==(1,center)))
test('measured left produces 103',lambda:check((call(single).mode,call(single).center,call(single).degraded)==(2,103,1)))
test('stopped single cannot authorize RUN',lambda:check(call(single,100,0).mode==0))
reflected=[[x for y in range(120) for x in single[0][y*188:(y+1)*188][::-1]],[187-x for x in single[3]],[187-x for x in single[2]],[187-x for x in single[1]]]
test('right-only mirror support',lambda:check((call(reflected).mode,call(reflected).center)==(3,84)))
def timeout():
 assert call(single,100).mode==2 and call(single,149).mode==2
 assert call(single,150).mode==0
 assert call(dual,151).mode==0
 assert call(dual,152,0).mode==1
 assert call(single,153).mode==2
test('bounded episode and stop/reset',timeout)
def flicker():
 assert call(single,100).mode==2
 for t in range(110,150,5):
  assert call(dual,t).degraded==1
  assert call(single,t+1).mode==2
 assert call(single,150).mode==0
test('one-frame dual flicker cannot reset deadline',flicker)
def recovery():
 call(single,100)
 assert call(dual,110).degraded==1 and call(dual,112).degraded==1
 assert call(dual,114).degraded==0
 assert call(single,151).mode==2 and call(single,151).since==151
test('three reliable dual frames recover',recovery)
def wrap():
 call(single,0xfffffff0)
 assert call(single,0x21).mode==2
 assert call(single,0x22).mode==0
test('uint32 tick wrap',wrap)
both=copy.deepcopy(single);both[1]=[0]*120;both[3]=[187]*120
test('both missing rejected',lambda:check(call(both).mode==0))
bad=copy.deepcopy(single);bad[1][89]=0
test('missing neighborhood edge rejected',lambda:check(call(bad).mode==0))
bad2=copy.deepcopy(single);bad2[1][89]+=20
test('edge discontinuity rejected',lambda:check(call(bad2).mode==0))
dark=copy.deepcopy(single);dark[0][90*188+103]=0
test('dark target rejected',lambda:check(call(dark).mode==0))
blocked=copy.deepcopy(single)
for x in range(55,80):blocked[0][90*188+x]=0
test('dark path rejected',lambda:check(call(blocked).mode==0))
outside=copy.deepcopy(single)
for y in range(88,93):outside[1][y]=150
test('offscreen prediction rejected',lambda:check(call(outside).mode==0))
def switch():
 call(single,100);assert call(reflected,102).mode==0
test('side switching rejected in episode',switch)
mismatch=copy.deepcopy(dual)
for y in range(88,93):mismatch[3][y]+=12;mismatch[2][y]=(mismatch[1][y]+mismatch[3][y])//2
test('startup width mismatch disarms model',lambda:check(call(mismatch,0,0).model_matches==0))
report={'implementation':'actual single_edge.c compiled as Windows DLL, same source as firmware','passed':passed,'case_count':len(passed),'scope':'host image/state logic; not MCU timing, mechanics or track validation','firmware_hex_sha256':hashlib.sha256((p.parent/'demo/mdk/Objects/rt1064.hex').read_bytes()).hexdigest()}
(p/'host-tests.json').write_text(json.dumps(report,indent=2),encoding='utf-8');print(json.dumps(report))
