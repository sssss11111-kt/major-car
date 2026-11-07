"""Read D1's known RAM after zero-output checks. No reset, flash or resume.
Only a matching same-frame SNAP CRC is accepted as a coherent current image.
Frozen per-frame trace records can also be recovered without slow radio export.
"""
from pathlib import Path
import argparse,importlib.util,json,re,struct,zlib
from pyocd.core.helpers import ConnectHelper
from pyocd.probe.aggregator import DebugProbeAggregator
ap=argparse.ArgumentParser();ap.add_argument('--output',type=Path,required=True);ap.add_argument('--snapshot-log',type=Path,required=True);a=ap.parse_args()
b=Path(__file__).resolve().parents[1];mapping=(b/'project/demo/mdk/Listings/rt1064.map').read_text(encoding='utf-8',errors='replace')
symbols={n:int(addr,16) for n,addr in re.findall(r'^\s+(\w+)\s+0x([0-9a-fA-F]+)\s+Data',mapping,re.M)}
a.output.mkdir(parents=True,exist_ok=False);result={'success':False,'reset':False,'resumed':False,'flash_written':False}
spec=importlib.util.spec_from_file_location('snap',Path(__file__).with_name('receive-stationary.py'));snap=importlib.util.module_from_spec(spec);spec.loader.exec_module(snap)
try:
 probes=DebugProbeAggregator.get_all_connected_probes();assert len(probes)==1,'Exactly one debugger required'
 session=ConnectHelper.session_with_chosen_probe(unique_id=probes[0].unique_id,target_override='mimxrt1064',frequency=1000000,options={'connect_mode':'attach','resume_on_disconnect':False})
 with session:
  t=session.target
  def read(n,size):return bytes(t.read_memory_block8(symbols[n],size))
  def num(n,size=4,signed=False):return int.from_bytes(read(n,size),'little',signed=signed)
  before={n:num(n,k,sign) for n,k,sign in [('trial_running',1,False),('trial_fault',1,False),('trial_left_pwm',4,True),('trial_right_pwm',4,True)]}
  result['before_halt']=before
  assert before['trial_running']==0 and before['trial_left_pwm']==before['trial_right_pwm']==0,'Vehicle not verified stopped; no halt performed'
  t.halt();result['halted']=True;result['pc']=hex(t.read_core_register('pc'))
  # Repeat after halt; motor outputs were already zero before the halt.
  assert num('trial_running',1)==0 and num('trial_left_pwm',4,True)==num('trial_right_pwm',4,True)==0
  count=num('frame_count');tick=num('frame_tick')
  payload=read('raw_image',22560)+read('left_line',120)+read('mid_line',120)+read('right_line',120)
  (a.output/'payload.bin').write_bytes(payload)
  result.update(frame=count,tick=tick,crc=f'{zlib.crc32(payload):08x}',state='HALTED')
  headers=re.findall(rb'SNAP9 w=188 h=120 n=22920 threshold=(\d+) frame=(\d+) tick=(\d+) crc=([0-9a-f]{8})',a.snapshot_log.read_bytes())
  assert headers,'No recorded SNAP header to establish same-frame coherence'
  threshold,frame,ftick,crc=headers[-1];result['recorded_snapshot_frame']=int(frame)
  assert count==int(frame) and tick==int(ftick) and zlib.crc32(payload)==int(crc,16),'RAM differs from SNAP header; image not accepted as processed same frame'
  s=snap.Snapshot();s.header={'w':188,'h':120,'n':22920,'threshold':int(threshold),'frame':count,'tick':tick,'crc':int(crc,16)};s.data=bytearray(payload);s.complete=True
  snap.save(s,a.output);result['snapshot_crc_matches_recorded_header']=True
  for name in ['s_ring_candidate','s_near_valid','s_valid','s_usable_rows','s_diag_near_rows']:
   result[name]=num(name,1)
  state=struct.unpack('<7I',read('rd29_state',28));w,n,triggered,after,frozen,trigger,reason=state
  result['trace_state']=dict(zip('write count triggered after frozen trigger_frame reason'.split(),state))
  if frozen and 0<n<=96 and w<96:
   trace=a.output/'frozen-trace';trace.mkdir()
   spec=importlib.util.spec_from_file_location('rd29',Path(__file__).with_name('receive-diagnostic.py'));rd=importlib.util.module_from_spec(spec);spec.loader.exec_module(rd)
   decoder=rd.Decoder(trace);decoder.begin={'count':n,'trigger':trigger,'reason':reason,'recovered_by_swd':1}
   for i in range(n):
    index=(w+96-n+i)%96
    data=bytes(t.read_memory_block8(symbols['rd29_frames']+index*23080,23080));crc=zlib.crc32(data)
    decoder.feed(f'RD29 FRAME i={i} n=23080 crc={crc:08x}\r\n'.encode()+data+b'\r\n')
   decoder.ended=True;summary=decoder.save();result['trace_complete']=summary['complete'];result['trace_frames']=len(decoder.frames)
  result['success']=True
except Exception as e:
 result['error']=str(e);raise
finally:
 (a.output/'result.json').write_text(json.dumps(result,indent=2),encoding='utf-8');print(json.dumps(result),flush=True)
