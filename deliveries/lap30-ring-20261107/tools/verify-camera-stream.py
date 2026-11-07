"""Enable stopped-only V2 wireless camera mode and verify three real packets.
No reset/start/bench/stop command is sent. Releases serial for Seekfree UI.
"""
from pathlib import Path
import argparse,importlib.util,json,re,time,hashlib
import serial
ap=argparse.ArgumentParser();ap.add_argument('--port',default='COM6');ap.add_argument('--output',type=Path,required=True);a=ap.parse_args();a.output.mkdir(parents=True,exist_ok=False)
spec=importlib.util.spec_from_file_location('rd',Path(__file__).with_name('receive-diagnostic.py'));rd=importlib.util.module_from_spec(spec);spec.loader.exec_module(rd)
pat=re.compile(rb'C11LAP29SF t=\d+ run=(\d+) fault=\d+ frame=\d+ good=\d+ age=\d+ e1=-?\d+ e2=-?\d+ p1=(-?\d+) p2=(-?\d+)\r?\n')
expected=bytes.fromhex('aa4902025e003c0001000000');raw=bytearray();buf=bytearray();frames=[];sent=False;error=None;last_status=None
try:
 p=serial.Serial(port=None,baudrate=115200,timeout=.2,write_timeout=1);p.dtr=False;p.rts=False;p.port=a.port
 with p:
  end=time.monotonic()+90
  while time.monotonic()<end:
   data=p.read(max(1,min(p.in_waiting,8192)));raw.extend(data);buf.extend(data)
   if not sent:
    matches=list(pat.finditer(buf))
    if matches:
     last_status=matches[-1].group().decode().strip();values=matches[-1].groups()
     if values==(b'0',b'0',b'0'):
      p.write(b'\nWIMG2\n');p.flush();sent=True;buf.clear();print('Verified stopped; WIMG2 sent',flush=True)
    if len(buf)>16384:del buf[:-4096]
   else:
    while True:
     i=buf.find(expected)
     if i<0:
      if len(buf)>12:del buf[:-12]
      break
     if len(buf)<i+12+5640:break
     gray=bytes(buf[i+12:i+12+5640]);del buf[:i+12+5640]
     n=len(frames);(a.output/f'gray-{n:02d}.bin').write_bytes(gray)
     # Native 94x60 PNG, without resampling.
     import struct,zlib
     def chunk(t,b):return struct.pack('>I',len(b))+t+b+struct.pack('>I',zlib.crc32(t+b))
     scan=b''.join(b'\0'+gray[y*94:(y+1)*94] for y in range(60))
     png=b'\x89PNG\r\n\x1a\n'+chunk(b'IHDR',struct.pack('>IIBBBBB',94,60,8,0,0,0,0))+chunk(b'IDAT',zlib.compress(scan))+chunk(b'IEND',b'')
     (a.output/f'gray-{n:02d}.png').write_bytes(png)
     frames.append({'index':n,'sha256':hashlib.sha256(gray).hexdigest(),'min':min(gray),'max':max(gray)})
     print('Valid V2 packet '+str(len(frames)),flush=True)
    if len(frames)>=3:break
  if len(frames)<3:raise TimeoutError('Not enough complete image packets; last state '+str(last_status))
except Exception as e:
 error=str(e);raise
finally:
 (a.output/'serial.bin').write_bytes(raw)
 result={'port':a.port,'baud':115200,'command_sent':sent,'last_stopped_status':last_status,'protocol':'Seekfree V2','width':94,'height':60,'frames':frames,'success':len(frames)>=3,'error':error,'view_mode_left_enabled':sent,'body_crc_available':False}
 (a.output/'result.json').write_text(json.dumps(result,indent=2),encoding='utf-8');print(json.dumps(result),flush=True)
