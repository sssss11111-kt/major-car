"""Verify real wired original-resolution Seekfree V2 camera packets.
Stopped-only UIMG2 commands are sent; firmware rejects them while running.
No start, stop, reset or flash command is sent. Motor power must remain off.
"""
from pathlib import Path
import argparse,hashlib,json,re,serial,struct,time,zlib
ap=argparse.ArgumentParser();ap.add_argument('--port',default='COM8');ap.add_argument('--baud',type=int,default=460800);ap.add_argument('--output',type=Path,required=True);ap.add_argument('--radio-control',action='store_true');a=ap.parse_args();a.output.mkdir(parents=True,exist_ok=False)
h=bytearray([0xaa,0,2,2,188,0,120,0,1,0,0,0]);h[1]=sum(h)&255;h=bytes(h)
raw=bytearray();buf=bytearray();frames=[];error=None;commands=0;times=[];start=time.monotonic()
def png(gray):
 def chunk(t,b):return struct.pack('>I',len(b))+t+b+struct.pack('>I',zlib.crc32(t+b))
 scan=b''.join(b'\0'+gray[y*188:(y+1)*188] for y in range(120))
 return b'\x89PNG\r\n\x1a\n'+chunk(b'IHDR',struct.pack('>IIBBBBB',188,120,8,0,0,0,0))+chunk(b'IDAT',zlib.compress(scan))+chunk(b'IEND',b'')
try:
 if a.radio_control:
  with serial.Serial('COM6',115200,timeout=.1,write_timeout=1) as radio:
   rb=bytearray();deadline=time.monotonic()+6
   while time.monotonic()<deadline:
    rb.extend(radio.read(max(1,min(radio.in_waiting,8192))))
    stopped=re.search(rb'C11LAP29SF2 t=\d+ run=0 fault=\d+ frame=\d+ good=\d+ age=\d+ e1=-?\d+ e2=-?\d+ p1=0 p2=0',rb)
    if stopped:
     radio.write(b'\nUIMG2\n');radio.flush();break
   else:raise RuntimeError('No fresh stopped zero-output SF2 telemetry on COM6')
   (a.output/'radio-control.bin').write_bytes(rb)
 with serial.Serial(a.port,a.baud,timeout=.1,write_timeout=1) as p:
  end=time.monotonic()+15;next_command=0
  while time.monotonic()<end:
   if commands<3 and not frames and time.monotonic()>=next_command:
    p.write(b'\nUIMG2\n');p.flush();commands+=1;next_command=time.monotonic()+3
   data=p.read(max(1,min(p.in_waiting,32768)));raw.extend(data);buf.extend(data)
   while True:
    i=buf.find(h)
    if i<0:
     if len(buf)>12:del buf[:-12]
     break
    if len(buf)<i+12+22560:break
    gray=bytes(buf[i+12:i+12+22560]);del buf[:i+12+22560];n=len(frames)
    (a.output/f'gray-{n:02d}.bin').write_bytes(gray);(a.output/f'gray-{n:02d}.png').write_bytes(png(gray));times.append(time.monotonic())
    frames.append({'index':n,'sha256':hashlib.sha256(gray).hexdigest(),'min':min(gray),'max':max(gray)})
    print('Full 188x120 image '+str(len(frames)),flush=True)
   if len(frames)>=3:break
  if not frames:raise TimeoutError('No complete V2 188x120 packets received')
except Exception as e:
 error=str(e)
finally:
 (a.output/'serial.bin').write_bytes(raw)
 result={'port':a.port,'baud':a.baud,'radio_control_used':a.radio_control,'elapsed_s':time.monotonic()-start,'bytes':len(raw),'commands_sent':commands,'command':'UIMG2','frames':frames,'width':188,'height':120,'success':len(frames)>=3,'observed_fps':(len(times)-1)/(times[-1]-times[0]) if len(times)>1 else None,'error':error,'protocol_body_crc_available':False}
 (a.output/'result.json').write_text(json.dumps(result,indent=2),encoding='utf-8');print(json.dumps(result),flush=True)
if error:raise SystemExit(1)
