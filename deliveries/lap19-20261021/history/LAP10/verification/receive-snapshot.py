from pathlib import Path
from datetime import datetime
import argparse,json,re,struct,time,zlib

class Snapshot:
 def __init__(self): self.header=None; self.data=bytearray(); self.complete=False
 def feed(self,line):
  if line=='SNAP9_ABORT': raise ValueError('board aborted transfer')
  if line.startswith('SNAP9 '):
   h=dict(re.findall(r'(\w+)=(\w+)',line))
   self.header={k:int(v,16 if k=='crc' else 10) for k,v in h.items()}
   assert self.header['w']==188 and self.header['h']==120 and self.header['n']==22920
   self.data=bytearray();self.complete=False
  elif line.startswith('PX9 ') and self.header:
   m=re.fullmatch(r'PX9 off=(\d+) data=([0-9a-f]+)',line)
   if not m or int(m[1])!=len(self.data):raise ValueError('missing, reordered or malformed chunk')
   b=bytes.fromhex(m[2]);assert 1<=len(b)<=32
   self.data.extend(b)
   if len(self.data)>self.header['n']:raise ValueError('payload overflow')
  elif line.startswith('END9 ') and self.header:
   m=re.fullmatch(r'END9 frame=(\d+) crc=([0-9a-f]{8})',line)
   if not m or int(m[1])!=self.header['frame']:raise ValueError('frame mismatch')
   if len(self.data)!=self.header['n'] or zlib.crc32(self.data)!=self.header['crc'] or int(m[2],16)!=self.header['crc']:raise ValueError('length/CRC mismatch')
   self.complete=True

def png(path,data,w,h):
 def chunk(k,b):return struct.pack('>I',len(b))+k+b+struct.pack('>I',zlib.crc32(k+b)&0xffffffff)
 scan=b''.join(b'\0'+data[y*w:(y+1)*w] for y in range(h))
 path.write_bytes(b'\x89PNG\r\n\x1a\n'+chunk(b'IHDR',struct.pack('>IIBBBBB',w,h,8,0,0,0,0))+chunk(b'IDAT',zlib.compress(scan))+chunk(b'IEND',b''))

def save(s,out):
 h=s.header;raw=bytes(s.data[:22560]);binary=bytes(255 if b>h['threshold'] else 0 for b in raw)
 png(out/'gray.png',raw,188,120);png(out/'binary.png',binary,188,120)
 (out/'payload.bin').write_bytes(s.data)
 lines={n:list(s.data[22560+i*120:22560+(i+1)*120]) for i,n in enumerate(['left','mid','right'])}
 meta={**h,'crc_ok':True,'kind':'requested post-stop image, not historical fault frame','binary':'reconstructed using same-frame transmitted threshold and > comparison','lines':lines}
 (out/'capture.json').write_text(json.dumps(meta,indent=2),'utf-8')
 svg=''
 for name,color in [('left','red'),('mid','lime'),('right','cyan')]:
  pts=' '.join(str(x)+','+str(y) for y,x in enumerate(lines[name]));svg+=f'<polyline points="{pts}" fill="none" stroke="{color}" stroke-width="1"/>'
 html=f"""<!doctype html><meta charset="utf-8"><title>停车单帧</title><style>body{{background:#222;color:white;font-family:sans-serif}}.pair{{display:flex;gap:16px}}img,svg{{width:564px;height:360px;image-rendering:pixelated}}</style><h2>停车位置无线单帧 · frame {h['frame']} · tick {h['tick']} · 阈值 {h['threshold']}</h2><p>当前抓取画面，不是历史触发帧。左：灰度原图；右：二值图与车端边界，黄色横线为第90行。</p><div class="pair"><img src="gray.png"><svg viewBox="0 0 188 120"><image href="binary.png" width="188" height="120"/>{svg}<path d="M0 90H188" stroke="yellow" stroke-width="0.5"/></svg></div>"""
 (out/'view.html').write_text(html,'utf-8')

def selftest():
 payload=bytes(i%256 for i in range(22920));crc=zlib.crc32(payload)
 header=f'SNAP9 w=188 h=120 n=22920 threshold=100 frame=12 tick=24 crc={crc:08x}'
 chunks=[f'PX9 off={off} data={payload[off:off+32].hex()}' for off in range(0,len(payload),32)]
 end=f'END9 frame=12 crc={crc:08x}'
 s=Snapshot()
 for l in [header,*chunks,end]:s.feed(l)
 assert s.complete and s.data==payload
 for lines in [[header,*chunks[1:],end],[header,*chunks[:-1],end],[header,*chunks,end.replace(f'{crc:08x}','00000000')]]:
  try:
   s=Snapshot()
   for l in lines:s.feed(l)
  except (ValueError,AssertionError):pass
  else:raise AssertionError('corruption accepted')
 print('PASS: complete frame, missing chunk, truncation, bad CRC')

def main():
 ap=argparse.ArgumentParser();ap.add_argument('--port',default='COM6');ap.add_argument('--self-test',action='store_true');a=ap.parse_args()
 if a.self_test:selftest();return
 import serial
 out=Path(__file__).resolve().parent/('snapshot-'+datetime.now().strftime('%Y%m%d-%H%M%S'));out.mkdir()
 s=Snapshot();raw=bytearray();sent=False;pending=bytearray()
 def read_line(port):
  if b'\n' not in pending:pending.extend(port.read(max(1,port.in_waiting)))
  if b'\n' not in pending:return b''
  n=pending.index(10)+1;b=bytes(pending[:n]);del pending[:n];return b

 try:
  with serial.Serial(port=None,baudrate=115200,timeout=.2,write_timeout=1) as port:
   port.dtr=False;port.rts=False;port.port=a.port;port.open()
   deadline=time.monotonic()+8;ready=0
   while time.monotonic()<deadline:
    b=read_line(port);raw.extend(b);l=b.decode('ascii',errors='replace').strip()
    m=re.fullmatch(r'C11LAP10 t=\d+ run=(\d+) fault=\d+ frame=\d+ good=\d+ age=\d+ e1=-?\d+ e2=-?\d+ p1=(\d+) p2=(\d+)',l)
    if m:
     if m.groups()!=('0','0','0'):raise RuntimeError('vehicle is not idle')
     ready+=1
     if ready>=5:break
   if ready<5:raise RuntimeError('no verified LAP10 idle telemetry; SNAP not sent')
   for b in b'\nSNAP\n':port.write(bytes([b]));port.flush();time.sleep(.015)
   sent=True;deadline=time.monotonic()+45
   while time.monotonic()<deadline:
    b=read_line(port);raw.extend(b)
    if b:s.feed(b.decode('ascii',errors='replace').strip())
    if s.complete:break
   if not s.complete:raise TimeoutError('snapshot incomplete; no image accepted')
  save(s,out);print('Verified snapshot: '+str(out))
 finally:
  (out/'serial.bin').write_bytes(raw+pending)
  (out/'result.json').write_text(json.dumps({'snapshot_requested':sent,'crc_verified':s.complete,'received_bytes':len(s.data)},indent=2))
if __name__=='__main__':main()
