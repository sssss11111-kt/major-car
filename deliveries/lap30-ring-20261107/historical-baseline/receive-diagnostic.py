"""Receive exact stopped-only RD29 frame exports; never starts/stops/resets car.
Default is passive. --auto-export sends TRACE only after three C11LAP29D1
stopped reports with zero motor PWM. A REQUEST_TRACE file is an explicit manual
export request and requires the same observed stopped state.
"""
from pathlib import Path
import argparse,csv,json,os,re,struct,time,zlib

FIELDS='frame tick running fault phase_before phase_after confirm_before confirm_after candidate_before near_before valid_before rows_before near_rows_before stripe_before valid_after rows_after near_after near_rows_after both_after evidence corner island_bottom entry_error near_error preview_error turn_error10 command10 slew guide just_entered route yaw10 e1 e2 p1 p2 process_us copy_us ready_cycle threshold'.split()
SIZE=23080
HEADER=re.compile(rb'RD29 FRAME i=(\d+) n=(\d+) crc=([0-9a-f]{8})\r?\n')
STATUS=re.compile(r'C11LAP29D1 t=(\d+) run=(\d+) fault=(\d+) frame=(\d+) good=(\d+) age=(\d+) e1=(-?\d+) e2=(-?\d+) p1=(-?\d+) p2=(-?\d+)')

def png(gray):
 def chunk(t,p):return struct.pack('>I',len(p))+t+p+struct.pack('>I',zlib.crc32(t+p))
 data=b''.join(b'\0'+gray[y*188:(y+1)*188] for y in range(120))
 return b'\x89PNG\r\n\x1a\n'+chunk(b'IHDR',struct.pack('>IIBBBBB',188,120,8,0,0,0,0))+chunk(b'IDAT',zlib.compress(data))+chunk(b'IEND',b'')

class Decoder:
 def __init__(self,out):
  self.out=Path(out);self.buf=b'';self.frames={};self.errors=[];self.begin=None;self.ended=False;self.stopped=0;self.lines=[];self.seen_running=False
 def feed(self,data):
  self.buf+=data
  while self.buf:
   m=HEADER.match(self.buf)
   if m:
    i,n,crc=int(m[1]),int(m[2]),int(m[3],16)
    if n!=SIZE or i>=96:
     self.errors.append('invalid header');self.buf=self.buf[1:];continue
    if len(self.buf)<m.end()+n:break
    payload=self.buf[m.end():m.end()+n]
    if zlib.crc32(payload)!=crc:
     self.errors.append('CRC failed frame '+str(i));self.buf=self.buf[1:];continue
    if i in self.frames and self.frames[i]['crc']!=crc:
     self.errors.append('conflicting frame '+str(i))
    meta=dict(zip(FIELDS,struct.unpack('<40i',payload[:160])))
    meta.update(index=i,crc=crc)
    self.frames[i]=meta
    (self.out/f'frame-{i:03d}.bin').write_bytes(payload)
    (self.out/f'frame-{i:03d}.png').write_bytes(png(payload[160:160+22560]))
    self.buf=self.buf[m.end()+n:];continue
   k=self.buf.find(b'\n')
   if k<0:
    if len(self.buf)>SIZE*2:self.buf=self.buf[-512:]
    break
   line=self.buf[:k].decode('ascii',errors='replace').strip();self.buf=self.buf[k+1:]
   if len(line)<512:self.lines.append(line)
   if line.startswith('RD29 BEGIN '):
    self.begin={k:int(v) for k,v in re.findall(r'(\w+)=(\d+)',line)}
   elif line=='RD29 END':self.ended=True
   else:
    s=STATUS.fullmatch(line)
    if s:
     vals=list(map(int,s.groups()))
     self.seen_running |= vals[1]==1
     self.stopped=self.stopped+1 if vals[1]==0 and vals[-2:]==[0,0] else 0
 def save(self):
  expected=self.begin.get('count') if self.begin else None
  rows=[self.frames[i] for i in sorted(self.frames)]
  summary={'expected':expected,'received':len(rows),'complete':self.ended and expected==len(rows) and not self.errors,'errors':self.errors,'begin':self.begin,'ended':self.ended,'rows':rows}
  (self.out/'decoded.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2),encoding='utf-8')
  if rows:
   with (self.out/'aligned.csv').open('w',encoding='utf-8-sig',newline='') as f:
    w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
  (self.out/'telemetry.txt').write_text('\n'.join(self.lines)+'\n',encoding='utf-8')
  doc='''<!doctype html><meta charset="utf-8"><title>环岛入口逐帧诊断</title>
<style>body{font:16px system-ui;max-width:1100px;margin:30px auto;background:#f4f6f8}canvas{width:752px;height:480px;image-rendering:pixelated;background:#111}pre{white-space:pre-wrap}input{width:750px}</style>
<h1>环岛入口逐帧诊断</h1><p>同帧原始灰度图与最终左/中/右线；绿色为目标中线。舵机数值是命令，不是实测轮角。</p>
<input id="pos" type="range" min="0" value="0"><p id="label"></p><canvas id="view" width="188" height="120"></canvas><pre id="detail"></pre>
<script>const rows=ROWS;const slider=document.getElementById('pos');slider.max=Math.max(0,rows.length-1);
const ctx=document.getElementById('view').getContext('2d');
async function show(){let r=rows[+slider.value];if(!r)return;document.getElementById('label').textContent=`索引 ${r.index} · frame ${r.frame} · tick ${r.tick} · phase ${r.phase_before}→${r.phase_after} · confirm ${r.confirm_before}→${r.confirm_after} · candidate ${r.candidate_before} · near ${r.near_before} · command ${r.command10/10}`;
document.getElementById('detail').textContent=JSON.stringify(r,null,2);let im=new Image();im.src=`frame-${String(r.index).padStart(3,'0')}.png`;await im.decode();ctx.drawImage(im,0,0);
try{let b=new Uint8Array(await(await fetch(`frame-${String(r.index).padStart(3,'0')}.bin`)).arrayBuffer());['#f65b59','#24db63','#51a9ff'].forEach((c,k)=>{ctx.fillStyle=c;for(let y=0;y<106;y++){let x=b[160+22560+k*120+y];if(x>0&&x<187)ctx.fillRect(x,y,1,1)}})}catch(e){} }
slider.oninput=show;show();</script>'''
  # Embed all line coordinates: file:// viewers can block fetch of binary files.
  for r in rows:
   payload=(self.out/f"frame-{r['index']:03d}.bin").read_bytes();r['lines']=list(payload[160+22560:])
  doc=doc.replace('ROWS',json.dumps(rows)).replace("try{let b=new Uint8Array(await(await fetch(`frame-${String(r.index).padStart(3,'0')}.bin`)).arrayBuffer());",'try{let b=r.lines;').replace('b[160+22560+k*120+y]','b[k*120+y]')
  (self.out/'viewer.html').write_text(doc,encoding='utf-8')
  return summary

def main():
 import serial
 ap=argparse.ArgumentParser();ap.add_argument('--port',default='COM6');ap.add_argument('--output',type=Path,required=True);ap.add_argument('--seconds',type=float,default=900);ap.add_argument('--auto-export',action='store_true');args=ap.parse_args()
 args.output.mkdir(parents=True,exist_ok=False);d=Decoder(args.output);start=last=time.monotonic();last_data=start;total=0;requested=False;failure=None
 try:
  port=serial.Serial(port=None,baudrate=115200,timeout=.2);port.dtr=False;port.rts=False;port.port=args.port
  with port,(args.output/'wire.bin').open('ab',buffering=0) as raw:
   print('PASSIVE RECORDING '+str(args.output)+'; TRACE only after verified stop; no START/STOP/RESET',flush=True)
   while time.monotonic()-start<args.seconds and not (args.output/'STOP_RECORDING').exists():
    data=port.read(max(1,min(port.in_waiting,16384)))
    if data:
     raw.write(data);os.fsync(raw.fileno());total+=len(data);last_data=time.monotonic();d.feed(data)
    manual=(args.output/'REQUEST_TRACE').exists()
    if not requested and ((args.auto_export and d.seen_running) or manual) and d.stopped>=3:
     port.write(b'\nTRACE\n');port.flush();requested=True;print('TRACE requested after three zero-output stopped states',flush=True)
    if d.ended:break
    if time.monotonic()-last>=10:
     print(json.dumps({'bytes':total,'crc_valid_frames':len(d.frames),'stopped_samples':d.stopped,'trace_requested':requested}),flush=True);last=time.monotonic()
   summary=d.save();summary.update(wire_bytes=total,wire_file_bytes=raw.tell(),trace_requested=requested)
 except Exception as e:
  failure=repr(e);summary=d.save();summary.update(error=failure,wire_bytes=total)
 finally:
  (args.output/'result.json').write_text(json.dumps(summary,indent=2),encoding='utf-8');print(json.dumps({k:v for k,v in summary.items() if k!='rows'}),flush=True)
 if failure:raise SystemExit(failure)
if __name__=='__main__':main()
