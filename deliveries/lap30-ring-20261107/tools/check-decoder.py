"""Protocol integrity and stopped-gate checks, with no hardware access."""
from pathlib import Path
import importlib.util,struct,tempfile,zlib
p=Path(__file__).with_name('receive-diagnostic.py');spec=importlib.util.spec_from_file_location('rd29',p);m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
payload=struct.pack('<40i',*range(40))+bytes([123])*22920
crc=zlib.crc32(payload)
frame=f'RD29 FRAME i=0 n=23080 crc={crc:08x}\r\n'.encode()+payload+b'\r\n'
wire=b'RD29 BEGIN format=1 count=1 bytes=23080 trigger=9 reason=1\r\n'+frame+b'RD29 END\r\n'
with tempfile.TemporaryDirectory() as t:
 d=m.Decoder(t)
 for i in range(0,len(wire),13):d.feed(wire[i:i+13])
 assert len(d.frames)==1 and d.ended and not d.errors
 s=d.save();assert s['complete'];assert (Path(t)/'viewer.html').is_file()
 assert (Path(t)/'frame-000.png').read_bytes().startswith(b'\x89PNG')
 assert (Path(t)/'frame-000.bin').read_bytes()==payload
with tempfile.TemporaryDirectory() as t:
 d=m.Decoder(t);damaged=bytearray(frame);damaged[100]^=1;d.feed(bytes(damaged)+b'RD29 END\r\n')
 assert not d.frames and d.errors and not d.save()['complete']
with tempfile.TemporaryDirectory() as t:
 d=m.Decoder(t);d.feed(wire[:-500]);assert not d.ended and not d.frames
 stopped=b'C11LAP29D1 t=10 run=0 fault=0 frame=1 good=1 age=0 e1=0 e2=0 p1=0 p2=0\r\n'
 d=m.Decoder(t);d.feed(stopped*3);assert d.stopped==3 and not d.seen_running
 d.feed(stopped.replace(b'run=0',b'run=1'));assert d.seen_running and d.stopped==0
 d.feed(stopped*3);assert d.stopped==3 and d.seen_running
print('PASS: fragmented binary CRC export, loss rejection, truncation rejection, PNG/viewer output, no auto-export before running')
