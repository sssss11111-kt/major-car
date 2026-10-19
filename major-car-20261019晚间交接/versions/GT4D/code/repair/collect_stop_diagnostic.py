"""Stop first, then read retained snapshot/diagnostics. No motion command."""
import argparse,json,time
from pathlib import Path
import serial
from serial.tools import list_ports
p=argparse.ArgumentParser();p.add_argument('--port',required=True);p.add_argument('--out',type=Path,required=True);a=p.parse_args()
ports=[p for p in list_ports.comports() if p.device.upper()==a.port.upper()]
if len(ports)!=1 or (ports[0].vid,ports[0].pid)!=(0x2e3c,0x5740):raise RuntimeError('Wrong wireless port')
a.out.mkdir(parents=True,exist_ok=False);raw=bytearray();events=[]
with serial.Serial(a.port,115200,timeout=.03,write_timeout=.2) as port:
 start=time.monotonic()
 def send(c):port.write(c.encode());events.append({'command':c,'t':time.monotonic()-start})
 send('s');schedule=[(.15,'Z'),(.8,'Z'),(1.4,'q'),(2.1,'p'),(2.5,'p'),(3.,'Z'),(3.6,'p')];end=start+4.3
 while time.monotonic()<end:
  while schedule and time.monotonic()-start>=schedule[0][0]:send(schedule.pop(0)[1])
  chunk=port.read(max(1,min(port.in_waiting,4096)))
  if chunk:events.append({'rx_t':time.monotonic()-start,'offset':len(raw),'length':len(chunk)});raw.extend(chunk)
(a.out/'serial.bin').write_bytes(raw);text=raw.decode('ascii','replace');(a.out/'serial.txt').write_text(text,encoding='utf-8');(a.out/'events.json').write_text(json.dumps(events,indent=2),encoding='utf-8')
lines=[x for x in text.splitlines() if x.startswith(('DIAG-STOP','UART8-DIAG','ms='))]
print('\n'.join(lines[-14:]))
