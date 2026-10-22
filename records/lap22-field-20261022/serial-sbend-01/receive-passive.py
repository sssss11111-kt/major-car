"""Passive recording only: never writes serial commands or starts the car."""
from pathlib import Path
from datetime import datetime, timezone
import argparse,json,re,time,serial

ap=argparse.ArgumentParser()
ap.add_argument('--port',required=True)
ap.add_argument('--output',type=Path,required=True)
ap.add_argument('--seconds',type=float,default=900)
args=ap.parse_args()
out=args.output.resolve();out.mkdir(parents=True,exist_ok=False)
start=time.monotonic();last=start;total=0;buf=b'';recent=[];status_count=0;run_seen=False;faults=set();error=None
stop_file=out/'STOP_RECORDING'
def save(finished=False):
    (out/'summary.json').write_text(json.dumps({'version_expected':'C11LAP22',
        'port':args.port,'baud':115200,'passive_only':True,'started_utc':started,
        'elapsed_s':round(time.monotonic()-start,2),'bytes':total,'complete_status_lines':status_count,
        'run_seen':run_seen,'faults_seen':sorted(faults),'finished':finished,'error':error},indent=2)+'\n',encoding='utf-8')
started=datetime.now(timezone.utc).isoformat()
try:
    port=serial.Serial(port=None,baudrate=115200,timeout=.25)
    port.dtr=False;port.rts=False;port.port=args.port
    with port, (out/'raw.bin').open('wb') as raw, (out/'telemetry.txt').open('w',encoding='utf-8') as log:
        print('LISTENING '+args.port+' 115200; PASSIVE ONLY; '+str(out),flush=True)
        save()
        while time.monotonic()-start<args.seconds and not stop_file.exists():
            data=port.read(max(1,min(port.in_waiting,8192)))
            if data:
                raw.write(data);raw.flush();total+=len(data);buf+=data
                while b'\n' in buf:
                    line,buf=buf.split(b'\n',1)
                    txt=line.decode('utf-8',errors='replace').rstrip('\r')
                    stamped=f'{time.monotonic()-start:.3f} {txt}'
                    log.write(stamped+'\n');log.flush()
                    if len(recent)<15:recent.append(stamped)
                    m=re.search(r'C11LAP22 t=\d+ run=(\d+) fault=(\d+) frame=\d+ good=\d+ age=\d+ e1=-?\d+ e2=-?\d+ p1=-?\d+ p2=-?\d+',txt)
                    if m:
                        status_count+=1;run_seen|=m[1]=='1';faults.add(int(m[2]))
                if len(buf)>65536:buf=buf[-4096:]
            if time.monotonic()-last>=10:
                save();print(json.dumps({'bytes':total,'status_lines':status_count,'recent':recent}),flush=True)
                recent=[];last=time.monotonic()
except Exception as e:
    error=repr(e);raise
finally:
    save(True)
    print((out/'summary.json').read_text(),flush=True)
