"""Passive wireless recording for user-operated R11 track testing."""
from pathlib import Path
from datetime import datetime, timezone
import json,time
import serial

folder=Path(__file__).resolve().parent/'wireless-01'
folder.mkdir(exist_ok=False)
root=folder.parent
state={'port':'COM6','baud':115200,'started_utc':datetime.now(timezone.utc).isoformat(),
       'running':True,'bytes':0,'commands':[],'error':None,'dtr':False,'rts':False}
def save():
    (folder/'status.json').write_text(json.dumps(state,indent=2),encoding='utf-8')
start=time.monotonic();save()
try:
    connection=serial.Serial(port=None,baudrate=115200,timeout=0.1)
    connection.dtr=False;connection.rts=False;connection.port='COM6';connection.open()
    print('COM6 opened115200, DTR/RTS disabled, passive recording active',flush=True)
    with connection,(folder/'raw.bin').open('wb') as raw,(folder/'events.jsonl').open('w',encoding='utf-8') as events:
        while time.monotonic()-start<3600 and not (root/'STOP-MONITOR').exists():
            request=root/'serial-command.txt'
            if request.exists():
                command=request.read_text(encoding='utf-8').strip()
                if command not in ['STOP','SNAP','TRACE','IMGOFF']:raise RuntimeError('Unsupported inspection command')
                connection.write((command+'\n').encode());connection.flush();request.unlink()
                state['commands'].append({'command':command,'elapsed_s':round(time.monotonic()-start,3)})
                print('Sent',command,flush=True)
            data=connection.read(connection.in_waiting or 1)
            if data:
                raw.write(data);raw.flush();state['bytes']+=len(data)
                events.write(json.dumps({'elapsed_s':round(time.monotonic()-start,3),'bytes':len(data)})+'\n');events.flush()
                print(data.decode('ascii',errors='replace')[-800:],end='',flush=True)
            save()
except Exception as exc:
    state['error']=str(exc)
    raise
finally:
    state.update(running=False,elapsed_s=round(time.monotonic()-start,3),finished_utc=datetime.now(timezone.utc).isoformat());save()
    print('\nWIRELESS FINAL',json.dumps(state),flush=True)
