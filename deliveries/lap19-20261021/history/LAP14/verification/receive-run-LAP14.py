import serial, time, json, pathlib, datetime
from serial.tools import list_ports
root=pathlib.Path(__file__).resolve().parent
out=root/('run-'+datetime.datetime.now().strftime('%Y%m%d-%H%M%S'))
out.mkdir()
print(json.dumps({'ports':[{'port':p.device,'description':p.description} for p in list_ports.comports()]},ensure_ascii=True),flush=True)
with serial.Serial('COM9',115200,timeout=0.25) as port, (out/'raw.bin').open('wb') as raw, (out/'telemetry.txt').open('w',encoding='utf-8') as log:
 print('LISTENING COM9 115200; '+str(out),flush=True)
 start=last=time.monotonic(); total=0; buf=b''; recent=[]
 try:
  while time.monotonic()-start<900:
   data=port.read(max(1,min(port.in_waiting,8192)))
   if data:
    raw.write(data);raw.flush();total+=len(data);buf+=data
    while b'\n' in buf:
     line,buf=buf.split(b'\n',1)
     txt=line.decode('utf-8',errors='replace').strip()
     if txt and sum(c.isprintable() and c!='\ufffd' for c in txt)/len(txt)>.85:
      stamped=f'{time.monotonic()-start:.3f} {txt}'
      log.write(stamped+'\n');log.flush();recent.append(stamped)
    if len(buf)>65536:buf=buf[-4096:]
   if time.monotonic()-last>=10:
    print(json.dumps({'elapsed_s':round(time.monotonic()-start,1),'bytes':total,'lines':recent[-15:]},ensure_ascii=True),flush=True);recent=[];last=time.monotonic()
 except KeyboardInterrupt: pass
 print(json.dumps({'finished':True,'bytes':total,'folder':str(out)},ensure_ascii=True),flush=True)
