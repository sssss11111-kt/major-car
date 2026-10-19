"""Offline archive integrity and recorded-data checks. Never accesses hardware."""
from pathlib import Path
import hashlib
import json
import math
import re

root=Path(__file__).resolve().parents[1]
def read(path): return json.loads((root/path).read_text(encoding='utf-8-sig'))
def check(ok,reason):
    if not ok: raise RuntimeError(reason)
def hex_size(path):
    data={};upper=0
    for line in path.read_text(encoding='ascii').splitlines():
        check(line.startswith(':'),'HEX prefix')
        raw=bytes.fromhex(line[1:]);check(sum(raw)%256==0,'HEX checksum')
        n,address,kind=raw[0],int.from_bytes(raw[1:3],'big'),raw[3]
        check(len(raw)==n+5,'HEX length')
        if kind==4:upper=int.from_bytes(raw[4:6],'big')<<16
        elif kind==2:upper=int.from_bytes(raw[4:6],'big')<<4
        elif kind==0:
            for i,value in enumerate(raw[4:4+n]):data[upper+address+i]=value
    return len(data)

for item in read('firmware-index.json'):
    path=root/item['path'];check(path.is_file(),'Missing firmware')
    check(hashlib.sha256(path.read_bytes()).hexdigest()==item['sha256'],'Firmware SHA mismatch')
    check(hex_size(path)==item['effective_bytes'],'Firmware effective size mismatch')
    if item['version']!='C1':
        flash=read(f'versions/{item["version"]}/evidence/pyocd-flash-result.json')
        check(flash['readback_verified'] and flash['image_sha256']==item['sha256'],'Flash proof mismatch')
        check(flash['verified_bytes']==item['effective_bytes'],'Flash byte count mismatch')

summary=read('versions/L3/low-summary.json');check(len(summary)==7,'L3 trial count')
ids=[]
for row in summary:
    check(row['measurement_complete'] and row['final_zero'] and not row['failure'],'Incomplete L3 trial')
    c=row['completion'];ids.append(c['id']);check(c['done']==1 and c['elapsed']==6000,'L3 completion')
    raw=read(row['evidence']+'/result.json');check(raw['completion']==c,'L3 summary/raw mismatch')
    for w in range(2):
        if row['motor']&(1<<w):
            check(c['n'][w]==300 and c['min'][w]<=c['sum'][w]/300<=c['max'][w],'L3 statistics')
check(len(set(ids))==7,'L3 duplicated IDs')
dual=[t for t in summary if t['motor']==3 and t['target']==20]
check(len(dual)==4 and dual[0]['outcome']=='needs_tuning','Keep first nonpassing dual trial')
for t in dual[-3:]:
    c=t['completion'];check(t['outcome']=='performance_pass','Three consecutive dual20 passes')
    for w in range(2):
        check(abs(c['sum'][w]/300-20)<=2 and math.sqrt(c['e2'][w]/300)<=5 and c['min'][w]>=2 and c['max'][w]<=40,'Dual20 performance')
    check(abs(c['sum'][0]-c['sum'][1])/300<=2,'Dual20 mean difference')
check(any(t['target']==30 and t['motor']==3 and t['outcome']=='performance_pass' for t in summary),'Cross-target proof')

hold=read('versions/H1/hold-summary.json')
check(sum(bool(t['measurement_complete']) for t in hold)==34,'H1 valid trial count')
check(sum(not bool(t['measurement_complete']) for t in hold)==1,'H1 aborted query retained')
sweep=read('versions/C3/sweep-summary.json')
check(sum(not bool(t['failure']) and bool(t['automatic_stop']) for t in sweep)==30,'Full sweep successful trials')
for row in sweep:
    check((root/row['evidence']/'result.json').exists(),'Sweep evidence link missing')
g1=read('versions/G1/VALIDATION.json')
check(g1['flashed'] and g1['cold_boot_verified'] and not g1['third_goal_completed'],'G1 current state')
check(not g1['ground_motion_sent'] and not g1['track_conditions_available'],'No ground trial claimed')
last=read('versions/G1/evidence/low-20261019-161126/result.json')['final']
check(last['mode']==0 and last['fault']==0 and last['pwm']==[0,0] and last['enc']==[0,0],'Last idle proof')

for row in read('provenance.json'):
    path=root/row['path'];check(path.is_file(),'Missing provenance file '+row['path'])
    check(hashlib.sha256(path.read_bytes()).hexdigest()==row['public_sha256'],'Public hash mismatch '+row['path'])
    if path.name=='serial.bin':check(row['public_sha256']==row['original_sha256'],'Raw serial bytes modified')

manifest=root/'SHA256SUMS.txt'
if manifest.exists():
    for line in manifest.read_text(encoding='utf-8').splitlines():
        digest,name=line.split('  ',1)
        check(hashlib.sha256((root/name).read_bytes()).hexdigest()==digest,'Archive hash mismatch '+name)
print(json.dumps({'offline_archive_check':'passed','firmware_versions':8,'flashed_readback_versions':7,
                  'L3_complete_trials':7,'H1_complete_trials':34,'sweep_complete_trials':30,
                  'G1_ground_test':'not performed'},ensure_ascii=False))
