"""Read-only artifact integrity checks; no hardware access."""
from pathlib import Path
import hashlib,json,re,zlib
from intelhex import IntelHex
base=Path(__file__).resolve().parents[1]
v=base/'project/verification';f=base/'project/demo/mdk/Objects/rt1064.hex'
s=json.loads((v/'LAP19-status.json').read_text(encoding='utf-8-sig'))
r=json.loads((v/'flash-LAP19-result.json').read_text(encoding='utf-8-sig'))
hash_hex=hashlib.sha256(f.read_bytes()).hexdigest()
assert hash_hex==s['hex_sha256']==r['hex_sha256']=='8c4c761340a3649fe1ff465dafa305add67036db0caa25b0241cb46bb30dc29e'
h=IntelHex(str(f));assert sum(b-a for a,b in h.segments())==r['verified_bytes']==64960
assert all(0x70000000<=a<b<=0x70010000 for a,b in h.segments())
assert s['flashed'] and r['readback_match']
header=(base/'project/demo/user/inc/main.h').read_text(encoding='utf-8-sig')
for macro,value in [('SERVO_MOTOR_L_MAX',124),('SERVO_MOTOR_R_MAX',61),('SERVO_MOTOR_MIDDLE',92.5),('TRIAL_DRIVE_TARGET',36),('TRIAL_DRIVE_DEGRADED_TARGET',24)]:
 m=re.search(r'^#define\s+'+macro+r'\s+\(([0-9.]+)f?\)',header,re.M);assert m and float(m[1])==value,macro
for folder in (v/'fixtures').iterdir():
 capture=json.loads((folder/'capture.json').read_text(encoding='utf-8-sig'));payload=(folder/'payload.bin').read_bytes()
 assert len(payload)==capture['n']==22920 and zlib.crc32(payload)==capture['crc'] and capture['crc_ok'],folder
manifest=base/'MANIFEST-SHA256.json'
if manifest.exists():
 entries=json.loads(manifest.read_text(encoding='utf-8'))
 for rel,meta in entries.items():
  p=base/rel;assert p.is_file() and p.stat().st_size==meta['bytes'] and hashlib.sha256(p.read_bytes()).hexdigest()==meta['sha256'],rel
print('PASS: LAP19 burned HEX, flash result, source parameters, 2 frame CRCs and published manifest')
