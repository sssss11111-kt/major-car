from pathlib import Path
import hashlib,json
from intelhex import IntelHex
from pyocd.core.helpers import ConnectHelper
from pyocd.flash.file_programmer import FileProgrammer
from pyocd.target.pack.flash_algo import PackFlashAlgo
from pyocd.core.memory_map import RamRegion
root=Path(__file__).resolve().parents[1]
import argparse
ap=argparse.ArgumentParser();ap.add_argument('--prepared',action='store_true');args=ap.parse_args()
if not args.prepared: raise SystemExit('Confirm motor power OFF, wheels lifted and debugger connected before --prepared.')
f=root/'demo/mdk/Objects/rt1064.hex'
h=IntelHex(str(f));segments=h.segments()
assert all(0x70000000<=a<b<=0x70400000 for a,b in segments)
s=ConnectHelper.session_with_chosen_probe(target_override='mimxrt1064',frequency=1000000,options={'connect_mode':'halt','resume_on_disconnect':False})
# Use the NXP FLM selected by the supplied MDK project, loaded in fixed OCRAM.
# Builtin algorithm initialization at DTCM failed; do not repeat that setup.
flm=PackFlashAlgo(str(root/'verification/MIMXRT1064_QSPI_4KB_SEC.FLM'))
assert flm.flash_start==0x70000000 and flm.flash_size==0x400000
region=s.target.memory_map.get_region_for_address(0x70000000)
region.algo=flm.get_pyocd_flash_algo(256,RamRegion(start=0x20200000,length=0x10000))
assert all(0x70000000<=a<b<=0x70010000 for a,b in segments)
with s:
 t=s.target;t.halt()
 backup=bytes(t.read_memory_block8(0x70000000,0x10000))
 (root/'verification/before-recalibrated-flash-64KiB.bin').write_bytes(backup)
 print('BACKUP '+hashlib.sha256(backup).hexdigest(),flush=True)
 FileProgrammer(s,chip_erase='sector',keep_unwritten=True).program(str(f))
 t.reset_and_halt();total=0
 for a,b in segments:
  for start in range(a,b,4096):
   end=min(start+4096,b)
   actual=bytes(t.read_memory_block8(start,end-start))
   assert actual==h.tobinarray(start=start,end=end-1).tobytes(),hex(start)
   total+=end-start
 result={'hex_sha256':hashlib.sha256(f.read_bytes()).hexdigest(),'verified_bytes':total,'segments':segments,'readback_match':True,'state':'halted; cold boot required'}
 (root/'verification/flash-recalibrated-result.json').write_text(json.dumps(result,indent=2))
 print(json.dumps(result),flush=True)



