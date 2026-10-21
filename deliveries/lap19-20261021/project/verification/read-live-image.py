from pathlib import Path
import json
import struct
import zlib
from elftools.elf.elffile import ELFFile
from pyocd.core.helpers import ConnectHelper

p = Path(__file__).resolve().parent
with (p.parent/'demo/mdk/Objects/rt1064.axf').open('rb') as f:
    symbols = {s.name:s['st_value'] for s in ELFFile(f).get_section_by_name('.symtab').iter_symbols()
               if s.name in {'raw_image','frame_count'}}
s = ConnectHelper.session_with_chosen_probe(target_override='mimxrt1064',frequency=1000000,
        blocking=False,options={'connect_mode':'attach','resume_on_disconnect':False})
with s:
    before = s.target.read32(symbols['frame_count'])
    raw = bytes(s.target.read_memory_block8(symbols['raw_image'],188*120))
    after = s.target.read32(symbols['frame_count'])
# Live read is deliberately non-blocking to the board; frame may change during read.
(p/'live-gray.bin').write_bytes(raw)
def chunk(k,b): return struct.pack('>I',len(b))+k+b+struct.pack('>I',zlib.crc32(k+b)&0xffffffff)
scan = b''.join(b'\0'+raw[y*188:(y+1)*188] for y in range(120))
png = b'\x89PNG\r\n\x1a\n'+chunk(b'IHDR',struct.pack('>IIBBBBB',188,120,8,0,0,0,0))+chunk(b'IDAT',zlib.compress(scan))+chunk(b'IEND',b'')
(p/'live-gray.png').write_bytes(png)
report = {'frame_before':before,'frame_after':after,'single_frame_confirmed':before==after,
          'kind':'Live debug read, may span frames; no halt or reset'}
(p/'live-gray.json').write_text(json.dumps(report,indent=2))
print(json.dumps(report))
