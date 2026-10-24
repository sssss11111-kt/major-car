from pathlib import Path
import hashlib
import json
from intelhex import IntelHex
from pyocd.core.helpers import ConnectHelper

p = Path(__file__).resolve().parent
image = IntelHex(str(p.parent/'demo/mdk/Objects/rt1064.hex'))
s = ConnectHelper.session_with_chosen_probe(target_override='mimxrt1064', frequency=100000,
        blocking=False, options={'connect_mode':'attach','resume_on_disconnect':False})
if s is None:
    raise RuntimeError('No debugger connected')
# Read Flash via the debug access port. Do not halt, reset, run or program.
actual = bytearray()
expected = bytearray()
with s:
    for a,b in image.segments():
        for start in range(a,b,4096):
            end = min(start+4096,b)
            actual.extend(s.target.read_memory_block8(start,end-start))
            expected.extend(image.tobinarray(start=start,end=end-1))
report = {'read_bytes':len(actual),'matches_LAP12':actual==expected,
          'actual_sha256':hashlib.sha256(actual).hexdigest(),
          'expected_sha256':hashlib.sha256(expected).hexdigest(),
          'operation':'attach and read Flash only; no reset/halt/program commands'}
(p/'board-image-check.json').write_text(json.dumps(report,indent=2))
print(json.dumps(report))
