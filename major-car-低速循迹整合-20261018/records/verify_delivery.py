"""Verify saved source and firmware fingerprints; no compiler or hardware needed."""
from pathlib import Path
import hashlib
import json

root = Path(__file__).resolve().parents[1]
manifest = json.loads((root / 'records/code-manifest.json').read_text(encoding='utf-8'))
for relative, digest in manifest.items():
    path = root / 'code' / relative
    if not path.is_file() or hashlib.sha256(path.read_bytes()).hexdigest() != digest:
        raise SystemExit('SOURCE MISMATCH: ' + relative)
for line in (root / 'firmware/SHA256SUMS.txt').read_text(encoding='ascii').splitlines():
    digest, name = line.split('  ', 1)
    if hashlib.sha256((root / 'firmware' / name).read_bytes()).hexdigest() != digest:
        raise SystemExit('FIRMWARE MISMATCH: ' + name)
log = (root / 'records/FINAL-host-tests.log').read_text(encoding='utf-8')
if 'ALL TESTS PASSED' not in log:
    raise SystemExit('Missing final host test evidence')
summary = json.loads((root / 'records/FINAL-build-summary.json').read_text(encoding='utf-8'))
if any(v['warnings'] != 0 for v in summary.values()):
    raise SystemExit('Build warnings remain; inspect FINAL logs')
for name in ('README.md', '修改记录.md', '实车验收清单.md'):
    if not (root / name).is_file():
        raise SystemExit('Missing ' + name)
print('DELIVERY VERIFIED: %d source files; 3 HEX + camera AXF; final tests/build records present.' % len(manifest))
