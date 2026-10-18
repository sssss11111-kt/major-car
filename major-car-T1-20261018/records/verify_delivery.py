"""Verify this T1 delivery's saved source, firmware and evidence. No hardware access."""
from pathlib import Path
import hashlib
import json

root=Path(__file__).resolve().parents[1]
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
manifest=json.loads((root/'records/source-manifest.json').read_text(encoding='utf-8'))
for relative,digest in manifest.items():
    p=root/'code'/relative
    assert p.is_file() and sha(p)==digest,'Source mismatch: '+relative
for line in (root/'firmware/SHA256SUMS.txt').read_text().splitlines():
    digest,name=line.split('  ',1)
    assert sha(root/'firmware'/name)==digest,'Firmware mismatch: '+name
summary=json.loads((root/'records/summary.json').read_text())
assert summary['version']=='20261018T1' and not summary['flashed'] and not summary['hardware_tested']
assert 'ALL TESTS PASSED' in (root/'records/host-tests-msvc.log').read_text(encoding='utf-8')
for mode in ('camera','mock','diag'):
    log=(root/'records'/('build-'+mode+'.log')).read_text(encoding='utf-8')
    assert log.rstrip().endswith('BUILD SUCCESS') and 'warning:' not in log.lower()
    assert summary['builds'][mode]['sha256']==sha(root/'firmware'/('20261018T1-'+mode+'.hex'))
assert all((root/p).is_file() for p in ('README.md','修改记录.md','实车验收清单.md'))
print('T1 DELIVERY VERIFIED:',len(manifest),'source files; firmware hashes and saved evidence match; hardware remains unverified.')
