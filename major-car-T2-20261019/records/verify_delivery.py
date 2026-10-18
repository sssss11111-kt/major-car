from pathlib import Path
import hashlib,json,re
root=Path(__file__).resolve().parents[1]
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
m=json.loads((root/'records/source-manifest.json').read_text())
for rel,digest in m.items():assert sha(root/'code'/rel)==digest,rel
for line in (root/'firmware/SHA256SUMS.txt').read_text().splitlines():
 digest,name=line.split('  ',1);assert sha(root/'firmware'/name)==digest,name
assert 'ALL TESTS PASSED' in (root/'records/host-tests-msvc.log').read_text(encoding='utf-8')
log=(root/'records/build-camera.log').read_text(encoding='utf-8')
assert log.rstrip().endswith('BUILD SUCCESS') and not re.search(r'(warning:|error:)',log,re.I)
print('T2 DELIVERY VERIFIED:',len(m),'source files; firmware and saved software evidence match.')
