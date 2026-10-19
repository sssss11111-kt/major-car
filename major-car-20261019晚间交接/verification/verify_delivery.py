"""Verify published files; never opens hardware or starts a car."""
from pathlib import Path
import hashlib
import json
import re

root = Path(__file__).resolve().parents[1]
manifest = json.loads((root / 'delivery-manifest.json').read_text(encoding='utf-8'))
for item in manifest:
    path = root / item['path']
    assert path.is_file(), item['path']
    assert hashlib.sha256(path.read_bytes()).hexdigest() == item['sha256'], item['path']
index = json.loads((root / 'firmware-index.json').read_text(encoding='utf-8'))
assert len(index) == 12
for item in index:
    assert hashlib.sha256((root / item['path']).read_bytes()).hexdigest() == item['sha256']
    assert any(x['readback_verified'] for x in item['flash_evidence']), item['version']
for file in (root / 'code').rglob('*'):
    if file.is_file():
        archived = root / 'versions/GT6/code' / file.relative_to(root / 'code')
        assert file.read_bytes() == archived.read_bytes(), str(file)
for file in root.glob('*.md'):
    for link in re.findall(r'\]\(([^)]+)\)', file.read_text(encoding='utf-8')):
        if '://' not in link and not link.startswith('#'):
            assert (file.parent / link.split('#')[0]).exists(), (file.name, link)
for file in root.rglob('*'):
    if file.is_file():
        assert not file.name.startswith(('before-flash', 'flash-GT', 'flash-GH', 'flash-GL'))
        assert file.suffix.lower() not in ('.axf', '.obj', '.exe', '.pyc', '.pdb')
status = json.loads((root / 'current-status.json').read_text(encoding='utf-8'))
assert status['latest_firmware'] == '20261019GT6'
assert status['camera_latest_user_confirmation'] == '已调整清晰，下一次从清晰图像识别继续'
assert status['right_curve_accepted'] is False
print(f'DELIVERY VERIFIED: {len(manifest)} files, 12 exact firmware versions, GT6 source mapping and handoff links')
