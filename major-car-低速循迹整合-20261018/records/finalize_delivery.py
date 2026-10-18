"""Create inspectable manifests/diff; reads immutable Git baseline, never touches hardware."""
from pathlib import Path
import difflib
import hashlib
import json
import shutil
import subprocess

ROOT = Path(__file__).resolve().parents[1]
CODE = ROOT / 'code'
GIT = Path(r'D:\桌面\智能车\.tmp\review-main-20261018')
BASE = '53d6dba300a88824b0fbc201af3e8d8aa9fce5d7'
sha = lambda data: hashlib.sha256(data).hexdigest()


def source_file(path):
    relative = path.relative_to(CODE).as_posix()
    return not (relative.startswith(('repair/build/', 'repair/evidence/')) or
                '/tests/out/' in relative or '__pycache__' in relative or
                path.suffix.lower() in ('.pyc', '.obj', '.exe', '.o'))


def decode(data):
    for encoding in ('utf-8', 'gb18030'):
        try:
            return data.decode(encoding)
        except UnicodeDecodeError:
            pass
    return data.decode('utf-8', errors='replace')


def validate_hex(path):
    upper, memory, ended = 0, {}, False
    for line in path.read_text(encoding='ascii').splitlines():
        if not line.startswith(':'):
            raise ValueError('Invalid HEX record')
        record = bytes.fromhex(line[1:])
        if len(record) != record[0] + 5 or sum(record) % 256:
            raise ValueError('Invalid HEX checksum/length')
        count, address, kind = record[0], int.from_bytes(record[1:3], 'big'), record[3]
        data = record[4:4 + count]
        if kind == 4:
            upper = int.from_bytes(data, 'big') << 16
        elif kind == 0:
            for index, byte in enumerate(data):
                absolute = upper + address + index
                if not 0x70000000 <= absolute < 0x70400000:
                    raise ValueError('Data outside project Flash layout')
                if absolute in memory and memory[absolute] != byte:
                    raise ValueError('Conflicting HEX data')
                memory[absolute] = byte
        elif kind == 1:
            ended = True
        elif kind != 5:
            raise ValueError('Unexpected HEX record kind')
    if not ended or not memory:
        raise ValueError('Missing EOF or image data')
    return {'data_bytes': len(memory), 'start': hex(min(memory)), 'last': hex(max(memory))}


def main():
    firmware = ROOT / 'firmware'
    firmware.mkdir(exist_ok=True)
    builds = {}
    for mode in ('camera', 'mock', 'diag'):
        folder = CODE / 'repair/build' / mode
        log = (folder / 'build.log').read_text(encoding='utf-8')
        if 'BUILD SUCCESS' not in log or 'error:' in log.lower():
            raise RuntimeError('Build not confirmed: ' + mode)
        shutil.copy2(folder / 'build.log', ROOT / 'records' / ('FINAL-build-' + mode + '.log'))
        shutil.copy2(folder / 'commands.json', ROOT / 'records' / ('FINAL-commands-' + mode + '.json'))
        target = firmware / ('20261018T-' + mode + '.hex')
        shutil.copy2(folder / ('camera-car-uart8-' + mode + '.hex'), target)
        builds[mode] = dict(validate_hex(target), sha256=sha(target.read_bytes()),
                            warnings=sum('warning:' in line.lower() for line in log.splitlines()))
    shutil.copy2(CODE / 'repair/build/camera/camera-car-uart8-camera.axf', firmware / '20261018T-camera.axf')
    shutil.copy2(CODE / 'repair/build/camera/link.map', ROOT / 'records/FINAL-camera-link.map')
    (ROOT / 'records/FINAL-build-summary.json').write_text(json.dumps(builds, indent=2), encoding='utf-8')
    baseline = json.loads((ROOT / 'records/baseline-manifest.json').read_text(encoding='utf-8'))
    current = {p.relative_to(CODE).as_posix(): sha(p.read_bytes())
               for p in sorted(CODE.rglob('*')) if p.is_file() and source_file(p)}
    (ROOT / 'records/code-manifest.json').write_text(json.dumps(current, ensure_ascii=False, indent=2), encoding='utf-8')
    rows, patch = [], []
    for name, digest in current.items():
        if baseline.get(name) == digest:
            continue
        rows.append((name, '新增' if name not in baseline else '修改'))
        if Path(name).suffix.lower() not in ('.c', '.h', '.py', '.md', '.cmd', '.ps1'):
            continue
        old = subprocess.check_output(['git', 'show', BASE + ':' + name], cwd=GIT) if name in baseline else b''
        patch.extend(difflib.unified_diff(decode(old).splitlines(keepends=True),
                                        decode((CODE / name).read_bytes()).splitlines(keepends=True),
                                        fromfile='baseline/' + name, tofile='code/' + name))
    (ROOT / 'records/changes.patch').write_text(''.join(patch), encoding='utf-8')
    (ROOT / 'records/CHANGED-FILES.md').write_text(
        '# 相对N基线的文件变更\n\n基线：' + BASE + '\n\n| 文件 | 类型 |\n|---|---|\n' +
        ''.join('| ' + name + ' | ' + kind + ' |\n' for name, kind in rows), encoding='utf-8')
    (firmware / 'SHA256SUMS.txt').write_text(''.join(
        sha(p.read_bytes()) + '  ' + p.name + '\n' for p in sorted(firmware.glob('*')) if p.suffix in ('.hex', '.axf')),
        encoding='ascii')
    print(json.dumps({'source_files': len(current), 'changed_files': len(rows), 'builds': builds}, indent=2))


if __name__ == '__main__':
    main()
