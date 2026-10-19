"""STEP-05 supervised tracking helper. Default sends only d/p queries; --run sends a.

No flashing. Physical preparation and static w validation must be done beforehand.
pyserial is required only for a real session, not for offline parser tests.
"""
import argparse
import csv
from datetime import datetime
import json
from pathlib import Path
import re
import time


def fields(line):
    return dict(re.findall(r'(\w+)=([^\s]+)', line))


def parse_status(line):
    if not line.startswith('ms='):
        return None
    try:
        values = fields(line)
        row = {key: int(values[key]) for key in
               ('ms', 'mode', 'fault', 'valid', 'us', 'target10', 'phase', 'cap_age',
                'ready', 'good_frames', 'accepted_age', 'trial_id', 'trial_done', 'trial_ms')}
        for key in ('enc', 'pwm'):
            pair = tuple(int(v) for v in values[key].split(','))
            if len(pair) != 2:
                return None
            row[key] = pair
        return row
    except (KeyError, ValueError):
        return None


def parse_config(line):
    values = fields(line)
    try:
        if not line.startswith('TRACK-CFG ') or values['v'] != '20261019T2':
            raise ValueError('unrecognized firmware')
        cfg = {key: int(values[key]) for key in
               ('target', 'cap', 'settle', 'duration', 'lease', 'simulated', 'left_bias', 'right_bias')}
        if not (0 < cfg['target'] <= 40 and 0 < cfg['cap'] <= 3000 and
                100 <= cfg['settle'] <= 1000 and 0 < cfg['duration'] <= 5000 and
                200 <= cfg['lease'] <= 1000 and cfg['simulated'] == 0
                and cfg['left_bias'] == 2200 and cfg['right_bias'] == 1500):
            raise ValueError('configuration outside first-trial envelope')
        return cfg
    except (KeyError, ValueError) as exc:
        raise RuntimeError('Real bounded tracking firmware not confirmed; no start') from exc


def check_status(row):
    if row is None:
        raise RuntimeError('Missing complete status')
    if (row['fault'] != 0 or row['mode'] not in (0, 1) or
            any(v < 0 or v > 3000 for v in row['pwm']) or
            any(v < -5 or v > 450 for v in row['enc']) or
            not 1350 <= row['us'] <= 1650):
        raise RuntimeError('Abnormal state/output/feedback; stop required')


def session(port, output, run=False):
    output.mkdir(parents=True, exist_ok=False)
    rows, config, latest = [], None, None
    pending = bytearray()
    motion_armed = False
    result = {'outcome': 'error', 'reason': 'Session did not finish', 'hardware_verified': False}
    raw = (output / 'serial.bin').open('wb')

    def send(command):
        port.write(command.encode('ascii'))

    def receive():
        nonlocal latest, config, pending
        chunk = port.read(max(1, min(getattr(port, 'in_waiting', 0), 4096)))
        raw.write(chunk)
        pending.extend(chunk)
        if len(pending) > 65536:
            raise RuntimeError('Unbounded/non-text serial data')
        new = []
        while b'\n' in pending:
            line, _, rest = pending.partition(b'\n')
            pending = bytearray(rest)
            text = line.decode('ascii', errors='replace').strip()
            if text.startswith('TRACK-CFG '):
                config = parse_config(text)
            if text.startswith('START REFUSED'):
                raise RuntimeError('Vehicle refused start; no automatic retry')
            row = parse_status(text)
            if row is not None:
                latest = row
                rows.append(row)
                new.append(row)
        return new

    try:
        deadline = time.monotonic() + 2
        query_at = 0.0
        send('d')
        while time.monotonic() < deadline:
            now = time.monotonic()
            if now >= query_at:
                send('p')
                query_at = now + .2
            receive()
            if latest:
                check_status(latest)
                if latest['mode'] != 0 or any(latest['pwm']):
                    raise RuntimeError('Idle and zero outputs required')
            if (config and latest and latest['ready'] == 1 and latest['good_frames'] >= 3
                    and latest['valid'] == 1 and latest['cap_age'] <= 150
                    and latest['accepted_age'] <= 150):
                break
        if not config:
            raise RuntimeError('No identified tracking configuration')
        check_status(latest)
        if latest['mode'] != 0 or any(latest['pwm']):
            raise RuntimeError('Idle and zero outputs required')
        if (latest['ready'] != 1 or latest['good_frames'] < 3 or latest['valid'] != 1
                or latest['cap_age'] > 150 or latest['accepted_age'] > 150):
            raise RuntimeError('Firmware not ready: three fresh processed valid frames required')
        expected_id = (latest['trial_id'] + 1) & 0xffffffff
        print(json.dumps({'configuration': config, 'status': latest}, ensure_ascii=False))
        if not run:
            result = {'outcome': 'preflight', 'reason': 'Firmware ready; no start sent',
                      'configuration': config, 'status': latest, 'hardware_verified': False}
            print('Preflight only. No start sent. --run explicitly starts one bounded trial.')
            return
        motion_armed = True  # Also send stop if write/ACK handling fails after this point.
        send('a')
        began = time.monotonic()
        fresh_at, query_at, last_ms = began, began, latest['ms']
        active = False
        moving = False
        limit = (config['settle'] + config['duration']) / 1000 + 1
        while time.monotonic() - began < limit:
            now = time.monotonic()
            if now >= query_at:
                send('p')
                query_at = now + .1
            for row in receive():
                check_status(row)
                if row['ms'] != last_ms:
                    fresh_at, last_ms = time.monotonic(), row['ms']
                if row['mode'] == 1:
                    if row['trial_id'] != expected_id:
                        raise RuntimeError('Unexpected trial identity; no automatic retry')
                    active = True
                    if (row['phase'] == 2 and row['target10'] > 0
                            and all(v > 0 for v in row['enc'])
                            and all(v > 0 for v in row['pwm'])):
                        moving = True
                elif active:
                    if any(row['pwm']) or row['us'] != 1500:
                        raise RuntimeError('Idle without zero PWM and neutral steering')
                    if (row['trial_id'] != expected_id or row['trial_done'] != 1
                            or row['trial_ms'] < config['settle'] + config['duration']):
                        raise RuntimeError('Trial ended early or without normal-expiry evidence')
                    if not moving:
                        raise RuntimeError('No observed two-wheel closed-loop motion; trial not accepted')
                    result = {'outcome': 'completed', 'reason': 'Normal expiry and motion feedback observed',
                              'configuration': config, 'completion': row, 'hardware_verified': False}
                    print('Normal expiry and two-wheel feedback observed; physical tracking/stopping still require observation.')
                    return
            if now - fresh_at > .7:
                raise RuntimeError('Telemetry stale; stop attempted (vehicle also has local lease)')
            if not active and now - began > .8:
                raise RuntimeError('No active trial observed; no automatic retry')
        raise RuntimeError('No observed automatic trial completion before deadline')
    except BaseException as exc:
        result = {'outcome': 'error', 'reason': str(exc) or type(exc).__name__,
                  'configuration': config, 'status': latest, 'hardware_verified': False}
        raise
    finally:
        if motion_armed:
            try:
                send('s')
            except Exception:
                pass  # A lost radio cannot guarantee delivery; vehicle-side lease is independent.
        raw.close()
        (output / 'result.json').write_text(json.dumps(result, indent=2), encoding='utf-8')
        (output / 'status.json').write_text(json.dumps(rows, indent=2), encoding='utf-8')
        if rows:
            with (output / 'status.csv').open('w', newline='', encoding='utf-8-sig') as file:
                writer = csv.DictWriter(file, fieldnames=list(rows[0]))
                writer.writeheader()
                writer.writerows(rows)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', required=True)
    parser.add_argument('--run', action='store_true', help='Send a after preflight; vehicle will move')
    parser.add_argument('--out', type=Path)
    args = parser.parse_args()
    output = args.out or Path(__file__).resolve().parent / 'evidence' / (
        'tracking-' + datetime.now().strftime('%Y%m%d-%H%M%S-%f'))
    import serial
    with serial.Serial(args.port, 115200, timeout=.02, write_timeout=.2) as port:
        session(port, output, args.run)


if __name__ == '__main__':
    main()
