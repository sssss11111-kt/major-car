"""Offline contracts; never opens a serial port."""
import unittest
from unittest.mock import patch
import tempfile
import os
from pathlib import Path
from tracking_gt1 import parse_status, parse_config, check_status, session

LINE = 'ms=100 mode=1 fault=0 frames=8 valid=1 pair=13/13 thr=30 err10=25 bench_target=20 bench_started=3 enc=20,21 pwm=1400,1450 us=1492 speed=20 target10=200 phase=2 quality=100 reject=0 seq=8 cap_age=10 proc_ms=0 used_age=0 drop_frames=0 ready=0 good_frames=3 accepted_age=10 trial_id=1 trial_done=0 trial_ms=100 pi_mask=3'
CFG = 'TRACK-CFG v=20261019GT1 target=20 cap=3000 settle=200 duration=3000 lease=500 simulated=0 profile=GL2 kp10=40 ki10=200 ff0=896,896 ff10=155,151 closed_cap=1500 kick_min=200 kick_max=300 transfer=200 entry=ff guard=120'

class Contracts(unittest.TestCase):
    @unittest.skipUnless(os.environ.get('TRACK_TEST_TELEMETRY'), 'Full runner supplies actual C telemetry')
    def test_actual_c_telemetry_is_complete_and_parseable(self):
        lines = Path(os.environ['TRACK_TEST_TELEMETRY']).read_text(encoding='ascii').splitlines()
        statuses = [parse_status(line) for line in lines if line.startswith('ms=')]
        self.assertTrue(statuses)
        self.assertTrue(all(row is not None for row in statuses))
        self.assertTrue(any(row['trial_done'] == 1 and row['trial_ms'] == 3200 for row in statuses))
        configs = [parse_config(line) for line in lines if line.startswith('TRACK-CFG ')]
        self.assertEqual(len(configs), 1)
        for row in statuses:
            check_status(row)
    def test_real_status_fields(self):
        row = parse_status(LINE)
        self.assertEqual(row['enc'], (20, 21))
        self.assertEqual(row['phase'], 2)
        check_status(row)
    def test_truncated_line_is_rejected(self):
        self.assertIsNone(parse_status('ms=10 mode=1 fault=0 enc=0,'))
        self.assertIsNone(parse_status('ordinary startup text'))
    def test_fault_or_bad_outputs_stop_monitor(self):
        for token, replacement in [('fault=0','fault=5'), ('pwm=1400,1450','pwm=3001,0'),
                                   ('enc=20,21','enc=-6,20'), ('us=1492','us=1700')]:
            with self.subTest(replacement=replacement), self.assertRaises(RuntimeError):
                check_status(parse_status(LINE.replace(token, replacement)))
    def test_only_identified_bounded_real_trial(self):
        self.assertEqual(parse_config(CFG)['duration'], 3000)
        for token, replacement in [('v=20261019GT1','v=20261018T'), ('lease=500','lease=0'),
                                   ('duration=3000','duration=0'), ('simulated=0','simulated=1'),
                                   ('ff0=896,896','ff0=2200,1500'), ('closed_cap=1500','closed_cap=3000')]:
            with self.subTest(replacement=replacement), self.assertRaises(RuntimeError):
                parse_config(CFG.replace(token, replacement))

    def exercise(self, scenario='normal', run=True, wrap=False):
        class Port:
            def __init__(self):
                self.commands = ''
                self.pending = bytearray()
                self.now = 0.0
                self.start = None
                self.sequence = 0
            @property
            def in_waiting(self):
                return len(self.pending)
            def write(self, data):
                self.commands += data.decode()
                for command in data.decode():
                    if command == 'd':
                        self.pending.extend((CFG + '\n').encode())
                    elif command == 'a':
                        if scenario == 'refused':
                            self.pending.extend(b'START REFUSED: not ready\n')
                        else:
                            self.start = self.now
                    elif command == 'p':
                        self.sequence += 1
                        elapsed = 0 if self.start is None else round((self.now-self.start)*1000)
                        active = self.start is not None and elapsed < (250 if scenario == 'early' else 3200)
                        done = int(self.start is not None and not active and scenario != 'early')
                        moving = active and elapsed >= 200 and scenario != 'no-motion'
                        ready = int(self.start is None and self.sequence >= 3 and scenario != 'not-ready')
                        mode = int(active)
                        fault = 0
                        if self.start is not None and scenario == 'fault':
                            mode, fault = 3, 6
                        device_ms = (round(self.now*1000) + (0xffffff00 if wrap else 0)) & 0xffffffff
                        if scenario == 'stale' and self.start is not None:
                            device_ms = 100
                        trial_id = int(self.start is not None)
                        if scenario == 'old-id' and self.start is not None:
                            trial_id = 0
                        line = ('ms=%d mode=%d fault=%d valid=1 us=1500 target10=%d phase=%d '
                                'cap_age=10 enc=%d,%d pwm=%d,%d ready=%d good_frames=%d '
                                'accepted_age=10 trial_id=%d trial_done=%d trial_ms=%d pi_mask=3\n') % (
                            device_ms, mode, fault, 200 if moving else 0, 2 if moving else 0,
                            20 if moving else 0, 21 if moving else 0, 1400 if moving else 0,
                            1450 if moving else 0, ready, 3 if ready else 1, trial_id, done, elapsed)
                        self.pending.extend(line.encode())
            def read(self, count):
                self.now += .02
                result = bytes(self.pending[:count])
                del self.pending[:count]
                return result
        port = Port()
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / 'trial'
            error = None
            with patch('tracking_trial.time.monotonic', side_effect=lambda: port.now):
                try:
                    session(port, output, run)
                except RuntimeError as exc:
                    error = exc
            import json
            result = json.loads((output / 'result.json').read_text())
        return port, result, error

    def test_preflight_waits_for_three_processed_frames(self):
        port, result, error = self.exercise(run=False)
        self.assertIsNone(error)
        self.assertGreaterEqual(port.sequence, 3)
        self.assertNotIn('a', port.commands)
        self.assertEqual(result['outcome'], 'preflight')

    def test_normal_full_trial_and_device_wrap(self):
        for wrap in (False, True):
            port, result, error = self.exercise(wrap=wrap)
            self.assertIsNone(error)
            self.assertEqual(result['outcome'], 'completed')
            self.assertGreaterEqual(result['completion']['trial_ms'], 3200)
            self.assertEqual(result['completion']['trial_id'], 1)
            self.assertTrue(port.commands.endswith('s'))

    def test_early_idle_and_never_moving_are_errors(self):
        for scenario in ('early', 'no-motion', 'fault', 'stale', 'old-id', 'refused'):
            with self.subTest(scenario=scenario):
                port, result, error = self.exercise(scenario)
                self.assertIsInstance(error, RuntimeError)
                self.assertEqual(result['outcome'], 'error')
                self.assertTrue(result['reason'])
                self.assertTrue(port.commands.endswith('s'))

    def test_not_ready_does_not_arm_and_still_saves_result(self):
        port, result, error = self.exercise('not-ready')
        self.assertIsInstance(error, RuntimeError)
        self.assertNotIn('a', port.commands)
        self.assertEqual(result['outcome'], 'error')


if __name__ == '__main__':
    unittest.main()

