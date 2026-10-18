"""Offline contracts; never opens a serial port."""
import unittest
from unittest.mock import patch
import tempfile
from pathlib import Path
from tracking_trial import parse_status, parse_config, check_status, session

LINE = 'ms=100 mode=1 fault=0 frames=8 valid=1 pair=13/13 thr=30 err10=25 bench_target=20 bench_started=3 enc=20,21 pwm=1400,1450 us=1492 speed=20 target10=200 phase=2 quality=100 reject=0 seq=8 cap_age=10 proc_ms=0 used_age=0 drop_frames=0'
CFG = 'TRACK-CFG v=20261018T target=20 cap=3000 settle=200 duration=3000 lease=500 simulated=0'

class Contracts(unittest.TestCase):
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
        for token, replacement in [('v=20261018T','v=20261018N'), ('lease=500','lease=0'),
                                   ('duration=3000','duration=0'), ('simulated=0','simulated=1')]:
            with self.subTest(replacement=replacement), self.assertRaises(RuntimeError):
                parse_config(CFG.replace(token, replacement))

    def test_session_queries_by_default_and_stops_after_explicit_run(self):
        class Port:
            def __init__(self):
                self.commands = ''
                self.pending = bytearray()
                self.now = 0
                self.active = False
                self.queries = 0
                self.bad_feedback = False
            @property
            def in_waiting(self):
                return len(self.pending)
            def write(self, data):
                self.commands += data.decode()
                for char in data.decode():
                    if char == 'd':
                        self.pending.extend((CFG + '\n').encode())
                    elif char == 'a':
                        self.active = True
                    elif char == 'p':
                        self.queries += 1
                        line = LINE.replace('ms=100', 'ms=' + str(self.queries * 100))
                        if not self.active or self.queries > 4:
                            line = line.replace('mode=1', 'mode=0').replace('pwm=1400,1450', 'pwm=0,0').replace('us=1492', 'us=1500')
                        if self.active and self.bad_feedback:
                            line = line.replace('enc=20,21', 'enc=-6,20')
                        self.pending.extend((line + '\n').encode())
            def read(self, count):
                self.now += .05
                data = bytes(self.pending[:count])
                del self.pending[:count]
                return data
        with tempfile.TemporaryDirectory() as directory:
            for run, bad in ((False, False), (True, False), (True, True)):
                port = Port()
                port.bad_feedback = bad
                output = Path(directory) / ('trial-%s-%s' % (run, bad))
                with patch('tracking_trial.time.monotonic', side_effect=lambda: port.now):
                    if bad:
                        with self.assertRaises(RuntimeError):
                            session(port, output, run)
                    else:
                        session(port, output, run)
                self.assertEqual('a' in port.commands, run)
                if run:
                    self.assertTrue(port.commands.endswith('s'))
                self.assertTrue((output / 'status.csv').is_file())
                self.assertTrue((output / 'serial.bin').is_file())

if __name__ == '__main__':
    unittest.main()
