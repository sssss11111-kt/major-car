"""Parser contracts and one real C-core replay (no hardware substitutes)."""
import importlib.util
import json
import os
from pathlib import Path
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("replay_vision", ROOT / "tools/replay_vision.py")
replay = importlib.util.module_from_spec(spec)
spec.loader.exec_module(replay)


class PgmTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.directory = Path(self.temporary.name)

    def tearDown(self):
        self.temporary.cleanup()

    def write(self, data):
        path = self.directory / "input.pgm"
        path.write_bytes(data)
        return path

    def test_comments_and_whitespace_valued_first_pixels(self):
        pixels = bytes([10, 13, 32, 9]) + bytes(188 * 120 - 4)
        path = self.write(b"P5\n# captured frame\n188 # width\n120\n255\n" + pixels)
        self.assertEqual(replay.read_pgm(path), pixels)
        path = self.write(b"P5\r\n188 120\r\n255\r\n" + pixels)
        self.assertEqual(replay.read_pgm(path), pixels)

    def test_bad_dimensions_maxval_magic_and_raster_lengths(self):
        pixels = bytes(188 * 120)
        for data in (b"P2\n188 120\n255\n" + pixels,
                     b"P5\n120 188\n255\n" + pixels,
                     b"P5\n188 120\n65535\n" + pixels,
                     b"P5\n188 120\n255\n" + pixels[:-1],
                     b"P5\n188 120\n255\n" + pixels + b"\n"):
            with self.subTest(header=data[:24]), self.assertRaises(ValueError):
                replay.read_pgm(self.write(data))

    def test_input_order_and_directory_sort(self):
        for name in ("b.pgm", "a.pgm", "ignored.txt"):
            (self.directory / name).write_bytes(b"")
        ordered = replay.input_paths([str(self.directory / "b.pgm"), str(self.directory / "a.pgm")])
        self.assertEqual([p.name for p in ordered], ["b.pgm", "a.pgm"])
        self.assertEqual([p.name for p in replay.input_paths([str(self.directory)])], ["a.pgm", "b.pgm"])

    def test_actual_core_and_outputs(self):
        compiler = ROOT / ".tools/ziglang" / ("zig.exe" if os.name == "nt" else "zig")
        if not compiler.exists():
            self.skipTest("Install the documented .tools Zig compiler for C integration")
        pixels = bytes(210 if 50 <= x <= 138 else 30 for y in range(120) for x in range(188))
        track = self.directory / "track.pgm"
        blank = self.directory / "blank.pgm"
        replay.write_pgm(track, pixels)
        replay.write_pgm(blank, bytes([255]) * (188 * 120))
        destination = self.directory / "report"
        records = replay.replay([track, track, blank], destination)
        self.assertEqual([r["valid"] for r in records], [1, 1, 0])
        self.assertEqual([r["frame"] for r in records], [0, 1, 2])
        self.assertLess(abs(records[0]["error"]), 1)
        self.assertGreaterEqual(records[0]["paired_rows"], 8)
        self.assertEqual(len(records[0]["row_threshold"]), 120)
        self.assertEqual(replay.read_pgm(destination / "000000-gray.pgm"), pixels)
        self.assertEqual(set(replay.read_pgm(destination / "000000-binary.pgm")), {0, 255})
        self.assertEqual(len(json.loads((destination / "results.json").read_text(encoding="utf-8"))), 3)
        self.assertTrue((destination / "results.csv").is_file())
        self.assertTrue((destination / "index.html").is_file())
        svg = (destination / "000000-overlay.svg").read_text()
        self.assertIn("data:image/png;base64,", svg)
        self.assertIn("Missing edges are not drawn", svg)


if __name__ == "__main__":
    unittest.main()
