"""Regression checks for increased central-brick clearance in offline previews."""

import unittest
import numpy as np

from preview_centerline import clearance, plan
from analyze import load_frames
from extract_replay import ROOT
import json


class ClearanceTests(unittest.TestCase):
    def test_central_brick_has_extra_clearance(self):
        binary = np.zeros((120, 188), dtype=bool)
        binary[:, 20:169] = True
        binary[60:71, 85:106] = False
        lines = np.array([[20] * 120, [94] * 120, [168] * 120], dtype=np.uint8)
        path, status = plan(binary, lines, [85, 60, 105, 70], 8)
        self.assertEqual(status, "pixel_route")
        for x, y in path:
            if 56 <= y <= 74:
                self.assertTrue(x < 85 - 20 or x > 105 + 20, (x, y))

    def test_narrow_road_rejects_increased_clearance(self):
        binary = np.zeros((120, 188), dtype=bool)
        binary[:, 65:125] = True
        binary[60:71, 85:106] = False
        lines = np.array([[65] * 120, [94] * 120, [124] * 120], dtype=np.uint8)
        self.assertIsNone(plan(binary, lines, [85, 60, 105, 70], 8)[0])

    def test_side_brick_keeps_existing_margin(self):
        lines = np.array([[20] * 120, [94] * 120, [168] * 120], dtype=np.uint8)
        self.assertEqual(clearance(lines, [140, 60, 155, 70], 8, 12), 8)

    def test_real_frame_200_moves_further_from_brick(self):
        report = json.loads((ROOT / "results/c-replay.json").read_text())
        box = next(r for r in report["frames"] if r["frame"] == 200)["results"]["sf2_frozen"]["box"]
        _, _, binary, lines, _, _ = next(row for row in load_frames() if row[0]["replay_frame"] == 200)
        gaps = []
        for extra in (0, 12):
            path, status = plan(binary, lines, box, 8, extra)
            self.assertEqual(status, "pixel_route")
            gaps.append(min(max(box[0] - x, x - box[2]) for x, y in path if box[1] <= y <= box[3]))
        self.assertEqual(gaps, [9, 21])


if __name__ == "__main__":
    unittest.main()
