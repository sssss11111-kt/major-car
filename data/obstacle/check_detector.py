"""Run the C detector against the published SF2 replay and GitHub main's camera pipeline."""

import ctypes as C
import json
import os
from pathlib import Path
import subprocess
import tempfile
import time
import hashlib
import zipfile

import numpy as np

from analyze import detect, intersection_over_union, load_frames
from extract_replay import ROOT

REPO = ROOT.parents[1]
PROJECT = REPO / "firmware/camera_car"


def compile_detector(output):
    src = PROJECT / "demo/user/src"
    package = REPO / "tools/track-replay/track-replay-source.zip"
    manifest = json.loads((ROOT / "manifest.json").read_text(encoding="utf-8"))
    with zipfile.ZipFile(package) as zipped:
        prefix = "track-replay-source/"
        hashes = json.loads(zipped.read(prefix + "algorithm-manifest.json"))["sha256"]
        assert hashes["algorithm/img_process.c"] == manifest["algorithm"]["source_sha256"]
        for name in ("replay_core.c", "algorithm/img_process.c", "algorithm/img_process.h",
                     "algorithm/mock_sdk.h", "algorithm/zf_common_headfile.h"):
            contents = zipped.read(prefix + name)
            assert hashlib.sha256(contents).hexdigest() == hashes[name], name
            target = output.parent / name
            target.parent.mkdir(exist_ok=True)
            target.write_bytes(contents)
    args = [os.environ.get("ZIG", "zig"), "cc", "-shared", "-O2", "-Wall", "-Wextra", "-Werror",
            "-I", str(output.parent / "algorithm"), "-I", str(src.parent / "inc"),
            str(ROOT / "detector_replay.c"), str(src / "obstacle_detect.c"),
            str(src / "car_core.c"), str(output.parent / "replay_core.c"), "-o", str(output)]
    subprocess.run(args, check=True)


def check(dll):
    u8 = C.POINTER(C.c_uint8)
    dll.replay_obstacle.argtypes = [u8, u8, u8, C.c_uint32, C.c_int, C.POINTER(C.c_uint16)]
    dll.replay_obstacle.restype = None
    dll.replay_process.argtypes = [u8, u8, u8, C.POINTER(C.c_int32)]
    dll.replay_process.restype = None
    totals = {mode: dict(tp=0, fn=0, fp=0, wrong_location=0, confirmed=0, suppressed=0)
              for mode in ("sf2_frozen", "sf2_recomputed", "main_measured")}
    records = []
    as_u8 = lambda value: value.ctypes.data_as(u8)
    dll.replay_reset_obstacle()
    dll.replay_reset()
    for frame, gray, binary, lines, box, negative in load_frames():
        binary_pixels = binary.astype(np.uint8)
        _, _, methods = detect(gray, binary, lines)
        expected = methods["gray_edge"]["candidates"]
        results = {}
        if frame["history_reset"]:
            dll.replay_reset_obstacle()
            dll.replay_reset()
        sf2_lines = np.zeros_like(lines)
        thresholds = np.zeros(gray.shape[0], dtype=np.uint8)
        metrics = (C.c_int32 * 13)()
        dll.replay_process(as_u8(gray), as_u8(sf2_lines), as_u8(thresholds), metrics)
        sf2_binary = (gray > thresholds[:, None]).astype(np.uint8)
        for index, (mode, total) in enumerate(totals.items()):
            raw = (C.c_uint16 * 13)()
            source_lines, source_binary = (sf2_lines, sf2_binary) if index == 1 else (lines, binary_pixels)
            start = time.perf_counter()
            dll.replay_obstacle(as_u8(gray), as_u8(source_binary), as_u8(source_lines),
                                frame["board_frame"], index, raw)
            elapsed = (time.perf_counter() - start) * 1000
            found = list(raw[1:5]) if raw[0] else None
            iou = intersection_over_union(box, found)
            if box:
                total["tp" if iou >= .35 else "fn"] += 1
                total["wrong_location"] += int(bool(found) and iou < .35)
            if negative and found:
                total["fp"] += 1
            total["confirmed"] += int(bool(raw[5]))
            total["suppressed"] += int(bool(raw[9]))
            if index == 0:
                assert found == (expected[0]["box"] if expected else None), (frame["replay_frame"], found, expected)
            assert not raw[10], (frame["replay_frame"], "unexpected queue overflow")
            results[mode] = {"box": found, "iou": round(iou, 3) if box else None,
                             "confirmed": bool(raw[5]), "track_hits": raw[6], "area": raw[7],
                             "binary_dark_permille": raw[8], "suppressed": bool(raw[9]),
                             "host_ms": round(elapsed, 4), "interpolated_rows": raw[12]}
        records.append({"frame": frame["replay_frame"], "label_box": box, "results": results})
    assert totals["sf2_frozen"]["tp"] >= 13 and totals["sf2_frozen"]["fp"] == 0
    print(json.dumps(totals, indent=2))
    assert dll.detector_unit_tests() == 0, "Synthetic detector/track safety case failed"
    print("PASS: ROI isolation, invalid rows, queue overflow, temporal confirmation/reset")
    return {"totals": totals, "frames": records}


if __name__ == "__main__":
    with tempfile.TemporaryDirectory(prefix="brick-host-") as tmp:
        output = Path(tmp) / "obstacle.dll"
        compile_detector(output)
        import _ctypes
        dll = C.CDLL(str(output))
        try:
            result = check(dll)
        finally:
            _ctypes.FreeLibrary(dll._handle)
        if path := os.environ.get("BRICK_REPLAY_REPORT"):
            Path(path).write_text(json.dumps(result, indent=2), encoding="utf-8")
