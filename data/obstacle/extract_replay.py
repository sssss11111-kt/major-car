"""Extract original pixels and frozen offline vision results without running the HTML."""

import argparse
import base64
import gzip
import hashlib
import json
from pathlib import Path
import re
import struct
import zlib

WIDTH, HEIGHT = 188, 120
SIZE = WIDTH * HEIGHT
FIRST, LAST = 100, 200  # One-based replay page numbers confirmed by the user.
ROOT = Path(__file__).resolve().parent


def extract(source, output):
    html = source.read_text(encoding="utf-8")
    encoded = re.search(r'<script id="embeddedRecord"[^>]*>([^<]+)</script>', html)[1]
    packed = gzip.decompress(base64.b64decode(encoded.strip(), validate=True))
    if packed[:4] != b"TRP1":
        raise ValueError("Expected a TRP1 portable replay")
    header_size, = struct.unpack_from("<I", packed, 4)
    start = 8 + header_size
    header = json.loads(packed[8:start])
    frames = header["frames"]
    if (header["format"] != "track-replay-v1" or
            (header["width"], header["height"]) != (WIDTH, HEIGHT) or
            len(frames) != 221 or len(packed) != start + SIZE * len(frames)):
        raise ValueError("Unexpected source replay dimensions or frame count")
    samples = []
    payload = bytearray()
    for index, frame in enumerate(frames):
        pixels = packed[start + SIZE * index:start + SIZE * (index + 1)]
        digest = hashlib.sha256(pixels).hexdigest()
        result, recorded = frame["result"], frame["recorded"]
        if digest != frame["sha256"] or digest != result["sha256"]:
            raise ValueError(f"Gray/result checksum mismatch at replay frame {index + 1}")
        if recorded.get("crc32") and f"{zlib.crc32(pixels):08x}" != recorded["crc32"].lower():
            raise ValueError(f"Camera CRC mismatch at replay frame {index + 1}")
        thresholds = base64.b64decode(result["thresholds"], validate=True)
        lines = base64.b64decode(result["lines"], validate=True)
        if len(thresholds) != HEIGHT or len(lines) != 3 * HEIGHT:
            raise ValueError(f"Invalid vision result at replay frame {index + 1}")
        if not FIRST <= index + 1 <= LAST:
            continue
        samples.append({
            "replay_frame": index + 1,
            "board_frame": recorded["board_frame"],
            "original_file": frame["filename"],
            "gray_sha256": digest,
            "pair_verified": True,
            "gray_offset": len(payload),
            "threshold_offset": len(payload) + SIZE,
            "lines_offset": len(payload) + SIZE + HEIGHT,
            "board_delta": result["board_delta"],
            "history_reset": bool(result["reset"] or frame["breakBefore"]),
        })
        payload.extend(pixels + thresholds + lines)
    output.mkdir(parents=True, exist_ok=True)
    manifest = {
        "source": source.name,
        "source_sha256": hashlib.sha256(source.read_bytes()).hexdigest(),
        "algorithm": header["algorithm"],
        "width": WIDTH, "height": HEIGHT,
        "binary_rule": "255 if gray[y,x] > thresholds[y], otherwise 0",
        "payload": "frames-100-200.bin.gz",
        "payload_sha256": hashlib.sha256(payload).hexdigest(),
        "samples": samples,
    }
    (output / "manifest.json").write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    (output / manifest["payload"]).write_bytes(gzip.compress(payload, mtime=0))
    print(f"Verified {len(frames)} source frames, extracted {len(samples)} same-image pairs")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("html", type=Path)
    parser.add_argument("--output", type=Path, default=ROOT)
    args = parser.parse_args()
    extract(args.html, args.output)
