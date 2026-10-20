"""Replay ordered 188x120 P5 PGM frames through the actual firmware C core.

No camera, serial port, or motor is accessed. Python standard library only.
Examples:
    python tools/replay_vision.py captures --out tests/out/replay
    python tools/replay_vision.py first.pgm second.pgm --out replay --fixed 100
"""
from __future__ import annotations

import argparse
import base64
import csv
import html
import json
import os
from pathlib import Path
import shutil
import struct
import subprocess
import tempfile
import zlib

WIDTH, HEIGHT = 188, 120
ROOT = Path(__file__).resolve().parents[1]
WHITESPACE = b" \t\r\n\v\f"


def read_pgm(path: Path) -> bytes:
    """Read exactly one 8-bit P5 image; never strip bytes from the raster."""
    data = Path(path).read_bytes()
    pos = 0

    def token() -> bytes:
        nonlocal pos
        while pos < len(data):
            if data[pos] in WHITESPACE:
                pos += 1
            elif data[pos] == ord("#"):
                end = data.find(b"\n", pos)
                if end < 0:
                    raise ValueError(f"{path}: unterminated header comment")
                pos = end + 1
            else:
                break
        begin = pos
        while pos < len(data) and data[pos] not in WHITESPACE and data[pos] != ord("#"):
            pos += 1
        if begin == pos:
            raise ValueError(f"{path}: incomplete PGM header")
        return data[begin:pos]

    magic, width, height, maximum = [token() for _ in range(4)]
    if magic != b"P5":
        raise ValueError(f"{path}: only binary P5 PGM is supported")
    try:
        dimensions = (int(width), int(height), int(maximum))
    except ValueError as exc:
        raise ValueError(f"{path}: noninteger PGM dimensions/maxval") from exc
    if dimensions != (WIDTH, HEIGHT, 255):
        raise ValueError(f"{path}: expected {WIDTH}x{HEIGHT} with maxval 255, got {dimensions}")
    if pos == len(data) or data[pos] not in WHITESPACE:
        raise ValueError(f"{path}: missing separator before raster")
    # PGM specifies one whitespace separator. Treat CRLF as one line ending;
    # do not use lstrip(): a first pixel of 9, 10, 13, or 32 is legitimate.
    pos += 2 if data[pos:pos + 2] == b"\r\n" else 1
    pixels = data[pos:]
    if len(pixels) != WIDTH * HEIGHT:
        raise ValueError(f"{path}: expected {WIDTH * HEIGHT} raster bytes, got {len(pixels)}")
    return pixels


def write_pgm(path: Path, pixels: bytes) -> None:
    path.write_bytes(f"P5\n{WIDTH} {HEIGHT}\n255\n".encode("ascii") + pixels)


def png_data(pixels: bytes) -> str:
    """Lossless grayscale PNG data URL, so the SVG has no external image dependency."""
    def chunk(kind: bytes, payload: bytes) -> bytes:
        return struct.pack(">I", len(payload)) + kind + payload + struct.pack(
            ">I", zlib.crc32(kind + payload) & 0xffffffff)
    rows = b"".join(b"\0" + pixels[y * WIDTH:(y + 1) * WIDTH] for y in range(HEIGHT))
    png = b"\x89PNG\r\n\x1a\n"
    png += chunk(b"IHDR", struct.pack(">IIBBBBB", WIDTH, HEIGHT, 8, 0, 0, 0, 0))
    png += chunk(b"IDAT", zlib.compress(rows))
    png += chunk(b"IEND", b"")
    return "data:image/png;base64," + base64.b64encode(png).decode("ascii")


def overlay(pixels: bytes, record: dict) -> str:
    parts = [
        f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {WIDTH} {HEIGHT + 30}" width="752" height="600">',
        '<rect width="188" height="150" fill="#161b22"/>',
        f'<image width="{WIDTH}" height="{HEIGHT}" href="{png_data(pixels)}" style="image-rendering:pixelated"/>',
    ]
    for row in record["rows"]:
        y, flags = row["y"], row["edge_flags"]
        for bit, key, color in ((1, "left", "#1eff60"), (2, "right", "#25dfff")):
            if flags & bit and row[key] >= 0:
                parts.append(f'<circle cx="{row[key]}" cy="{y}" r="1" fill="{color}"/>')
        if flags and row["center"] >= 0:
            if flags == 3:
                parts.append(f'<circle cx="{row["center"]}" cy="{y}" r="1" fill="#ffff00"/>')
            else:
                parts.append(f'<circle cx="{row["center"]}" cy="{y}" r="1.6" '
                             'fill="none" stroke="#ff9d23" stroke-width=".6"/>')
    parts.append('<g fill="white" font-family="sans-serif" font-size="4">'
                 '<text x="2" y="126">Green/cyan: measured left/right. Yellow: paired centre.</text>'
                 '<text x="2" y="132">Orange ring: inferred centre. Missing edges are not drawn.</text>')
    parts.append(f'<text x="2" y="139">valid={record["valid"]} '
                 f'paired={record["paired_rows"]}/{record["scanned_rows"]} '
                 f'inferred={record["inferred_rows"]} error={record["error"]:.2f}</text></g></svg>')
    return "\n".join(parts)


def compile_adapter(directory: Path, cc: str | None) -> Path:
    compiler = cc or str(ROOT / ".tools" / "ziglang" / ("zig.exe" if os.name == "nt" else "zig"))
    executable = directory / ("vision_replay.exe" if os.name == "nt" else "vision_replay")
    command = [compiler]
    if Path(compiler).stem.lower() == "zig":
        command.append("cc")
    command += ["-std=c99", "-Wall", "-Wextra", "-Werror", "-O1",
                "-I", str(ROOT / "demo/user/inc"), str(ROOT / "tests/vision_replay.c"),
                str(ROOT / "demo/user/src/car_core.c"), "-o", str(executable)]
    subprocess.run(command, check=True)
    return executable


def input_paths(arguments: list[str]) -> list[Path]:
    result = []
    for argument in arguments:
        path = Path(argument)
        if path.is_dir():
            result.extend(sorted((p for p in path.iterdir() if p.is_file() and p.suffix.lower() == ".pgm"),
                                 key=lambda p: p.name))
        elif path.is_file():
            result.append(path)
        else:
            raise ValueError(f"Input does not exist: {path}")
    if not result:
        raise ValueError("No PGM frames supplied")
    return result


def replay(paths: list[Path], destination: Path, cc: str | None = None,
           fixed: int | None = None, dark_road: bool = False) -> list[dict]:
    # Validate every frame before producing outputs; retain explicit input order.
    frames = [read_pgm(path) for path in paths]
    destination.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="vision-replay-") as temporary:
        work = Path(temporary)
        executable = compile_adapter(work, cc)
        sequence = work / "sequence.raw"
        sequence.write_bytes(b"".join(frames))
        completed = subprocess.run(
            [str(executable), str(sequence), str(work),
             "-1" if fixed is None else str(fixed), str(int(dark_road))],
            check=True, text=True, capture_output=True)
        records = [json.loads(line) for line in completed.stdout.splitlines() if line.strip()]
        if len(records) != len(paths):
            raise RuntimeError("Core returned an unexpected number of frames")
        for index, (path, pixels, record) in enumerate(zip(paths, frames, records)):
            stem = f"{index:06d}"
            record["source"] = str(path.resolve())
            record["row_threshold"] = [row["threshold"] for row in record["rows"]]
            write_pgm(destination / f"{stem}-gray.pgm", pixels)
            shutil.copyfile(work / f"{stem}-binary.pgm", destination / f"{stem}-binary.pgm")
            (destination / f"{stem}-overlay.svg").write_text(overlay(pixels, record), encoding="utf-8")
    (destination / "results.json").write_text(json.dumps(records, indent=2, ensure_ascii=False), encoding="utf-8")
    columns = ["frame", "source", "valid", "paired_rows", "valid_rows", "scanned_rows",
               "inferred_rows", "threshold", "contrast", "error", "row_threshold"]
    with (destination / "results.csv").open("w", newline="", encoding="utf-8-sig") as stream:
        writer = csv.DictWriter(stream, fieldnames=columns, extrasaction="ignore")
        writer.writeheader()
        writer.writerows({**r, "row_threshold": json.dumps(r["row_threshold"])} for r in records)
    cards = []
    for record in records:
        stem = f'{record["frame"]:06d}'
        cards.append(f'<section><h2>{stem} {html.escape(Path(record["source"]).name)}</h2>'
                     f'<img src="{stem}-overlay.svg" alt="Observed edges and centre">'
                     f'<p><a href="{stem}-gray.pgm">Gray PGM</a> · '
                     f'<a href="{stem}-binary.pgm">Binary PGM</a></p></section>')
    (destination / "index.html").write_text(
        '<!doctype html><meta charset="utf-8"><title>Vision replay</title>'
        '<style>body{font:16px sans-serif;max-width:900px;margin:24px auto;background:#eee}'
        'img{width:752px;max-width:100%}section{margin:30px 0}</style>'
        '<h1>Vision replay</h1><p>Frames use one persistent vision state in the supplied order. '
        'Green/cyan are measured edges; orange rings are inferred centres. '
        'Software replay does not validate real-car control or execution timing.</p>'
        '<p><a href="results.json">JSON</a> · <a href="results.csv">CSV</a></p>' + "".join(cards),
        encoding="utf-8")
    return records


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("inputs", nargs="+", help="PGM files in sequence order, or directories sorted by filename")
    parser.add_argument("--out", type=Path, required=True, help="New or empty output directory")
    parser.add_argument("--cc", help="C compiler path (Zig, GCC, or Clang); default is bundled .tools Zig")
    parser.add_argument("--fixed", type=int, choices=range(256), metavar="0..255", help="Use fixed threshold")
    parser.add_argument("--dark-road", action="store_true")
    args = parser.parse_args()
    try:
        if args.out.exists() and any(args.out.iterdir()):
            raise ValueError("Output directory must be empty, to preserve earlier replay evidence")
        records = replay(input_paths(args.inputs), args.out, args.cc, args.fixed, args.dark_road)
    except (ValueError, OSError, subprocess.CalledProcessError) as exc:
        parser.exit(2, f"Replay failed: {exc}\n")
    print(f'Replayed {len(records)} frames; {sum(bool(r["valid"]) for r in records)} valid. '
          f'Open {args.out.resolve() / "index.html"}')
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
