"""Render image-space bypass previews using real frames and C detector outputs."""

import argparse
import html
import json
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont
from scipy.ndimage import binary_erosion

from analyze import intersection_over_union, load_frames
from extract_replay import ROOT


def plan(binary, lines, box, margin):
    """Row DP with bounded image slope/acceleration, returning a complete path."""
    top, bottom = max(36, box[1] - 10), 105
    rows = list(range(bottom, top - 1, -1))
    allowed = binary_erosion(binary, structure=np.ones((1, margin * 2 + 1)), border_value=0)
    for y in rows:
        left, mid, right = map(int, lines[:, y])
        if not (2 < left < mid < right < 185):
            return None, "missing_lane"
        allowed[y, :left + margin + 1] = False
        allowed[y, right - margin:] = False
    x0, y0, x1, y1 = box
    allowed[max(0, y0 - 4):min(120, y1 + 5),
            max(0, x0 - margin):min(188, x1 + margin + 1)] = False
    start = int(lines[1, bottom])
    if not allowed[bottom, start]:
        return None, "blocked_start"
    costs = np.full((188, 7), np.inf)
    costs[start, 3] = 0
    parents = []
    for y in rows[1:]:
        next_costs = np.full_like(costs, np.inf)
        previous = np.full((188, 7), -1, dtype=np.int8)
        for delta in range(-3, 4):
            xs = np.arange(max(0, delta), min(188, 188 + delta))
            old_xs = xs - delta
            for old_delta in range(max(-3, delta - 1), min(3, delta + 1) + 1):
                score = costs[old_xs, old_delta + 3] + (
                    (xs - int(lines[1, y])) ** 2 * .01 + delta ** 2 * .8 +
                    (delta - old_delta) ** 2 * 3)
                better = allowed[y, xs] & (score < next_costs[xs, delta + 3])
                selected = xs[better]
                next_costs[selected, delta + 3] = score[better]
                previous[selected, delta + 3] = old_delta
        parents.append(previous)
        costs = next_costs
    if not np.isfinite(costs).any():
        return None, "no_corridor"
    x, state = np.unravel_index(np.argmin(costs), costs.shape)
    path = [(int(x), rows[-1])]
    for i in range(len(parents) - 1, -1, -1):
        delta = state - 3
        old_delta = int(parents[i][x, state])
        x -= delta
        state = old_delta + 3
        path.append((int(x), rows[i]))
    path.reverse()
    # Validate the returned path independently of DP costs.
    assert len(path) == len(rows) and path[0] == (start, bottom)
    assert all(allowed[y, x] for x, y in path)
    slopes = [b[0] - a[0] for a, b in zip(path, path[1:])]
    assert all(abs(s) <= 3 for s in slopes)
    assert all(abs(b - a) <= 1 for a, b in zip(slopes, slopes[1:]))
    return path, "pixel_route"


def crosses(path, box):
    if not path or box is None:
        return None
    return any(box[0] <= x <= box[2] and box[1] <= y <= box[3] for x, y in path)


def draw_line(draw, path, color, dashed=False):
    for index, (a, b) in enumerate(zip(path, path[1:])):
        if not dashed or index % 4 < 2:
            draw.line((a, b), fill=color, width=1)


def panel(frame, gray, binary, lines, label, detection, path, status, margin):
    scale = 3
    sheet = Image.new("RGB", (188 * 3, 151), "#182d3c")
    sheet.paste(Image.fromarray(gray).convert("RGB"), (0, 0))
    sheet.paste(Image.fromarray(binary.astype(np.uint8) * 255).convert("RGB"), (188, 0))
    sheet.paste(Image.fromarray(gray).convert("RGB"), (376, 0))
    draw = ImageDraw.Draw(sheet)
    for col in range(3):
        original = [(int(lines[1, y]) + col * 188, y) for y in range(36, 106)
                    if 0 < lines[1, y] < 187]
        draw_line(draw, original, "#ffdb56")
    box = detection["box"]
    if label:
        draw.rectangle((label[0] + 376, label[1], label[2] + 376, label[3]), outline="#f269c7")
    if box:
        draw.rectangle((box[0] + 376, box[1], box[2] + 376, box[3]), outline="#ff594e")
    if path:
        draw_line(draw, [(x + 376, y) for x, y in path], "#00e6eb",
                  dashed=not detection["confirmed"])
    draw.text((4, 122), f"Frame {frame} | original", fill="#ffdb56")
    draw.text((192, 122), "Original SF2 binary", fill="white")
    draw.text((380, 122), f"{status} / margin {margin}px", fill="#00e6eb")
    draw.text((4, 137), "Yellow original  Cyan preview  Red detected  Pink face label", fill="white")
    return sheet.resize((sheet.width * scale, sheet.height * scale), Image.Resampling.NEAREST)


def run(report_file, output, margin):
    output.mkdir(parents=True, exist_ok=True)
    reports = {r["frame"]: r["results"]["sf2_frozen"]
               for r in json.loads(report_file.read_text(encoding="utf-8"))["frames"]}
    items, thumbnails = [], []
    for frame, gray, binary, lines, label, negative in load_frames():
        number = frame["replay_frame"]
        detection = reports[number]
        path, status = (None, "not_detected")
        if detection["box"]:
            path, status = plan(binary, lines, detection["box"], margin)
        original = [(int(lines[1, y]), y) for y in range(36, 106) if lines[1, y]]
        record = dict(frame=number, board_frame=frame["board_frame"],
                      label=label, negative=negative, detection=detection, status=status,
                      path=path, original_crosses_face=crosses(original, label),
                      new_crosses_face=crosses(path, label),
                      matched=bool(label and intersection_over_union(label, detection["box"]) >= .35))
        items.append(record)
        image = panel(number, gray, binary, lines, label, detection, path, status, margin)
        image.save(output / f"{number}.png")
        if number in (104, 118, 142, 162, 172, 200):
            thumbnails.append(image)
    labeled = [r for r in items if r["label"]]
    jumps = []
    for previous, current in zip(items, items[1:]):
        if current["board_frame"] - previous["board_frame"] != 1 or not previous["path"] or not current["path"]:
            continue
        a = {y: x for x, y in previous["path"]}
        b = {y: x for x, y in current["path"]}
        common = set(a) & set(b)
        jumps.append({"frames": [previous["frame"], current["frame"]],
                      "max_common_row_shift": max(abs(a[y] - b[y]) for y in common)})
    summary = {
        "frames": len(items), "labeled_frames": len(labeled), "pixel_margin": margin,
        "detected_labeled": sum(r["matched"] for r in labeled),
        "correct_detection_and_pixel_route": sum(r["matched"] and r["path"] is not None and
                                                  not r["new_crosses_face"] for r in labeled),
        "confirmed_correct_detection_and_pixel_route": sum(r["matched"] and r["path"] is not None and
                                                            not r["new_crosses_face"] and
                                                            r["detection"]["confirmed"] for r in labeled),
        "original_crosses_face": sum(r["original_crosses_face"] for r in labeled),
        "pixel_routes_all_frames": sum(r["path"] is not None for r in items),
        "statuses": {s: sum(r["status"] == s for r in items) for s in sorted({r["status"] for r in items})},
        "consecutive_route_pairs": len(jumps),
        "max_common_row_shift_px": max((r["max_common_row_shift"] for r in jumps), default=None),
        "pairs_with_shift_over_8px": sum(r["max_common_row_shift"] > 8 for r in jumps),
        "limitations": ["Pixel margins are experimental and have no vehicle-width calibration.",
                        "SF2 displayed lane boundaries include historical estimates.",
                        "Labels describe the visible front face, not the full brick footprint.",
                        "Every frame is planned independently. Confirmed status comes from the C detector.",
                        "The calibrated C bypass planner cannot run without physical calibration."]}
    (output / "centerlines.json").write_text(json.dumps(dict(summary=summary, frames=items, temporal_jumps=jumps), indent=2),
                                            encoding="utf-8")
    sheet = Image.new("RGB", (thumbnails[0].width, sum(im.height for im in thumbnails)))
    y = 0
    for im in thumbnails:
        sheet.paste(im, (0, y))
        y += im.height
    sheet.save(output / "comparison.png")
    metadata = json.dumps([{"frame": r["frame"], "status": r["status"],
                           "confirmed": r["detection"]["confirmed"]} for r in items])
    document = """<!doctype html><meta charset="utf-8"><title>红砖中线回放</title>
<style>body{background:#182d3c;color:white;font:16px system-ui;max-width:1720px;margin:24px auto}
img{width:100%;image-rendering:pixelated}button,input{font:inherit;margin:8px}pre{white-space:pre-wrap}</style>
<h1>真实图像上的红砖中线候选回放</h1>
<p>左为原灰度与原中线，中为原二值图，右为新候选中线。黄色原中线，青色候选线，红框检测，粉框人工正面初标。青色虚线表示尚未多帧确认。</p>
<p>像素裕量为 MARGIN px，未做车宽和距离标定。逐帧生成，不代表车辆已完成连续绕行。停止或漏检时右图只保留原中线。</p>
<button id="prev">上一张</button><button id="play">播放</button><button id="next">下一张</button>
<input id="seek" type="range" min="0" max="100" value="0"><span id="info"></span>
<img id="image"><pre>SUMMARY</pre><script>
const data=METADATA;let index=0,timer;const seek=document.getElementById('seek');
function show(i){index=Math.max(0,Math.min(100,i));seek.value=index;const f=data[index];
document.getElementById('image').src=f.frame+'.png';
document.getElementById('info').textContent='第 '+f.frame+' 张 / '+f.status+' / 多帧确认 '+f.confirmed;}
document.getElementById('prev').onclick=()=>show(index-1);document.getElementById('next').onclick=()=>show(index+1);
seek.oninput=()=>show(+seek.value);document.getElementById('play').onclick=()=>{if(timer){clearInterval(timer);timer=null;}
else timer=setInterval(()=>show((index+1)%101),250);};show(0);</script>"""
    document = document.replace("MARGIN", str(margin)).replace("SUMMARY", html.escape(json.dumps(summary, ensure_ascii=False, indent=2)))
    (output / "index.html").write_text(document.replace("METADATA", metadata), encoding="utf-8")
    print(json.dumps(summary, indent=2))
    return summary


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--report", type=Path, default=ROOT / "results/c-replay.json")
    parser.add_argument("--output", type=Path, default=ROOT / "centerline-preview")
    parser.add_argument("--margin", type=int, default=8)
    args = parser.parse_args()
    if not 1 <= args.margin <= 20:
        parser.error("margin must be 1..20 image pixels")
    run(args.report, args.output, args.margin)
