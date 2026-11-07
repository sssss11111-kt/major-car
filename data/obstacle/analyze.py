"""Analyze the real camera replay and compare four front-face segmentation methods."""

import argparse
import gzip
import hashlib
import json
from pathlib import Path
import time

import numpy as np
from PIL import Image, ImageDraw
from scipy import ndimage

from extract_replay import HEIGHT, ROOT, SIZE, WIDTH

TOP, BOTTOM = 36, 100  # Experimental: front-face labels span rows 43–106.
MARGIN = 11  # Includes margin for projecting a lower lane measurement upward.
ANCHOR_OFFSET = 7
CONTRAST = 30  # Face-to-road median differences in the labeled sequence: 53–100 DN.
EDGE = 23
METHODS = ("gray", "binary", "fusion", "gray_edge")


def load_frames():
    manifest = json.loads((ROOT / "manifest.json").read_text(encoding="utf-8"))
    labels = json.loads((ROOT / "labels.json").read_text(encoding="utf-8"))
    data = gzip.decompress((ROOT / manifest["payload"]).read_bytes())
    if hashlib.sha256(data).hexdigest() != manifest["payload_sha256"]:
        raise ValueError("Replay payload checksum mismatch")
    for frame in manifest["samples"]:
        number = str(frame["replay_frame"])
        gray = np.frombuffer(data, np.uint8, SIZE, frame["gray_offset"]).reshape(HEIGHT, WIDTH)
        rows = np.frombuffer(data, np.uint8, HEIGHT, frame["threshold_offset"])
        lines = np.frombuffer(data, np.uint8, 3 * HEIGHT, frame["lines_offset"]).reshape(3, HEIGHT)
        yield frame, gray, gray > rows[:, None], lines, labels["positive"].get(number), (
            frame["replay_frame"] in labels["negative"])


def road_reference(gray, lines):
    roi = np.zeros_like(gray, dtype=bool)
    reference = np.zeros(HEIGHT, dtype=np.int16)
    lefts, centers, rights = lines.astype(int)
    valid = ((lefts > 2) & (lefts < centers) & (centers < rights) & (rights < WIDTH - 3)
             & (rights - lefts >= 35) & (rights - lefts <= 170))
    for y in range(TOP, BOTTOM + 1):
        # Use a row just below the front face as a corridor proxy, limiting
        # false boundaries caused by the face itself. SF2 display rows are
        # approximate. On board the input must carry measured-row provenance.
        anchor = min(y + ANCHOR_OFFSET, 105)
        if valid[anchor]:
            left, right = lefts[anchor], rights[anchor]
        else:
            before = next((i for i in range(anchor - 1, max(-1, anchor - 13), -1) if valid[i]), None)
            after = next((i for i in range(anchor + 1, min(106, anchor + 13)) if valid[i]), None)
            if before is None or after is None or after - before > 12:
                continue
            if abs(lefts[after] - lefts[before]) > 24 or abs(rights[after] - rights[before]) > 24:
                continue
            left = lefts[before] + int((lefts[after] - lefts[before]) * (anchor - before) / (after - before))
            right = rights[before] + int((rights[after] - rights[before]) * (anchor - before) / (after - before))
        l, r = left + MARGIN, right - MARGIN
        roi[y, l:r + 1] = True
        row = gray[y, l:r + 1]
        rank = 3 * len(row) // 4
        reference[y] = np.partition(row, rank)[rank]
    return roi, reference


def candidates(mask, dark, edge):
    ids, count = ndimage.label(mask, structure=np.ones((3, 3)))
    result = []
    for label, (ys, xs) in enumerate(ndimage.find_objects(ids), 1):
        width, height = xs.stop - xs.start, ys.stop - ys.start
        if not (10 <= width <= 56 and 3 <= height <= 28 and width >= height):
            continue
        blob = ids[ys, xs] == label
        area = int(blob.sum())
        fill = 1000 * area // (width * height)
        if fill < 440 or area < (28 if ys.stop <= 65 else 48):
            continue
        bottom = slice(max(ys.start, ys.stop - 5), ys.stop)
        support = np.any((ids[bottom, xs] == label) & edge[bottom, xs], axis=0)
        edge_columns = int(support.sum())
        result.append({
            "box": [xs.start, ys.start, xs.stop - 1, ys.stop - 1],
            "area": area, "fill_permille": fill, "edge_columns": edge_columns,
            "contrast": int(dark[ys, xs][blob].mean()),
        })
    return sorted(result, key=lambda x: (x["edge_columns"], x["area"]), reverse=True), count


def detect(gray, binary, lines):
    roi, reference = road_reference(gray, lines)
    gray = gray.astype(np.int16)
    dark = reference[:, None] - gray
    edge = np.zeros_like(roi)
    edge[:-4] = (gray[4:] - gray[:-4] >= EDGE) & (gray[4:] >= reference[:-4, None] - 18)
    dark_mask = roi & (dark >= CONTRAST)
    binary_mask = roi & ~binary
    masks = {"gray": dark_mask, "binary": binary_mask, "fusion": dark_mask & binary_mask,
             "gray_edge": dark_mask}
    methods = {}
    for method, mask in masks.items():
        items, total = candidates(mask, dark, edge)
        if method == "gray_edge":
            items = [item for item in items if item["edge_columns"] * 2 >=
                     item["box"][2] - item["box"][0] + 1]
        elif method != "fusion":
            items.sort(key=lambda x: x["area"], reverse=True)
        methods[method] = {"components": total, "candidates": items}
    return roi, masks, methods


def intersection_over_union(a, b):
    if not a or not b:
        return 0.0
    intersection = max(0, min(a[2], b[2]) - max(a[0], b[0]) + 1) * max(
        0, min(a[3], b[3]) - max(a[1], b[1]) + 1)
    area = lambda r: (r[2] - r[0] + 1) * (r[3] - r[1] + 1)
    return intersection / (area(a) + area(b) - intersection)


def stats(values):
    return {"count": int(values.size), "min": int(values.min()), "max": int(values.max()),
            "mean": round(float(values.mean()), 3), "median": float(np.median(values)),
            "std": round(float(values.std()), 3),
            "histogram": np.bincount(values.ravel(), minlength=256).tolist()}


def face_stats(gray, binary, box, roi):
    x, y, right, bottom = box
    face = gray[y:bottom + 1, x:right + 1]
    local_binary = binary[y:bottom + 1, x:right + 1]
    road = gray[bottom + 2:bottom + 6, x:right + 1]
    dark_labels, _ = ndimage.label(~binary, structure=np.ones((3, 3)))
    components = set(np.unique(dark_labels[y:bottom + 1, x:right + 1])) - {0}
    outer = set(np.unique(dark_labels[:, [0, -1]])) - {0}
    # Below-face pixels provide same-column road context. The adjacent
    # one-row band is a transition sample, since the gray frame alone
    # does not establish a separate ground-truth shadow boundary.
    transition = gray[bottom + 1:bottom + 2, x:right + 1]
    return {"face": stats(face), "road_below": stats(road), "transition": stats(transition),
            "roi": stats(gray[roi]) if roi.any() else None,
            "binary_0": int((~local_binary).sum()), "binary_255": int(local_binary.sum()),
            "binary_1": 0, "binary_white_ratio": float(local_binary.mean()),
            "binary_components_in_box": len(components), "connected_to_image_side": bool(outer & components),
            "dark_pixels_by_row": (~local_binary).sum(axis=1).tolist(),
            "vertical_profile": gray[max(0, y - 5):bottom + 7, (x + right) // 2].tolist()}


def panel(number, gray, binary, roi, masks, box, methods):
    images = [gray, binary, roi, masks["gray"], masks["binary"], masks["fusion"], gray]
    canvas = Image.new("RGB", (7 * WIDTH, HEIGHT + 20), "#1c3242")
    draw = ImageDraw.Draw(canvas)
    for col, (name, pixels) in enumerate(zip(
            (f"{number} Gray / label", "SF2 Binary", "Lane ROI", "A Gray", "B Binary",
             "C Fusion", "D Gray + bottom edge"), images)):
        view = pixels.astype(np.uint8) * 255 if pixels.dtype == bool else pixels
        canvas.paste(Image.fromarray(view), (col * WIDTH, 0))
        draw.text((col * WIDTH + 3, HEIGHT + 2), name, fill="white")
        if box and col in (0, 6):
            x, y, r, b = box
            draw.rectangle((x + col * WIDTH, y, r + col * WIDTH, b), outline="#00e0ae")
    for col, method in zip((3, 4, 5, 6), METHODS):
        for index, item in enumerate(methods[method]["candidates"][:4]):
            x, y, r, b = item["box"]
            draw.rectangle((x + col * WIDTH, y, r + col * WIDTH, b),
                           outline="#f5a623" if index == 0 else "#f6dc79")
    return canvas


def run(output):
    output.mkdir(parents=True, exist_ok=True)
    records = []
    totals = {method: dict(tp=0, fp=0, fn=0, wrong_location=0) for method in METHODS}
    representatives = []
    for frame, gray, binary, lines, box, negative in load_frames():
        start = time.perf_counter()
        roi, masks, methods = detect(gray, binary, lines)
        elapsed = (time.perf_counter() - start) * 1000
        number = frame["replay_frame"]
        view = panel(number, gray, binary, roi, masks, box, methods)
        view.save(output / f"{number:03d}.png")
        if number in (104, 118, 142, 162, 172, 200):
            representatives.append(view)
        for method, result in methods.items():
            best = result["candidates"][0]["box"] if result["candidates"] else None
            iou = intersection_over_union(best, box)
            result["iou"] = round(iou, 3) if box else None
            if box:
                totals[method]["tp" if iou >= 0.35 else "fn"] += 1
                totals[method]["wrong_location"] += int(bool(best) and iou < 0.35)
            if negative and best:
                totals[method]["fp"] += 1
        records.append({"frame": number, "board_frame": frame["board_frame"], "label_box": box,
                        "negative": negative, "gray": stats(gray), "binary_0": int((~binary).sum()),
                        "binary_255": int(binary.sum()), "methods": methods,
                        "regions": face_stats(gray, binary, box, roi) if box else None,
                        "host_ms": round(elapsed, 3)})
    for total in totals.values():
        total["recall"] = round(total["tp"] / (total["tp"] + total["fn"]), 3)
        detections = total["tp"] + total["fp"] + total["wrong_location"]
        total["precision"] = round(total["tp"] / detections, 3) if detections else None
    (output / "report.json").write_text(json.dumps({"totals": totals, "frames": records}, indent=2),
                                        encoding="utf-8")
    sheet = Image.new("RGB", (representatives[0].width, len(representatives) * representatives[0].height))
    for i, view in enumerate(representatives):
        sheet.paste(view, (0, i * view.height))
    sheet.save(output / "comparison.png")
    print(json.dumps(totals, indent=2))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path)
    run(parser.parse_args().output)
