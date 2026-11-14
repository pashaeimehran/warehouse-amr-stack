#!/usr/bin/env python3
"""Measures a saved occupancy map against the true world geometry.

The ground truth comes from warehouse_sim's world generator (the same constants
that wrote the SDF): every obstacle the 2D LiDAR can see at its mounting height
(walls, shelf cabinets with end panels, the drop-off table) as an axis-aligned
rectangle in the map frame. The SLAM map frame starts at the dock, which is the
world origin, so no alignment is estimated; a drift would show up as error.

Reported:
  - distance from each occupied map cell centre to the nearest true obstacle
    outline: median, p95, max, and the share within 0.10 m (two cells);
  - outline coverage: share of points sampled every 5 cm along the true
    outlines (where the robot could see them) that have an occupied cell
    within 0.10 m.

Usage:
    python3 tools/analysis/map_accuracy.py src/warehouse_bringup/maps/warehouse.yaml
"""

from __future__ import annotations

import argparse
import importlib.util
import json
import math
import sys
from pathlib import Path

import yaml

REPO = Path(__file__).resolve().parents[2]
GENERATOR = REPO / "src" / "warehouse_sim" / "scripts" / "generate_world.py"


def load_generator():
    spec = importlib.util.spec_from_file_location("generate_world", GENERATOR)
    module = importlib.util.module_from_spec(spec)
    sys.modules["generate_world"] = module
    spec.loader.exec_module(module)
    return module


def obstacles(gen) -> list[tuple[float, float, float, float]]:
    """Rectangles (x_min, y_min, x_max, y_max) visible at LiDAR height."""
    (x0, x1), (y0, y1) = gen.HALL_X, gen.HALL_Y
    t = gen.WALL_THICKNESS
    rects = [(x0 - t, y1, x1 + t, y1 + t), (x0 - t, y0 - t, x1 + t, y0),
             (x0 - t, y0, x0, y1), (x1, y0, x1 + t, y1)]
    half = gen.SHELF_DEPTH / 2
    e = gen.END_PANEL_THICKNESS
    for row_y in gen.SHELF_ROWS_Y:
        rects.append((gen.SHELF_X[0] - e, row_y - half, gen.SHELF_X[1] + e, row_y + half))
    tx, ty, sx, sy, _ = gen.DROP_OFF_TABLE
    rects.append((tx - sx / 2, ty - sy / 2, tx + sx / 2, ty + sy / 2))
    return rects


def distance_to_outline(x: float, y: float, rect) -> float:
    x_min, y_min, x_max, y_max = rect
    dx = max(x_min - x, 0.0, x - x_max)
    dy = max(y_min - y, 0.0, y - y_max)
    if dx > 0.0 or dy > 0.0:
        return math.hypot(dx, dy)
    return min(x - x_min, x_max - x, y - y_min, y_max - y)   # inside: to the nearest edge


def read_pgm(path: Path) -> tuple[int, int, bytes]:
    data = path.read_bytes()
    tokens, pos = [], 0
    while len(tokens) < 4:   # magic, width, height, maxval (skipping comments)
        while data[pos:pos + 1].isspace():
            pos += 1
        if data[pos:pos + 1] == b"#":
            pos = data.index(b"\n", pos)
            continue
        end = pos
        while not data[end:end + 1].isspace():
            end += 1
        tokens.append(data[pos:end])
        pos = end
    if tokens[0] != b"P5":
        raise ValueError(f"{path}: only binary PGM (P5) is supported")
    width, height = int(tokens[1]), int(tokens[2])
    return width, height, data[pos + 1:pos + 1 + width * height]


def percentile(sorted_values: list[float], q: float) -> float:
    rank = q / 100 * (len(sorted_values) - 1)
    lo = int(math.floor(rank))
    hi = min(lo + 1, len(sorted_values) - 1)
    return sorted_values[lo] + (rank - lo) * (sorted_values[hi] - sorted_values[lo])


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("map_yaml", type=Path)
    parser.add_argument("--tolerance", type=float, default=0.10)
    args = parser.parse_args()

    meta = yaml.safe_load(args.map_yaml.read_text())
    width, height, pixels = read_pgm(args.map_yaml.parent / meta["image"])
    resolution = float(meta["resolution"])
    ox, oy = float(meta["origin"][0]), float(meta["origin"][1])
    negate = int(meta.get("negate", 0))
    occupied_thresh = float(meta["occupied_thresh"])

    occupied = []
    for row in range(height):
        for col in range(width):
            value = pixels[row * width + col]
            p = value / 255.0 if negate else (255 - value) / 255.0   # occupancy probability
            if p > occupied_thresh:
                # PGM row 0 is the top (max y) of the map.
                occupied.append((ox + (col + 0.5) * resolution,
                                 oy + (height - row - 0.5) * resolution))

    rects = obstacles(load_generator())
    distances = sorted(min(distance_to_outline(x, y, r) for r in rects) for x, y in occupied)

    # Outline coverage on the sides facing open floor (outer wall faces are never seen).
    samples, covered = 0, 0
    occupied_set = {(round((x - ox) / resolution - 0.5), round((y - oy) / resolution - 0.5))
                    for x, y in occupied}
    reach = int(math.ceil(args.tolerance / resolution))
    for i, (x_min, y_min, x_max, y_max) in enumerate(rects):
        if i < 4:   # walls: only the inner faces
            edges = [((x_min, y_min), (x_min, y_max)) if i == 3 else
                     ((x_max, y_min), (x_max, y_max)) if i == 2 else
                     ((x_min, y_min), (x_max, y_min)) if i == 0 else
                     ((x_min, y_max), (x_max, y_max))]
        else:
            edges = [((x_min, y_min), (x_max, y_min)), ((x_min, y_max), (x_max, y_max)),
                     ((x_min, y_min), (x_min, y_max)), ((x_max, y_min), (x_max, y_max))]
        for (ax, ay), (bx, by) in edges:
            n = max(1, int(math.hypot(bx - ax, by - ay) / 0.05))
            for k in range(n + 1):
                px, py = ax + (bx - ax) * k / n, ay + (by - ay) * k / n
                cx, cy = int((px - ox) / resolution), int((py - oy) / resolution)
                if not (0 <= cx < width and 0 <= cy < height):
                    continue
                samples += 1
                if any((cx + dx, cy + dy) in occupied_set
                       for dx in range(-reach, reach + 1) for dy in range(-reach, reach + 1)):
                    covered += 1

    result = {
        "map": str(args.map_yaml),
        "size_cells": [width, height],
        "resolution_m": resolution,
        "occupied_cells": len(distances),
        "distance_to_truth_m": {
            "median": round(percentile(distances, 50), 4),
            "p95": round(percentile(distances, 95), 4),
            "max": round(distances[-1], 4),
        },
        f"occupied_within_{args.tolerance:.2f}_m": round(
            sum(d <= args.tolerance for d in distances) / len(distances), 4),
        "outline_coverage": round(covered / samples, 4) if samples else None,
    }
    print(json.dumps(result, indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())
