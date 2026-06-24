from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional

import numpy as np

from config import (
    CONTOUR_MIN_PIXELS, RESAMPLE_SPACING_MM, TOOL_ORIENTATION,
)


@dataclass
class ExtractedPath:
    strokes: list[list[tuple[float, float]]]  # pixel-coordinate strokes, ordered for efficient travel
    total_strokes: int
    total_points: int


def extract_from_edges(
    edges: np.ndarray,
    min_contour_pixels: int = CONTOUR_MIN_PIXELS,
    offset: tuple[int, int] = (0, 0),
) -> ExtractedPath:
    """
    Turn a binary groove image (1-px-wide centrelines, white on black) into
    ordered, resampled pixel strokes.

    The live groove preview and the final captured path come from identical
    processing. ``offset`` (x0, y0) shifts every point back into full-frame pixel
    coordinates when the grooves were computed on a cropped sub-image, so the
    workspace mapping in pixels_to_robot_coords stays correct.
    """
    strokes = _chains_from_edges(edges, min_contour_pixels)

    if not strokes:
        return ExtractedPath(strokes=[], total_strokes=0, total_points=0)

    # Compute pixel spacing from workspace config if available — fall back to 10px
    spacing_px = 10.0
    strokes_smoothed  = [smooth_stroke(s) for s in strokes]
    strokes_resampled = [resample_stroke(s, spacing_px) for s in strokes_smoothed]
    strokes_ordered   = _order_strokes(strokes_resampled)

    ox, oy = offset
    if ox or oy:
        strokes_ordered = [[(x + ox, y + oy) for x, y in s] for s in strokes_ordered]

    total_pts = sum(len(s) for s in strokes_ordered)
    return ExtractedPath(
        strokes=strokes_ordered,
        total_strokes=len(strokes_ordered),
        total_points=total_pts,
    )


def pixels_to_robot_coords(
    strokes: list[list[tuple[int, int]]],
    workspace,
    frame_width: int,
    frame_height: int,
    draw_z_offset: float = 0.0,
) -> list[list[list[float]]]:
    """
    Convert pixel strokes to robot base-frame poses.

    Assumes the camera looks straight down and covers the full workspace rectangle.
    Image rows (v) increase downward, but the workspace Y axis increases upward, so v
    is flipped to keep the preview the same way up as the camera image:
      wx = (u / frame_width)          * workspace.x_extent
      wy = (1 - v / frame_height)     * workspace.y_extent
      p  = origin + wx*x_axis + wy*y_axis + draw_z_offset*z_axis

    Returns list of strokes, each stroke is a list of [x, y, z, rx, ry, rz].
    """
    o  = workspace.origin
    xa = workspace.x_axis
    ya = workspace.y_axis
    za = workspace.z_axis
    xe = workspace.x_extent
    ye = workspace.y_extent
    rx, ry, rz = TOOL_ORIENTATION

    robot_strokes: list[list[list[float]]] = []
    for stroke in strokes:
        robot_stroke: list[list[float]] = []
        for u, v in stroke:
            wx = (u / frame_width)        * xe
            wy = (1.0 - v / frame_height) * ye   # flip: image row grows down, world Y grows up
            px = o[0] + wx * xa[0] + wy * ya[0] + draw_z_offset * za[0]
            py = o[1] + wx * xa[1] + wy * ya[1] + draw_z_offset * za[1]
            pz = o[2] + wx * xa[2] + wy * ya[2] + draw_z_offset * za[2]
            robot_stroke.append([px, py, pz, rx, ry, rz])
        if robot_stroke:
            robot_strokes.append(robot_stroke)

    return robot_strokes


def _chains_from_edges(
    edge_img: np.ndarray,
    min_len: int,
) -> list[list[tuple[int, int]]]:
    """
    Extract ordered pixel chains from a binary groove image via 8-connectivity
    chain-following. Each edge pixel is visited once, giving the true centerline
    without the double-tracing artefact that cv2.findContours produces on thin edges.
    """
    ys, xs = np.where(edge_img > 0)
    if len(xs) == 0:
        return []
    h, w = edge_img.shape
    remaining: set[tuple[int, int]] = set(zip(xs.tolist(), ys.tolist()))

    def nbrs(x: int, y: int) -> list[tuple[int, int]]:
        return [
            (x + dx, y + dy)
            for dx in (-1, 0, 1) for dy in (-1, 0, 1)
            if (dx, dy) != (0, 0)
            and 0 <= x + dx < w and 0 <= y + dy < h
            and (x + dx, y + dy) in remaining
        ]

    # Pre-find endpoints (≤1 neighbour) to start chains from tips rather than middles.
    endpoint_q = [px for px in remaining if len(nbrs(*px)) <= 1]
    ep_idx = 0
    chains: list[list[tuple[int, int]]] = []

    while remaining:
        start: tuple[int, int] | None = None
        while ep_idx < len(endpoint_q):
            cand = endpoint_q[ep_idx]
            ep_idx += 1
            if cand in remaining:
                start = cand
                break
        if start is None:
            start = next(iter(remaining))

        chain: list[tuple[int, int]] = []
        x, y = start
        while True:
            chain.append((x, y))
            remaining.discard((x, y))
            nn = nbrs(x, y)
            if not nn:
                break
            x, y = nn[0]

        if len(chain) >= min_len:
            chains.append(chain)

    return chains


def smooth_stroke(
    pts: list[tuple[float, float]],
    iterations: int = 2,
) -> list[tuple[float, float]]:
    """Chaikin corner-cutting: each iteration replaces segments with two interior points."""
    if len(pts) < 3:
        return pts
    for _ in range(iterations):
        new_pts: list[tuple[float, float]] = [pts[0]]
        for i in range(len(pts) - 1):
            x0, y0 = pts[i]
            x1, y1 = pts[i + 1]
            new_pts.append((0.75 * x0 + 0.25 * x1, 0.75 * y0 + 0.25 * y1))
            new_pts.append((0.25 * x0 + 0.75 * x1, 0.25 * y0 + 0.75 * y1))
        new_pts.append(pts[-1])
        pts = new_pts
    return pts


def resample_stroke(
    stroke: list[tuple[float, float]],
    spacing_px: float,
) -> list[tuple[float, float]]:
    """Resample a stroke to approximately `spacing_px` pixel intervals."""
    if len(stroke) < 2:
        return stroke

    cum = [0.0]
    for i in range(1, len(stroke)):
        dx = stroke[i][0] - stroke[i - 1][0]
        dy = stroke[i][1] - stroke[i - 1][1]
        cum.append(cum[-1] + math.sqrt(dx * dx + dy * dy))

    total = cum[-1]
    if total < spacing_px:
        return [stroke[0], stroke[-1]]

    result: list[tuple[float, float]] = []
    target = 0.0
    j = 0

    while target <= total + 1e-9:
        while j + 1 < len(cum) and cum[j + 1] < target:
            j += 1
        if j + 1 >= len(stroke):
            break
        seg_len = cum[j + 1] - cum[j]
        t = (target - cum[j]) / seg_len if seg_len > 1e-9 else 0.0
        x = stroke[j][0] + t * (stroke[j + 1][0] - stroke[j][0])
        y = stroke[j][1] + t * (stroke[j + 1][1] - stroke[j][1])
        result.append((x, y))
        target += spacing_px

    if not result or result[-1] != stroke[-1]:
        result.append(stroke[-1])

    return result


def _order_strokes(
    strokes: list[list[tuple[float, float]]],
) -> list[list[tuple[float, float]]]:
    """
    Nearest-neighbour TSP: order strokes to minimise total pen-up travel.
    Each stroke can be reversed if its end is closer to the current position.
    """
    if not strokes:
        return []

    remaining = list(range(len(strokes)))
    ordered: list[list[tuple[float, float]]] = []

    first = strokes[remaining.pop(0)]
    ordered.append(first)
    cx, cy = first[-1]

    while remaining:
        best_i     = None
        best_dist  = float("inf")
        best_rev   = False

        for idx in remaining:
            s = strokes[idx]
            d_fwd = _dist2(cx, cy, s[0][0],  s[0][1])
            d_rev = _dist2(cx, cy, s[-1][0], s[-1][1])
            if d_fwd <= d_rev and d_fwd < best_dist:
                best_dist = d_fwd
                best_i    = idx
                best_rev  = False
            elif d_rev < d_fwd and d_rev < best_dist:
                best_dist = d_rev
                best_i    = idx
                best_rev  = True

        chosen = strokes[best_i]
        if best_rev:
            chosen = list(reversed(chosen))
        ordered.append(chosen)
        remaining.remove(best_i)
        cx, cy = chosen[-1]

    return ordered


def _dist2(x1: float, y1: float, x2: float, y2: float) -> float:
    dx = x1 - x2
    dy = y1 - y2
    return dx * dx + dy * dy
