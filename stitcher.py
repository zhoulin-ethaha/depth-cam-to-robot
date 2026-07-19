"""
Pure math for the dual-camera stitching prototype (no hardware, no server).

Two D435i cameras look down at the sand side by side with a small (~5-10%)
overlap. Each camera's metric depth image is deprojected to 3D points with its
own intrinsics, camera 2's points are moved into camera 1's frame by a fixed
rig transform (StitchCalib — calibrated once per mounting), and both clouds are
rasterized onto one shared top-down grid. The result is a single "virtual
overhead camera" heightmap: uniform mm-per-pixel, no perspective seam, grooves
still read as locally-larger depth so `depth_extractor.grooves_and_mask` works
on it unchanged. Where the two cameras overlap, samples are averaged (the
overlap band ends up LESS noisy than either camera alone).

Frame conventions (RealSense camera frame): x right, y down, z forward into
the scene. The output grid keeps camera 1's x/y axes, so v grows down exactly
like the single-camera pipeline. Heightmap values are metres along camera 1's
z axis — "depth below camera 1" — so valleys are larger values, same as raw
depth. The rig transform models a level side-by-side mounting: translation
(tx, ty, tz) plus yaw about the viewing axis. Small pitch/roll differences are
absorbed later by the detrend stage of groove detection.

RGB: the colour stream is aligned to depth per camera, so each depth pixel
carries a colour sample where the (narrower-FOV) RGB lens covers it. Those
samples are rasterized onto the same grid; the strip the RGB lenses miss stays
black (expected — the depth image is the product, RGB is reference only).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Optional

import cv2
import numpy as np

from config import (
    DEPTH_WIDTH, DEPTH_HEIGHT,
    STITCH_MM_PER_PX, STITCH_MAX_GRID_W, STITCH_MAX_GRID_H,
    STITCH_NOMINAL_HFOV_DEG, STITCH_NOMINAL_VFOV_DEG,
)


# ── data types ────────────────────────────────────────────────────────────────

@dataclass(frozen=True)
class Intrinsics:
    """Pinhole intrinsics of one depth stream (pixels)."""
    fx: float
    fy: float
    cx: float
    cy: float
    width: int = DEPTH_WIDTH
    height: int = DEPTH_HEIGHT

    @classmethod
    def nominal(cls, width: int = DEPTH_WIDTH, height: int = DEPTH_HEIGHT) -> "Intrinsics":
        """D435 depth intrinsics estimated from the datasheet FOV (synthetic mode)."""
        fx = (width / 2) / math.tan(math.radians(STITCH_NOMINAL_HFOV_DEG) / 2)
        fy = (height / 2) / math.tan(math.radians(STITCH_NOMINAL_VFOV_DEG) / 2)
        return cls(fx=fx, fy=fy, cx=width / 2, cy=height / 2, width=width, height=height)


@dataclass
class StitchCalib:
    """
    Rig transform mapping camera-2 points into camera 1's frame:
    p1 = Rz(yaw) @ p2 + [tx, ty, tz]. Millimetres/degrees because that is what
    the UI edits; converted to metres internally.
    """
    tx_mm: float = 400.0    # cam2 offset along cam1 +x (side-by-side baseline)
    ty_mm: float = 0.0      # offset along cam1 +y (down-image)
    tz_mm: float = 0.0      # mounting-height difference along the view axis
    yaw_deg: float = 0.0    # rotation about the viewing axis
    swap: bool = False      # swap which physical camera plays cam1/cam2

    @classmethod
    def from_dict(cls, d: dict | None) -> "StitchCalib":
        d = d or {}
        c = cls()
        for k in ("tx_mm", "ty_mm", "tz_mm", "yaw_deg"):
            if k in d and d[k] is not None:
                setattr(c, k, float(d[k]))
        c.swap = bool(d.get("swap", False))
        return c

    def to_dict(self) -> dict:
        return {"tx_mm": self.tx_mm, "ty_mm": self.ty_mm, "tz_mm": self.tz_mm,
                "yaw_deg": self.yaw_deg, "swap": self.swap}


@dataclass
class CameraFrame:
    """One camera's averaged capture: metric depth + valid mask (+ aligned RGB)."""
    depth_m: np.ndarray                 # float32 HxW, metres
    valid: np.ndarray                   # bool HxW
    intr: Intrinsics
    rgb: Optional[np.ndarray] = None    # uint8 HxWx3 BGR aligned to depth, or None


@dataclass
class StitchResult:
    depth_m: np.ndarray                 # float32 HxW — depth below cam1 plane (m)
    valid: np.ndarray                   # bool HxW
    rgb: np.ndarray                     # uint8 HxWx3 BGR (black where no colour)
    rgb_valid: np.ndarray               # bool HxW
    overlap: np.ndarray                 # bool HxW — both cameras contributed
    mm_per_px: float
    origin_xy: tuple[float, float]      # world (cam1-frame) x,y of pixel (0,0), metres
    # Per-camera height grids on the same grid (used by refine_shift).
    h1: np.ndarray = field(repr=False, default=None)
    v1: np.ndarray = field(repr=False, default=None)
    h2: np.ndarray = field(repr=False, default=None)
    v2: np.ndarray = field(repr=False, default=None)


# ── geometry ──────────────────────────────────────────────────────────────────

def deproject(depth_m: np.ndarray, valid: np.ndarray, intr: Intrinsics
              ) -> tuple[np.ndarray, np.ndarray]:
    """
    Depth image → Nx3 points in the camera frame (metres), plus the flat pixel
    indices of each point (for looking up the aligned RGB sample later).
    """
    h, w = depth_m.shape
    vs, us = np.nonzero(valid)
    z = depth_m[vs, us].astype(np.float64)
    x = (us.astype(np.float64) - intr.cx) / intr.fx * z
    y = (vs.astype(np.float64) - intr.cy) / intr.fy * z
    pts = np.column_stack([x, y, z])
    return pts, vs * w + us


def transform_points(pts: np.ndarray, calib: StitchCalib) -> np.ndarray:
    """Apply the cam2→cam1 rig transform: yaw about the view axis, then translate."""
    a = math.radians(calib.yaw_deg)
    c, s = math.cos(a), math.sin(a)
    out = np.empty_like(pts)
    out[:, 0] = c * pts[:, 0] - s * pts[:, 1] + calib.tx_mm / 1000.0
    out[:, 1] = s * pts[:, 0] + c * pts[:, 1] + calib.ty_mm / 1000.0
    out[:, 2] = pts[:, 2] + calib.tz_mm / 1000.0
    return out


def _auto_mm_per_px(frame: CameraFrame) -> float:
    """Grid resolution matching cam1's native pixel size at the median depth."""
    z = frame.depth_m[frame.valid]
    if z.size == 0:
        return 2.0
    med = float(np.median(z))
    return max(0.5, med * 1000.0 / frame.intr.fx)


def _rasterize(pts: np.ndarray, xmin: float, ymin: float, res: float,
               gw: int, gh: int, extra_weights: list[np.ndarray] | None = None
               ) -> tuple[np.ndarray, np.ndarray, list[np.ndarray]]:
    """
    Bin points onto the grid. Returns (count, height_sum, [extra_sum, ...])
    each as float32 gh×gw. `extra_weights` lets RGB channels ride along using
    the same bin indices.
    """
    iu = np.floor((pts[:, 0] - xmin) / res).astype(np.int64)
    iv = np.floor((pts[:, 1] - ymin) / res).astype(np.int64)
    m = (iu >= 0) & (iu < gw) & (iv >= 0) & (iv < gh)
    idx = iv[m] * gw + iu[m]
    n = gw * gh
    cnt = np.bincount(idx, minlength=n).astype(np.float32).reshape(gh, gw)
    hsum = np.bincount(idx, weights=pts[m, 2], minlength=n).astype(np.float32).reshape(gh, gw)
    extras = []
    for w in (extra_weights or []):
        extras.append(np.bincount(idx, weights=w[m], minlength=n)
                      .astype(np.float32).reshape(gh, gw))
    return cnt, hsum, extras


def fill_small_holes(height: np.ndarray, valid: np.ndarray, iters: int = 2
                     ) -> tuple[np.ndarray, np.ndarray]:
    """
    Fill 1-2 px rasterization holes with the mean of valid 3×3 neighbours.
    Large gaps (outside both footprints) stay invalid.
    """
    h = height.astype(np.float32).copy()
    v = valid.copy()
    for _ in range(iters):
        vf = cv2.blur(v.astype(np.float32), (3, 3))
        hf = cv2.blur(np.where(v, h, 0.0).astype(np.float32), (3, 3))
        fill = (vf > 0) & ~v
        if not fill.any():
            break
        h[fill] = hf[fill] / vf[fill]
        v = v | fill
    return h, v


def stitch(f1: CameraFrame, f2: CameraFrame, calib: StitchCalib,
           mm_per_px: float = STITCH_MM_PER_PX) -> StitchResult:
    """
    Merge two camera frames into one top-down heightmap in cam1's frame.
    """
    if calib.swap:
        f1, f2 = f2, f1

    p1, idx1 = deproject(f1.depth_m, f1.valid, f1.intr)
    p2, idx2 = deproject(f2.depth_m, f2.valid, f2.intr)
    p2 = transform_points(p2, calib)

    if mm_per_px <= 0:
        mm_per_px = _auto_mm_per_px(f1)
    res = mm_per_px / 1000.0

    allx = np.concatenate([p1[:, 0], p2[:, 0]]) if p2.size else p1[:, 0]
    ally = np.concatenate([p1[:, 1], p2[:, 1]]) if p2.size else p1[:, 1]
    if allx.size == 0:
        empty = np.zeros((2, 2), np.float32)
        return StitchResult(empty, empty.astype(bool),
                            np.zeros((2, 2, 3), np.uint8), empty.astype(bool),
                            empty.astype(bool), mm_per_px, (0.0, 0.0))
    # Percentile bounds resist stray outlier points; quantized so the grid does
    # not jitter frame to frame.
    q = res * 8
    xmin = math.floor(np.percentile(allx, 0.5) / q) * q
    ymin = math.floor(np.percentile(ally, 0.5) / q) * q
    xmax = math.ceil(np.percentile(allx, 99.5) / q) * q
    ymax = math.ceil(np.percentile(ally, 99.5) / q) * q
    gw = int(round((xmax - xmin) / res))
    gh = int(round((ymax - ymin) / res))
    # Cap the grid by coarsening resolution, never by cropping coverage.
    scale = max(gw / STITCH_MAX_GRID_W, gh / STITCH_MAX_GRID_H, 1.0)
    if scale > 1.0:
        res *= scale
        mm_per_px *= scale
        gw = max(1, int(round((xmax - xmin) / res)))
        gh = max(1, int(round((ymax - ymin) / res)))
    gw = max(gw, 16)
    gh = max(gh, 16)

    def rgb_weights(f: CameraFrame, idx: np.ndarray) -> tuple[list[np.ndarray], np.ndarray]:
        n = idx.shape[0]
        if f.rgb is None:
            z = np.zeros(n, np.float64)
            return [z, z, z], np.zeros(n, bool)
        flat = f.rgb.reshape(-1, 3).astype(np.float64)
        samples = flat[idx]
        has = samples.sum(axis=1) > 0    # aligned-RGB black = outside colour FOV
        return [samples[:, 0] * has, samples[:, 1] * has, samples[:, 2] * has], has

    w1, has1 = rgb_weights(f1, idx1)
    w2, has2 = rgb_weights(f2, idx2)

    c1, s1, e1 = _rasterize(p1, xmin, ymin, res, gw, gh, w1 + [has1.astype(np.float64)])
    c2, s2, e2 = _rasterize(p2, xmin, ymin, res, gw, gh, w2 + [has2.astype(np.float64)])

    cnt = c1 + c2
    valid = cnt > 0
    height = np.zeros((gh, gw), np.float32)
    height[valid] = (s1 + s2)[valid] / cnt[valid]
    overlap = (c1 > 0) & (c2 > 0)

    # Per-camera grids kept for refine_shift, BEFORE hole filling.
    h1 = np.zeros_like(height); v1 = c1 > 0
    h1[v1] = s1[v1] / c1[v1]
    h2 = np.zeros_like(height); v2 = c2 > 0
    h2[v2] = s2[v2] / c2[v2]

    height, valid = fill_small_holes(height, valid)

    # RGB: last extra channel is the per-bin colour-sample count.
    rgb = np.zeros((gh, gw, 3), np.uint8)
    if e1 and e2:
        rc = e1[3] + e2[3]
        rgb_valid = rc > 0
        for ch in range(3):
            plane = np.zeros((gh, gw), np.float32)
            plane[rgb_valid] = (e1[ch] + e2[ch])[rgb_valid] / rc[rgb_valid]
            rgb[:, :, ch] = np.clip(plane, 0, 255).astype(np.uint8)
        for ch in range(3):  # fill pinholes so the colour view reads cleanly
            filled, _ = fill_small_holes(rgb[:, :, ch].astype(np.float32), rgb_valid)
            rgb[:, :, ch] = np.clip(filled, 0, 255).astype(np.uint8)
        _, rgb_valid = fill_small_holes(np.zeros_like(height), rgb_valid)
        rgb[~rgb_valid] = 0
    else:
        rgb_valid = np.zeros((gh, gw), bool)

    return StitchResult(height, valid, rgb, rgb_valid, overlap, mm_per_px,
                        (xmin, ymin), h1=h1, v1=v1, h2=h2, v2=v2)


# ── overlap auto-refine ───────────────────────────────────────────────────────

def refine_shift(result: StitchResult, min_overlap_px: int = 400
                 ) -> Optional[tuple[float, float]]:
    """
    Estimate the residual XY misalignment between the two cameras from the
    overlap band of an existing stitch, by phase-correlating the two detrended
    height patches. Returns (dtx_mm, dty_mm) to ADD to the calibration's
    tx_mm/ty_mm, or None if the overlap is too small / correlation too weak.
    Needs some relief (grooves, objects) in the overlap — flat sand has nothing
    to correlate.
    """
    both = result.v1 & result.v2
    if int(both.sum()) < min_overlap_px:
        return None
    vs, us = np.nonzero(both)
    y0, y1 = vs.min(), vs.max() + 1
    x0, x1 = us.min(), us.max() + 1

    def patch(h: np.ndarray, v: np.ndarray) -> np.ndarray:
        p = h[y0:y1, x0:x1].astype(np.float32)
        m = v[y0:y1, x0:x1]
        mean = p[m].mean() if m.any() else 0.0
        p = np.where(m, p, mean)
        p -= cv2.blur(p, (31, 31))       # detrend: align on relief, not tilt
        std = p.std()
        return (p / std if std > 1e-9 else p).astype(np.float32)

    pa, pb = patch(result.h1, result.v1), patch(result.h2, result.v2)
    ph, pw = pa.shape
    if ph < 32 or pw < 32:
        return None
    # Exhaustive translation search: an interior crop of cam2's patch slid over
    # cam1's patch. Deterministic (no convergence basin needed — the grooves
    # can be narrower than the misalignment) and the margin sets the maximum
    # detectable shift. ±1 px ≈ ±1 grid-mm accuracy, plenty for a rig trim.
    my = min(40, ph // 4)
    mx = min(40, pw // 4)
    templ = pb[my:ph - my, mx:pw - mx]
    scores = cv2.matchTemplate(pa, templ, cv2.TM_CCOEFF_NORMED)
    _, best, _, loc = cv2.minMaxLoc(scores)
    if best < 0.2:                       # nothing distinctive in the overlap
        return None
    # Template found at `loc` in pa; cam2's content sits shifted by
    # (m - loc) grid px, and the correction is the negative of that shift.
    return (float(loc[0] - mx) * result.mm_per_px,
            float(loc[1] - my) * result.mm_per_px)


# ── synthetic scene (no-hardware fallback + tests) ────────────────────────────

def _world_relief_mm(x: np.ndarray, y: np.ndarray) -> np.ndarray:
    """
    Groove pattern carved into the synthetic sand, in mm below the flat surface.
    Spans a wide footprint so strokes cross the seam between the two cameras.
    x, y in metres (cam1 frame).
    """
    relief = np.zeros_like(x)
    # A long wavy groove running left-right across both footprints.
    cy = 0.05 * np.sin(x * 7.0)
    relief += 2.5 * np.exp(-((y - cy) ** 2) / (2 * 0.006 ** 2))
    # A diagonal straight groove.
    d = (y - 0.12) - 0.35 * (x - 0.2)
    relief += 2.0 * np.exp(-(d ** 2) / (2 * 0.005 ** 2))
    return relief


def synthetic_frame(intr: Intrinsics, cam_from_world: StitchCalib | None,
                    plane_z_m: float = 0.8, noise_mm: float = 0.35,
                    rng: np.random.Generator | None = None) -> CameraFrame:
    """
    Render what one camera sees of the synthetic sand plane. `cam_from_world`
    is the camera's pose in cam1's frame (None/identity for cam1 itself); the
    surface lives at z = plane_z_m below cam1.
    """
    rng = rng or np.random.default_rng(0)
    h, w = intr.height, intr.width
    us, vs = np.meshgrid(np.arange(w, dtype=np.float64),
                         np.arange(h, dtype=np.float64))
    calib = cam_from_world or StitchCalib(tx_mm=0, ty_mm=0, tz_mm=0, yaw_deg=0)
    a = math.radians(calib.yaw_deg)
    c, s = math.cos(a), math.sin(a)
    z0 = plane_z_m - calib.tz_mm / 1000.0          # flat-plane depth from this camera
    xr = (us - intr.cx) / intr.fx * z0             # rays at the plane, camera frame
    yr = (vs - intr.cy) / intr.fy * z0
    xw = c * xr - s * yr + calib.tx_mm / 1000.0    # → world (cam1) frame
    yw = s * xr + c * yr + calib.ty_mm / 1000.0
    depth = z0 + _world_relief_mm(xw, yw) / 1000.0
    depth += rng.normal(0.0, noise_mm / 1000.0, size=depth.shape)
    valid = rng.random(depth.shape) > 0.01         # ~1% dropout speckle
    depth = depth.astype(np.float32)
    depth[~valid] = 0.0

    # Fake aligned RGB: sand colour, black band at the frame edges imitating
    # the narrower colour FOV (so the stitched RGB shows the expected gap).
    rgb = np.full((h, w, 3), (96, 130, 168), np.uint8)   # BGR sand tone
    shade = np.clip(1.0 - 0.12 * _world_relief_mm(xw, yw), 0.0, 1.0)
    rgb = (rgb.astype(np.float32) * shade[..., None]).astype(np.uint8)
    edge = int(w * 0.10)
    rgb[:, :edge] = 0
    rgb[:, -edge:] = 0
    return CameraFrame(depth_m=depth, valid=valid, intr=intr, rgb=rgb)


def synthetic_pair(overlap_frac: float = 0.08, plane_z_m: float = 0.8,
                   yaw2_deg: float = 1.5, seed: int = 0
                   ) -> tuple[CameraFrame, CameraFrame, StitchCalib]:
    """
    Two synthetic cameras side by side with the requested footprint overlap.
    Returns (frame1, frame2, true_calib) — feed true_calib to stitch() for a
    perfect merge, or perturb it to exercise refine_shift().
    """
    intr = Intrinsics.nominal()
    rng = np.random.default_rng(seed)
    footprint_w = plane_z_m * intr.width / intr.fx
    baseline = footprint_w * (1.0 - overlap_frac)
    true_calib = StitchCalib(tx_mm=baseline * 1000.0, ty_mm=0.0, tz_mm=0.0,
                             yaw_deg=yaw2_deg)
    f1 = synthetic_frame(intr, None, plane_z_m, rng=rng)
    # Render cam2 with the INVERSE view: pixels of cam2 map to world through
    # its pose, which is exactly what synthetic_frame(cam_from_world) does.
    f2 = synthetic_frame(intr, true_calib, plane_z_m, rng=rng)
    return f1, f2, true_calib
