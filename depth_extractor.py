"""
depth_extractor.py — turn a DEPTH frame into groove centrelines.

Grooves raked into sand are a few-millimetre physical depression — invisible to
an RGB camera but directly measured by a depth camera (Intel RealSense D435i).
This module is the whole vision engine for the app: it colorizes depth for the
browser view, detects the hand-drawn grooves, and crops/thins them into the
1-pixel-wide centrelines the path extractor consumes:

    color = colorize_depth(depth_m, valid)               # the live/captured view
    proc  = process_depth(depth_m, valid, crop, params)  # colorize + crop + detect
    ext   = extract_from_edges(proc.grooves, 20, proc.origin)   # path_extractor.py
    strokes = pixels_to_robot_coords(ext.strokes, ws, W, H)

Why "valley detection" rather than literal "lines of equal depth":
    A perfectly level sandbox would let you threshold an absolute depth band, but
    real surfaces sag and tilt, so a fixed depth picks up the slope, not the marks.
    Instead we estimate the smooth bare-sand surface and subtract it, leaving only
    the *local* relief — then a groove is simply "a few mm deeper than its immediate
    surroundings" anywhere on the surface. (An absolute iso-depth band is still
    available via detect="band".)

Sensor-agnostic: pass a HxW float depth array in metres. A short RealSense
capture+averaging helper is at the bottom.
"""
from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np

from config import (
    GROOVE_SMOOTH_SIGMA_PX, GROOVE_DETREND_SIGMA_PX, GROOVE_DEPTH_MM,
    GROOVE_MIN_BLOB_PX, GROOVE_DETECT, DEPTH_COLOR_NEAR_M, DEPTH_COLOR_FAR_M,
    DEPTH_FPS, DEPTH_WIDTH, DEPTH_HEIGHT, DEPTH_AVERAGE_FRAMES,
)

try:
    from skimage.morphology import skeletonize as _sk_skeletonize
    _HAVE_SKIMAGE = True
except ImportError:
    _HAVE_SKIMAGE = False


# ── Crop ────────────────────────────────────────────────────────────────────
@dataclass
class Crop:
    """Crop rectangle in normalized [0, 1] coordinates of the full frame."""
    x: float = 0.0
    y: float = 0.0
    w: float = 1.0
    h: float = 1.0

    @classmethod
    def from_dict(cls, d: dict | None) -> "Crop":
        if not d:
            return cls()
        try:
            c = cls(
                float(d.get("x", 0.0)),
                float(d.get("y", 0.0)),
                float(d.get("w", 1.0)),
                float(d.get("h", 1.0)),
            )
        except (TypeError, ValueError):
            return cls()
        return c.clamped()

    def clamped(self) -> "Crop":
        x = min(max(self.x, 0.0), 1.0)
        y = min(max(self.y, 0.0), 1.0)
        w = min(max(self.w, 0.0), 1.0 - x)
        h = min(max(self.h, 0.0), 1.0 - y)
        if w <= 1e-4 or h <= 1e-4:
            return Crop()  # degenerate → treat as full frame
        return Crop(x, y, w, h)

    def pixel_box(self, width: int, height: int) -> tuple[int, int, int, int]:
        """Return (x0, y0, x1, y1) integer pixel bounds within a width×height image."""
        x0 = int(round(self.x * width))
        y0 = int(round(self.y * height))
        x1 = int(round((self.x + self.w) * width))
        y1 = int(round((self.y + self.h) * height))
        x0 = max(0, min(x0, width - 1))
        y0 = max(0, min(y0, height - 1))
        x1 = max(x0 + 1, min(x1, width))
        y1 = max(y0 + 1, min(y1, height))
        return x0, y0, x1, y1


# ── Groove detection parameters ───────────────────────────────────────────────
@dataclass
class DepthGrooveParams:
    smooth_sigma_px: float = GROOVE_SMOOTH_SIGMA_PX     # denoise the depth map first
    detrend_sigma_px: float = GROOVE_DETREND_SIGMA_PX   # blur radius for the bare surface
    groove_depth_mm: float = GROOVE_DEPTH_MM            # mm deeper than surface = a groove
    detect: str = GROOVE_DETECT                         # "valley" | "ridge" | "band"
    band_center_mm: float = 0.0      # for detect="band": target depth below surface
    band_width_mm: float = 1.0       # for detect="band": half-width of the accepted band
    min_blob_px: int = GROOVE_MIN_BLOB_PX              # discard specks smaller than this
    near_m: float = DEPTH_COLOR_NEAR_M  # colormap near plane, 0 = auto
    far_m: float = DEPTH_COLOR_FAR_M    # colormap far plane,  0 = auto

    @classmethod
    def from_dict(cls, d: dict | None) -> "DepthGrooveParams":
        d = d or {}

        def _f(key, default, lo, hi):
            try:
                return min(max(float(d.get(key, default)), lo), hi)
            except (TypeError, ValueError):
                return default

        def _i(key, default, lo, hi):
            try:
                return int(min(max(round(float(d.get(key, default))), lo), hi))
            except (TypeError, ValueError):
                return default

        detect = str(d.get("detect", GROOVE_DETECT))
        if detect not in ("valley", "ridge", "band"):
            detect = "valley"

        return cls(
            smooth_sigma_px=_f("smooth_sigma_px", GROOVE_SMOOTH_SIGMA_PX, 0.0, 10.0),
            detrend_sigma_px=_f("detrend_sigma_px", GROOVE_DETREND_SIGMA_PX, 1.0, 200.0),
            groove_depth_mm=_f("groove_depth_mm", GROOVE_DEPTH_MM, 0.1, 30.0),
            detect=detect,
            band_center_mm=_f("band_center_mm", 0.0, -50.0, 50.0),
            band_width_mm=_f("band_width_mm", 1.0, 0.1, 30.0),
            min_blob_px=_i("min_blob_px", GROOVE_MIN_BLOB_PX, 0, 5000),
            near_m=_f("near_m", DEPTH_COLOR_NEAR_M, 0.0, 5.0),
            far_m=_f("far_m", DEPTH_COLOR_FAR_M, 0.0, 5.0),
        )


def grooves_from_depth(
    depth_m: np.ndarray,
    valid: np.ndarray | None = None,
    params: DepthGrooveParams = DepthGrooveParams(),
    skeleton: bool = True,
) -> np.ndarray:
    """
    depth_m  : HxW float, metres. 0 or NaN = no reading.
    valid    : optional HxW bool mask of trustworthy pixels (else inferred).
    skeleton : True → thin to 1-px centrelines (path source); False → return the
               cleaned binary blob (cheaper, used for the live preview).
    returns  : HxW uint8 binary (white on black), ready for
               path_extractor.extract_from_edges().
    """
    d = np.asarray(depth_m, dtype=np.float32).copy()
    if valid is None:
        valid = np.isfinite(d) & (d > 0)
    d[~valid] = np.nan

    # Fill gaps so blurring doesn't bleed invalid pixels into the surface estimate.
    d_filled = _fill_invalid(d)

    if params.smooth_sigma_px > 0:
        d_filled = cv2.GaussianBlur(d_filled, (0, 0), params.smooth_sigma_px)

    # Bare-sand surface = low-frequency component. Subtract → local relief in mm.
    # Positive = farther from the (top-down) camera = a depression = a groove.
    surface = cv2.GaussianBlur(d_filled, (0, 0), params.detrend_sigma_px)
    relief_mm = (d_filled - surface) * 1000.0

    if params.detect == "ridge":
        mask = relief_mm < -params.groove_depth_mm
    elif params.detect == "band":
        lo = params.band_center_mm - params.band_width_mm
        hi = params.band_center_mm + params.band_width_mm
        mask = (relief_mm >= lo) & (relief_mm <= hi)
    else:  # "valley" (default)
        mask = relief_mm > params.groove_depth_mm
    mask = mask & valid

    mask_u8 = (mask.astype(np.uint8)) * 255
    mask_u8 = cv2.morphologyEx(mask_u8, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
    mask_u8 = _remove_small(mask_u8, params.min_blob_px)

    return _skeletonize(mask_u8) if skeleton else mask_u8


# ── Depth → colour for display ────────────────────────────────────────────────
def colorize_depth(
    depth_m: np.ndarray,
    valid: np.ndarray | None = None,
    near_m: float = 0.0,
    far_m: float = 0.0,
) -> np.ndarray:
    """
    Map a metric depth array to a BGR image so depth reads as colour (TURBO ramp:
    near = blue, far = red). Invalid pixels are black. ``near_m``/``far_m`` set the
    range in metres; 0 for either means auto (2nd–98th percentile of valid depth).
    """
    d = np.asarray(depth_m, dtype=np.float32)
    if valid is None:
        valid = np.isfinite(d) & (d > 0)

    if near_m <= 0.0 or far_m <= 0.0:
        vals = d[valid]
        if vals.size:
            near = float(np.percentile(vals, 2.0))
            far = float(np.percentile(vals, 98.0))
        else:
            near, far = 0.0, 1.0
    else:
        near, far = near_m, far_m
    if far <= near:
        far = near + 1e-3

    norm = np.clip((d - near) / (far - near), 0.0, 1.0)
    color = cv2.applyColorMap((norm * 255.0).astype(np.uint8), cv2.COLORMAP_TURBO)
    color[~valid] = (0, 0, 0)
    return color


@dataclass
class ProcessedDepth:
    color_full: np.ndarray   # BGR uint8, FULL frame colorized depth (the view)
    grooves: np.ndarray      # uint8 binary, cropped — groove centrelines (path source)
    origin: tuple[int, int]  # (x0, y0) pixel offset of the crop in the full frame


def process_depth(
    depth_m: np.ndarray,
    valid: np.ndarray | None,
    crop: Crop,
    params: DepthGrooveParams,
) -> ProcessedDepth:
    """
    Colorize the FULL depth frame (so the crop box overlays the same image the
    user sees), then run groove detection on the cropped depth. Returns the full
    colorized view, the cropped groove centrelines, and the crop's pixel origin
    so extracted strokes can be shifted back into full-frame coordinates.
    """
    d = np.asarray(depth_m, dtype=np.float32)
    if valid is None:
        valid = np.isfinite(d) & (d > 0)
    h, w = d.shape[:2]

    color_full = colorize_depth(d, valid, params.near_m, params.far_m)

    x0, y0, x1, y1 = crop.pixel_box(w, h)
    sub_d = d[y0:y1, x0:x1]
    sub_v = valid[y0:y1, x0:x1]
    grooves = grooves_from_depth(sub_d, sub_v, params, skeleton=True)

    return ProcessedDepth(color_full=color_full, grooves=grooves, origin=(x0, y0))


def encode_jpeg(img_gray_or_bgr: np.ndarray, quality: int = 80) -> bytes | None:
    """Encode a uint8 image (grayscale or BGR) to JPEG bytes."""
    ok, buf = cv2.imencode(".jpg", img_gray_or_bgr, [cv2.IMWRITE_JPEG_QUALITY, quality])
    return buf.tobytes() if ok else None


# ── helpers ───────────────────────────────────────────────────────────────────
def _fill_invalid(d: np.ndarray) -> np.ndarray:
    """Replace NaNs with the nearest valid depth (cheap inpaint via distance transform)."""
    nan = ~np.isfinite(d)
    if not nan.any():
        return d
    if nan.all():
        return np.zeros_like(d)
    filled = d.copy()
    # Nearest-valid-pixel fill keeps the surface estimate stable near holes.
    _, labels = cv2.distanceTransformWithLabels(
        nan.astype(np.uint8), cv2.DIST_L2, 3, labelType=cv2.DIST_LABEL_PIXEL
    )
    valid_vals = d[~nan]
    # Map each label back to a source pixel value.
    src = np.zeros(valid_vals.size + 1, dtype=np.float32)
    src[1:] = valid_vals
    filled[nan] = src[labels[nan]]
    return filled


def _remove_small(mask_u8: np.ndarray, min_px: int) -> np.ndarray:
    n, lbl, stats, _ = cv2.connectedComponentsWithStats((mask_u8 > 0).astype(np.uint8), 8)
    out = np.zeros_like(mask_u8)
    for i in range(1, n):
        if stats[i, cv2.CC_STAT_AREA] >= min_px:
            out[lbl == i] = 255
    return out


def _skeletonize(mask_u8: np.ndarray) -> np.ndarray:
    """Thin a thick binary mask to 1-px centrelines. Uses skimage if present."""
    binary = mask_u8 > 0
    if not binary.any():
        return np.zeros_like(mask_u8)
    if _HAVE_SKIMAGE:
        return (_sk_skeletonize(binary).astype(np.uint8)) * 255
    if hasattr(cv2, "ximgproc"):
        return cv2.ximgproc.thinning(mask_u8)
    return _zhang_suen_thinning(binary)


def _zhang_suen_thinning(binary: np.ndarray) -> np.ndarray:
    """
    Pure-numpy Zhang-Suen thinning fallback (used when neither scikit-image nor
    opencv-contrib is installed). Fine for a single static capture; for speed,
    `pip install scikit-image` and this is bypassed automatically.
    """
    img = binary.astype(np.uint8).copy()
    changed = True
    while changed:
        changed = False
        for step in (0, 1):
            p = np.pad(img, 1)
            P2 = p[:-2, 1:-1]; P3 = p[:-2, 2:]; P4 = p[1:-1, 2:]; P5 = p[2:, 2:]
            P6 = p[2:, 1:-1];  P7 = p[2:, :-2]; P8 = p[1:-1, :-2]; P9 = p[:-2, :-2]
            neighbours = [P2, P3, P4, P5, P6, P7, P8, P9]
            B = sum(neighbours)
            seq = neighbours + [P2]
            A = sum(((seq[i] == 0) & (seq[i + 1] == 1)).astype(np.uint8) for i in range(8))
            if step == 0:
                c = (P2 * P4 * P6 == 0) & (P4 * P6 * P8 == 0)
            else:
                c = (P2 * P4 * P8 == 0) & (P2 * P6 * P8 == 0)
            cond = (img == 1) & (B >= 2) & (B <= 6) & (A == 1) & c
            if cond.any():
                img[cond] = 0
                changed = True
    return img * 255


# ── Optional: RealSense capture with temporal averaging ───────────────────────
def average_realsense_depth(n_frames: int = DEPTH_AVERAGE_FRAMES):
    """
    Capture and average N depth frames from an Intel RealSense (D435i). The sand
    is static, so averaging many frames is the single biggest win for sub-mm
    grooves: it cuts per-pixel depth noise by ~sqrt(n_frames).

    Returns (depth_m HxW float32, valid HxW bool). Requires `pip install pyrealsense2`.
    """
    import pyrealsense2 as rs  # noqa: local import so the module loads without it

    pipe = rs.pipeline()
    cfg = rs.config()
    cfg.enable_stream(rs.stream.depth, DEPTH_WIDTH, DEPTH_HEIGHT, rs.format.z16, DEPTH_FPS)
    profile = pipe.start(cfg)
    scale = profile.get_device().first_depth_sensor().get_depth_scale()  # → metres/unit
    try:
        acc = None
        cnt = None
        for _ in range(n_frames):
            frame = pipe.wait_for_frames().get_depth_frame()
            z = np.asarray(frame.get_data(), dtype=np.float32) * scale  # metres
            ok = z > 0
            if acc is None:
                acc = np.zeros_like(z)
                cnt = np.zeros_like(z)
            acc[ok] += z[ok]
            cnt += ok
    finally:
        pipe.stop()

    valid = cnt > 0
    depth_m = np.zeros_like(acc)
    depth_m[valid] = acc[valid] / cnt[valid]
    return depth_m, valid
