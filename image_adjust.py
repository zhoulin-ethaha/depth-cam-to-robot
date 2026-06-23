"""
Still-image processing for the capture → crop → adjust → path pipeline.

The goal is to pull a *very subtle* signal (e.g. shallow markings raked into a
sandbox) out of a captured frame and turn it into clean Canny edges. The user
tunes crop + tonal controls in the browser; the exact same code runs for the
live preview and for the final path extraction, so what you see is what gets
drawn.

All tonal work is done on a float32 grayscale image in [0, 1] — grayscale is
what Canny consumes anyway, and it keeps the controls intuitive.
"""
from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np


# ── Crop ──────────────────────────────────────────────────────────────────────
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


# ── Tonal adjustments ─────────────────────────────────────────────────────────
@dataclass
class Adjustments:
    """
    Tonal + edge controls. Defaults are neutral / match the old live pipeline.

    brightness  additive,        [-1, 1]   0 = none
    exposure    stops (2**ev),   [-2, 2]    0 = none
    contrast    multiplicative,  [0, 3]     1 = none
    highlights  push bright tones[-1, 1]    0 = none
    shadows     lift dark tones, [-1, 1]    0 = none
    gamma       tone curve,      [0.2, 3]   1 = none
    clahe       local contrast equalization (great for subtle texture)
    invert      flip black/white (dark marks on light sand vs. opposite)
    blur        Gaussian kernel before Canny (odd; >=1)
    canny_low / canny_high   Canny hysteresis thresholds
    """
    brightness: float = 0.0
    exposure: float = 0.0
    contrast: float = 1.0
    highlights: float = 0.0
    shadows: float = 0.0
    gamma: float = 1.0
    clahe: bool = False
    clahe_clip: float = 2.0
    invert: bool = False
    blur: int = 5
    canny_low: int = 50
    canny_high: int = 150
    # Auto touch-up: reveal shallow relief (grooves) automatically.
    auto: bool = False
    auto_strength: float = 0.6   # 0 = mild, 1 = aggressive
    auto_canny: bool = False     # pick Canny thresholds from image statistics

    @classmethod
    def from_dict(cls, d: dict | None) -> "Adjustments":
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

        return cls(
            brightness=_f("brightness", 0.0, -1.0, 1.0),
            exposure=_f("exposure", 0.0, -2.0, 2.0),
            contrast=_f("contrast", 1.0, 0.0, 3.0),
            highlights=_f("highlights", 0.0, -1.0, 1.0),
            shadows=_f("shadows", 0.0, -1.0, 1.0),
            gamma=_f("gamma", 1.0, 0.2, 3.0),
            clahe=bool(d.get("clahe", False)),
            clahe_clip=_f("clahe_clip", 2.0, 0.1, 10.0),
            invert=bool(d.get("invert", False)),
            blur=_i("blur", 5, 1, 31),
            canny_low=_i("canny_low", 50, 0, 500),
            canny_high=_i("canny_high", 150, 0, 500),
            auto=bool(d.get("auto", False)),
            auto_strength=_f("auto_strength", 0.6, 0.0, 1.0),
            auto_canny=bool(d.get("auto_canny", False)),
        )


def _apply_tone(g: np.ndarray, adj: Adjustments) -> np.ndarray:
    """Apply the tonal stack to a float32 grayscale image in [0, 1]."""
    x = g

    # Exposure first (multiplicative, in stops) — mimics camera exposure comp.
    if adj.exposure:
        x = x * (2.0 ** adj.exposure)

    # Brightness (linear lift).
    if adj.brightness:
        x = x + adj.brightness

    # Shadows / highlights: tone-weighted local lifts. Shadow weight is largest
    # in dark regions, highlight weight largest in bright regions, so each knob
    # mostly leaves the other end of the range alone.
    if adj.shadows:
        x = np.clip(x, 0.0, 1.0)
        w_shadow = (1.0 - x) ** 2
        x = x + adj.shadows * 0.5 * w_shadow
    if adj.highlights:
        x = np.clip(x, 0.0, 1.0)
        w_high = x ** 2
        x = x + adj.highlights * 0.5 * w_high

    # Contrast around mid-gray.
    if adj.contrast != 1.0:
        x = (x - 0.5) * adj.contrast + 0.5

    x = np.clip(x, 0.0, 1.0)

    # Gamma curve.
    if adj.gamma and adj.gamma != 1.0:
        x = np.power(x, 1.0 / adj.gamma)

    if adj.invert:
        x = 1.0 - x

    return np.clip(x, 0.0, 1.0)


def _auto_enhance(gray: np.ndarray, strength: float) -> np.ndarray:
    """
    Reveal shallow relief (grooves raked into sand) from a uint8 grayscale image.

    Grooves are a low-amplitude, directional shading signal — micro shadows and
    highlights along each furrow — rather than a flat tonal difference. The chain:

    1. Robust contrast stretch (1st–99th percentile) — spreads the narrow band of
       values a flat sandy surface occupies across the full 0–255 range, ignoring
       glare/dark specks that would wreck a naive min/max stretch.
    2. CLAHE — local histogram equalization so faint texture pops everywhere, not
       just where the surface happens to be well lit.
    3. Unsharp/high-pass relief boost — subtracts a blurred copy to amplify the
       fine ridges while leaving the broad lighting gradient alone.

    ``strength`` (0–1) scales the CLAHE clip limit and the relief gain.
    """
    s = max(0.0, min(1.0, strength))

    # 1. Robust percentile contrast stretch.
    lo, hi = np.percentile(gray, 1.0), np.percentile(gray, 99.0)
    if hi - lo < 1.0:
        stretched = gray.copy()
    else:
        stretched = np.clip((gray.astype(np.float32) - lo) * (255.0 / (hi - lo)), 0, 255).astype(np.uint8)

    # 2. Edge-preserving denoise — smooth sand grain so the later boost amplifies
    #    groove ridges, not speckle. Bilateral keeps the furrow edges crisp.
    denoised = cv2.bilateralFilter(stretched, d=7, sigmaColor=40, sigmaSpace=7)

    # 3. Local contrast equalization.
    clahe = cv2.createCLAHE(clipLimit=2.0 + 4.0 * s, tileGridSize=(8, 8))
    eq = clahe.apply(denoised)

    # 4. Unsharp mask — emphasize groove ridges.
    blur = cv2.GaussianBlur(eq, (0, 0), sigmaX=3.0)
    gain = 0.6 + 1.4 * s
    sharp = cv2.addWeighted(eq, 1.0 + gain, blur, -gain, 0)

    return sharp


def _auto_canny_thresholds(img: np.ndarray, sigma: float = 0.33) -> tuple[int, int]:
    """Pick Canny hysteresis thresholds from the image median (Otsu-like heuristic)."""
    v = float(np.median(img))
    lower = int(max(0, (1.0 - sigma) * v))
    upper = int(min(255, (1.0 + sigma) * v))
    if upper <= lower:
        upper = min(255, lower + 1)
    return lower, upper


@dataclass
class ProcessedStill:
    full_adjusted: np.ndarray       # uint8 grayscale, FULL frame — the tuned image (edit preview)
    edges: np.ndarray               # uint8 binary, cropped — Canny result (path source)
    origin: tuple[int, int]         # (x0, y0) pixel offset of the crop in the full frame


def process_still(frame_bgr: np.ndarray, crop: Crop, adj: Adjustments) -> ProcessedStill:
    """
    Grayscale → tonal stack → (optional CLAHE) on the FULL frame, then crop →
    blur → Canny.

    The tonal stack is applied to the whole frame so the edit preview visibly
    reflects every adjustment (and the crop box overlays the tuned image). Only
    edge detection is restricted to the crop. Returns the full adjusted image,
    the cropped Canny edge map, and the crop's pixel origin so extracted strokes
    can be shifted back into full-frame coordinates.
    """
    h, w = frame_bgr.shape[:2]

    gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)

    # Auto touch-up runs first (normalize + local equalize + relief boost) so the
    # manual tonal sliders, if touched, fine-tune the already-enhanced image.
    if adj.auto:
        gray = _auto_enhance(gray, adj.auto_strength)

    g = gray.astype(np.float32) / 255.0
    g = _apply_tone(g, adj)
    full_adjusted = (g * 255.0).astype(np.uint8)

    # CLAHE works on uint8 and is excellent at revealing faint, low-contrast
    # texture like shallow rake marks; apply after the global tonal stack.
    if adj.clahe:
        clahe = cv2.createCLAHE(clipLimit=max(0.1, adj.clahe_clip), tileGridSize=(8, 8))
        full_adjusted = clahe.apply(full_adjusted)

    x0, y0, x1, y1 = crop.pixel_box(w, h)
    sub = full_adjusted[y0:y1, x0:x1]

    k = adj.blur if adj.blur % 2 == 1 else adj.blur + 1
    blurred = cv2.GaussianBlur(sub, (k, k), 0) if k > 1 else sub

    if adj.auto_canny:
        lo, hi = _auto_canny_thresholds(blurred)
    else:
        lo, hi = adj.canny_low, adj.canny_high
        if hi < lo:
            lo, hi = hi, lo
    edges = cv2.Canny(blurred, lo, hi)

    return ProcessedStill(full_adjusted=full_adjusted, edges=edges, origin=(x0, y0))


def encode_jpeg(img_gray_or_bgr: np.ndarray, quality: int = 80) -> bytes | None:
    """Encode a uint8 image (grayscale or BGR) to JPEG bytes."""
    ok, buf = cv2.imencode(".jpg", img_gray_or_bgr, [cv2.IMWRITE_JPEG_QUALITY, quality])
    return buf.tobytes() if ok else None
