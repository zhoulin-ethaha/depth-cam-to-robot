"""
Unit tests for depth_extractor.py — the depth → groove engine. Pure numpy/cv2,
no RealSense hardware required.

Also covers the no-hardware paths of DepthCameraThread (lifecycle + empty
capture); the live RealSense streaming itself is exercised in test_integration.py.
"""
import cv2
import numpy as np
import pytest

from depth_extractor import (
    Crop,
    DepthGrooveParams,
    ProcessedDepth,
    colorize_depth,
    groove_mask,
    grooves_and_mask,
    grooves_from_depth,
    process_depth,
)


def _rows_with_grooves(skel):
    """Return the set of approximate row-bands (centre y) that contain skeleton px."""
    return np.where(skel > 0)[0]


def _has_row(skel, y, tol=20):
    ys = _rows_with_grooves(skel)
    return bool(((ys > y - tol) & (ys < y + tol)).any())
from path_extractor import extract_from_edges
from camera_thread import DepthCameraThread


# ─────────────────────────────────────────────────────────────────────────────
# grooves_from_depth
# ─────────────────────────────────────────────────────────────────────────────

class TestGroovesFromDepth:

    def test_flat_surface_has_no_grooves(self, flat_depth):
        out = grooves_from_depth(flat_depth)
        assert out.shape == flat_depth.shape
        assert out.dtype == np.uint8
        assert int(out.sum()) == 0

    def test_carved_groove_is_detected(self, depth_with_groove):
        out = grooves_from_depth(depth_with_groove)
        assert out.max() == 255
        # The detected centreline should run along the carved row (y≈240).
        ys = np.where(out > 0)[0]
        assert 230 <= ys.mean() <= 250

    def test_groove_feeds_path_extractor(self, depth_with_groove):
        out = grooves_from_depth(depth_with_groove)
        extracted = extract_from_edges(out, min_contour_pixels=20)
        assert extracted.total_strokes >= 1
        assert extracted.total_points > 0

    def test_skeleton_is_thinner_than_raw_mask(self, depth_with_groove):
        thin = grooves_from_depth(depth_with_groove, skeleton=True)
        thick = grooves_from_depth(depth_with_groove, skeleton=False)
        assert (thin > 0).sum() <= (thick > 0).sum()
        assert (thick > 0).sum() > 0

    def test_grooves_and_mask_matches_individual_calls(self, depth_with_groove):
        mask, skel = grooves_and_mask(depth_with_groove)
        # mask == the thick mask; skel == its skeleton.
        assert (mask == groove_mask(depth_with_groove)).all()
        assert (skel == grooves_from_depth(depth_with_groove, skeleton=True)).all()
        assert (skel > 0).sum() <= (mask > 0).sum()

    def test_ridge_mode_ignores_a_valley(self, depth_with_groove):
        # The synthetic groove is a depression, so ridge detection finds nothing.
        params = DepthGrooveParams(detect="ridge")
        out = grooves_from_depth(depth_with_groove, params=params)
        assert int(out.sum()) == 0

    def test_higher_threshold_rejects_shallow_groove(self, depth_with_groove):
        # The groove is ~3 mm deep; a 10 mm threshold should reject it.
        params = DepthGrooveParams(groove_depth_mm=10.0)
        out = grooves_from_depth(depth_with_groove, params=params)
        assert int(out.sum()) == 0

    def test_all_invalid_does_not_crash(self):
        d = np.zeros((64, 64), dtype=np.float32)  # all zero = all invalid
        out = grooves_from_depth(d)
        assert int(out.sum()) == 0


# ─────────────────────────────────────────────────────────────────────────────
# colorize_depth
# ─────────────────────────────────────────────────────────────────────────────

class TestColorizeDepth:

    def test_returns_bgr_uint8(self, flat_depth):
        color = colorize_depth(flat_depth)
        assert color.shape == (480, 640, 3)
        assert color.dtype == np.uint8

    def test_invalid_pixels_are_black(self, flat_depth):
        valid = np.ones(flat_depth.shape, dtype=bool)
        valid[:100, :100] = False
        color = colorize_depth(flat_depth, valid)
        assert np.all(color[:100, :100] == 0)

    def test_explicit_range_runs(self, depth_with_groove):
        color = colorize_depth(depth_with_groove, near_m=0.25, far_m=0.35)
        assert color.shape == (480, 640, 3)


# ─────────────────────────────────────────────────────────────────────────────
# process_depth
# ─────────────────────────────────────────────────────────────────────────────

class TestProcessDepth:

    def test_full_frame(self, depth_with_groove):
        proc = process_depth(depth_with_groove, None, Crop(), DepthGrooveParams())
        assert isinstance(proc, ProcessedDepth)
        assert proc.color_full.shape == (480, 640, 3)
        assert proc.grooves.shape == (480, 640)
        assert proc.mask.shape == (480, 640)
        assert proc.origin == (0, 0)
        assert proc.grooves.max() == 255
        # Mask is the thick detected region; skeleton is its thinning.
        assert (proc.grooves > 0).sum() <= (proc.mask > 0).sum()

    def test_crop_shifts_origin_and_shrinks_grooves(self, depth_with_groove):
        crop = Crop(0.25, 0.25, 0.5, 0.5)
        proc = process_depth(depth_with_groove, None, crop, DepthGrooveParams())
        assert proc.origin == (160, 120)              # 0.25*640, 0.25*480
        assert proc.grooves.shape == (240, 320)       # 0.5*480, 0.5*640
        assert proc.mask.shape == (240, 320)
        # The colorized view is always the full frame so the crop box overlays it.
        assert proc.color_full.shape == (480, 640, 3)


# ─────────────────────────────────────────────────────────────────────────────
# Crop
# ─────────────────────────────────────────────────────────────────────────────

class TestCrop:

    def test_default_is_full_frame(self):
        assert Crop().pixel_box(640, 480) == (0, 0, 640, 480)

    def test_from_dict_clamps_out_of_range(self):
        c = Crop.from_dict({"x": -1, "y": 0.5, "w": 5, "h": 0.5})
        assert 0.0 <= c.x <= 1.0
        assert c.x + c.w <= 1.0 + 1e-9

    def test_degenerate_crop_becomes_full_frame(self):
        c = Crop.from_dict({"x": 0.5, "y": 0.5, "w": 0.0, "h": 0.0})
        assert (c.x, c.y, c.w, c.h) == (0.0, 0.0, 1.0, 1.0)


# ─────────────────────────────────────────────────────────────────────────────
# DepthGrooveParams.from_dict
# ─────────────────────────────────────────────────────────────────────────────

class TestDepthGrooveParams:

    def test_defaults_when_empty(self):
        p = DepthGrooveParams.from_dict({})
        assert p.detect == "valley"
        assert p.groove_depth_mm > 0

    def test_unknown_detect_falls_back_to_valley(self):
        p = DepthGrooveParams.from_dict({"detect": "nonsense"})
        assert p.detect == "valley"

    def test_values_are_clamped(self):
        p = DepthGrooveParams.from_dict({"groove_depth_mm": 9999, "min_blob_px": -5})
        assert p.groove_depth_mm <= 30.0
        assert p.min_blob_px >= 0

    def test_garbage_values_use_defaults(self):
        p = DepthGrooveParams.from_dict({"groove_depth_mm": "abc"})
        assert isinstance(p.groove_depth_mm, float)


# ─────────────────────────────────────────────────────────────────────────────
# Natural-groove rejection (reference subtraction + consistency/length filters)
# ─────────────────────────────────────────────────────────────────────────────

class TestNaturalGrooveRejection:

    def test_reference_subtraction_cancels_preexisting_groove(self):
        # reference = natural groove at y=100; current adds a drawn groove at y=300.
        ref = np.full((480, 640), 0.30, dtype=np.float32)
        cv2.line(ref, (100, 100), (500, 100), 0.303, 4)
        cur = ref.copy()
        cv2.line(cur, (100, 300), (500, 300), 0.303, 4)

        # Without reference: both grooves detected.
        no_ref = grooves_from_depth(cur)
        assert _has_row(no_ref, 100) and _has_row(no_ref, 300)

        # With full reference subtraction: the natural (y=100) groove cancels.
        p = DepthGrooveParams(ref_strength=1.0)
        with_ref = grooves_from_depth(cur, params=p, reference=ref)
        assert _has_row(with_ref, 300)            # drawn groove kept
        assert not _has_row(with_ref, 100)        # natural groove removed

    def test_min_length_drops_short_grooves(self):
        d = np.full((480, 640), 0.30, dtype=np.float32)
        cv2.line(d, (100, 240), (500, 240), 0.303, 4)   # long ~400 px
        cv2.line(d, (100, 100), (140, 100), 0.303, 4)   # short ~40 px
        mm_per_px = 0.5                                  # 40 px = 20 mm, 400 px = 200 mm

        base = grooves_from_depth(d)
        assert _has_row(base, 240) and _has_row(base, 100)

        p = DepthGrooveParams(min_length_mm=50.0)        # 50 mm = 100 px
        filt = grooves_from_depth(d, params=p, mm_per_px=mm_per_px)
        assert _has_row(filt, 240)                        # long kept
        assert not _has_row(filt, 100)                    # short removed
        assert (filt > 0).sum() < (base > 0).sum()

    def test_min_mean_depth_drops_shallow_grooves(self):
        d = np.full((480, 640), 0.30, dtype=np.float32)
        cv2.line(d, (100, 240), (500, 240), 0.308, 4)   # deep ~8 mm
        cv2.line(d, (100, 100), (500, 100), 0.302, 4)   # shallow ~2 mm
        p_all = DepthGrooveParams(groove_depth_mm=1.0)   # both pass per-pixel threshold
        base = grooves_from_depth(d, params=p_all)
        assert _has_row(base, 240) and _has_row(base, 100)

        p = DepthGrooveParams(groove_depth_mm=1.0, min_mean_depth_mm=4.0)
        filt = grooves_from_depth(d, params=p)
        assert _has_row(filt, 240)                        # deep kept
        assert not _has_row(filt, 100)                    # shallow removed

    def test_width_band_drops_thin_grooves(self):
        d = np.full((480, 640), 0.30, dtype=np.float32)
        cv2.line(d, (100, 240), (500, 240), 0.303, 10)  # wide ~10 px
        cv2.line(d, (100, 100), (500, 100), 0.303, 3)   # thin ~3 px
        p = DepthGrooveParams(min_width_mm=3.0)          # 3 mm = 6 px at 0.5 mm/px
        filt = grooves_from_depth(d, params=p, mm_per_px=0.5)
        assert _has_row(filt, 240)                        # wide kept
        assert not _has_row(filt, 100)                    # thin removed

    def test_filters_off_by_default(self, depth_with_groove):
        # No reference, no mm scale, all thresholds 0 → identical to the plain mask.
        m1, s1 = grooves_and_mask(depth_with_groove)
        m2 = groove_mask(depth_with_groove)
        assert (m1 == m2).all()

    def test_from_dict_parses_and_clamps_new_keys(self):
        p = DepthGrooveParams.from_dict({
            "ref_strength": 2.0, "min_length_mm": -5,
            "min_width_mm": 100, "min_mean_depth_mm": 3,
        })
        assert p.ref_strength <= 1.0
        assert p.min_length_mm >= 0
        assert p.min_width_mm <= 50
        assert p.min_mean_depth_mm == 3.0


# ─────────────────────────────────────────────────────────────────────────────
# DepthCameraThread (no-hardware paths)
# ─────────────────────────────────────────────────────────────────────────────

class TestDepthCameraThreadNoHardware:

    def test_not_running_before_start(self, shared_state_and_lock):
        state, lock = shared_state_and_lock
        ct = DepthCameraThread(state, lock)
        assert ct.running is False

    def test_stop_before_start_is_safe(self, shared_state_and_lock):
        state, lock = shared_state_and_lock
        ct = DepthCameraThread(state, lock)
        ct.stop()  # must not raise

    def test_capture_frame_none_before_any_frame(self, shared_state_and_lock):
        state, lock = shared_state_and_lock
        ct = DepthCameraThread(state, lock)
        assert ct.capture_frame() is None
