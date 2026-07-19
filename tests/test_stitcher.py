"""Unit tests for the dual-camera stitching math (stitcher.py). No hardware."""

import numpy as np
import pytest

from depth_extractor import DepthGrooveParams, grooves_and_mask
from stitcher import (
    CameraFrame, Intrinsics, StitchCalib,
    deproject, fill_small_holes, refine_shift, stitch, synthetic_pair,
    transform_points,
)


def _flat_frame(depth=0.8, w=160, h=120, rgb=False):
    intr = Intrinsics.nominal(w, h)
    d = np.full((h, w), depth, np.float32)
    v = np.ones((h, w), bool)
    color = np.full((h, w, 3), 120, np.uint8) if rgb else None
    return CameraFrame(depth_m=d, valid=v, intr=intr, rgb=color)


class TestGeometry:
    def test_deproject_center_pixel_on_axis(self):
        f = _flat_frame()
        pts, idx = deproject(f.depth_m, f.valid, f.intr)
        center = np.argmin(np.abs(pts[:, 0]) + np.abs(pts[:, 1]))
        assert pts[center, 2] == pytest.approx(0.8)
        assert abs(pts[center, 0]) < 0.01 and abs(pts[center, 1]) < 0.01
        assert idx.max() < f.depth_m.size

    def test_transform_translation_and_yaw(self):
        pts = np.array([[0.1, 0.0, 0.8]])
        out = transform_points(pts, StitchCalib(tx_mm=200, ty_mm=50, tz_mm=10, yaw_deg=90))
        assert out[0, 0] == pytest.approx(0.2, abs=1e-9)          # x rotated away, +tx
        assert out[0, 1] == pytest.approx(0.1 + 0.05, abs=1e-9)   # x → +y under 90° yaw
        assert out[0, 2] == pytest.approx(0.81, abs=1e-9)

    def test_calib_dict_roundtrip(self):
        c = StitchCalib(tx_mm=123.4, ty_mm=-5.0, tz_mm=2.0, yaw_deg=1.25, swap=True)
        assert StitchCalib.from_dict(c.to_dict()) == c


class TestFillHoles:
    def test_fills_isolated_hole_keeps_large_gap(self):
        h = np.full((20, 20), 5.0, np.float32)
        v = np.ones((20, 20), bool)
        v[10, 10] = False                    # isolated pinhole
        v[0:20, 0:3] = False                 # wide gap
        hf, vf = fill_small_holes(h, v, iters=2)
        assert vf[10, 10] and hf[10, 10] == pytest.approx(5.0)
        assert not vf[5, 0]                  # interior of the wide gap stays invalid


class TestStitch:
    def test_single_flat_plane_roundtrip(self):
        f = _flat_frame()
        r = stitch(f, CameraFrame(np.zeros((120, 160), np.float32),
                                  np.zeros((120, 160), bool), f.intr), StitchCalib())
        assert r.valid.any()
        assert r.depth_m[r.valid].mean() == pytest.approx(0.8, abs=0.002)
        assert 0.5 < r.mm_per_px < 20.0

    def test_pair_widens_footprint_and_overlaps(self):
        f1, f2, calib = synthetic_pair(overlap_frac=0.08)
        single = stitch(f1, CameraFrame(np.zeros_like(f1.depth_m),
                                        np.zeros_like(f1.valid), f1.intr), calib)
        both = stitch(f1, f2, calib)
        w_single = single.valid.any(axis=0).sum() * single.mm_per_px
        w_both = both.valid.any(axis=0).sum() * both.mm_per_px
        assert w_both > 1.7 * w_single       # ~2× coverage minus the 8% overlap
        ol = both.overlap.sum() / max(1, (both.v1 > 0).sum())
        assert 0.02 < ol < 0.25              # a real but small overlap band

    def test_overlap_averaging_reduces_noise(self):
        f1, f2, calib = synthetic_pair(overlap_frac=0.30, yaw2_deg=0.0)
        r = stitch(f1, f2, calib)
        core = r.overlap & r.v1 & r.v2
        assert core.sum() > 1000
        # Compare high-frequency noise inside the overlap band: averaged vs one cam.
        import cv2
        def hf_std(img):
            p = img.astype(np.float32)
            return float((p - cv2.blur(p, (9, 9)))[core].std())
        assert hf_std(r.depth_m) < hf_std(r.h1) * 0.95

    def test_rgb_merges_with_gap(self):
        f1, f2, calib = synthetic_pair()
        r = stitch(f1, f2, calib)
        assert r.rgb_valid.any()
        assert r.rgb[r.rgb_valid].mean() > 20      # colour actually landed
        assert not r.rgb_valid.all()               # the RGB-FOV gap exists
        assert (r.rgb[~r.rgb_valid] == 0).all()

    def test_swap_exchanges_cameras(self):
        f1, f2, calib = synthetic_pair()
        a = stitch(f1, f2, calib)
        calib.swap = True
        b = stitch(f2, f1, calib)                  # swapped inputs + swap flag = same scene
        assert abs(a.valid.sum() - b.valid.sum()) / a.valid.sum() < 0.02

    def test_groove_detection_on_stitched_output(self):
        f1, f2, calib = synthetic_pair(overlap_frac=0.08)
        r = stitch(f1, f2, calib)
        params = DepthGrooveParams(groove_depth_mm=1.2, min_blob_px=30)
        mask, skel = grooves_and_mask(r.depth_m, r.valid, params,
                                      None, r.mm_per_px)
        assert mask.any() and skel.any()
        # The wavy groove spans the full width — including the seam region.
        cols = np.nonzero(mask.any(axis=0))[0]
        assert (cols.max() - cols.min()) > 0.8 * mask.shape[1]


class TestRefine:
    def test_recovers_introduced_offset(self):
        f1, f2, true_calib = synthetic_pair(overlap_frac=0.20, yaw2_deg=0.0)
        wrong = StitchCalib(**{**true_calib.to_dict(), "swap": False})
        wrong.tx_mm += 12.0
        wrong.ty_mm -= 8.0
        r = stitch(f1, f2, wrong)
        d = refine_shift(r)
        assert d is not None
        dtx, dty = d
        assert dtx == pytest.approx(-12.0, abs=4.0)
        assert dty == pytest.approx(8.0, abs=4.0)

    def test_no_overlap_returns_none(self):
        f1, f2, calib = synthetic_pair(overlap_frac=0.08)
        apart = StitchCalib(**calib.to_dict())
        apart.tx_mm += 500.0                       # push footprints fully apart
        r = stitch(f1, f2, apart)
        assert refine_shift(r) is None
