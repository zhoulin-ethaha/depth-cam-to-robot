"""
Unit tests for path_export.py — URScript + JSON + bundle saving. No hardware.
"""
import base64
import json
import math

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from path_export import build_urscript, build_json, save_bundle, _offset_pose, stroke_blend

_PI = math.pi

# Two short strokes, tool-down orientation ([0, π, 0] → tool Z points −Z).
STROKES = [
    [[0.4, 0.0, 0.2, 0.0, _PI, 0.0], [0.45, 0.0, 0.2, 0.0, _PI, 0.0]],
    [[0.4, 0.1, 0.2, 0.0, _PI, 0.0], [0.4, 0.15, 0.2, 0.0, _PI, 0.0], [0.4, 0.2, 0.2, 0.0, _PI, 0.0]],
]


# ─────────────────────────────────────────────────────────────────────────────
# URScript
# ─────────────────────────────────────────────────────────────────────────────

class TestUrscript:

    def test_structure(self):
        s = build_urscript(STROKES, speed=0.3, safety=0.05)
        assert s.startswith("# ") or s.startswith("def ")  # header or def
        assert "def draw_path():" in s
        assert s.strip().endswith("draw_path()")
        assert "end" in s

    def test_travels_use_movel_draws_use_movep(self):
        s = build_urscript(STROKES, speed=0.3, safety=0.05)
        assert s.count("movel(") == len(STROKES) * 3   # approach + start + lift each
        # movep for every drawing point after the start of each stroke
        assert s.count("movep(") == sum(len(st) - 1 for st in STROKES)

    def test_poses_formatted(self):
        s = build_urscript(STROKES, speed=0.25, safety=0.05)
        assert "p[0.40000, 0.00000, 0.20000, 0.00000, 3.14159, 0.00000]" in s
        assert "v=0.2500" in s

    def test_safety_retract_is_above_for_tool_down(self):
        # tool-down retract adds +Z; first movel target should be z = 0.2 + 0.05
        s = build_urscript([STROKES[0]], speed=0.3, safety=0.05)
        first = s.split("movel(")[1]
        assert "0.25000" in first   # 0.20 + 0.05 safety on Z


# ─────────────────────────────────────────────────────────────────────────────
# Blend radius (the exec-bar Radius slider)
# ─────────────────────────────────────────────────────────────────────────────

class TestBlendRadius:

    def test_default_blend_in_script(self):
        s = build_urscript(STROKES, speed=0.3, safety=0.05)
        assert "r=0.0005" in s                      # MOVEP_BLEND_M default

    def test_custom_blend_in_script(self):
        s = build_urscript(STROKES, speed=0.3, safety=0.05, blend_m=0.003)
        assert "r=0.0030" in s
        assert "r=0.0005" not in s

    def test_stroke_blend_passthrough_when_small(self):
        # 50 mm segments: a 3 mm request is far below the 45% cap.
        assert stroke_blend(STROKES[1], 0.003) == pytest.approx(0.003)

    def test_stroke_blend_clamped_to_shortest_segment(self):
        # 10 mm then 4 mm tail segment: 5 mm request must clamp to 0.45 × 4 mm.
        stroke = [[0.4, 0.0, 0.2, 0.0, _PI, 0.0],
                  [0.41, 0.0, 0.2, 0.0, _PI, 0.0],
                  [0.414, 0.0, 0.2, 0.0, _PI, 0.0]]
        assert stroke_blend(stroke, 0.005) == pytest.approx(0.45 * 0.004)

    def test_stroke_blend_zero_and_degenerate(self):
        assert stroke_blend(STROKES[0], 0.0) == 0.0
        assert stroke_blend(STROKES[0], -1.0) == 0.0
        assert stroke_blend([STROKES[0][0]], 0.005) == 0.005  # 1 pt: nothing to clamp

    def test_clamped_blend_lands_in_script(self):
        stroke = [[0.4, 0.0, 0.2, 0.0, _PI, 0.0],
                  [0.41, 0.0, 0.2, 0.0, _PI, 0.0],
                  [0.414, 0.0, 0.2, 0.0, _PI, 0.0]]
        s = build_urscript([stroke], speed=0.3, safety=0.05, blend_m=0.005)
        assert "r=0.0018" in s                      # 0.45 × 4 mm = 1.8 mm


# ─────────────────────────────────────────────────────────────────────────────
# JSON
# ─────────────────────────────────────────────────────────────────────────────

class TestJson:

    def test_shape_and_meta(self):
        j = build_json(STROKES, {"mode": "surface"})
        assert j["meta"]["mode"] == "surface"
        assert len(j["strokes"]) == 2
        assert len(j["strokes"][1]) == 3
        assert "pose" in j["strokes"][0][0]
        assert "plane" in j["strokes"][0][0]

    def test_plane_is_orthonormal_frame(self):
        j = build_json(STROKES, {})
        pl = j["strokes"][0][0]["plane"]
        x, y, z = np.array(pl["xaxis"]), np.array(pl["yaxis"]), np.array(pl["zaxis"])
        assert abs(np.linalg.norm(x) - 1) < 1e-4
        assert abs(np.dot(x, y)) < 1e-4               # orthogonal
        assert np.allclose(np.cross(x, y), z, atol=1e-4)   # right-handed
        # tool-down: approach axis (z) points down
        assert np.allclose(z, [0, 0, -1], atol=1e-4)

    def test_plane_origin_matches_pose(self):
        j = build_json(STROKES, {})
        wp = j["strokes"][0][0]
        assert wp["plane"]["origin"] == wp["pose"][:3]


# ─────────────────────────────────────────────────────────────────────────────
# Offset + bundle
# ─────────────────────────────────────────────────────────────────────────────

class TestOffset:

    def test_offset_lifts_along_normal(self):
        p = [0.4, 0.0, 0.2, 0.0, _PI, 0.0]
        out = _offset_pose(p, 0.01)     # tool-down → +Z
        assert abs(out[2] - 0.21) < 1e-9
        assert out[:2] == p[:2] and out[3:] == p[3:]


class TestSaveBundle:

    def test_creates_three_files(self, tmp_path):
        # 1×1 transparent PNG
        png_b64 = ("data:image/png;base64,"
                   "iVBORw0KGgoAAAANSUhEUgAAAAEAAAABCAQAAAC1HAwCAAAAC0lEQVR42mNk+M9QDwADhgGAWjR9awAAAABJRU5ErkJggg==")
        folder = save_bundle(STROKES, speed=0.3, safety_m=0.05, offset_m=0.0,
                             meta={"mode": "surface"}, preview_png_data_url=png_b64,
                             base_dir=tmp_path)
        assert (folder / "path.script").exists()
        assert (folder / "path.json").exists()
        assert (folder / "preview.png").exists()
        assert (folder / "preview.png").stat().st_size > 0

    def test_json_is_valid_and_offset_applied(self, tmp_path):
        folder = save_bundle(STROKES, speed=0.3, safety_m=0.05, offset_m=0.005,
                             meta={"mode": "planar"}, base_dir=tmp_path)
        data = json.loads((folder / "path.json").read_text())
        # first waypoint z lifted by 5 mm (tool-down → +Z)
        assert abs(data["strokes"][0][0]["pose"][2] - 0.205) < 1e-6

    def test_no_image_skips_png(self, tmp_path):
        folder = save_bundle(STROKES, speed=0.3, safety_m=0.05, offset_m=0.0,
                             meta={}, preview_png_data_url=None, base_dir=tmp_path)
        assert not (folder / "preview.png").exists()

    def test_collision_makes_distinct_folder(self, tmp_path):
        a = save_bundle(STROKES, 0.3, 0.05, 0.0, {}, base_dir=tmp_path)
        b = save_bundle(STROKES, 0.3, 0.05, 0.0, {}, base_dir=tmp_path)
        assert a != b        # same-second saves don't overwrite
