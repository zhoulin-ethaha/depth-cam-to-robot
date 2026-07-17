"""
Unit tests for surface.py — 2D → 3D surface projection. Pure numpy/trimesh,
no hardware required. Synthetic meshes are built in-memory.
"""
import numpy as np
import pytest
import trimesh
from scipy.spatial.transform import Rotation

from surface import SurfaceModel, SurfacePose

W, H = 640, 480   # drawing frame (4:3 — matches the 0.4×0.3 m test meshes)


def _flat_plane():
    """0.4×0.3 m rectangle at z=0 (two triangles), normals up."""
    v = np.array([[0, 0, 0], [0.4, 0, 0], [0.4, 0.3, 0], [0, 0.3, 0]], dtype=float)
    f = np.array([[0, 1, 2], [0, 2, 3]])
    return SurfaceModel(trimesh.Trimesh(vertices=v, faces=f, process=False), "flat")


def _tilted_plane():
    """Plane z = x (45° about Y): same 0.4×0.3 footprint."""
    v = np.array([[0, 0, 0], [0.4, 0, 0.4], [0.4, 0.3, 0.4], [0, 0.3, 0]], dtype=float)
    f = np.array([[0, 1, 2], [0, 2, 3]])
    return SurfaceModel(trimesh.Trimesh(vertices=v, faces=f, process=False), "tilted")


def _half_plane():
    """Single triangle covering only half its bbox — the rest is a hole."""
    v = np.array([[0, 0, 0], [0.4, 0, 0], [0, 0.3, 0]], dtype=float)
    f = np.array([[0, 1, 2]])
    return SurfaceModel(trimesh.Trimesh(vertices=v, faces=f, process=False), "half")


def _vertical_plane():
    """
    Plane modelled VERTICAL in the file (XZ plane, 0.4 wide × 0.3 tall,
    normals facing −Y) — as exported straight from Rhino without laying it flat.
    """
    v = np.array([[0, 0, 0], [0.4, 0, 0], [0.4, 0, 0.3], [0, 0, 0.3]], dtype=float)
    f = np.array([[0, 1, 2], [0, 2, 3]])   # wound so face normals point −Y
    return SurfaceModel(trimesh.Trimesh(vertices=v, faces=f, process=False), "vertical")


IDENTITY = SurfacePose(tx=0, ty=0, tz=0)


# ─────────────────────────────────────────────────────────────────────────────
# Projection basics (flat plane)
# ─────────────────────────────────────────────────────────────────────────────

class TestFlatProjection:

    def test_points_land_on_surface(self):
        strokes = [[(160, 120), (320, 240), (480, 360)]]
        out = _flat_plane().project_strokes(strokes, W, H, IDENTITY)
        assert len(out) == 1
        for pose in out[0]:
            assert len(pose) == 6
            assert abs(pose[2]) < 1e-9            # z on the plane

    def test_center_pixel_maps_to_bbox_center(self):
        out = _flat_plane().project_strokes([[(320, 240), (321, 240)]], W, H, IDENTITY)
        p = out[0][0]
        assert abs(p[0] - 0.2) < 1e-6
        assert abs(p[1] - 0.15) < 1e-6

    def test_v_axis_is_flipped(self):
        # Top-of-image pixel (v=0) → far (+Y) edge of the surface.
        out = _flat_plane().project_strokes([[(320, 0), (320, 2)]], W, H, IDENTITY)
        assert out[0][0][1] > 0.29

    def test_tool_is_perpendicular(self):
        out = _flat_plane().project_strokes([[(320, 240), (400, 240)]], W, H, IDENTITY)
        for pose in out[0]:
            tool_z = Rotation.from_rotvec(pose[3:]).apply([0, 0, 1])
            assert np.allclose(tool_z, [0, 0, -1], atol=1e-6)   # approach along −normal

    def test_offset_lifts_along_normal(self):
        out = _flat_plane().project_strokes([[(320, 240), (400, 240)]], W, H,
                                            IDENTITY, offset_m=0.005)
        for pose in out[0]:
            assert abs(pose[2] - 0.005) < 1e-9

    def test_empty_strokes(self):
        assert _flat_plane().project_strokes([], W, H, IDENTITY) == []


# ─────────────────────────────────────────────────────────────────────────────
# Placement (SurfacePose)
# ─────────────────────────────────────────────────────────────────────────────

class TestPose:

    def test_translation_moves_the_path(self):
        pose = SurfacePose(tx=0.5, ty=-0.2, tz=0.1)
        out = _flat_plane().project_strokes([[(320, 240), (321, 240)]], W, H, pose)
        p = out[0][0]
        assert abs(p[0] - (0.2 + 0.5)) < 1e-6
        assert abs(p[1] - (0.15 - 0.2)) < 1e-6
        assert abs(p[2] - 0.1) < 1e-6

    def test_rz_rotation(self):
        pose = SurfacePose(tx=0, ty=0, tz=0, rz=90.0)
        out = _flat_plane().project_strokes([[(320, 240), (321, 240)]], W, H, pose)
        p = out[0][0]
        # (0.2, 0.15) rotated 90° about Z → (−0.15, 0.2)
        assert abs(p[0] + 0.15) < 1e-6
        assert abs(p[1] - 0.2) < 1e-6

    def test_pose_from_dict_clamps(self):
        p = SurfacePose.from_dict({"tx": 99, "rz": -999})
        assert p.tx <= 3.0
        assert -180.0 <= p.rz <= 180.0


# ─────────────────────────────────────────────────────────────────────────────
# Non-planar behaviour
# ─────────────────────────────────────────────────────────────────────────────

class TestTiltedPlane:

    def test_points_lie_on_tilted_plane(self):
        strokes = [[(160, 240), (320, 240), (480, 240)]]
        out = _tilted_plane().project_strokes(strokes, W, H, IDENTITY)
        for pose in out[0]:
            assert abs(pose[2] - pose[0]) < 1e-6    # plane z = x

    def test_tool_follows_tilted_normal(self):
        out = _tilted_plane().project_strokes([[(320, 240), (400, 240)]], W, H, IDENTITY)
        n = np.array([-1, 0, 1]) / np.sqrt(2)       # outward normal of z=x
        for pose in out[0]:
            tool_z = Rotation.from_rotvec(pose[3:]).apply([0, 0, 1])
            assert np.allclose(tool_z, -n, atol=1e-6)

    def test_offset_is_along_normal_not_z(self):
        off = 0.01
        base = _tilted_plane().project_strokes([[(320, 240), (400, 240)]], W, H, IDENTITY)
        high = _tilted_plane().project_strokes([[(320, 240), (400, 240)]], W, H,
                                               IDENTITY, offset_m=off)
        d = np.array(high[0][0][:3]) - np.array(base[0][0][:3])
        n = np.array([-1, 0, 1]) / np.sqrt(2)
        assert np.allclose(d, off * n, atol=1e-6)


class TestVerticalInFile:
    """A mesh exported vertical (not laid flat) must still receive the drawing —
    projection follows the mesh's dominant normal, not a fixed local −Z."""

    def test_projection_hits_the_vertical_plane(self):
        strokes = [[(160, 240), (320, 240), (480, 240)]]
        out = _vertical_plane().project_strokes(strokes, W, H, IDENTITY)
        assert len(out) == 1 and len(out[0]) == 3
        for pose in out[0]:
            assert abs(pose[1]) < 1e-9              # points lie on the y=0 plane

    def test_image_up_maps_to_plane_up(self):
        # Top-of-image pixel → top of the vertical plane (max local Z).
        out = _vertical_plane().project_strokes([[(320, 0), (320, 2)]], W, H, IDENTITY)
        assert out[0][0][2] > 0.29

    def test_tool_perpendicular_to_vertical_plane(self):
        out = _vertical_plane().project_strokes([[(320, 240), (400, 240)]], W, H, IDENTITY)
        for pose in out[0]:
            tool_z = Rotation.from_rotvec(pose[3:]).apply([0, 0, 1])
            assert np.allclose(tool_z, [0, 1, 0], atol=1e-6)   # approach along +Y

    def test_offset_pulls_off_the_plane(self):
        out = _vertical_plane().project_strokes([[(320, 240), (400, 240)]], W, H,
                                                IDENTITY, offset_m=0.02)
        for pose in out[0]:
            assert abs(pose[1] + 0.02) < 1e-9       # 20 mm toward −Y (outward)

    def test_mm_per_px_uses_true_footprint(self):
        # 0.4 m fitted across 640 px → 0.625 mm/px.
        assert abs(_vertical_plane().drawing_mm_per_px(W, H) - 0.625) < 1e-6


class TestMisses:

    def test_rays_off_the_mesh_split_the_stroke(self):
        # A horizontal line across the bbox middle crosses the empty half of the
        # triangle → the points over the hole are dropped.
        strokes = [[(x, 240) for x in range(40, 640, 40)]]
        out = _half_plane().project_strokes(strokes, W, H, IDENTITY)
        total_in = sum(len(s) for s in strokes)
        total_out = sum(len(s) for s in out)
        assert total_out < total_in
        for stroke in out:
            assert len(stroke) >= 2


# ─────────────────────────────────────────────────────────────────────────────
# Loading + payload
# ─────────────────────────────────────────────────────────────────────────────

class TestLoadAndPayload:

    def test_load_scales_mm_to_m(self, tmp_path):
        v = np.array([[0, 0, 0], [400, 0, 0], [400, 300, 0], [0, 300, 0]], dtype=float)
        f = np.array([[0, 1, 2], [0, 2, 3]])
        p = tmp_path / "plane_mm.stl"
        trimesh.Trimesh(vertices=v, faces=f, process=False).export(p)
        model = SurfaceModel.load(p)                 # default mm → m
        size = model.info()["bbox"]["size"]
        assert abs(size[0] - 0.4) < 1e-6
        assert abs(size[1] - 0.3) < 1e-6

    def test_mesh_payload_shape(self):
        payload = _flat_plane().mesh_payload()
        assert len(payload["vertices"]) % 3 == 0
        assert len(payload["faces"]) % 3 == 0
        assert max(payload["faces"]) < len(payload["vertices"]) // 3

    def test_mesh_payload_corners_match_model(self):
        # Registration markers in the browser must use the same numbering as
        # the server-side solve — both come from corner_points().
        model = _flat_plane()
        payload = model.mesh_payload()
        assert payload["corners"] == model.corner_points()
        assert len(payload["corners"]) == 4          # flat sheet → 4 corners
