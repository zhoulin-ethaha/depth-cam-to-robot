"""
surface.py — project 2D sand drawings onto a non-planar 3D surface.

The surface is authored in Rhino, meshed, and exported as STL/OBJ (millimetres).
This module loads it with trimesh and turns the extracted pixel strokes into
6-DOF robot poses that lie ON the surface, with the tool axis perpendicular to
it (Rhino's "project curve onto surface" + surface-normal tool orientation):

    model = SurfaceModel.load("surfaces/dune.stl")            # mm → m
    strokes = model.project_strokes(px_strokes, W, H, pose, offset_m=0.002)

Frames and conventions
    • The mesh lives in its own LOCAL frame. ``SurfacePose`` (translation in m +
      XYZ Euler in degrees) places it in the ROBOT BASE frame — editable live
      from the browser so the surface can be moved relative to the robot.
    • The 2D drawing (full camera frame, aspect W:H) is fitted centred onto the
      mesh's local XY bounding box (aspect preserved) and projected along
      local −Z ("shone down" onto the surface). Draw-side = local +Z.
    • Tool orientation: tool Z approaches along the inward normal (−n), with
      the tool X chained point-to-point for minimal twist so the wrist doesn't
      flip mid-stroke. Orientations are UR rotation vectors.
    • ``offset_m`` shifts each waypoint along the OUTWARD normal: 0 = contact,
      positive = hover above the surface, negative = plunge into it.

Points whose projection ray misses the mesh are dropped and strokes are split
at the gaps, so a drawing larger than the surface simply falls off its edges.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, asdict
from pathlib import Path

import numpy as np

try:
    import trimesh as _trimesh
    _HAVE_TRIMESH = True
except ImportError:
    _HAVE_TRIMESH = False

from scipy.spatial.transform import Rotation

from config import SURFACE_UNITS_TO_M, SURFACE_MAX_FACES


# ── Surface placement (edited live from the browser) ─────────────────────────
@dataclass
class SurfacePose:
    """Rigid placement of the surface mesh in the robot base frame."""
    tx: float = 0.4    # m
    ty: float = 0.0
    tz: float = 0.0
    rx: float = 0.0    # deg, XYZ Euler
    ry: float = 0.0
    rz: float = 0.0

    @classmethod
    def from_dict(cls, d: dict | None) -> "SurfacePose":
        d = d or {}

        def _f(key, default, lo, hi):
            try:
                return min(max(float(d.get(key, default)), lo), hi)
            except (TypeError, ValueError):
                return default

        return cls(
            tx=_f("tx", 0.4, -3.0, 3.0),
            ty=_f("ty", 0.0, -3.0, 3.0),
            tz=_f("tz", 0.0, -3.0, 3.0),
            rx=_f("rx", 0.0, -180.0, 180.0),
            ry=_f("ry", 0.0, -180.0, 180.0),
            rz=_f("rz", 0.0, -180.0, 180.0),
        )

    def to_dict(self) -> dict:
        return asdict(self)

    def matrix(self) -> np.ndarray:
        """4×4 local→base transform."""
        m = np.eye(4)
        m[:3, :3] = Rotation.from_euler(
            "xyz", [self.rx, self.ry, self.rz], degrees=True
        ).as_matrix()
        m[:3, 3] = [self.tx, self.ty, self.tz]
        return m


# ── Surface model ─────────────────────────────────────────────────────────────
class SurfaceModel:
    """A triangulated target surface loaded from STL/OBJ (local frame, metres)."""

    def __init__(self, mesh, name: str) -> None:
        self.mesh = mesh
        self.name = name

    # ── loading ────────────────────────────────────────────────────────────
    @classmethod
    def load(cls, path: str | Path, units_to_m: float = SURFACE_UNITS_TO_M) -> "SurfaceModel":
        """Load an STL/OBJ mesh. ``units_to_m`` scales file units (Rhino mm → m)."""
        if not _HAVE_TRIMESH:
            raise RuntimeError(
                "trimesh is not installed — run `pip install trimesh` to use surfaces."
            )
        path = Path(path)
        mesh = _trimesh.load(str(path), force="mesh")
        if mesh.is_empty or len(mesh.faces) == 0:
            raise ValueError(f"{path.name}: no triangles found in mesh.")
        if units_to_m != 1.0:
            mesh.apply_scale(units_to_m)
        if len(mesh.faces) > SURFACE_MAX_FACES:
            print(f"[surface] WARNING: {len(mesh.faces)} faces — browser preview may "
                  f"be slow; consider a coarser mesh (< {SURFACE_MAX_FACES}).")
        return cls(mesh, path.name)

    # ── info / browser payload ─────────────────────────────────────────────
    def info(self) -> dict:
        lo, hi = self.mesh.bounds
        return {
            "name": self.name,
            "faces": int(len(self.mesh.faces)),
            "bbox": {
                "size": [round(float(v), 4) for v in (hi - lo)],
                "min": [round(float(v), 4) for v in lo],
                "max": [round(float(v), 4) for v in hi],
            },
        }

    def drawing_mm_per_px(self, frame_width: int, frame_height: int) -> float:
        """
        Millimetres per drawing pixel once the frame is fitted onto this surface —
        the surface-mode replacement for the workspace-derived scale (feeds the
        mm-based groove filters).
        """
        lo, hi = self.mesh.bounds
        bw, bh = float(hi[0] - lo[0]), float(hi[1] - lo[1])
        aspect = frame_width / frame_height
        rw = min(bw, bh * aspect)
        return (rw / frame_width) * 1000.0

    def mesh_payload(self) -> dict:
        """Vertices/faces in the LOCAL frame; the browser applies the pose matrix."""
        return {
            "vertices": np.round(self.mesh.vertices, 5).ravel().tolist(),
            "faces": self.mesh.faces.ravel().tolist(),
        }

    # ── projection ─────────────────────────────────────────────────────────
    def project_strokes(
        self,
        strokes_px: list[list[tuple[float, float]]],
        frame_width: int,
        frame_height: int,
        pose: SurfacePose,
        offset_m: float = 0.0,
    ) -> list[list[list[float]]]:
        """
        Pixel strokes → 6-DOF robot poses on the surface.

        Each stroke point is mapped onto the mesh's local XY bounding box
        (centred, aspect preserved), ray-cast along local −Z onto the mesh,
        offset along the outward normal, transformed to the robot base frame by
        ``pose``, and given a tool orientation perpendicular to the surface.
        """
        if not strokes_px:
            return []

        lo, hi = self.mesh.bounds
        bw, bh = float(hi[0] - lo[0]), float(hi[1] - lo[1])
        if bw < 1e-9 or bh < 1e-9:
            return []

        # Fit the drawing rect (aspect W:H) centred into the local XY bbox.
        aspect = frame_width / frame_height
        rw = min(bw, bh * aspect)
        rh = rw / aspect
        ox = float(lo[0]) + (bw - rw) / 2.0
        oy = float(lo[1]) + (bh - rh) / 2.0

        # Flatten all points to batch the ray-cast (one query for everything).
        counts = [len(s) for s in strokes_px]
        pts = np.array([p for s in strokes_px for p in s], dtype=np.float64)
        lx = ox + (pts[:, 0] / frame_width) * rw
        ly = oy + (1.0 - pts[:, 1] / frame_height) * rh   # image v grows down

        z_start = float(hi[2]) + 0.05
        origins = np.column_stack([lx, ly, np.full(len(pts), z_start)])
        dirs = np.tile([0.0, 0.0, -1.0], (len(pts), 1))

        hit_pt, hit_n = self._raycast_down(origins, dirs)   # NaN rows = miss

        # Offset along the outward (local +Z-ish) normal, then local → base.
        m = pose.matrix()
        rot = m[:3, :3]
        with np.errstate(invalid="ignore"):
            pts_local = hit_pt + offset_m * hit_n
        pts_base = pts_local @ rot.T + m[:3, 3]
        nrm_base = hit_n @ rot.T

        # Rebuild strokes, splitting at misses, and attach orientations.
        out: list[list[list[float]]] = []
        i = 0
        for c in counts:
            seg_pts: list[np.ndarray] = []
            seg_nrm: list[np.ndarray] = []
            for k in range(i, i + c):
                if np.isfinite(pts_base[k]).all():
                    seg_pts.append(pts_base[k])
                    seg_nrm.append(nrm_base[k])
                else:                       # ray missed the surface → split here
                    _flush(out, seg_pts, seg_nrm)
                    seg_pts, seg_nrm = [], []
            _flush(out, seg_pts, seg_nrm)
            i += c
        return out

    def _raycast_down(self, origins: np.ndarray, dirs: np.ndarray):
        """Nearest hit per ray. Returns (points, outward normals); NaN = miss."""
        locs, ray_idx, tri_idx = self.mesh.ray.intersects_location(
            origins, dirs, multiple_hits=True
        )
        n = len(origins)
        pts = np.full((n, 3), np.nan)
        nrm = np.full((n, 3), np.nan)
        if len(locs) == 0:
            return pts, nrm

        # Keep the FIRST hit along the ray (max z, since rays point down).
        best_z = np.full(n, -np.inf)
        for loc, ri, ti in zip(locs, ray_idx, tri_idx):
            if loc[2] > best_z[ri]:
                best_z[ri] = loc[2]
                pts[ri] = loc
                fn = self.mesh.face_normals[ti]
                nrm[ri] = fn if fn[2] >= 0 else -fn   # outward = toward local +Z
        return pts, nrm


# ── helpers ───────────────────────────────────────────────────────────────────
def _flush(out: list, seg_pts: list, seg_nrm: list) -> None:
    """Close a contiguous run of hits into a stroke with chained orientations."""
    if len(seg_pts) < 2:
        return
    rotvecs = _normals_to_rotvecs(np.array(seg_nrm))
    out.append([[float(p[0]), float(p[1]), float(p[2]),
                 float(r[0]), float(r[1]), float(r[2])]
                for p, r in zip(seg_pts, rotvecs)])


def _normals_to_rotvecs(normals: np.ndarray) -> np.ndarray:
    """
    Per-point tool orientation: tool Z along −normal (approach perpendicular to
    the surface), tool X chained from the previous point (minimal twist so the
    wrist doesn't flip between neighbouring waypoints). → UR rotation vectors.
    """
    out = np.zeros_like(normals)
    x_ref = np.array([1.0, 0.0, 0.0])
    for i, n in enumerate(normals):
        z = -n / (np.linalg.norm(n) + 1e-12)
        x = x_ref - np.dot(x_ref, z) * z
        if np.linalg.norm(x) < 1e-6:                 # normal ∥ x_ref → fall back
            x = np.array([0.0, 1.0, 0.0]) - np.dot([0.0, 1.0, 0.0], z) * z
        x /= np.linalg.norm(x)
        y = np.cross(z, x)
        out[i] = Rotation.from_matrix(np.column_stack([x, y, z])).as_rotvec()
        x_ref = x                                     # chain for minimal twist
    return out
