"""
Capture thread for the Dual-Cam Vision prototype (stitch_main.py).

Owns TWO RealSense D435i pipelines (selected by serial number), keeps a short
rolling buffer per camera for temporal averaging, and every STITCH_EVERY_S
rebuilds the stitched heightmap + RGB + groove mask/skeleton JPEGs into
shared_state. If fewer than two cameras are available (or pyrealsense2 is
missing, or the main app holds the devices), it falls back to a SYNTHETIC
scene so the UI and calibration workflow stay testable.

Deliberately separate from camera_thread.DepthCameraThread — this prototype
must not touch the single-camera pipeline. One process per RealSense still
applies: close the main app before running this tool.
"""

from __future__ import annotations

import threading
import time
from collections import deque
from typing import Optional

import cv2
import numpy as np

from config import (
    DEPTH_WIDTH, DEPTH_HEIGHT, DEPTH_FPS,
    STITCH_AVERAGE_FRAMES, STITCH_EVERY_S, STITCH_MM_PER_PX,
)
from depth_extractor import DepthGrooveParams, colorize_depth, encode_jpeg, grooves_and_mask
from stitcher import (
    CameraFrame, Intrinsics, StitchCalib, apply_orientation, auto_align,
    refine_shift, stitch, synthetic_pair,
)


class DualCameraThread:
    def __init__(self, shared_state: dict, state_lock: threading.Lock) -> None:
        self._state = shared_state
        self._state_lock = state_lock
        self._thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()
        self._params = DepthGrooveParams()
        self._calib = StitchCalib()
        self._refine_requested = threading.Event()
        self._align_requested = threading.Event()
        self._mm_per_px = STITCH_MM_PER_PX
        self._stitch_on = True

    @property
    def running(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    def start(self) -> None:
        if self.running:
            return
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._run, daemon=True,
                                        name="dual_camera_thread")
        self._thread.start()

    def stop(self) -> None:
        self._stop_event.set()
        if self._thread:
            self._thread.join(timeout=3.0)
            self._thread = None

    # ── setters (atomic swaps, same pattern as DepthCameraThread) ────────────
    def set_params(self, params: DepthGrooveParams) -> None:
        self._params = params

    def set_calib(self, calib: StitchCalib) -> None:
        self._calib = calib
        self._publish_calib()

    def get_calib(self) -> StitchCalib:
        return self._calib

    def request_refine(self) -> None:
        """Run overlap auto-refine on the next stitch cycle."""
        self._refine_requested.set()

    def request_align(self) -> None:
        """Run the full automatic overlap search on the next cycle."""
        self._align_requested.set()

    def set_stitch(self, on: bool) -> None:
        """Stitch ON = combined views; OFF = per-camera setup views only."""
        self._stitch_on = bool(on)
        with self._state_lock:
            self._state["stitch_on"] = self._stitch_on

    def get_stitch(self) -> bool:
        return self._stitch_on

    def _publish_calib(self, refine_msg: Optional[dict] = None) -> None:
        with self._state_lock:
            self._state["stitch_calib"] = self._calib.to_dict()
            if refine_msg is not None:
                self._state["stitch_refine_result"] = refine_msg

    # ── main loop ────────────────────────────────────────────────────────────
    def _run(self) -> None:
        pipes = self._try_start_cameras()
        if pipes is None:
            self._run_synthetic()
        else:
            try:
                self._run_real(pipes)
            finally:
                for pipe, _, _, _ in pipes:
                    try:
                        pipe.stop()
                    except Exception:
                        pass
        with self._state_lock:
            for k in ("stitch_depth_jpg", "stitch_rgb_jpg",
                      "stitch_mask_jpg", "stitch_skel_jpg",
                      "stitch_left_depth_jpg", "stitch_left_rgb_jpg",
                      "stitch_right_depth_jpg", "stitch_right_rgb_jpg"):
                self._state[k] = None
        print("[stitch] stopped")

    def _try_start_cameras(self):
        """Start both RealSense pipelines; None → caller uses synthetic mode."""
        try:
            import pyrealsense2 as rs
        except ImportError:
            self._set_note("pyrealsense2 not installed — SYNTHETIC scene")
            return None
        try:
            ctx = rs.context()
            serials = sorted(d.get_info(rs.camera_info.serial_number)
                             for d in ctx.query_devices())
        except Exception as exc:
            self._set_note(f"RealSense enumeration failed ({exc}) — SYNTHETIC scene")
            return None
        if len(serials) < 2:
            self._set_note(f"{len(serials)} camera(s) found, need 2 "
                           "(is the main app running?) — SYNTHETIC scene")
            return None

        pipes = []
        for serial in serials[:2]:
            pipe = rs.pipeline()
            cfg = rs.config()
            cfg.enable_device(serial)
            cfg.enable_stream(rs.stream.depth, DEPTH_WIDTH, DEPTH_HEIGHT,
                              rs.format.z16, DEPTH_FPS)
            cfg.enable_stream(rs.stream.color, DEPTH_WIDTH, DEPTH_HEIGHT,
                              rs.format.bgr8, DEPTH_FPS)
            try:
                profile = pipe.start(cfg)
            except Exception as exc:
                for p, _, _, _ in pipes:
                    p.stop()
                self._set_note(f"cannot start camera {serial} ({exc}) — SYNTHETIC scene")
                return None
            scale = profile.get_device().first_depth_sensor().get_depth_scale()
            ri = (profile.get_stream(rs.stream.depth)
                  .as_video_stream_profile().get_intrinsics())
            intr = Intrinsics(fx=ri.fx, fy=ri.fy, cx=ri.ppx, cy=ri.ppy,
                              width=ri.width, height=ri.height)
            align = rs.align(rs.stream.depth)
            pipes.append((pipe, align, scale, intr))
            print(f"[stitch] camera {serial}: fx={ri.fx:.1f} fy={ri.fy:.1f}")
        self._serials = serials[:2]
        return pipes

    def _run_real(self, pipes) -> None:
        buffers = [deque(maxlen=STITCH_AVERAGE_FRAMES) for _ in pipes]
        last_rgb: list[Optional[np.ndarray]] = [None, None]
        last_stitch = 0.0
        while not self._stop_event.is_set():
            got = False
            for i, (pipe, align, scale, _intr) in enumerate(pipes):
                try:
                    frames = pipe.poll_for_frames()
                except Exception:
                    continue
                if not frames:
                    continue
                frames = align.process(frames)
                df = frames.get_depth_frame()
                if not df:
                    continue
                z = np.asarray(df.get_data(), dtype=np.float32) * scale
                buffers[i].append((z, z > 0))
                cf = frames.get_color_frame()
                if cf:
                    last_rgb[i] = np.asarray(cf.get_data())
                got = True

            now = time.monotonic()
            if (now - last_stitch) >= STITCH_EVERY_S and all(buffers):
                last_stitch = now
                frames2 = []
                for i, (_p, _a, _s, intr) in enumerate(pipes):
                    depth, valid = _average(buffers[i])
                    frames2.append(CameraFrame(depth, valid, intr, last_rgb[i]))
                self._process(frames2[0], frames2[1],
                              synthetic=False, serials=self._serials)
            if not got:
                time.sleep(0.005)

    def _run_synthetic(self) -> None:
        i = 0
        while not self._stop_event.is_set():
            f1, f2, _true = synthetic_pair(overlap_frac=0.08, seed=i)
            i += 1
            self._process(f1, f2, synthetic=True, serials=["SYN-1", "SYN-2"])
            # Sleep in small steps so stop() stays responsive.
            deadline = time.monotonic() + STITCH_EVERY_S
            while not self._stop_event.is_set() and time.monotonic() < deadline:
                time.sleep(0.05)

    # ── one stitch + detection cycle ─────────────────────────────────────────
    def _per_camera_views(self, f1: CameraFrame, f2: CameraFrame,
                          calib: StitchCalib, params: DepthGrooveParams) -> dict:
        """Left/right oriented previews for the setup screen (stitch OFF)."""
        left, right = apply_orientation(f1, f2, calib)
        out = {}
        for name, f in (("left", left), ("right", right)):
            out[f"stitch_{name}_depth_jpg"] = encode_jpeg(
                colorize_depth(f.depth_m, f.valid, params.near_m, params.far_m))
            rgb = f.rgb
            if rgb is None:
                rgb = np.zeros((*f.depth_m.shape, 3), np.uint8)
                cv2.putText(rgb, "no RGB frame yet", (12, 32),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (120, 120, 120), 1,
                            cv2.LINE_AA)
            out[f"stitch_{name}_rgb_jpg"] = encode_jpeg(rgb)
        return out

    def _process(self, f1: CameraFrame, f2: CameraFrame,
                 synthetic: bool, serials: list[str]) -> None:
        params = self._params
        calib = self._calib

        refine_msg = None
        if self._align_requested.is_set():
            self._align_requested.clear()
            found = auto_align(f1, f2, calib, self._mm_per_px)
            if found is None:
                refine_msg = {"success": False,
                              "message": "Auto-align found no overlap — rake a "
                                         "groove across the seam, then press "
                                         "Find overlap."}
            else:
                self._calib = calib = found
                refine_msg = {"success": True,
                              "message": f"Overlap found: tx {found.tx_mm:.0f} mm, "
                                         f"ty {found.ty_mm:.0f} mm",
                              "tx_mm": found.tx_mm, "ty_mm": found.ty_mm}

        # Published in BOTH modes so the setup screen is live the moment the
        # stitch is toggled off.
        cam_jpgs = self._per_camera_views(f1, f2, calib, params)

        if not self._stitch_on:
            with self._state_lock:
                self._state.update(cam_jpgs)
                for k in ("stitch_depth_jpg", "stitch_rgb_jpg",
                          "stitch_mask_jpg", "stitch_skel_jpg"):
                    self._state[k] = None
                self._state["stitch_info"] = {"synthetic": synthetic,
                                              "serials": serials}
                self._state["stitch_calib"] = calib.to_dict()
                self._state["stitch_on"] = False
                if refine_msg is not None:
                    self._state["stitch_refine_result"] = refine_msg
            return

        result = stitch(f1, f2, calib, self._mm_per_px)

        if self._refine_requested.is_set():
            self._refine_requested.clear()
            delta = refine_shift(result)
            if delta is None:
                refine_msg = {"success": False,
                              "message": "Refine failed: overlap too small or "
                                         "featureless — rake a groove across the seam."}
            else:
                dtx, dty = delta
                calib = StitchCalib(**calib.to_dict())
                calib.tx_mm += dtx
                calib.ty_mm += dty
                self._calib = calib
                refine_msg = {"success": True,
                              "message": f"Refined: Δtx {dtx:+.1f} mm, Δty {dty:+.1f} mm",
                              "dtx_mm": dtx, "dty_mm": dty}
                result = stitch(f1, f2, calib, self._mm_per_px)

        mask, skel = grooves_and_mask(result.depth_m, result.valid, params,
                                      None, result.mm_per_px)

        # Depth view: colorized heightmap with the overlap band outlined cyan.
        depth_view = colorize_depth(result.depth_m, result.valid,
                                    params.near_m, params.far_m)
        contours, _ = cv2.findContours(result.overlap.astype(np.uint8),
                                       cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(depth_view, contours, -1, (255, 220, 80), 1)

        gh, gw = result.depth_m.shape
        n_valid = int(result.valid.sum())
        info = {
            "synthetic": synthetic,
            "serials": serials,
            "mm_per_px": round(result.mm_per_px, 3),
            "size": [gw, gh],
            "overlap_px": int(result.overlap.sum()),
            "overlap_pct": round(float(100.0 * result.overlap.sum() / max(1, n_valid)), 1),
        }
        with self._state_lock:
            self._state.update(cam_jpgs)
            self._state["stitch_depth_jpg"] = encode_jpeg(depth_view)
            self._state["stitch_rgb_jpg"] = encode_jpeg(result.rgb)
            self._state["stitch_mask_jpg"] = encode_jpeg(mask)
            self._state["stitch_skel_jpg"] = encode_jpeg(skel)
            self._state["stitch_info"] = info
            self._state["stitch_calib"] = calib.to_dict()
            self._state["stitch_on"] = True
            if refine_msg is not None:
                self._state["stitch_refine_result"] = refine_msg

    def _set_note(self, msg: str) -> None:
        print(f"[stitch] {msg}")
        with self._state_lock:
            self._state["stitch_note"] = msg


def _average(buffer: deque) -> tuple[np.ndarray, np.ndarray]:
    """Temporal average of a (depth, valid) buffer — same math as capture_frame."""
    frames = list(buffer)
    acc = np.zeros_like(frames[0][0], dtype=np.float32)
    cnt = np.zeros_like(acc, dtype=np.float32)
    for z, ok in frames:
        acc[ok] += z[ok]
        cnt += ok
    valid = cnt > 0
    depth = np.zeros_like(acc)
    depth[valid] = acc[valid] / cnt[valid]
    return depth, valid
