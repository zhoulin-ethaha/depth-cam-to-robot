# CLAUDE.md

## What this is
depth-cam-to-robot: a browser-controlled pipeline that watches a sandbox with an
Intel RealSense **D435i** depth camera, detects hand-raked grooves (mm-deep — raw
metric depth, no RGB vision), converts them to strokes, projects them onto a
Rhino-authored 3D target surface, and has a **UR10e** (ur-rtde) retrace them with
the TCP perpendicular to the surface. Artistic context: gestures in sand guide a
robot depositing a living seeded substrate — the code's job ends at toolpath
execution/export. Includes a projector subsystem that shines the detected mask
back onto the sand, and a Save feature exporting URScript + JSON toolpaths.

## Run / test
- Run: `run.bat` or `.venv\Scripts\python.exe main.py` → UI at http://localhost:5005.
  Closing the last browser tab kills the server (deliberate, via SIGINT).
- ALWAYS use `.venv\Scripts\python.exe`; never bare `pip` (broken launcher risk —
  use `python -m pip`). The venv is NOT relocatable.
- Unit tests: `.venv/Scripts/python.exe -m pytest -q -m "not integration"` (138, no
  hardware). Integration: `-m integration`, needs RealSense/robot + TEST_ROBOT_IP.
- No CLI modes. Hardware vs no-robot is in the UI: "Test Mode (no robot)" button
  unlocks capture with a synthetic workspace; Run stays gated on a robot connection.
- Robot bring-up: UR10e in **Remote Control** mode, pendant speed slider 100%
  (servoL lags otherwise), static link-local IP (e.g. 169.254.10.10), TCP+payload
  set on pendant. PC on same subnet.

## Pipeline (stage → owner → I/O)
1. **Capture** `camera_thread.DepthCameraThread` — RealSense depth+RGB 640×480@30,
   colour aligned to depth. Rolling buffer; `capture_frame()` → (depth_m float32
   HxW, valid bool, rgb BGR|None), temporally averaged (~30 frames ≈1 s, noise ↓√N).
   Live JPEGs into shared_state keys (`last_depth_color_jpg` etc.).
2. **Groove detection** `depth_extractor` — `grooves_and_mask(depth, valid, params,
   reference, mm_per_px)`: gap-fill → denoise → detrend (subtract blurred surface)
   → threshold (valley/ridge/band, mm relief) → morph close/min-blob → per-stroke
   filters (reference subtraction, min mean depth, min/max width, min length) →
   (thick mask, 1-px skeleton). `process_depth` adds crop; coords stay full-frame.
3. **Stroke extraction** `path_extractor.extract_from_edges` — 8-conn chain follow
   → Chaikin smooth → resample (**hardcoded 10 px**; RESAMPLE_SPACING_MM unused —
   known bug) → nearest-neighbour TSP ordering → pixel strokes.
4. **Mapping** `surface.SurfaceModel.project_strokes` — STL/OBJ (Rhino, mm→m) via
   trimesh; camera frame fitted centred (aspect kept) onto the footprint ⟂ the
   mesh's dominant normal; ray-cast; TCP ⟂ surface with minimal twist; offset
   along outward normal. Placement = `SurfacePose` (m + XYZ euler deg, base frame),
   set by UI sliders — no camera↔robot calibration exists. Planar fallback:
   `path_extractor.pixels_to_robot_coords` + `workspace.WorkspaceConfig` (Test Mode).
5. **Reach check** `reach.reach_flags` — envelope only (1.30 m sphere − 0.18 m axis
   cylinder). No IK/joint-limit/collision model. Red segments in preview.
6. **Execution** `path_executor.PathExecutor` — per stroke: retract along tool axis
   (Safety mm) → movel travel → servoL 125 Hz 6-DOF stream (smoothstep ease);
   uniform speed = UI % of MAX_TCP_SPEED (1.0 m/s); run-time normal offset baked
   into waypoints. `robot_controller` = thread-safe ur-rtde wrapper.
7. **Export** `path_export.save_bundle` → `paths/<YYYY-MM-DD_HH-MM-SS>/` with
   `path.script` (URScript movel/movep), `path.json` (poses + per-waypoint plane:
   origin + orthonormal x/y/z axes, z = approach), `preview.png`.
8. **Server/UI** `server.py` (aiohttp) + `viewer/` — MJPEG: /depth /rgb
   /depth/grooves /depth/mask /depth/mask/full; WS /ws (JSON); POST /surface/upload;
   GET /status (compact state JSON for tools); /projection (+?cal). viewer.js =
   single-page app w/ three.js preview; projection.html = corner-pin homography.
9. **Projector** — full-frame mask composed ONLY while a projection window is
   connected (`projection_clients`); corners persist in settings.json; Capture
   auto-blanks projector and waits for buffer refill before averaging.

## Conventions
- Pose = `[x, y, z, rx, ry, rz]`: metres + UR rotation vector (rad), robot base
  frame. Tool approach = tool-frame +Z; outward surface normal = −(R@[0,0,1]).
- Pixels 640×480, v grows down (flipped to world/robot Y-up). Crops normalized
  [0,1]; stroke coords always shifted back to full frame before mapping.
- Mesh files + UI depth params in mm; everything robot-side in m.
- `config.py` = every constant. `settings.json` = last robot IP + projector
  corners. Gitignored: `surfaces/`, `paths/`, `settings.json`, `.venv/`.
- Phases: idle → previewing → editing → captured → executing → done | error.

## Key WS messages (browser ↔ server; external tools may use these)
- in: `connect{ip}`, `disconnect`, `simulate_workspace`, `capture_image`,
  `preview_adjust{params}`, `generate_path{params:{crop,adjustments}}`,
  `run{params:{speed_pct,offset_mm,safety_mm}}`, `cancel`,
  `save_path{params:{speed_pct,offset_mm,safety_mm,image}}`,
  `set_groove_params{params}`, `set_reference`/`clear_reference`,
  `set_surface_pose{params:{pose,offset_mm}}`, `clear_surface`,
  `projection_hello`, `projection_corners{corners}`.
- out: `state` (20 Hz), `capture_result{stroke_count,point_count,strokes,
  reach_flags,reach_out}`, `still`, `preview`, `surface_status`, `save_result`,
  `reference_status`, `execution_update`, `connection_result`.

## Don't touch / gotchas
- **Never `import main` from tools/scripts** — import starts the camera thread and
  pollers (hardware side effects). Import the stage modules instead.
- One process per RealSense; one RTDE controller per robot. The running app owns
  both — external tools must go through HTTP/WS, not open hardware directly.
- Safety constants (`MAX_TCP_SPEED`, `UR_REACH_M`, speeds/accels, `DRAW_Z`) only
  change on explicit user request.
- Projection windows intentionally open on `127.0.0.1` (not localhost): Chrome
  caps 6 HTTP/1.1 connections per host and MJPEG streams hold theirs forever.
- `/`, `/projection`, `/static/*` are served no-cache — but Python changes still
  need an app restart.
- `movep`/servoL orientation interp assumes neighbouring waypoints don't flip the
  wrist — surface projection chains tool-X for minimal twist; keep that property.
- Test count reference: 138 unit (+6 hardware-gated). Keep green.
