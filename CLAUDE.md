# CLAUDE.md

## Maintenance rule (apply on EVERY pipeline/feature change)
When the pipeline, WS/HTTP API, or features change: (1) update this file —
pipeline stages, WS message list, conventions, gotchas, test count; (2) check
`mcp_server/server.py` — its tools wrap the WS/HTTP API, so renamed/changed
messages, params or reply fields break them; update tools + `mcp_server/README.md`
to match; (3) update README.md user docs. Do this in the same commit as the change.

## What this is
depth-cam-to-robot: a browser-controlled pipeline that watches a sandbox with an
Intel RealSense **D435i** depth camera, detects hand-raked grooves (mm-deep — raw
metric depth, no RGB vision), converts them to strokes, projects them onto a
Rhino-authored 3D target surface, and has a **UR10e** (ur-rtde) retrace them with
the TCP perpendicular to the surface. Artistic context: gestures in sand guide a
robot depositing a living seeded substrate — the code's job ends at toolpath
execution/export. Includes a projector subsystem that shines the detected mask
back onto the sand, and a Save feature exporting URScript + JSON toolpaths.
Two modes: **Developer Mode** (`/`, all manual controls) and **Participant
Mode** (the ⧉ popup on the Depth viewport, `/depths`): an Auto toggle + depth
trigger run the whole pipeline automatically and lock the manual buttons.

## Run / test
- Run: `run.bat` or `.venv\Scripts\python.exe main.py` → Developer Mode at
  http://localhost:5005 (Participant Mode = its ⧉ popup). Closing the last
  browser tab kills the server (deliberate, via SIGINT).
- ALWAYS use `.venv\Scripts\python.exe`; never bare `pip` (broken launcher risk —
  use `python -m pip`). The venv is NOT relocatable.
- Unit tests: `.venv/Scripts/python.exe -m pytest -q -m "not integration"` (177, no
  hardware). Integration: `-m integration`, needs RealSense/robot + TEST_ROBOT_IP.
- No CLI modes. Hardware vs no-robot is in the UI: "Test Mode (no robot)" button
  unlocks capture with a synthetic workspace; Run stays gated on a robot connection.
- Robot bring-up: UR10e in **Remote Control** mode, pendant speed slider 100%
  (or programmed speeds scale down), static link-local IP (e.g. 169.254.10.10),
  TCP+payload set on pendant. PC on same subnet.

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
   → Chaikin smooth → resample at `spacing_mm` (UI Spacing slider 10–100 mm,
   default RESAMPLE_SPACING_MM=10; falls back to 10 px w/o a mm scale) →
   nearest-neighbour TSP ordering → pixel strokes. Also returns `strokes_dense`
   (~2 mm) for the white on-surface skeleton line in the 3D preview.
4. **Mapping** `surface.SurfaceModel.project_strokes` — STL/OBJ (Rhino, mm→m) via
   trimesh; camera frame fitted centred (aspect kept) onto the footprint ⟂ the
   mesh's dominant normal; ray-cast; TCP ⟂ surface with minimal twist; offset
   along outward normal. Placement = `SurfacePose` (m + XYZ euler deg, base frame),
   set by UI sliders OR by corner→TCP touch-off (`registration.py`: pick a mesh
   corner — click its marker in the 3D preview or the dialog list, hover
   highlights it cyan (dialog is non-modal, preview stays visible) — then
   freedrive the tool tip onto it, confirm —
   1-point = translation only, keeps slider rotation; Kabsch ≥3-point solver
   already implemented for a future multi-point UI; corners = mesh vertices
   nearest the bbox corners, shipped in `mesh_payload()["corners"]`, same
   indices browser + server). No camera↔robot calibration exists. Planar fallback:
   `path_extractor.pixels_to_robot_coords` + `workspace.WorkspaceConfig` (Test Mode).
5. **Reach check** `reach.reach_flags` — envelope only (1.30 m sphere − 0.18 m axis
   cylinder). No IK/joint-limit/collision model. Red segments in preview.
6. **Execution** `path_executor.PathExecutor` — per stroke: retract along tool axis
   (Safety mm) → movel travel → movel onto the first waypoint → **movep** blended
   process path through the rest (MOVEP_BLEND_M=0.5 mm; async movePath, polled so
   cancel stays responsive) — same actuation as the saved path.script; uniform
   speed = UI % of MAX_TCP_SPEED (1.0 m/s); run-time normal offset baked into
   waypoints. `robot_controller` = thread-safe ur-rtde wrapper.
7. **Export** `path_export.save_bundle` → `paths/<YYYY-MM-DD_HH-MM-SS>/` with
   `path.script` (URScript movel/movep), `path.json` (poses + per-waypoint plane:
   origin + orthonormal x/y/z axes, z = approach), `preview.png`.
8. **Server/UI** `server.py` (aiohttp) + `viewer/` — MJPEG: /depth /rgb
   /depth/grooves /depth/mask /depth/mask/full; WS /ws (JSON); POST /surface/upload;
   GET /status (compact state JSON for tools); GET/POST /presets + GET
   /presets/{name} (Detection-Parameter slider presets → `presets/<date_time>.json`,
   gitignored; browser-only, not exposed to MCP tools); /projection (+?cal);
   /depths (the Participant Mode popup: absolute mm-from-camera labels on the
   live depth view + Auto toggle + trigger box + big status chip — labels
   computed in camera_thread ONLY while a popup is connected, gated by
   `depth_overlay_clients`, throttled DEPTH_LABELS_EVERY). viewer.js =
   Developer-Mode single-page app w/ three.js preview; projection.html =
   corner-pin homography; depth_view.html + depth_overlay.js = the popup.
9. **Projector** — full-frame mask composed ONLY while a projection window is
   connected (`projection_clients`); corners persist in settings.json; Capture
   auto-blanks projector and waits for buffer refill before averaging.
10. **Participant Mode** `automation.ParticipantAutomation` (pure state machine)
    + `_participant_loop`/`_participant_pipeline` in main.py. Lives in the
    /depths popup: an **Auto toggle** (`set_automation{on}`) + a trigger
    distance (mm, `set_trigger`); camera thread flags frames with
    ≥TRIGGER_MIN_AREA_PX valid px closer than the trigger (`trigger_below`,
    `depth_extractor.depth_below_threshold`). Auto ON → **Auto On**; anything
    below trigger → **Alerted**; frame clear for PARTICIPANT_CLEAR_S →
    **Sensing** (waits buffer refill, then capture) → **Generating Paths**
    (current Dev-Mode crop/adjustments/spacing) → **Actuating** (save_bundle,
    then run if robot connected; skipped otherwise) → back to **Auto On**
    (**Auto Off** when toggled off). While Auto is ON the manual
    capture/generate/run WS calls are refused server-side (`_manual_locked`,
    also blocks MCP tools) and the Dev-Mode buttons grey out; automation
    itself calls the SAME handlers via `server.broadcast_ws()` (a ws shim
    fanning out to all browser clients), so Developer windows watch it live.
    Statuses shown big top-right in the popup via `state.participant`.

## Conventions
- Pose = `[x, y, z, rx, ry, rz]`: metres + UR rotation vector (rad), robot base
  frame. Tool approach = tool-frame +Z; outward surface normal = −(R@[0,0,1]).
- Pixels 640×480, v grows down (flipped to world/robot Y-up). Crops normalized
  [0,1]; stroke coords always shifted back to full frame before mapping.
- Mesh files + UI depth params in mm; everything robot-side in m.
- `config.py` = every constant. `settings.json` = last robot IP + projector
  corners. Gitignored: `surfaces/`, `paths/`, `presets/`, `settings.json`, `.venv/`.
- Phases: idle → previewing → editing → captured → executing → done | error.

## Key WS messages (browser ↔ server; external tools may use these)
- in: `connect{ip}`, `disconnect`, `simulate_workspace`, `capture_image`,
  `preview_adjust{params}`, `generate_path{params:{crop,adjustments,spacing_mm}}`,
  `run{params:{speed_pct,offset_mm,safety_mm}}`, `cancel`,
  `save_path{params:{speed_pct,offset_mm,safety_mm,image}}`,
  `set_groove_params{params}`, `set_reference`/`clear_reference`,
  `set_surface_pose{params:{pose,offset_mm}}`, `clear_surface`,
  `projection_hello`, `projection_corners{corners}`,
  `depth_overlay_hello`, `depth_overlay_params{params:{interval_mm}}`,
  `register_freedrive{params:{on}}`, `register_corner{params:{corner_index}}`,
  `set_trigger{params:{threshold_mm|null}}` (trigger distance; null/empty clears),
  `set_automation{params:{on}}` (Participant Auto toggle; ON locks manual
  capture/generate/run for every other client incl. MCP tools),
  `set_exec_params{params:{speed_pct,offset_mm,safety_mm,spacing_mm}}` (live,
  debounced sync of the exec bar so Participant Mode + reopened windows match).
- out: `state` (20 Hz, incl. `participant{auto,status,message,trigger_mm,below}`;
  `init` carries the same block plus `detect{crop,adjustments,spacing_mm}` +
  `exec{speed_pct,offset_mm,safety_mm}` — the browser restores its controls
  from these on (re)open), `capture_result{stroke_count,point_count,strokes,
  reach_flags,reach_out,skeleton,exec_viz:{blend_m,reach_m,min_reach_m,
  spacing_mm}}`, `still`, `preview`, `surface_status`, `save_result`,
  `reference_status`, `execution_update`, `connection_result`,
  `register_result{success,message,pose,error}`,
  `depth_labels{labels:[[u,v,mm],...]}` (only to /depths popups, ~4 Hz).
  (`skeleton` = dense on-surface [x,y,z] polylines for the white preview line;
  `exec_viz` lets the browser rebuild the toolpath viz client-side on
  Offset/Safety edits.)

## Don't touch / gotchas
- **Never `import main` from tools/scripts** — import starts the camera thread and
  pollers (hardware side effects). Import the stage modules instead.
- One process per RealSense; one RTDE controller per robot. The running app owns
  both — external tools must go through HTTP/WS, not open hardware directly.
- Safety constants (`MAX_TCP_SPEED`, `UR_REACH_M`, speeds/accels, `DRAW_Z`) only
  change on explicit user request.
- Projection windows intentionally open on `127.0.0.1` (not localhost): Chrome
  caps 6 HTTP/1.1 connections per host and MJPEG streams hold theirs forever.
- `/`, `/projection`, `/depths`, `/static/*` are served no-cache — but Python
  changes still need an app restart.
- Participant Sensing waits DEPTH_AVERAGE_FRAMES/DEPTH_FPS before capturing:
  the averaged still uses the PAST second, which would contain the hand
  otherwise. Keep that wait ≥ the buffer length.
- `movep` orientation interp assumes neighbouring waypoints don't flip the
  wrist — surface projection chains tool-X for minimal twist; keep that property.
- Live drawing and the saved path.script both use movep with MOVEP_BLEND_M —
  keep them in sync (that equivalence is the point of the movep executor).
- MOVEP_BLEND_M must stay < half the minimum waypoint spacing (10 mm) or the
  controller rejects the path.
- Test count reference: 177 unit (+6 hardware-gated). Keep green.
