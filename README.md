# depth-cam-to-robot

Turn hand-drawn grooves raked into sand into a UR robot drawing. An Intel RealSense
depth camera looks straight down at the sandbox, you click **Capture Image**, tune
the groove detection until the marks stand out as clean centrelines, click **Generate
Path**, then **Run** — the robot retraces every groove with a pen (or rake tool).

Grooves are a *few-millimetre physical depression*. An RGB camera can't see them —
the shading is far too subtle, even after aggressive contrast touch-up — but a depth
camera measures them directly. This project works on the raw metric depth the
RealSense reports, colorizing it only for display.

---

## Overview

`depth-cam-to-robot` bridges a depth camera and robot motion:

1. A RealSense D435i looks down at the sand and streams metric depth.
2. The depth is detrended (the smooth bare-sand surface is estimated and subtracted)
   so each groove shows up as local relief — "a few mm deeper than its surroundings."
3. The thresholded grooves are thinned to 1-pixel centrelines and turned into an
   ordered list of strokes.
4. The strokes are **projected onto a target surface** — a mesh authored in Rhino
   and loaded as STL/OBJ (flat, tilted, vertical, or fully non-planar) — with the
   TCP oriented perpendicular to the surface at every waypoint.
5. A Universal Robots arm traces the 6-DOF path with smooth `servoL` streaming at
   125 Hz, at a user-set speed, hover offset, and safety retract distance.

All interaction happens in a browser-based UI that opens automatically. No ROS, no
offline programming.

---

## How it works

```
RealSense depth frame (metres)
    │
    ▼
temporal averaging (N frames on Capture)   ← cuts per-pixel depth noise ~√N
    │
    ▼
gap-fill ─► denoise (Gaussian) ─► subtract smooth surface ─► local relief (mm)
    │
    ▼
threshold "a few mm deeper"   ─► morphological close ─► drop small blobs
    │
    ▼
skeletonize                   ← 1-px-wide groove centrelines
    │
    ▼
_chains_from_edges()          ← 8-connected pixel chain follower (visit once)
    │
    ▼
smooth_stroke()               ← Chaikin corner-cutting (2 iterations)
    │
    ▼
resample_stroke()             ← uniform arc-length resampling (~10 px spacing)
    │
    ▼
_order_strokes()              ← TSP nearest-neighbour; minimises pen-up travel
    │
    ▼
SurfaceModel.project_strokes()  ← ray-cast onto the target mesh (STL/OBJ);
    │                             TCP perpendicular to the surface per waypoint
    │                             (planar fallback: pixels_to_robot_coords)
    ▼
reach check                   ← flags waypoints outside the arm's envelope (red)
    │
    ▼
PathExecutor._run()
    ├─ moveL  retract along the tool axis (safety distance)
    ├─ moveL  travel to the retracted stroke start
    └─ servoL stream  (125 Hz, smoothstep ease-in/out, 6-DOF)
         ── repeats per stroke ──
    └─ moveL  final retract
```

---

## Groove detection from depth

### Why "valley detection", not a fixed depth band

A perfectly level sandbox would let you threshold an absolute depth band, but real
surfaces sag and tilt, so a fixed depth picks up the *slope* of the sand, not the
marks. Instead we estimate the smooth bare-sand surface (a heavily blurred copy of
the depth map) and subtract it, leaving only the **local relief**. A groove is then
simply "a few mm deeper than its immediate surroundings" anywhere on the surface —
regardless of how the sandbox tilts. (An absolute iso-depth band is still available
via the **Band** mode.)

### The stages (`depth_extractor.grooves_from_depth`)

**1. Gap fill** — invalid depth pixels (0 / NaN) are filled from the nearest valid
neighbour so blurring doesn't bleed holes into the surface estimate.

**2. Denoise** (`smooth_sigma_px`) — a small Gaussian smooths per-pixel depth noise.

**3. Detrend** (`detrend_sigma_px`) — the bare-sand surface is estimated as the
low-frequency component (a large-radius Gaussian) and subtracted, giving relief in
millimetres. Positive relief = farther from the top-down camera = a depression.

**4. Threshold** (`groove_depth_mm`, `detect` mode):
| Mode | Keeps |
|------|-------|
| `valley` (default) | relief deeper than `groove_depth_mm` — the grooves |
| `ridge` | relief raised more than `groove_depth_mm` — bumps/ridges |
| `band` | relief within `band_center_mm ± band_width_mm` — an absolute iso-depth slice |

**5. Clean** — a morphological close bridges 1-px gaps, then connected components
smaller than `min_blob_px` are discarded as noise.

**6. Skeletonize** — the thick mask is thinned to 1-pixel-wide centrelines (the same
"each pixel once" property the chain extractor relies on). Uses scikit-image if
installed, else opencv-contrib `ximgproc.thinning`, else a pure-numpy Zhang-Suen
fallback.

### From centrelines to robot strokes

The skeleton is a binary mask of white pixels on black. `_chains_from_edges()` walks
each centreline via 8-connectivity, starting from endpoints (pixels with ≤1
neighbour) and removing each visited pixel, so every pixel is visited exactly once
and each chain is an ordered tip-to-tip path. (`cv2.findContours` would trace each
thin line down one side and back the other, drawing it twice.)

### Tuning

| Goal | What to change |
|------|----------------|
| Catch fainter grooves | Lower `groove_depth_mm` |
| Reject noise / grain | Raise `groove_depth_mm`, or raise `smooth_sigma_px` (Denoise) |
| Flatten broad undulations | Lower `detrend_sigma_px` (Surface scale) |
| Keep thin marks | Lower `smooth_sigma_px` |
| Discard speckle | Raise `min_blob_px` |
| Trace raised lines instead | Switch **Mode** to `ridge` |

The single biggest quality win is **temporal averaging**: the sand is static, so
Capture averages `DEPTH_AVERAGE_FRAMES` frames, cutting per-pixel depth noise by
~√N before any detection runs.

### Rejecting natural grooves

Sand often has pre-existing ripples/texture that look like grooves. Four optional
filters (in the **Reject natural grooves** panel section) suppress them — each is
independent and **disabled at 0**, so you can A/B the difference:

- **Reference subtraction** (`ref_strength`) — capture the *undrawn* sand with **Set
  Reference**, then subtract that baseline. Pre-existing grooves appear in both the
  reference and the live frame and **cancel**, leaving only what you drew. The single
  most reliable discriminator (camera + sandbox must stay still between the two).
- **Min mean depth** (`min_mean_depth_mm`) — drop whole grooves whose *average* relief
  is shallow. Hand-raked grooves are consistently a few mm deep; faint ripples aren't.
- **Min / Max width** (`min_width_mm` / `max_width_mm`) — keep only grooves matching the
  raking tool's width; rejects thin scratches and broad dishes.
- **Min length** (`min_length_mm`) — drop short grooves; natural texture breaks into
  short fragments. (Width and length get their mm scale from the drawing's fit onto
  the loaded surface, or from the Test-Mode workspace.)

---

## Hardware requirements

| Component | Requirement |
|-----------|-------------|
| Robot | Universal Robots UR3 / UR5 / UR10 / UR16 (any with RTDE support) |
| Robot mode | **Remote Control** enabled on Teach Pendant (Settings → System → Remote Control) |
| Camera | Intel RealSense D435i (any RealSense depth camera should work) |
| Camera position | Top-down view covering the full sandbox |
| Python | 3.11+ |

A short-range RealSense (e.g. D405) resolves sub-mm grooves even better, but the
D435i is the reference setup here.

---

## Installation

```bash
git clone https://github.com/zhoulin-ethaha/depth-cam-to-robot.git
cd depth-cam-to-robot
python -m venv .venv
.venv\Scripts\activate           # Windows  (macOS/Linux: source .venv/bin/activate)
pip install -r requirements.txt
```

**Dependencies:**

| Package | Purpose |
|---------|---------|
| `pyrealsense2 >= 2.54` | RealSense depth capture |
| `opencv-python >= 4.8` | Depth filtering, colorizing, JPEG encoding |
| `scikit-image >= 0.22` | Fast skeletonization (a pure-numpy fallback runs without it) |
| `ur-rtde >= 1.6` | UR robot RTDE control (moveL, servoL, TCP pose) |
| `aiohttp >= 3.9` | Async web server, MJPEG streaming, WebSocket |
| `numpy >= 1.26` | Array operations |
| `trimesh >= 4.0` + `rtree` | Target-surface mesh loading and ray-casting |
| `scipy >= 1.11` | Rotations (surface-normal TCP orientations, retracts) |

---

## Running

```bash
python main.py
```

The browser opens automatically at **`http://localhost:5005`**. (Port 5005 is
deliberately off the common 8080/8000 range so this app can run alongside other
tools without a port clash.) Closing the browser tab stops the server.

---

## Workflow

1. **Connect** — enter the robot's IP (e.g. `192.168.1.100`) and click **Connect**.

2. **Load the drawing target** — an overlay prompts for the target surface: mesh
   your Rhino surface, export it as **STL/OBJ in millimetres**, and load it. (There
   is no manual robot calibration step — the surface's position relative to the
   robot is set with the Surface X/Y/Z + rotation sliders and verified visually in
   the Path Preview.)

3. **Aim the RealSense** straight down so it covers the whole sandbox. The four
   viewports show **Depth** (near = blue → far = red), **RGB**, **Skeleton** (the
   1-px centrelines that become the path) and **Mask** (the thick detected region,
   which shows groove *width* and is handy while tuning).

4. **Tune detection live** — the **Detect Grooves** panel is available *before* you
   capture. Pick a **Mode** (Valley / Ridge / Band) and adjust **Groove depth**,
   **Surface scale**, **Denoise**, and **Min blob**; the groove viewports update in
   real time. You can also drag a **crop** rectangle directly on the Depth view to
   limit the region — RGB/Skeleton/Mask then show only the cropped area.

5. **Capture Image** — freezes a temporally averaged depth (+ aligned colour) frame;
   the crop you drew carries over (adjust it on the still: drag inside to move, corners
   to resize; **Reset Crop** restores the full frame). Detection — and the generated
   path — cover only the cropped region. (Depth-view range sliders are display-only.)

6. **Generate Path** — the 3D viewer shows the surface and the extracted path
   (green = pen-down, grey = pen-up travel). Waypoints outside the arm's estimated
   reach are highlighted **red** with a header warning — move the surface closer or
   crop smaller before running. The **Path | Order** toggle switches to a numbered
   view (stroke order, green start / red end dots, size slider); **⧉ Pop out**
   opens the preview in its own window. Re-tune and regenerate freely, or
   **Retake** for a fresh capture.

7. **Run** — set **Speed** (% of max TCP speed — governs the *entire* motion,
   travels included), **Offset** (mm off the surface along the local normal) and
   **Safety** (retract distance, mm) in the preview's execution bar, then Run. The
   blue dot tracks the live TCP along the strokes. A progress bar tracks execution;
   **Cancel** stops mid-stroke; failures show "Run failed: …" in the header.

   **💾 Save** (execution bar) writes the toolpath — with the current
   Speed/Offset/Safety baked in — to a timestamped folder under `paths/` (see
   *Saving toolpaths* below).

### Test mode (no robot)

Click **Test Mode (no robot)** to set a synthetic workspace and exercise the
depth → groove → path-preview pipeline without connecting a robot. Run stays gated
on a real connection.

### How the drawing maps onto the surface

1. In Rhino, `Mesh` your surface and **export as STL/OBJ in millimetres**. The mesh
   may be modelled **flat, tilted, or vertical** — projection follows the mesh's
   dominant (area-weighted average) face normal, and the drawing lands on the side
   the normals point (flip them in Rhino with `Dir` if the paths appear on the back).
2. The full camera frame (4:3) is **fitted centred** onto the surface's footprint,
   aspect preserved — so each stroke lands at the same relative position it has in
   the camera view, and the scale is fixed by the surface size (frame width ↦
   fitted width). Cropping only selects which grooves exist; it doesn't move or
   zoom the drawing.
3. Every waypoint gets a tool orientation **perpendicular to the surface**, with
   minimal wrist twist between points. Rays that miss the mesh split the stroke,
   so drawings larger than the surface fall off its edges.
4. Placement is live: the **Surface X/Y/Z + Rot X/Y/Z** sliders position the mesh
   in the robot base frame (the axes marker in the preview is the base origin).
   The **TCP offset (mm)** slider bakes a hover distance at Generate time; the
   execution bar's **Offset** box adds more at Run time without regenerating.
5. Surface contact depth comes from the offsets — the planar `DRAW_Z` is not
   applied in surface mode. Retracts follow the tool axis, so they pull *away*
   from tilted/vertical surfaces instead of sliding along them.

**Clear Surface** returns to the flat-workspace mapping (Test Mode). If the robot
draws on a *real* physical surface, the virtual placement must match reality —
set the sliders to where the object sits relative to the robot base and verify
with the preview and a slow, offset-first run.

### Saving toolpaths

**💾 Save** in the execution bar writes the generated toolpath to a **timestamped
subfolder** under `paths/` (e.g. `paths/2026-07-13_14-32-08/`) containing three
files:

- **`path.script`** — a **URScript** program (native to UR controllers): `movel`
  travels + `movep` drawing moves, with the current Speed/Offset/Safety baked in.
  Directly runnable — verify the TCP/payload on the pendant and run slow first.
- **`path.json`** — the strokes as 6-DOF poses **plus a full plane/frame per
  waypoint** (`origin` + orthonormal `xaxis`/`yaxis`/`zaxis`, z = tool approach).
  This is the format for frame/plane-guided workflows (Grasshopper, custom
  motion) rather than bare points.
- **`preview.png`** — the 3D Path Preview image, so the operator can identify the
  saved path at a glance.

`paths/` is gitignored. The header of each `.script` records the mode, surface,
speed, offset, safety and stroke count.

### Projecting the mask onto the sand

A projector pointed at the sandbox can light up the detected grooves in place —
the **⧉ Project** button on the **Mask** viewport opens `/projection` in its own
window; drag it onto the projector display and press **F11**. No extra software
is needed (a corner-pin homography in the browser does the mapping), and the
projector-side stream is only computed while the window is open.

- **Calibrate once:** rake reference marks into the sand corners, then drag the
  projected corner handles **1–4** until the white mask lands on the physical
  marks (arrow keys nudge 1 px, Shift = 10 px). Saved to `settings.json`;
  **C** re-enters calibration, **B** blanks manually.
- The projection uses the **full-frame** mask (stable coordinates regardless of
  the crop box), so it always matches the camera's view of the sand.
- **Capture auto-blanks** the projector and waits ~1 s for the depth buffer to
  refill, so projected light never contaminates the captured depth.
- Projector setup: keystone OFF, no digital zoom (the corner-pin handles all
  geometry), fixed mount — recalibrate after any bump. Valid for the calibrated
  sand plane; a dimmer room gives crisper grooves.

---

## Configuration reference

All parameters live in `config.py`.

### Server

| Variable | Default | Description |
|----------|---------|-------------|
| `HTTP_HOST` | `"localhost"` | Bind address |
| `HTTP_PORT` | `5005` | Web UI port |

### Depth camera (RealSense)

| Variable | Default | Description |
|----------|---------|-------------|
| `DEPTH_WIDTH` | `640` | Depth stream width (px) |
| `DEPTH_HEIGHT` | `480` | Depth stream height (px) |
| `DEPTH_FPS` | `30` | Depth stream frame rate |
| `DEPTH_AVERAGE_FRAMES` | `30` | Frames temporally averaged on Capture |
| `DEPTH_COLOR_NEAR_M` / `DEPTH_COLOR_FAR_M` | `0.0` | Colormap range in metres (0 = auto) |

### Groove detection

| Variable | Default | Description |
|----------|---------|-------------|
| `GROOVE_DETECT` | `"valley"` | `valley` / `ridge` / `band` |
| `GROOVE_DEPTH_MM` | `1.5` | mm deeper than surface to count as a groove |
| `GROOVE_DETREND_SIGMA_PX` | `25.0` | blur radius estimating the bare surface |
| `GROOVE_SMOOTH_SIGMA_PX` | `1.5` | depth denoise before detection |
| `GROOVE_MIN_BLOB_PX` | `40` | discard detected specks smaller than this |
| `CONTOUR_MIN_PIXELS` | `20` | discard chains shorter than this many pixels |

### Path extraction

| Variable | Default | Description |
|----------|---------|-------------|
| `RESAMPLE_SPACING_MM` | `5.0` mm | Target spacing between resampled waypoints |

### Target surface

| Variable | Default | Description |
|----------|---------|-------------|
| `SURFACE_DIR` | `surfaces/` | Uploaded STL/OBJ meshes are stored here |
| `SURFACE_UNITS_TO_M` | `0.001` | File-unit scale (Rhino mm → m; set 1.0 for metres) |
| `SURFACE_MAX_FACES` | `80000` | Warn above this — browser preview gets heavy |

### Robot motion

| Variable | Default | Units | Description |
|----------|---------|-------|-------------|
| `DRAW_Z` | `-0.010` | m | Planar-mode pen contact offset (not used in surface mode) |
| `TRAVEL_Z` | `0.050` | m | Default safety retract (UI Safety box overrides per run) |
| `DRAW_SPEED` | `0.05` | m/s | Default speed = 5% (UI Speed slider overrides per run) |
| `MAX_TCP_SPEED` | `1.0` | m/s | 100% on the Speed slider (UR10e rated max tool speed) |
| `DRAW_ACCEL` | `0.3` | m/s² | Drawing acceleration |
| `TRAVEL_ACCEL` | `0.5` | m/s² | Travel/retract acceleration |
| `TOOL_ORIENTATION` | `[0, π, 0]` | rad | Planar-mode TCP orientation (surface mode derives it per waypoint) |
| `UR_REACH_M` | `1.30` | m | Reach-check envelope radius around the base |
| `UR_MIN_REACH_M` | `0.18` | m | Reach-check inner cylinder around the base axis |

### RTDE servo streaming

| Variable | Default | Description |
|----------|---------|-------------|
| `RTDE_FREQUENCY` | `125` Hz | servoL command rate |
| `SERVO_LOOKAHEAD_TIME` | `0.1` s | Lookahead buffer for smooth interpolation |
| `SERVO_GAIN` | `300` | Proportional gain for servo control |
| `SERVO_VELOCITY_DEFAULT` | `0.10` m/s | Velocity hint passed to servoL |
| `SERVO_ACCELERATION` | `0.5` m/s² | Acceleration hint passed to servoL |

### Robot home position

| Variable | Default | Description |
|----------|---------|-------------|
| `START_JOINT_ANGLES` | `[0, -π/2, π/2, -π/2, -π/2, 0]` | Joint angles for safe home pose (rad) |
| `START_SPEED` | `0.3` m/s | Speed for moveJ to home |
| `START_ACCEL` | `0.5` m/s² | Acceleration for moveJ to home |

---

## Project structure

```
depth_cam-to-robot/
├── main.py                  # Entry point: shared state, callbacks, startup, TCP poller
├── config.py                # All configurable parameters
├── server.py                # aiohttp server: MJPEG feeds, WebSocket, surface upload
├── camera_thread.py         # DepthCameraThread: RealSense → depth/RGB/skeleton/mask streams
├── depth_extractor.py       # Depth → groove engine: colorize, detect, filter, skeletonize
├── path_extractor.py        # Grooves → pixel chains → smooth → resample → TSP
├── surface.py               # Target mesh: STL/OBJ load, projection, normal TCP orientations
├── path_export.py           # Save toolpath → URScript + JSON (poses+frames) + preview PNG
├── path_executor.py         # Background thread: retract/travel/servoL per stroke, progress
├── robot_controller.py      # Thread-safe ur-rtde wrapper (moveL, servoL, EE pose)
├── workspace.py             # Planar fallback mapping (Test Mode)
├── settings.py              # Persistent JSON settings (last robot IP)
├── conftest.py              # Pytest shared fixtures
├── pytest.ini               # Test configuration
├── requirements.txt         # Dependencies
├── settings.json            # Auto-generated: saved app settings
├── surfaces/                # Uploaded target meshes (gitignored)
├── paths/                   # Saved toolpaths: dated folders of .script/.json/.png (gitignored)
├── tests/                   # Unit + hardware-gated integration tests
└── viewer/
    ├── index.html           # Single-page app
    ├── viewer.js            # WebSocket client, UI handlers, Three.js 3D path preview
    ├── style.css            # Responsive layout
    └── lib/
        ├── three.min.js     # Three.js (3D rendering)
        └── OrbitControls.js # Mouse/touch orbit controls
```

---

## Architecture notes

### Threading model

Three background daemon threads run alongside the async event loop:

- **`DepthCameraThread`** — continuously reads depth + colour frames from the
  RealSense, serves four live streams (colorized depth, aligned RGB, skeleton,
  mask — the latter three cropped to the live crop box), and buffers the recent
  raw metric depth so Capture can return a temporally averaged frame.
- **`PathExecutor`** — runs the per-stroke retract/travel/draw sequence.
  Long-running RTDE calls (`moveL`, `servoL`) happen here so the WebSocket
  broadcast loop is never blocked.
- **TCP poller** — reads the robot's actual TCP pose at 10 Hz while connected so
  the preview's blue dot tracks the arm in real time.

All cross-thread communication goes through a single `shared_state: dict` protected
by `state_lock: threading.Lock`. The aiohttp event loop offloads blocking work via
`loop.run_in_executor(None, fn)`.

### Surface placement (instead of calibration)

There is no robot-side calibration: the target surface's pose in the robot base
frame is set directly with the placement sliders and verified visually (the axes
marker in the preview is the base origin). `SurfacePose` (translation + XYZ Euler)
transforms projected waypoints and normals into the base frame. The planar
`WorkspaceConfig` mapping remains as the Test-Mode fallback.

### Smooth robot motion

Drawing strokes use `servoL` rather than `moveL`:

- Commands stream at **125 Hz** (8 ms timesteps), interpolating position *and*
  orientation (6-DOF) between waypoints.
- Position advances using **arc-length parameterisation** — the speed set on the
  Speed slider holds regardless of waypoint density, and the same speed is used
  for retract/travel moves so the whole actuation is uniform.
- **Smoothstep ease-in/out** (`f(α) = 3α² − 2α³`) zeroes velocity at stroke
  start/end (peaking at 1.5× mid-stroke), eliminating jerk at pen-down/up.

---

## Development & testing

```bash
pip install -r requirements.txt

# Unit tests (no hardware required)
pytest -q -m "not integration"

# Integration tests (require a RealSense and/or UR robot)
set TEST_ROBOT_IP=192.168.1.100      # Windows  (macOS/Linux: export ...)
pytest -m integration -v
```

| Test file | What it covers |
|-----------|---------------|
| `test_depth_extractor.py` | Depth → groove detection, natural-groove filters, colorize, crop/process |
| `test_path_extractor.py` | Chain extraction, resampling, TSP ordering, coordinate mapping |
| `test_surface.py` | Mesh projection (flat/tilted/vertical), normal TCP orientations, offsets, placement, misses |
| `test_path_executor.py` | Stroke sequencing, servoL streaming, uniform speed, tool-axis retracts, cancel |
| `test_path_export.py` | URScript generation, JSON poses+frames, offset baking, timestamped bundle saving |
| `test_robot_controller.py` | RTDE port probe, connect/disconnect, motion commands, thread safety |
| `test_integration.py` | Live RealSense feed, full depth→groove→robot pipeline (hardware-gated) |

All unit tests mock hardware (robot) or use synthetic depth — no physical devices
needed.

---

## References

- Robot communication based on [UR-hand-control](https://github.com/f-scotto/UR-hand-control)
- UR RTDE interface: [ur-rtde documentation](https://sdurobotics.gitlab.io/ur_rtde/)
- Intel RealSense SDK (`pyrealsense2`): [librealsense](https://github.com/IntelRealSense/librealsense)
