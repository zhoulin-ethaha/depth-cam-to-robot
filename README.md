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
   ordered list of robot poses.
4. A Universal Robots arm traces the path with smooth `servoL` streaming at 125 Hz.

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
pixels_to_robot_coords()      ← 3-point workspace calibration maps px → metres
    │
    ▼
PathExecutor._run()
    ├─ moveL  lift above current position
    ├─ moveL  travel to above stroke start
    └─ servoL stream  (125 Hz, smoothstep ease-in/out)
         ── repeats per stroke ──
    └─ moveL  final pen-up
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
git clone <your-repo-url>
cd depth_cam-to-robot
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
| `ur-rtde >= 1.6` | UR robot RTDE control (moveL, servoL, freedrive, TCP pose) |
| `aiohttp >= 3.9` | Async web server, MJPEG streaming, WebSocket |
| `numpy >= 1.26` | Array operations |

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

### First time

1. **Connect** — enter the robot's IP (e.g. `192.168.1.100`) and click **Connect**.

2. **Workspace setup** — a setup overlay appears.
   - Click **Activate Freedrive** — the arm goes compliant.
   - Move the TCP to three reference points on the sandbox and **Record** each:
     - **P0** — workspace origin (e.g. bottom-left corner)
     - **Px** — a point along the X direction (e.g. bottom-right corner)
     - **Py** — a point along the Y direction (e.g. top-left corner)
   - Click **Confirm & Start** — saved to `workspace.json` for future sessions.

3. **Aim the RealSense** straight down so it covers the whole sandbox. The left panel
   shows the colorized depth (near = blue → far = red); the right panel shows a live
   groove preview.

4. **Capture Image** — freezes a temporally averaged depth frame and switches to the
   editing view.

5. **Detect Grooves** — in the panel:
   - Drag on the depth still to draw a **crop** rectangle (drag inside to move,
     corners to resize; **Reset Crop** restores the full frame). The crop selects
     which part of the workspace gets drawn — strokes stay correctly positioned.
   - Pick a **Mode** (Valley / Ridge / Band) and tune **Groove depth**, **Surface
     scale**, **Denoise**, and **Min blob** until the grooves show as clean white
     centrelines with little background noise.
   - The right panel toggles between **Grooves** (what becomes the path) and **Depth**
     (the colorized depth). Depth-view range sliders are display-only.

6. **Generate Path** — the 3D viewer shows the extracted path (green = pen-down,
   grey = pen-up travel). Re-tune and regenerate freely, or **Retake** for a fresh
   capture.

7. **Run** — the robot lifts, travels to the first stroke, and draws. A progress bar
   tracks execution; **Cancel** stops mid-stroke.

### Subsequent sessions

After the first calibration, a **"Use This Workspace"** banner appears on connect.
Click it to skip recalibration.

### Test mode (no robot)

Click **Test Mode (no robot)** to set a synthetic workspace and exercise the
depth → groove → path-preview pipeline without connecting a robot. Run stays gated
on a real connection.

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

### Robot motion

| Variable | Default | Units | Description |
|----------|---------|-------|-------------|
| `DRAW_Z` | `-0.010` | m | Pen-contact Z offset below workspace surface origin |
| `TRAVEL_Z` | `0.050` | m | Pen-up travel height above workspace surface origin |
| `DRAW_SPEED` | `0.05` | m/s | Average drawing speed (smoothstep peak is 1.5×) |
| `DRAW_ACCEL` | `0.3` | m/s² | Drawing acceleration |
| `TRAVEL_SPEED` | `0.15` | m/s | Pen-up travel speed |
| `TRAVEL_ACCEL` | `0.5` | m/s² | Pen-up travel acceleration |
| `TOOL_ORIENTATION` | `[0, π, 0]` | rad | TCP orientation (rx, ry, rz) for tool-down |

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
├── main.py                  # Entry point: shared state, callbacks, startup
├── config.py                # All configurable parameters
├── server.py                # aiohttp server: MJPEG feeds, WebSocket, static files
├── camera_thread.py         # DepthCameraThread: RealSense → colorized-depth + groove streams
├── depth_extractor.py       # Depth → groove engine: colorize, detect, crop, skeletonize
├── path_extractor.py        # Grooves → pixel chains → smooth → resample → TSP → robot coords
├── path_executor.py         # Background thread: lift/travel/servoL per stroke, progress
├── robot_controller.py      # Thread-safe ur-rtde wrapper (moveL, servoL, freedrive, EE pose)
├── workspace.py             # 3-point workspace calibration; pixel ↔ robot coord mapping
├── settings.py              # Persistent JSON settings (last robot IP)
├── conftest.py              # Pytest shared fixtures
├── pytest.ini               # Test configuration
├── requirements.txt         # Dependencies
├── workspace.json           # Auto-generated: saved workspace calibration
├── settings.json            # Auto-generated: saved app settings
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

Two background daemon threads run alongside the async event loop:

- **`DepthCameraThread`** — continuously reads depth frames from the RealSense,
  colorizes them and computes a (throttled) live groove preview, encodes both as
  JPEG into `shared_state`, and buffers the recent raw metric depth so Capture can
  return a temporally averaged frame. The MJPEG endpoints read at ~30 fps.
- **`PathExecutor`** — runs the per-stroke lift/travel/draw sequence. Long-running
  RTDE calls (`moveL`, `servoL`) happen here so the WebSocket broadcast loop is
  never blocked.

All cross-thread communication goes through a single `shared_state: dict` protected
by `state_lock: threading.Lock`. The aiohttp event loop offloads blocking work via
`loop.run_in_executor(None, fn)`.

### Workspace calibration

Three robot TCP positions define the drawing surface (P0 origin, Px along X, Py along
Y). `WorkspaceConfig.from_points()` builds an orthonormal frame and extents in metres;
pixel coordinates map linearly: `world_x = (u / frame_width) × x_extent`, expressed in
the robot base frame by `p = origin + wx·x̂ + wy·ŷ`.

### Smooth robot motion

Drawing strokes use `servoL` rather than `moveL`:

- Commands stream at **125 Hz** (8 ms timesteps).
- Position advances using **arc-length parameterisation** — constant `DRAW_SPEED`
  regardless of waypoint density.
- **Smoothstep ease-in/out** (`f(α) = 3α² − 2α³`) zeroes velocity at stroke start/end
  (peaking at 1.5× `DRAW_SPEED` mid-stroke), eliminating jerk at pen-down/up.

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
| `test_depth_extractor.py` | Depth → groove detection, colorize, crop/process, param parsing, thread no-hardware paths |
| `test_path_extractor.py` | Chain extraction, resampling, TSP ordering, coordinate mapping |
| `test_path_executor.py` | Stroke sequencing, servoL streaming, ease-in/out, progress, cancel |
| `test_robot_controller.py` | RTDE port probe, connect/disconnect, motion commands, thread safety, freedrive |
| `test_integration.py` | Live RealSense feed, full depth→groove→robot pipeline (hardware-gated) |

All unit tests mock hardware (robot) or use synthetic depth — no physical devices
needed.

---

## References

- Robot communication based on [UR-hand-control](https://github.com/f-scotto/UR-hand-control)
- UR RTDE interface: [ur-rtde documentation](https://sdurobotics.gitlab.io/ur_rtde/)
- Intel RealSense SDK (`pyrealsense2`): [librealsense](https://github.com/IntelRealSense/librealsense)
