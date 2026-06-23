# camera-to-robot

Turn any webcam image into a UR robot drawing. Point the camera at an object or sketch, click **Capture Image**, crop and adjust the still until the marking stands out, click **Generate Path**, then **Run** — the robot traces every detected edge with a pen.

The capture/adjust step is built for *subtle* subjects — e.g. shallow markings raked into a sandbox — where the signal is barely above the background. You freeze a frame, crop to the area of interest, and tune exposure / brightness / contrast / highlights / shadows / gamma / local contrast (CLAHE) / invert plus the Canny thresholds, with a live edge preview, before committing to a tool path.

---

## Overview

`camera-to-robot` is a Python application that bridges computer vision and robot motion:

1. A webcam looks down at a flat surface (paper, whiteboard, sand…)
2. OpenCV's Canny edge detector extracts the outlines of whatever is in frame
3. The edges are turned into an ordered list of robot poses
4. A Universal Robots arm traces the path with a pen, using smooth `servoL` streaming at 125 Hz

All interaction happens in a browser-based UI that opens automatically. No ROS, no offline programming.

---

## How it works

```
Camera frame
    │
    ▼
Gaussian blur  ──►  Canny edge detection  ──►  Binary edge image (1-px wide edges)
    │
    ▼
_chains_from_edges()          ← 8-connected pixel chain follower
    │                           visits each pixel once; no double-tracing
    ▼
smooth_stroke()               ← Chaikin corner-cutting (2 iterations)
    │
    ▼
resample_stroke()             ← Uniform arc-length resampling (~10 px spacing)
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

## Canny edge detection

### What it is

The **Canny edge detector** (John Canny, 1986) is a multi-stage algorithm that finds object boundaries in a greyscale image and outputs a 1-pixel-wide binary edge map. It is widely considered the gold standard for edge detection because it finds real edges while suppressing noise.

### The four stages

**1. Gaussian blur** (`CANNY_BLUR_KERNEL = 5`, a 5×5 kernel)

Smooths the image before gradient computation. Camera noise — random pixel variations — would otherwise produce false edges. A larger kernel removes more noise but blurs sharp corners; a smaller kernel preserves fine detail but picks up more noise.

**2. Gradient computation** (Sobel operator)

Applies a 3×3 Sobel filter in both X and Y directions to estimate how rapidly pixel intensity changes at each location. The result is a *gradient magnitude* (how strong the edge is) and a *gradient direction* (which way the edge runs).

**3. Non-maximum suppression**

Looks along the gradient direction at each pixel and keeps only the local maximum — i.e. the single pixel where the gradient is strongest across the edge width. Every other pixel on the same edge is set to zero.

This is why Canny edges are always **exactly 1 pixel wide** — a property our pixel-chain extractor relies on.

**4. Hysteresis thresholding** (two-pass)

Two thresholds control which gradient peaks become edges:

| Threshold | Parameter | Default | Meaning |
|-----------|-----------|---------|---------|
| High | `CANNY_THRESHOLD_HIGH` | `150` | Above this → strong edge (always kept) |
| Low | `CANNY_THRESHOLD_LOW` | `50` | Between low and high → weak edge (kept only if connected to a strong edge) |

The two-pass approach connects broken edges (which would otherwise stop a robot stroke mid-path) while discarding isolated noise specks that happen to exceed the low threshold.

### From Canny output to robot strokes

The edge image is a binary mask of white pixels on black. We need to turn those scattered pixels into ordered lists (strokes) the robot can trace.

**Why not `cv2.findContours`?**
`findContours` traces the *boundary* of white regions. For a 1-pixel-wide edge, the boundary goes along one side of the pixel and comes back along the other — visiting every pixel twice. The robot would draw each stroke, then retrace it in the opposite direction.

**What we do instead — `_chains_from_edges()`:**
1. Collect all white pixels into a set
2. Pre-compute each pixel's neighbour count; pixels with ≤1 neighbour are *endpoints* (chain tips)
3. Start each new chain from an endpoint (ensuring we begin at a tip, not the middle of a stroke)
4. Walk the chain pixel-by-pixel via 8-connectivity, removing each visited pixel from the set
5. Stop when no unvisited neighbours remain

Result: each pixel is visited exactly once and each chain is an ordered path from one tip to the other.

### Tuning the edge detector

| Goal | What to change |
|------|----------------|
| Fewer edges (less noise) | Increase `CANNY_THRESHOLD_HIGH` |
| Connect broken edges | Decrease `CANNY_THRESHOLD_LOW` |
| Reduce noise (at the cost of fine detail) | Increase `CANNY_BLUR_KERNEL` (must stay odd: 3, 5, 7…) |
| Discard tiny noise fragments | Increase `CONTOUR_MIN_PIXELS` |

---

## Hardware requirements

| Component | Requirement |
|-----------|-------------|
| Robot | Universal Robots UR3 / UR5 / UR10 / UR16 (any with RTDE support) |
| Robot mode | **Remote Control** enabled on Teach Pendant (Settings → System → Remote Control) |
| Camera | Any USB webcam or built-in laptop camera |
| Camera position | Top-down view covering the full drawing surface |
| Host OS | macOS or Linux (Windows not tested) |
| Python | 3.11+ |

---

## Installation

```bash
git clone https://github.com/f-scotto/camera-to-robot.git
cd camera-to-robot
python -m venv .venv
source .venv/bin/activate        # Windows: .venv\Scripts\activate
pip install -r requirements.txt
```

**Dependencies:**

| Package | Purpose |
|---------|---------|
| `opencv-python >= 4.8` | Camera capture, Gaussian blur, Canny, JPEG encoding |
| `ur-rtde >= 1.6` | UR robot RTDE control (moveL, servoL, freedrive, TCP pose) |
| `aiohttp >= 3.9` | Async web server, MJPEG streaming, WebSocket |
| `numpy >= 1.26` | Array operations in workspace geometry |

---

## Running

```bash
python main.py
```

The browser opens automatically at `http://localhost:8080`. Closing the browser tab stops the server.

---

## Workflow

### First time

1. **Connect** — enter the robot's IP address (e.g. `192.168.1.100`) and click **Connect**. The status dot turns green.

2. **Workspace setup** — a setup overlay appears.
   - Click **Start Freedrive** — the robot arm goes compliant (you can move it by hand).
   - Physically position the robot's TCP (tool tip) at three reference points on the drawing surface and click **Record** for each:
     - **P0** — the workspace origin (e.g. bottom-left corner of the paper)
     - **Px** — a point along the X direction (e.g. bottom-right corner)
     - **Py** — a point along the Y direction (e.g. top-left corner)
   - Click **Confirm Workspace** — the geometry is saved to `workspace.json` for all future sessions.

3. **Point the camera** at your subject so it covers the whole work surface.

4. **Capture Image** — click **Capture Image** to freeze the current frame. The live feeds are replaced by the captured still (left) and a processed preview (right).

5. **Crop & adjust** — in the **Adjust Image** panel:
   - **✨ Auto Touch-Up** — one click to reveal faint relief like grooves raked into sand. It robustly stretches contrast (1st–99th percentile), denoises grain with an edge-preserving bilateral filter, applies CLAHE local equalization, and sharpens the ridges, then auto-picks the Canny thresholds from the image. Use **Auto strength** to scale how aggressive it is, then fine-tune with the manual sliders on top. Raise **Blur** if grain still produces speckle edges.
   - Drag on the still to draw a **crop** rectangle (drag inside to move, corners to resize; **Reset Crop** restores the full frame). The crop selects which part of the workspace gets drawn — strokes stay positioned correctly within the calibrated area.
   - Tune **Exposure / Brightness / Contrast / Highlights / Shadows / Gamma**, toggle **Local contrast (CLAHE)** for faint texture, or **Invert** for light marks on a dark ground.
   - Tune **Blur** and **Canny low/high** to control edge sensitivity.
   - The right panel updates live — switch between **Edges** (what becomes the path) and **Adjusted** (the tuned image). Aim for clean white edges on the marking with little background noise.

6. **Generate Path** — click **Generate Path**. The 3D viewer shows the extracted path (green = pen-down strokes, grey = pen-up travel). Re-adjust and regenerate as many times as you like, or **Retake** to grab a fresh frame.

7. **Run** — click **Run**. The robot lifts, travels to the first stroke, and draws. A progress bar tracks execution. Click **Cancel** at any time to stop mid-stroke.

### Subsequent sessions

After the first workspace calibration, a **"Use This Workspace"** banner appears on connect. Click it to skip recalibration and start immediately.

---

## Configuration reference

All parameters live in `config.py`. Edit that file to tune the system for your setup.

### Server

| Variable | Default | Description |
|----------|---------|-------------|
| `HTTP_HOST` | `"localhost"` | Bind address |
| `HTTP_PORT` | `8080` | Web UI port |

### Camera

| Variable | Default | Description |
|----------|---------|-------------|
| `CAMERA_INDEX` | `0` | Device index (`0` = built-in / first camera) |
| `CAMERA_WIDTH` | `640` | Capture resolution width (px) |
| `CAMERA_HEIGHT` | `480` | Capture resolution height (px) |

### Canny edge detection

| Variable | Default | Description |
|----------|---------|-------------|
| `CANNY_BLUR_KERNEL` | `5` | Gaussian blur kernel size (must be odd) |
| `CANNY_THRESHOLD_LOW` | `50` | Lower hysteresis threshold |
| `CANNY_THRESHOLD_HIGH` | `150` | Upper hysteresis threshold |
| `CONTOUR_MIN_PIXELS` | `20` | Discard chains shorter than this many pixels |

### Path extraction

| Variable | Default | Description |
|----------|---------|-------------|
| `RESAMPLE_SPACING_MM` | `5.0` mm | Target spacing between resampled waypoints |

### Robot motion

| Variable | Default | Units | Description |
|----------|---------|-------|-------------|
| `DRAW_Z` | `-0.010` | m | Pen-contact Z offset below workspace surface origin |
| `TRAVEL_Z` | `0.050` | m | Pen-up travel height above workspace surface origin |
| `DRAW_SPEED` | `0.05` | m/s | Average drawing speed (with smoothstep, peak is 1.5×) |
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
camera-to-robot/
├── main.py                  # Entry point: shared state, callbacks, startup
├── config.py                # All configurable parameters
├── server.py                # aiohttp server: MJPEG feeds, WebSocket, static files
├── camera_thread.py         # Background camera capture → dual MJPEG streams
├── path_extractor.py        # Canny → pixel chains → smooth → resample → TSP → robot coords
├── path_executor.py         # Background thread: lift/travel/servoL per stroke, progress
├── robot_controller.py      # Thread-safe ur-rtde wrapper (moveL, servoL, freedrive, EE pose)
├── workspace.py             # 3-point workspace calibration; pixel ↔ robot coord mapping
├── settings.py              # Persistent JSON settings (last robot IP)
├── conftest.py              # Pytest shared fixtures
├── pytest.ini               # Test configuration
├── requirements.txt         # Production dependencies
├── requirements-dev.txt     # Development dependencies (pytest, coverage)
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

- **`CameraThread`** — continuously reads frames from `cv2.VideoCapture`, encodes two JPEG streams (raw + Canny), and writes them to `shared_state` under `_state_lock`. The MJPEG HTTP endpoints read from `shared_state` at ~30 fps.
- **`PathExecutor`** — runs the per-stroke lift/travel/draw sequence. Long-running RTDE calls (`moveL`, `servoL`) happen here so the WebSocket broadcast loop is never blocked.

All cross-thread communication goes through a single `shared_state: dict` protected by `state_lock: threading.Lock`. The aiohttp event loop offloads blocking calls via `loop.run_in_executor(None, fn)`.

### Workspace calibration

Three robot TCP positions define the drawing surface:

- **P0** — origin
- **Px** — a point along the camera's horizontal axis
- **Py** — a point along the camera's vertical axis

`WorkspaceConfig.from_points()` computes an orthonormal coordinate frame (origin, x-axis, y-axis, z-axis) and the extents in metres. Pixel coordinates are then mapped linearly: `world_x = (u / frame_width) × x_extent`, and the result is expressed in robot base-frame by `p = origin + wx·x̂ + wy·ŷ`.

### Smooth robot motion

Drawing strokes use `servoL` (UR RTDE servo mode) rather than `moveL`:

- Commands stream at **125 Hz** (8 ms timesteps)
- Position is advanced using **arc-length parameterisation** — the robot moves at `DRAW_SPEED` metres per second regardless of waypoint density
- **Smoothstep ease-in/out** (`f(α) = 3α² − 2α³`) is applied to the parameterisation: velocity is 0 at stroke start and end, peaking at 1.5× `DRAW_SPEED` at the midpoint. This eliminates abrupt jerks at pen-down and pen-up transitions.

---

## Development & testing

```bash
pip install -r requirements-dev.txt

# Unit tests (no hardware required)
pytest -q --ignore=tests/test_integration.py

# With coverage
pytest --cov=. --ignore=tests/test_integration.py

# Integration tests (requires webcam and robot)
export TEST_ROBOT_IP=192.168.1.100
pytest -m integration -v
```

The test suite has **89 unit tests** across four modules:

| Test file | What it covers |
|-----------|---------------|
| `test_path_extractor.py` | Canny pipeline, chain extraction, resampling, TSP ordering, coordinate mapping |
| `test_path_executor.py` | Stroke sequencing, servoL streaming, ease-in/out, progress tracking, cancel |
| `test_robot_controller.py` | RTDE port probe, connect/disconnect, motion commands, thread safety, freedrive |
| `test_camera_thread.py` | Frame capture, dual MJPEG encoding, lock separation |

All unit tests mock hardware (robot and camera) — no physical devices needed.

---

## References

- Robot communication based on [UR-hand-control](https://github.com/f-scotto/UR-hand-control)
- UR RTDE interface: [ur-rtde documentation](https://sdurobotics.gitlab.io/ur_rtde/)
- Canny edge detection: J. Canny, "A computational approach to edge detection," *IEEE TPAMI*, 1986
