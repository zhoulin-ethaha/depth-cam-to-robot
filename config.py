import math
from pathlib import Path

# ── Server ────────────────────────────────────────────────────────────────────
# Port 5005 is deliberately off the common 8080/8000 range so this depth app can
# run alongside other tools (TouchDesigner, earlier RGB prototypes, etc.) without
# a port clash.
HTTP_HOST    = "localhost"
HTTP_PORT    = 5005
DEPTH_PATH   = "/depth"             # MJPEG: colorized depth (the live view)
RGB_PATH     = "/rgb"               # MJPEG: aligned colour image
GROOVE_PATH  = "/depth/grooves"     # MJPEG: detected groove centrelines (skeleton)
MASK_PATH    = "/depth/mask"        # MJPEG: thick detected-region mask
WS_PATH      = "/ws"
STATIC_PATH  = "/static"

# ── Persistent files ──────────────────────────────────────────────────────────
WORKSPACE_FILE = Path("workspace.json")
SETTINGS_FILE  = Path("settings.json")

# ── Target surface (Rhino mesh export) ────────────────────────────────────────
# STL/OBJ meshes uploaded from the browser land here. Rhino exports are usually
# in millimetres, so files are scaled by SURFACE_UNITS_TO_M on load (set to 1.0
# if you export in metres).
SURFACE_DIR        = Path("surfaces")
SURFACE_UPLOAD_URL = "/surface/upload"
SURFACE_UNITS_TO_M = 0.001
SURFACE_MAX_FACES  = 80000   # warn above this — browser preview gets heavy

# ── Robot start position (joint angles in radians) ───────────────────────────
START_JOINT_ANGLES = [0.0, -math.pi / 2, math.pi / 2, -math.pi / 2, -math.pi / 2, 0.0]
START_SPEED = 0.3
START_ACCEL = 0.5

# ── RTDE servo parameters ─────────────────────────────────────────────────────
RTDE_FREQUENCY       = 125
SERVO_LOOKAHEAD_TIME = 0.1
SERVO_GAIN           = 300
SERVO_VELOCITY_DEFAULT = 0.10
SERVO_ACCELERATION   = 0.5

# ── Depth camera (Intel RealSense D435i) ──────────────────────────────────────
# The D435i streams metric depth directly, so grooves raked into sand — a few-mm
# physical depression invisible to an RGB camera — are measured, not inferred
# from shading. We keep the raw depth internally and only colorize it for display.
DEPTH_WIDTH          = 640
DEPTH_HEIGHT         = 480
DEPTH_FPS            = 30
DEPTH_AVERAGE_FRAMES = 30     # frames temporally averaged on Capture (cuts noise ~√n)

# Colormap range (metres) for the live depth view. 0 = auto (per-frame percentile).
DEPTH_COLOR_NEAR_M = 0.0
DEPTH_COLOR_FAR_M  = 0.0

# ── Groove detection (depth → groove centrelines) ─────────────────────────────
# See depth_extractor.grooves_from_depth for the algorithm. These are the live-feed
# defaults and the initial values of the browser's Adjust controls.
GROOVE_SMOOTH_SIGMA_PX  = 1.5    # denoise the depth map first
GROOVE_DETREND_SIGMA_PX = 25.0   # blur radius estimating the bare-sand surface
GROOVE_DEPTH_MM         = 1.5    # how much deeper than the surface counts as a groove
GROOVE_MIN_BLOB_PX      = 40     # discard connected specks smaller than this
GROOVE_DETECT           = "valley"  # "valley"=grooves, "ridge"=raised lines, "band"=iso-depth

# ── Path extraction ───────────────────────────────────────────────────────────
CONTOUR_MIN_PIXELS  = 20   # discard contours shorter than this many pixels
RESAMPLE_SPACING_MM = 5.0  # target spacing between robot waypoints in mm

# ── Robot drawing ─────────────────────────────────────────────────────────────
DRAW_Z           = -0.010  # m — pen contact (negative = below workspace surface origin)
TRAVEL_Z         =  0.050  # m — pen-up travel height above workspace surface origin
DRAW_SPEED       = 0.05    # m/s during drawing strokes (default; UI Speed slider overrides)
MAX_TCP_SPEED    = 1.0     # m/s — 100% on the Speed slider (UR10e rated max tool speed)

# ── Reach estimate (UR10e) ────────────────────────────────────────────────────
# Rough reachability envelope used to warn about waypoints the arm cannot get
# to: a sphere of REACH_M around the base, minus a thin inner cylinder around
# the base axis where the wrist cannot fold in. An estimate — not full IK.
UR_REACH_M     = 1.30
UR_MIN_REACH_M = 0.18
DRAW_ACCEL       = 0.3     # m/s²
TRAVEL_SPEED     = 0.15    # m/s during pen-up travel moves
TRAVEL_ACCEL     = 0.5     # m/s²
TOOL_ORIENTATION = [0.0, math.pi, 0.0]  # tool-down [rx, ry, rz]

# ── Visualization ─────────────────────────────────────────────────────────────
VIS_INTERVAL = 0.05  # seconds between WebSocket state broadcasts
