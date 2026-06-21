import math
from pathlib import Path

# ── Server ────────────────────────────────────────────────────────────────────
HTTP_HOST             = "localhost"
HTTP_PORT             = 8080
CAMERA_RAW_PATH       = "/camera"
CAMERA_PROCESSED_PATH = "/camera/processed"
WS_PATH               = "/ws"
STATIC_PATH           = "/static"

# ── Persistent files ──────────────────────────────────────────────────────────
WORKSPACE_FILE = Path("workspace.json")
SETTINGS_FILE  = Path("settings.json")

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

# ── Camera ────────────────────────────────────────────────────────────────────
CAMERA_INDEX  = 0      # 0 = built-in laptop webcam (always index 0 on macOS)
CAMERA_WIDTH  = 640
CAMERA_HEIGHT = 480

# ── Canny edge detection ──────────────────────────────────────────────────────
CANNY_BLUR_KERNEL    = 5    # Gaussian blur kernel size (must be odd)
CANNY_THRESHOLD_LOW  = 50
CANNY_THRESHOLD_HIGH = 150

# ── Path extraction ───────────────────────────────────────────────────────────
CONTOUR_MIN_PIXELS  = 20   # discard contours shorter than this many pixels
RESAMPLE_SPACING_MM = 5.0  # target spacing between robot waypoints in mm

# ── Robot drawing ─────────────────────────────────────────────────────────────
DRAW_Z           = -0.010  # m — pen contact (negative = below workspace surface origin)
TRAVEL_Z         =  0.050  # m — pen-up travel height above workspace surface origin
DRAW_SPEED       = 0.05    # m/s during drawing strokes
DRAW_ACCEL       = 0.3     # m/s²
TRAVEL_SPEED     = 0.15    # m/s during pen-up travel moves
TRAVEL_ACCEL     = 0.5     # m/s²
TOOL_ORIENTATION = [0.0, math.pi, 0.0]  # tool-down [rx, ry, rz]

# ── Visualization ─────────────────────────────────────────────────────────────
VIS_INTERVAL = 0.05  # seconds between WebSocket state broadcasts
