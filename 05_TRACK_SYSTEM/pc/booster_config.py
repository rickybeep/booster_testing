"""Sanitized defaults for the partner-facing V10 tracker subset.

Site-specific hosts, tag IDs, anchors, heading offsets, speed caps, and
waypoints live in ``config/tracker.local.json``.  ``arena_runner.py`` applies
those validated values at runtime.  This module intentionally contains no
production IP addresses, credentials, capture endpoints, or private names.
"""

# HTTP / robot feedback. ROBOT_HOSTS is intentionally empty: callers inject
# per-robot hosts from the local configuration.
ROBOT_HOSTS = ()
PANEL_PORT = 8080
PANEL_TIMEOUT_S = 0.35
NAV_STATE_HZ = 20
STATUS_POLL_HZ = 2
NAV_YAW_STALE_S = 0.65
MODE_STALE_S = 1.25

# MotionController ceilings match the V10 robot guard.
MAX_FORWARD = 0.20
MAX_TURN = 0.40
MAX_FORWARD_HARD = 0.40
MAX_TURN_HARD = 0.80
CONTROL_HZ = 20
DEADMAN_SECONDS = 0.90
K1_TURN_CMD_SIGN = -1
IMU_YAW_SIGN = 1
IMU_YAW_OFFSET_DEG = 0.0
K1_MODE_LABELS = {
    "damp": "Damping (limp!)",
    "prep": "Prepare (stand)",
    "walk": "Walking",
    "custom": "Custom",
}

# Simple head-scan limits used by the minimal partner runner.
HEAD_SLEW_RAD_S = 0.50
HEAD_YAW_LIM = 1.10
HEAD_PITCH_LIM = 0.30

# Arena and Navigator defaults. The example config uses an 18 ft square, but
# the runner replaces ARENA and SAFE_MARGIN_M from local validated values.
ARENA = (0.0, 0.0, 5.4864, 5.4864)
SAFE_MARGIN_M = 1.50
ARENA_MARGIN = SAFE_MARGIN_M
WAYPOINT_RADIUS = 0.50
REPULSE_MARGIN_M = SAFE_MARGIN_M + 0.40
HEADING_TOL_DEG = 12.0
FACE_TOL_DEG = 8.0
FORWARD_CONE_DEG = 80.0
SLOW_RADIUS = 0.80
MIN_TARGET_STEP = 1.0
TURN_KP = 1.4
TURN_SIGN = 1
ARRIVE_MIN_FRAC = 0.15
RECOVER_FORWARD_FRAC = 0.80
RECOVER_EXIT_M = 0.20
NEAR_ALIGN_ENTER_DEG = 45.0
NEAR_ALIGN_EXIT_DEG = 15.0
ARC_FULL_SPEED_DEG = 20.0
ARC_MAX_DEG = 70.0
ARC_MIN_SPEED_FRAC = 0.28

# Circular peer/static-obstacle planning and reactive layers.
ROBOT_AVOID_CLEAR_M = 1.50
ROBOT_AVOID_MARGIN_M = 1.80
ROBOT_AVOID_GAIN = 1.8
ROBOT_HOLD_DIST_M = 0.90
ROBOT_HOLD_HYST_M = 0.30
ROBOT_HOLD_FLOOR_M = 0.75
PLAN_DETOUR_MARGIN_M = 0.30
PLAN_CLEAR_HYST_M = 0.10
PLAN_MATCH_M = 0.75
PLAN_MAX_WAYPOINTS = 6
PEER_DISENGAGE_CONE_DEG = 45.0

# Required by Navigator's optional AUTO target helpers. The partner runner
# uses fixed waypoints, but keeping these values makes the vendored module
# complete and import-compatible.
AUTO_TARGET_PICK_ATTEMPTS = 200
AUTO_TARGET_REPICK_COOLDOWN_S = 2.0
AUTO_TARGET_RETRY_S = 1.5

# Legacy rectangular keepout is disabled. Static keepouts in the partner
# schema are circular obstacles so path planning, repulsion, and hold all see
# the same geometry.
KEEPOUT = None
KEEPOUT_BUFFER = 0.45
KEEPOUT_AVOID = 0.80
KEEPOUT_GAIN = 1.6
