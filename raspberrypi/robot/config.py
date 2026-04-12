import os
from dotenv import load_dotenv

load_dotenv()

# =============================== API CONFIGURATION ===============================
AUTH_TOKEN = os.getenv("AUTH_TOKEN", "ooops")  # Should be set in environment variables

# ================================ CAMERA SETTINGS ================================
# Camera resolution
CAMERA_SENSOR_WIDTH = 4608
CAMERA_SENSOR_HEIGHT = 2592
CAMERA_FOV_DEG = 120

# Camera performance
CAMERA_BUFFER_COUNT = 2
CAMERA_MAX_FPS = 60
CAMERA_MIN_FRAME_INTERVAL = 1.0 / CAMERA_MAX_FPS

# Frame size calculations
FRAME_WIDTH  = 1536
FRAME_HEIGHT = 864
FRAME_SIZE_B = FRAME_HEIGHT * FRAME_WIDTH * 3  # RGB888 format (3 bytes per pixel, no alpha)

# =========================== SERIAL COMMUNICATION ================================
# Teensy serial port settings
TEENSY_PORT = "/dev/ttyAMA0"
TEENSY_BAUD = 168000
TEENSY_TIMEOUT = 0.1

COMMUNICATION_LOOP_FREQUENCY = 120
COMMUNICATION_LOOP_PERIOD = 1.0 / COMMUNICATION_LOOP_FREQUENCY

# =========================== CONTROL LOOP SETTINGS ===============================
# Logic loop timing
LOGIC_LOOP_FREQUENCY = 120
LOGIC_LOOP_PERIOD = 1.0 / LOGIC_LOOP_FREQUENCY

# Idle mode settings
IDLE_SLEEP_DURATION = 0.1

# ============================ API SERVER SETTINGS ================================
# Video streaming
API_VIDEO_JPEG_QUALITY = 70  # JPEG quality (0-100, higher is better quality/larger size)
API_VIDEO_TARGET_FPS = 30

# Server settings
API_HOST = '0.0.0.0'
API_PORT = 5000

# ============================= LOGGING SETTINGS ==================================
LOG_LEVEL = "DEBUG"  # Log level: DEBUG, INFO, WARNING, ERROR, CRITICAL
# Log buffer
LOG_BUFFER_MAX_ENTRIES = 100

# ============================== MOTOR CONTROL ====================================
MOTOR_COUNT = 4

# Motor angles (degrees)
MOTOR_LOCATIONS = [135, 225, 315, 45]

# Motor speed limits
MOTOR_SPEED_MIN = -9999
MOTOR_SPEED_MAX = 9999

# Motor acceleration (logic loops required to go from 0 to max speed)
MOTOR_ACCELERATION_LOGIC_LOOPS = 4

# =========================== LINE SENSOR SETTINGS ================================
LINE_SENSOR_COUNT = 12
LINE_SENSOR_LOCATIONS = [i * (360 / LINE_SENSOR_COUNT) for i in range(LINE_SENSOR_COUNT)]

# Default values, can be calibrated/changed via API
DEFAULT_LINE_DETECTION_THRESHOLDS = [[400, 600]] * LINE_SENSOR_COUNT  # Array of [min, max] values

# ========================== CALIBRATION STORAGE ==================================
CALIBRATION_FILE_PATH = "calibration_data.json"

# ======================== OBJECT DETECTION SETTINGS ==============================
# Scale factor for processing frames (0.5 means half resolution)
DETECTION_FRAME_SIZE_SCALE = 0.5
DETECTION_FRAME_WIDTH = int(FRAME_WIDTH * DETECTION_FRAME_SIZE_SCALE)
DETECTION_FRAME_HEIGHT = int(FRAME_HEIGHT * DETECTION_FRAME_SIZE_SCALE)


# ========================= GOAL DETECTION SETTINGS ===============================
# Real-world goal height in millimeters
GOAL_HEIGHT_MM = 100.0
DEFAULT_FOCAL_LENGTH_PIXELS = 1000.0

# ============================== LOGIC SETTINGS ===================================
DEFAULT_LINE_AVOIDING_ENABLED = True
DEFAULT_ROTATION_CORRECTION_ENABLED = True

# ===================== AUTONOMOUS BEHAVIOUR SETTINGS =============================

# --- General ---
# Global speed multiplier for all autonomous movements (reduce for debugging)
AUTO_SPEED_MULTIPLIER = 1.0
# Angle offset to apply to the ball angle (degrees)
IR_BALL_ANGLE_OFFSET_DEG = -10.0

# --- Approach ---
# Forward speed component while approaching the ball
AUTO_APPROACH_SPEED = 0.7
# The ratio between the ball angle & distance and the angle that the robot should move in
# while approaching the ball. This value is multiplied with the IR ball angle and distance to
# calculate the movement angle for approaching. A higher value means the robot will move
# more sideways to approach the ball from the front.
# NOTE: The IR ball distance is reverted: 3000 means the ball is the closest, <3000 means
#       it's farther. This means that the robot will approach the ball more from behind
#       when it's closer.
AUTO_IR_BALL_APPROACH_ANGLE_RATIO = 0.001
# Similar ratio for camera-based ball tracking, using the camera ball angle instead of IR.
# The distance is ignored in this ratio
AUTO_CAM_BALL_APPROACH_ANGLE_RATIO = 1.3
# The threshold distance to consider the ball "close enough" to initiate pushing (3000 nearest, 0 farthest)
AUTO_BALL_CLOSE_THRESHOLD = 2500
# Angular window around 0° where ball is considered "in front" of the robot
AUTO_BALL_FRONT_THRESHOLD_DEG = 15.0

# --- Pushing ---
# Speed when pushing the ball toward the goal
AUTO_PUSH_SPEED = 1.0
# Maximum angle to apply for steering while pushing (deg, when ball is at edge of possession area)
AUTO_PUSH_STEERING_MAX_ANGLE_DEG = 45.0
# Distance at which we consider the goal "scored" (stop pushing)
AUTO_GOAL_SCORED_DISTANCE_MM = 600.0

# --- Position-based speed scaling ---
# When enabled, the robot uses goal distance to slow down near the enemy line
DEFAULT_POSITION_BASED_SPEED_ENABLED = True
# Minimum speed multiplier when close to any line (0.0 to 1.0)
AUTO_POSITION_SLOW_MIN_SPEED = 0.5
# Distance from center (x coordinate) at which to start slowing down (mm)
AUTO_POSITION_SLOW_START_DISTANCE_X_MM = 700.0
# Range where slow speed isn't applied (y coordinate, mm)
AUTO_POSITION_SLOW_START_DISTANCE_Y_MIN_MM = 300.0
AUTO_POSITION_SLOW_START_DISTANCE_Y_MAX_MM = 1900.0
# Apply lowest speed multiplier when distance from nearest slow speed start is >= this (mm)
AUTO_POSITION_SLOW_END_DISTANCE_MM = 200.0

# --- Ball possession camera check ---
# Ball possession area as percentage of frame dimensions
# % of frame width (centered horizontally)
AUTO_BALL_POSSESSION_AREA_WIDTH_PERCENT = 30.0
AUTO_BALL_POSSESSION_AREA_WIDTH_DEG = CAMERA_FOV_DEG * AUTO_BALL_POSSESSION_AREA_WIDTH_PERCENT / 100.0
# % of frame height (from bottom)
AUTO_BALL_POSSESSION_AREA_HEIGHT_PERCENT = 40.0
# Minimum fraction of orange pixels in possession area to consider the ball possessed
AUTO_BALL_POSSESSION_MIN_RATIO = 0.02
# Default HSV calibration for the ball (orange)
DEFAULT_BALL_CALIBRATION_HSV = [5, 150, 150, 25, 255, 255]  # [h_min,s_min,v_min, h_max,s_max,v_max]
# IR angle range considered "inside" the robot if the ball was possessed by camera in previous frame (deg)
AUTO_BALL_INSIDE_ROBOT_IR_ANGLE_RANGE_DEG = 40.0

# --- Camera-based ball tracking (for approach state) ---
# Enable using camera ball detection for approach instead of IR
AUTO_CAMERA_BALL_TRACKING_ENABLED = True
# Use camera detection as primary up to this distance (mm), then fall back to IR
AUTO_CAMERA_BALL_TRACKING_MAX_DISTANCE_MM = 1000.0
# Max allowed angle difference between camera and IR ball angles to use camera tracking
AUTO_CAMERA_BALL_TRACKING_MAX_IR_DIFF_DEG = 45.0

# --- Goal-tracking rotation ---
# Gain applied to goal alignment error to produce rotation command while approaching/pushing
AUTO_GOAL_TRACK_ROTATE_GAIN = 0.7
# Rotation speed used while searching for the goal (no goal visible)
AUTO_GOAL_SEARCH_ROTATE_SPEED = 1.0

