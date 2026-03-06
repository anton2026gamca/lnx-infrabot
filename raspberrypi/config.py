# ==================== CAMERA SETTINGS ====================
# Camera resolution
CAMERA_SENSOR_WIDTH = 4608
CAMERA_SENSOR_HEIGHT = 2592
CAMERA_RESOLUTION_SCALE = 0.25  # 0.25 = 1152x648
CAMERA_FOV_DEG = 120

# Camera performance
CAMERA_BUFFER_COUNT = 2
CAMERA_MAX_FPS = 30
CAMERA_MIN_FRAME_INTERVAL = 1.0 / CAMERA_MAX_FPS

# Frame size calculations
FRAME_HEIGHT = int(CAMERA_SENSOR_HEIGHT * CAMERA_RESOLUTION_SCALE)
FRAME_WIDTH = int(CAMERA_SENSOR_WIDTH * CAMERA_RESOLUTION_SCALE)
FRAME_SIZE_B = FRAME_HEIGHT * FRAME_WIDTH * 3  # RGB888

# ==================== SERIAL COMMUNICATION ====================
# Teensy serial port settings
TEENSY_PORT = "/dev/ttyAMA0"
TEENSY_BAUD = 168000
TEENSY_TIMEOUT = 0.1

COMMUNICATION_LOOP_FREQUENCY = 120
COMMUNICATION_LOOP_PERIOD = 1.0 / COMMUNICATION_LOOP_FREQUENCY

# ==================== CONTROL LOOP SETTINGS ====================
# Logic loop timing
LOGIC_LOOP_FREQUENCY = 120
LOGIC_LOOP_PERIOD = 1.0 / LOGIC_LOOP_FREQUENCY

# Idle mode settings
IDLE_SLEEP_DURATION = 0.1

# ==================== API SERVER SETTINGS ====================
# Video streaming
API_VIDEO_JPEG_QUALITY = 60  # 0-100
API_VIDEO_TARGET_FPS = 30

# Server settings
API_HOST = '0.0.0.0'
API_PORT = 5000

# ==================== LOGGING SETTINGS ====================
LOG_LEVEL = "DEBUG"  # DEBUG, INFO, WARNING, ERROR, CRITICAL
# Log buffer
LOG_BUFFER_MAX_ENTRIES = 100

# ==================== MOTOR CONTROL ====================
MOTOR_COUNT = 4

# Motor angles (degrees)
MOTOR_LOCATIONS = [135, 225, 315, 45]

# Motor speed limits
MOTOR_SPEED_MIN = -9999
MOTOR_SPEED_MAX = 9999

# ==================== LINE SENSOR SETTINGS ====================
LINE_SENSOR_COUNT = 12
LINE_SENSOR_LOCATIONS = [i * (360 / LINE_SENSOR_COUNT) for i in range(LINE_SENSOR_COUNT)]

# Default values, can be calibrated/changed via api
DEFAULT_LINE_DETECTION_THRESHOLDS = [[400, 600]] * LINE_SENSOR_COUNT # Array of [min, max] values

# ==================== CALIBRATION STORAGE ====================
CALIBRATION_FILE_PATH = "calibration_data.json"

# ==================== GOAL DETECTION SETTINGS ====================
# Real-world goal height in millimeters
GOAL_HEIGHT_MM = 100.0
DEFAULT_FOCAL_LENGTH_PIXELS = 1000.0

# ==================== LOGIC SETTINGS ====================
DEFAULT_LINE_AVOIDING_ENABLED = False
DEFAULT_ROTATION_CORRECTION_ENABLED = True

# ==================== AUTONOMOUS BEHAVIOUR SETTINGS ====================

# --- General ---
# Global speed multiplier for all autonomous movements (reduce for debugging)
AUTO_SPEED_MULTIPLIER = 1.0
# Angle offset to apply to the ball angle (degrees)
AUTO_BALL_ANGLE_OFFSET_DEG = -10.0

# --- Get-behind-ball (ORBIT state) ---
# If abs(ball_angle - 180) > this threshold the robot needs to go around the ball
# to approach from behind (so that forward = toward goal).
AUTO_BEHIND_BALL_THRESHOLD_DEG = 90.0
# Arc speed while orbiting around the ball to get behind it
AUTO_ORBIT_SPEED = 0.5

# --- Approach ---
# Forward speed component while approaching the ball
AUTO_APPROACH_SPEED = 0.6
# The ratio between the ball angle and the angle that the robot should move in while approaching the ball.
AUTO_BALL_TO_ROBOT_ANGLE_TIMES_DISTANCE_RATIO = 2.5 / 2500.0
# How far away (IR distance units) counts as "ball is close enough to align"
AUTO_BALL_CLOSE_THRESHOLD = 2800
# Angular window around 0° where ball is considered "in front" of the robot
AUTO_BALL_FRONT_THRESHOLD_DEG = 10.0

# --- Goal alignment (rotate in place) ---
# Rotation gain while aligning goal to center
AUTO_ALIGN_ROTATE_GAIN = 0.6

# --- Pushing ---
# Speed when pushing the ball toward the goal
AUTO_PUSH_SPEED = 1.0
# Distance at which we consider the goal "scored" (stop pushing)
AUTO_GOAL_SCORED_DISTANCE_MM = 1000.0

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
# Apply lowest speed multiplier when distance from the nearest slow speed start is equal or greater to this (mm)
AUTO_POSITION_SLOW_END_DISTANCE_MM = 200.0

# --- Line avoidance ---
# How long to back away from a line (seconds)
AUTO_LINE_REVERSE_DURATION = 0.4
# Retreat speed when a line is detected
AUTO_LINE_REVERSE_SPEED = 0.8

# --- Ball possession camera check ---
# Height of the bottom strip (in pixels) to scan for ball (orange) pixels
AUTO_BALL_BOTTOM_STRIP_HEIGHT = 30
# Minimum fraction of orange pixels in strip to consider the ball possessed
AUTO_BALL_POSSESSION_MIN_RATIO = 0.03
# Default HSV calibration for the ball (orange)
DEFAULT_BALL_CALIBRATION_HSV = [5, 150, 150, 25, 255, 255]  # [h_min,s_min,v_min, h_max,s_max,v_max]

# --- Goal-tracking rotation ---
# Gain applied to goal alignment error to produce rotation command while approaching/pushing
AUTO_GOAL_TRACK_ROTATE_GAIN = 0.7
# Rotation speed used while searching for the goal (no goal visible)
AUTO_GOAL_SEARCH_ROTATE_SPEED = 0.25

