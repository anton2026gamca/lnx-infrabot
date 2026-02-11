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
# Log buffer
LOG_BUFFER_MAX_ENTRIES = 100
LOG_LEVEL = "DEBUG"  # DEBUG, INFO, WARNING, ERROR, CRITICAL

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
LINE_DETECTION_THRESHOLDS = [[400, 600]] * LINE_SENSOR_COUNT # Array of [min, max] values

# ==================== CALIBRATION STORAGE ====================
CALIBRATION_FILE_PATH = "calibration_data.json"

# ==================== GOAL DETECTION SETTINGS ====================
# Real-world goal height in millimeters
GOAL_HEIGHT_MM = 100.0
DEFAULT_FOCAL_LENGTH_PIXELS = 1000.0
