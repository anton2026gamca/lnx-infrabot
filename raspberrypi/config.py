# ==================== CAMERA SETTINGS ====================
# Camera resolution
CAMERA_SENSOR_WIDTH = 4608
CAMERA_SENSOR_HEIGHT = 2592
CAMERA_RESOLUTION_SCALE = 0.25  # 0.25 = 1152x648

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
TEENSY_BAUD = 38400
TEENSY_TIMEOUT = 0.1

# ==================== CONTROL LOOP SETTINGS ====================
# Logic loop timing
LOGIC_LOOP_FREQUENCY = 60
LOGIC_LOOP_PERIOD = 1.0 / LOGIC_LOOP_FREQUENCY

# Idle mode settings
IDLE_SLEEP_DURATION = 0.1

# ==================== API SERVER SETTINGS ====================
# Video streaming
API_VIDEO_JPEG_QUALITY = 85  # 0-100
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
MOTOR_LOCATIONS = [45, 135, 225, 315]

# Motor speed limits
MOTOR_SPEED_MIN = -9999
MOTOR_SPEED_MAX = 9999
