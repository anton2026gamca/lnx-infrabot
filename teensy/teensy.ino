#include <i2c_driver.h>
#include "imx_rt1060/imx_rt1060_i2c_driver.h"
#include <math.h>
#include <string.h>


// ================================================
// =                Definitions                   =
// ================================================


struct Button {
  int pin;
  int lastState;
  int state;
  unsigned long lastDebounceTime;
};

enum DebugLevel {
  DEBUG_INFO,
  DEBUG_WARN,
  DEBUG_ERROR
};


#define TARGET_UPS 120
#define INTERVAL_US (1000000 / TARGET_UPS)


// ========== Control Switches ==========
#define MAIN_SWITCH_PIN 31
#define MAIN_SWITCH_LED_PIN 28
#define MODULE_SWITCH_PIN 32
#define MODULE_SWITCH_LED_PIN 29
#define MODULE_PIN 33
#define MODULE_LED_PIN 30

volatile bool module_value = true;
volatile bool is_running = false;

Button main_switch = {MAIN_SWITCH_PIN, HIGH, HIGH, 0};
bool main_switch_enabled = false;
Button module_switch = {MODULE_SWITCH_PIN, HIGH, HIGH, 0};
bool bt_module_enabled = true;


// ========== Serial Communication ==========
#define RASPBERRY_SERIAL Serial8
#define RASPBERRY_SERIAL_SPEED 230400

#define RASPBERRY_SERIAL_BUFFER_SIZE 128
char message_buffer[RASPBERRY_SERIAL_BUFFER_SIZE];

#define SENSOR_DATA_MESSAGE_LENGTH 43


#define DEBUG_PRINTS_ENABLED false
#define DEBUG_LOGS_ENABLED false
#define DEBUG_PROFILING_ENABLED true
#define DEBUG_PROFILING_PRINT_INTERVAL_S 1
#define DEBUG_SERIAL Serial
#define DEBUG_SERIAL_SPEED 38400

#if DEBUG_PRINTS_ENABLED
  template<typename T>
  inline void DEBUG_PRINT(const T& msg) {
    DEBUG_SERIAL.print(msg);
  }

  template<typename T>
  inline void DEBUG_PRINTLN(const T& msg) {
    DEBUG_SERIAL.println(msg);
  }

  inline void DEBUG_PRINTLN() {
    DEBUG_SERIAL.println();
  }
#else
  template<typename T> inline void DEBUG_PRINT(const T&) {}
  template<typename T> inline void DEBUG_PRINTLN(const T&) {}
  inline void DEBUG_PRINTLN() {}
#endif

#if DEBUG_LOGS_ENABLED
  template<typename T>
  inline void DEBUG_LOG(DebugLevel level, const T& msg) {
    switch (level) {
      case DEBUG_INFO:  DEBUG_SERIAL.print("[INFO]  "); break;
      case DEBUG_WARN:  DEBUG_SERIAL.print("[WARN]  "); break;
      case DEBUG_ERROR: DEBUG_SERIAL.print("[ERROR] "); break;
    }
    DEBUG_SERIAL.println(msg);
  }
#else
  template<typename T> inline void DEBUG_LOG(DebugLevel, const T&) {}
#endif

#if DEBUG_PROFILING_ENABLED
  struct TimingStat {
    unsigned long duration_sum = 0;
    unsigned long duration_min = UINT32_MAX;
    unsigned long duration_max = 0;
    
    unsigned long interval_sum = 0;
    unsigned long interval_min = UINT32_MAX;
    unsigned long interval_max = 0;
    
    unsigned long call_count = 0;
    unsigned long last_call_time = 0;
  };

  const int MAX_PROFILING_POINTS = 20;
  struct ProfilingData {
    const char* names[MAX_PROFILING_POINTS];
    TimingStat stats[MAX_PROFILING_POINTS];
    int count = 0;
  };
  
  ProfilingData profiling_data;
  static unsigned long last_timing_print = 0;

  int get_or_create_profiling_point(const char* name) {
    for (int i = 0; i < profiling_data.count; i++) {
      if (profiling_data.names[i] == name) {
        return i;
      }
    }
    if (profiling_data.count < MAX_PROFILING_POINTS) {
      profiling_data.names[profiling_data.count] = name;
      return profiling_data.count++;
    }
    return -1;
  }

  inline void DEBUG_PROFILING_START(unsigned long& checkpoint) {
    checkpoint = micros();
  }

  inline void DEBUG_PROFILING_RECORD(unsigned long checkpoint, const char* label) {
    unsigned long elapsed = micros() - checkpoint;
    int idx = get_or_create_profiling_point(label);
    if (idx >= 0) {
      TimingStat& stat = profiling_data.stats[idx];
      stat.duration_sum += elapsed;
      if (elapsed < stat.duration_min) stat.duration_min = elapsed;
      if (elapsed > stat.duration_max) stat.duration_max = elapsed;
      
      unsigned long now = micros();
      if (stat.last_call_time > 0) {
        unsigned long interval = now - stat.last_call_time;
        stat.interval_sum += interval;
        if (interval < stat.interval_min) stat.interval_min = interval;
        if (interval > stat.interval_max) stat.interval_max = interval;
      }
      stat.last_call_time = now;
      stat.call_count++;
    }
  }

  void print_profiling_value(unsigned long value, int width, bool is_null) {
    if (is_null) {
      DEBUG_SERIAL.print("");
      for (int j = 0; j < width; j++) DEBUG_SERIAL.print(" ");
    } else {
      String val_str = String(value);
      DEBUG_SERIAL.print(val_str);
      for (int j = val_str.length(); j < width; j++) DEBUG_SERIAL.print(" ");
    }
  }

  void DEBUG_PROFILING_PRINT_IF_READY() {
    unsigned long current_time = millis();
    if (current_time - last_timing_print >= DEBUG_PROFILING_PRINT_INTERVAL_S * 1000) {
      last_timing_print = current_time;
      
      DEBUG_SERIAL.println();
      DEBUG_SERIAL.println("┌──────────────────────────┬────────────────────────────────┬────────────────────────────────┬─────────┐");
      DEBUG_SERIAL.println("│ Function                 │ Duration (µs)                  │ Interval (µs)                  │ Calls/s │");
      DEBUG_SERIAL.println("│                          │ Avg      Min      Max          │ Avg      Min      Max          │         │");
      DEBUG_SERIAL.println("├──────────────────────────┼────────────────────────────────┼────────────────────────────────┼─────────┤");
      
      for (int i = 0; i < profiling_data.count; i++) {
        TimingStat& stat = profiling_data.stats[i];
        if (stat.call_count > 0) {
          bool has_duration_data = (stat.duration_min != UINT32_MAX);
          bool has_interval_data = (stat.call_count > 1);
          
          DEBUG_SERIAL.print("│ ");
          DEBUG_SERIAL.print(profiling_data.names[i]);
          int name_len = strlen(profiling_data.names[i]);
          for (int j = name_len; j < 24; j++) DEBUG_SERIAL.print(" ");
          
          DEBUG_SERIAL.print(" │ ");
          if (has_duration_data) {
            unsigned long duration_avg = stat.duration_sum / stat.call_count;
            print_profiling_value(duration_avg, 8, false);
            DEBUG_SERIAL.print(" ");
            print_profiling_value(stat.duration_min, 8, false);
            DEBUG_SERIAL.print(" ");
            print_profiling_value(stat.duration_max, 9, false);
          } else {
            print_profiling_value(0, 8, true);
            DEBUG_SERIAL.print(" ");
            print_profiling_value(0, 8, true);
            DEBUG_SERIAL.print(" ");
            print_profiling_value(0, 9, true);
          }
          
          DEBUG_SERIAL.print("    │ ");
          if (has_interval_data) {
            unsigned long interval_avg = stat.interval_sum / (stat.call_count - 1);
            print_profiling_value(interval_avg, 8, false);
            DEBUG_SERIAL.print(" ");
            print_profiling_value(stat.interval_min, 8, false);
            DEBUG_SERIAL.print(" ");
            print_profiling_value(stat.interval_max, 9, false);
          } else {
            print_profiling_value(0, 8, true);
            DEBUG_SERIAL.print(" ");
            print_profiling_value(0, 8, true);
            DEBUG_SERIAL.print(" ");
            print_profiling_value(0, 9, true);
          }
          
          DEBUG_SERIAL.print("    │ ");
          if (has_interval_data) {
            unsigned long interval_avg = stat.interval_sum / (stat.call_count - 1);
            unsigned long calls_per_sec = (interval_avg > 0) ? (1000000 / interval_avg) : 0;
            String cps_str = String(calls_per_sec);
            DEBUG_SERIAL.print(cps_str);
            for (int j = cps_str.length(); j < 6; j++) DEBUG_SERIAL.print(" ");
          } else {
            DEBUG_SERIAL.print("      ");
          }
          
          DEBUG_SERIAL.println("  │");
        }
      }
      
      DEBUG_SERIAL.println("└──────────────────────────┴────────────────────────────────┴────────────────────────────────┴─────────┘");
      
      for (int i = 0; i < profiling_data.count; i++) {
        profiling_data.stats[i] = {0, UINT32_MAX, 0, 0, UINT32_MAX, 0, 0, 0};
      }
    }
  }
#else
  struct TimingStat {};
  inline void DEBUG_PROFILING_START(unsigned long&) {}
  inline void DEBUG_PROFILING_RECORD(unsigned long, const char*) {}
  void DEBUG_PROFILING_PRINT_IF_READY() {}
#endif


// ========== Motors Configuration ==========
#define MOTOR_COUNT 4
#define PWM_INDEX 0
#define DIR_INDEX 1
#define MOTOR_STOPPED 0
#define KICKER_OUT 1
#define KICKER_IN 0

// [PWM_PIN, DIR_PIN]
const int motor_pin_config[MOTOR_COUNT][2] = {
  { 6, 5 },
  { 8, 7 },
  { 10, 9 },
  { 12, 11 },
};
const int kicker_pin = 4;

struct MotorsData {
  int16_t motor_speed[MOTOR_COUNT];
  int8_t kicker_position;
};

MotorsData motors_data = { { 0, 0, 0, 0 }, KICKER_IN };


// ========== BNO055 Compass Sensor ==========
#define BNO055_I2C_ADDRESS 0x28
#define BNO055_CHIP_ID 0xA0
#define BNO055_CHIP_ID_ADDR 0x00
#define BNO055_EULER_H_LSB_ADDR 0x1A
#define BNO055_OPR_MODE_ADDR 0x3D
#define BNO055_PWR_MODE_ADDR 0x3E
#define BNO055_SYS_TRIGGER_ADDR 0x3F
#define BNO055_PAGE_ID_ADDR 0x07
#define BNO055_OPR_MODE_CONFIG 0x00
#define BNO055_OPR_MODE_NDOF 0x08
#define BNO055_POWER_MODE_NORMAL 0x00

I2CMaster& i2c_master = Master;
bool bno_initialized = false;
unsigned long bno_next_retry_at_ms = 0;
unsigned long i2c_req_checkpoint;

struct CompassData {
  float heading;
  float pitch;
  float roll;
};
CompassData compass_data = { 0.0, 0.0, 0.0 };


// ========== MRM-IR-Finder3 Sensor ==========
#define IR_I2C_ADDRESS 0x10
#define IR_ANGLE_AND_DISTANCE_REGISTER 0x00
#define IR_ANGLE_AND_DISTANCE_BYTES_COUNT 4
#define IR_RAW_REGISTER 0x01
#define IR_RAW_BYTES_COUNT 12
#define IR_SENSOR_COUNT 12
#define IR_NO_SIGNAL 999
#define IR_NO_SIGNAL_ANGLE 180
#define IR_NO_SIGNAL_DISTANCE 0

struct IRData {
  uint16_t angle;
  uint16_t distance;
  uint8_t sensor_IR[IR_SENSOR_COUNT];
};
IRData ir_data = { IR_NO_SIGNAL, 0, { 0 } };


// ========== I2C State Machine ==========
#define I2C_ASYNC_TIMEOUT_MS 4
#define I2C_BNO_RETRY_DEBOUNCE_MS 500
#define I2C_BNO_RESET_DELAY_MS 700
#define I2C_BNO_MODE_SWITCH_DELAY_MS 30

enum AsyncI2CState {
  I2C_STATE_WAIT_FOR_REQUEST = 0,
  I2C_STATE_BNO_CHECK_ID_SET_REGISTER,
  I2C_STATE_BNO_CHECK_ID_READ,
  I2C_STATE_BNO_SET_CONFIG_MODE,
  I2C_STATE_BNO_RESET,
  I2C_STATE_BNO_WAIT_AFTER_RESET,
  I2C_STATE_BNO_WAIT_ID_SET_REGISTER,
  I2C_STATE_BNO_WAIT_ID_READ,
  I2C_STATE_BNO_SET_POWER_MODE,
  I2C_STATE_BNO_SET_PAGE,
  I2C_STATE_BNO_SET_SYS_TRIGGER,
  I2C_STATE_BNO_SET_IMUPLUS_MODE,
  I2C_STATE_BNO_WAIT_AFTER_OP_MODE,
  I2C_STATE_IR_SET_ANGLE_REGISTER,
  I2C_STATE_IR_READ_ANGLE,
  I2C_STATE_IR_SET_RAW_REGISTER,
  I2C_STATE_IR_READ_RAW,
  I2C_STATE_BNO_SET_EULER_REGISTER,
  I2C_STATE_BNO_READ_EULER
};

enum AsyncI2CReadKind {
  I2C_READ_NONE = 0,
  I2C_READ_BNO_ID,
  I2C_READ_IR_ANGLE_DISTANCE,
  I2C_READ_IR_RAW,
  I2C_READ_BNO_EULER
};

AsyncI2CState i2c_state = I2C_STATE_BNO_CHECK_ID_SET_REGISTER;
AsyncI2CState i2c_next_state = I2C_STATE_BNO_CHECK_ID_SET_REGISTER;
AsyncI2CReadKind i2c_read_kind = I2C_READ_NONE;
bool i2c_op_in_progress = false;
unsigned long i2c_op_started_at_ms = 0;
unsigned long i2c_state_started_at_ms = 0;
uint8_t i2c_tx_byte = 0;
uint8_t i2c_tx_buffer[2] = { 0, 0 };
uint8_t bno_chip_id_buffer[1] = { 0 };
uint8_t bno_euler_buffer[6] = { 0 };
uint8_t ir_angle_distance_buffer[IR_ANGLE_AND_DISTANCE_BYTES_COUNT] = { 0 };
uint8_t ir_raw_buffer[IR_RAW_BYTES_COUNT] = { 0 };

bool compass_read_requested = false;
bool ir_read_requested = false;
bool ir_read_completed = false;
bool compass_read_completed = false;


// ========== Line Sensors ==========
#define LINE_SENSOR_COUNT 12

const int line_sensors_pin_config[LINE_SENSOR_COUNT] = {
  23, 22, 21, 20,
  17, 16, 15, 14,
  41, 40, 39, 38,
};

struct LineData {
  uint16_t sensor_line[LINE_SENSOR_COUNT];
  uint16_t sensor_line_min[LINE_SENSOR_COUNT];
  uint16_t sensor_line_max[LINE_SENSOR_COUNT];
};
LineData line_data = { { 0 } };


// ========== Data Collection ==========
struct SensorData {
  MotorsData motors_data;
  CompassData compass_data;
  IRData ir_data;
  LineData line_data;
};

SensorData sensor_data;



// ================================================
// =                 Functions                    =
// ================================================

// ========== Button Functions ==========
bool isButtonPressed(Button &b) {
  int currentReading = digitalRead(b.pin);
  bool clicked = false;
  const unsigned long debounceDelay = 50;

  if (currentReading != b.lastState) {
    b.lastDebounceTime = millis();
  }

  if ((millis() - b.lastDebounceTime) > debounceDelay) {
    if (currentReading == LOW && b.state == HIGH) {
      clicked = true;
    }
    b.state = currentReading;
  }

  b.lastState = currentReading;
  return clicked;
}

// ========== Control State Management ==========
void update_running_state() {
  module_value = bt_module_enabled ? (digitalRead(MODULE_PIN) == HIGH) : true;
  is_running = (main_switch_enabled && module_value);
}

// ========== Debug Functions ==========
void print_sensor_debug_info() {
  // Motor status
  DEBUG_PRINT("Motors: ");
  for (int i = 0; i < MOTOR_COUNT; i++) {
    DEBUG_PRINT("M");
    DEBUG_PRINT(i);
    DEBUG_PRINT("=");
    DEBUG_PRINT(sensor_data.motors_data.motor_speed[i]);
    DEBUG_PRINT(" ");
  }
  DEBUG_PRINT(" | Kicker=");
  DEBUG_PRINTLN(sensor_data.motors_data.kicker_position);

  // Compass orientation
  DEBUG_PRINT("Compass: H=");
  DEBUG_PRINT(sensor_data.compass_data.heading);
  DEBUG_PRINT(" P=");
  DEBUG_PRINT(sensor_data.compass_data.pitch);
  DEBUG_PRINT(" R=");
  DEBUG_PRINTLN(sensor_data.compass_data.roll);

  // IR sensor ball detection
  DEBUG_PRINT("IR Ball: Angle=");
  DEBUG_PRINT(sensor_data.ir_data.angle);
  DEBUG_PRINT(" Dist=");
  DEBUG_PRINT(sensor_data.ir_data.distance);
  DEBUG_PRINT("cm | Sensors: ");
  for (int i = 0; i < IR_SENSOR_COUNT; i++) {
    DEBUG_PRINT(sensor_data.ir_data.sensor_IR[i]);
    DEBUG_PRINT(" ");
  }
  DEBUG_PRINTLN();

  // Line sensors
  DEBUG_PRINT("Line: Value: ");
  for (int i = 0; i < LINE_SENSOR_COUNT; i++) {
    DEBUG_PRINT(sensor_data.line_data.sensor_line[i]);
    DEBUG_PRINT(" ");
  }
  DEBUG_PRINT("Min: ");
  for (int i = 0; i < LINE_SENSOR_COUNT; i++) {
    DEBUG_PRINT(sensor_data.line_data.sensor_line_min[i]);
    DEBUG_PRINT(" ");
  }
  DEBUG_PRINT("Max: ");
  for (int i = 0; i < LINE_SENSOR_COUNT; i++) {
    DEBUG_PRINT(sensor_data.line_data.sensor_line_max[i]);
    DEBUG_PRINT(" ");
  }
  DEBUG_PRINTLN();
  DEBUG_PRINTLN("========================================");
}

// ========== Motor Control Functions ==========
void set_motor_speed(int motor_ID, int speed) {
  if (motor_ID < 0 || motor_ID >= MOTOR_COUNT) return;

  int pwm = constrain(abs(speed), 0, 255);
  digitalWrite(motor_pin_config[motor_ID][DIR_INDEX], (speed >= 0) ? HIGH : LOW);
  analogWrite(motor_pin_config[motor_ID][PWM_INDEX], pwm);
}

void set_all_motors_speed(int16_t m_speed[MOTOR_COUNT]) {
  for (int i = 0; i < MOTOR_COUNT; i++) {
    set_motor_speed(i, m_speed[i]);
    motors_data.motor_speed[i] = m_speed[i];
  }
}

void set_kicker_position(int8_t position) {
  position = constrain(position, KICKER_IN, KICKER_OUT);
  digitalWrite(kicker_pin, position);
  motors_data.kicker_position = position;
}

void stop_motors() {
  for (int i = 0; i < MOTOR_COUNT; i++) {
    set_motor_speed(i, MOTOR_STOPPED);
    motors_data.motor_speed[i] = MOTOR_STOPPED;
  }
  set_kicker_position(KICKER_IN);
}

// ========== Sensor Reading Functions ==========
inline void set_compass_invalid() {
  if (bno_initialized) {
    bno_next_retry_at_ms = millis() + (
      i2c_state == I2C_STATE_BNO_SET_EULER_REGISTER ||
      i2c_state == I2C_STATE_BNO_READ_EULER
      ? I2C_BNO_RETRY_DEBOUNCE_MS
      : 0
    );
    bno_initialized = false;
  }

  compass_data.heading = 999;
  compass_data.pitch = 999;
  compass_data.roll = 999;
}

inline void set_ir_no_signal_data() {
  ir_data.angle = IR_NO_SIGNAL;
  ir_data.distance = 0;
  for (int i = 0; i < IR_SENSOR_COUNT; i++) {
    ir_data.sensor_IR[i] = 0;
  }
}

inline void reset_i2c_state_machine() {
  DEBUG_LOG(DEBUG_INFO, "Resetting State Machine");
  i2c_op_in_progress = false;
  i2c_read_kind = I2C_READ_NONE;
  i2c_state = I2C_STATE_WAIT_FOR_REQUEST;
  i2c_next_state = I2C_STATE_WAIT_FOR_REQUEST;
  i2c_state_started_at_ms = 0;
}

inline bool is_bno_state(int state) {
  return state == (int)I2C_STATE_BNO_CHECK_ID_SET_REGISTER ||
         state == (int)I2C_STATE_BNO_CHECK_ID_READ ||
         state == (int)I2C_STATE_BNO_SET_CONFIG_MODE ||
         state == (int)I2C_STATE_BNO_RESET ||
         state == (int)I2C_STATE_BNO_WAIT_AFTER_RESET ||
         state == (int)I2C_STATE_BNO_WAIT_ID_SET_REGISTER ||
         state == (int)I2C_STATE_BNO_WAIT_ID_READ ||
         state == (int)I2C_STATE_BNO_SET_POWER_MODE ||
         state == (int)I2C_STATE_BNO_SET_PAGE ||
         state == (int)I2C_STATE_BNO_SET_SYS_TRIGGER ||
         state == (int)I2C_STATE_BNO_SET_IMUPLUS_MODE ||
         state == (int)I2C_STATE_BNO_WAIT_AFTER_OP_MODE ||
         state == (int)I2C_STATE_BNO_SET_EULER_REGISTER ||
         state == (int)I2C_STATE_BNO_READ_EULER;
}

inline void start_i2c_write(uint8_t address, const uint8_t* buffer, size_t length, bool send_stop, int next_state) {
#if DEBUG_PROFILING_ENABLED
  DEBUG_PROFILING_START(i2c_req_checkpoint);
#endif
  i2c_master.write_async(address, buffer, length, send_stop);
  i2c_op_in_progress = true;
  i2c_read_kind = I2C_READ_NONE;
  i2c_next_state = (AsyncI2CState)next_state;
  i2c_op_started_at_ms = millis();
}

inline void start_i2c_read(uint8_t address, uint8_t* buffer, size_t length, bool send_stop, int read_kind, int next_state) {
#if DEBUG_PROFILING_ENABLED
  DEBUG_PROFILING_START(i2c_req_checkpoint);
#endif
  i2c_master.read_async(address, buffer, length, send_stop);
  i2c_op_in_progress = true;
  i2c_read_kind = (AsyncI2CReadKind)read_kind;
  i2c_next_state = (AsyncI2CState)next_state;
  i2c_op_started_at_ms = millis();
}

inline void request_ir_compass_read() {
  compass_read_requested = true;
  ir_read_requested = true;
  ir_read_completed = false;
  compass_read_completed = false;
}

inline bool are_ir_compass_reads_completed() {
  return ir_read_completed && compass_read_completed;
}

void process_i2c_async() {
#if DEBUG_PROFILING_ENABLED
  unsigned long i2c_start = 0;
  DEBUG_PROFILING_START(i2c_start);

  static unsigned long ir_checkpoint = 0;
  static unsigned long compass_checkpoint = 0;
#endif

  if (i2c_op_in_progress) {
    if (!i2c_master.finished()) {
      if (millis() - i2c_op_started_at_ms > I2C_ASYNC_TIMEOUT_MS) {
        bool is_bno = is_bno_state(i2c_state);
        DEBUG_LOG(DEBUG_ERROR, "Async I2C transaction timeout, read_kind=" + String(i2c_read_kind) + " i2c_state=" + String(i2c_state) + " is_bno=" + String(is_bno));
        if (!is_bno) {
          set_ir_no_signal_data();
        }
        reset_i2c_state_machine();
#if DEBUG_PROFILING_ENABLED
        DEBUG_PROFILING_RECORD(i2c_req_checkpoint, "I2C Request Timeout");
#endif
      }
#if DEBUG_PROFILING_ENABLED
      DEBUG_PROFILING_RECORD(i2c_start, "I2C Update");
#endif
      return;
    }

    i2c_op_in_progress = false;

    if (i2c_master.has_error()) {
      DEBUG_LOG(DEBUG_ERROR, "Async I2C transaction failed, code=" + String((int)i2c_master.error()) + " read_kind=" + String(i2c_read_kind) + " i2c_state=" + String(i2c_state) + " is_bno=" + String(is_bno_state(i2c_state)));
      if (is_bno_state(i2c_state) || i2c_read_kind == I2C_READ_BNO_ID || i2c_read_kind == I2C_READ_BNO_EULER) {
        set_compass_invalid();
      } else {
        set_ir_no_signal_data();
      }
      reset_i2c_state_machine();
#if DEBUG_PROFILING_ENABLED
      DEBUG_PROFILING_RECORD(i2c_start, "I2C Update");
      DEBUG_PROFILING_RECORD(i2c_req_checkpoint, "I2C Request Error");
#endif
      return;
    }

#if DEBUG_PROFILING_ENABLED
    DEBUG_PROFILING_RECORD(i2c_req_checkpoint, "I2C Request Success");
#endif

    if (i2c_read_kind == I2C_READ_BNO_ID) {
      if (bno_chip_id_buffer[0] != BNO055_CHIP_ID) {
        set_compass_invalid();
        i2c_state = I2C_STATE_WAIT_FOR_REQUEST;
        i2c_read_kind = I2C_READ_NONE;
#if DEBUG_PROFILING_ENABLED
        DEBUG_PROFILING_RECORD(i2c_start, "I2C Update");
#endif
        return;
      }
    } else if (i2c_read_kind == I2C_READ_IR_ANGLE_DISTANCE) {
      uint16_t angle = ((uint16_t)ir_angle_distance_buffer[0] << 8) | ir_angle_distance_buffer[1];
      uint16_t distance = ((uint16_t)ir_angle_distance_buffer[2] << 8) | ir_angle_distance_buffer[3];
      if (angle == IR_NO_SIGNAL_ANGLE && distance == IR_NO_SIGNAL_DISTANCE) {
        ir_data.angle = IR_NO_SIGNAL;
        ir_data.distance = 0;
      } else {
        ir_data.angle = angle;
        ir_data.distance = distance;
      }
      ir_read_completed = true;
      DEBUG_LOG(DEBUG_INFO, "Recieved valid ir angle data");
#if DEBUG_PROFILING_ENABLED
      DEBUG_PROFILING_RECORD(ir_checkpoint, "IR Read Success");
#endif
    } else if (i2c_read_kind == I2C_READ_IR_RAW) {
      for (int i = 0; i < IR_SENSOR_COUNT; i++) {
        ir_data.sensor_IR[i] = ir_raw_buffer[i];
      }
      DEBUG_LOG(DEBUG_INFO, "Recieved valid ir raw data");
    } else if (i2c_read_kind == I2C_READ_BNO_EULER) {
      int16_t heading_raw = ((int16_t)bno_euler_buffer[1] << 8) | bno_euler_buffer[0];
      int16_t pitch_raw = ((int16_t)bno_euler_buffer[3] << 8) | bno_euler_buffer[2];
      int16_t roll_raw = ((int16_t)bno_euler_buffer[5] << 8) | bno_euler_buffer[4];

      compass_data.heading = fmod(((float)heading_raw / 16.0f) + 360.0f, 360.0f);
      compass_data.pitch = ((float)pitch_raw / 16.0f);
      compass_data.roll = ((float)roll_raw / 16.0f);

      bno_initialized = true;

      compass_read_completed = true;

#if DEBUG_PROFILING_ENABLED
      DEBUG_PROFILING_RECORD(compass_checkpoint, "Compass Read Success");
#endif
      DEBUG_LOG(DEBUG_INFO, "Recieved valid compass data: " + String(compass_data.heading));
    }

    i2c_read_kind = I2C_READ_NONE;
    i2c_state = i2c_next_state;
  }

  switch (i2c_state) {
    case I2C_STATE_WAIT_FOR_REQUEST:
      if (ir_read_requested) {
        i2c_state = I2C_STATE_IR_SET_ANGLE_REGISTER;
        ir_read_requested = false;
        break;
      }

      if (compass_read_requested) {
        i2c_state = I2C_STATE_BNO_SET_EULER_REGISTER;
        compass_read_requested = false;
      }
      
      if (!bno_initialized && millis() >= bno_next_retry_at_ms) {
        i2c_state = I2C_STATE_BNO_CHECK_ID_SET_REGISTER;
        break;
      }

      break;

    case I2C_STATE_BNO_CHECK_ID_SET_REGISTER:
      DEBUG_LOG(DEBUG_INFO, "Initializing BNO");
      i2c_tx_byte = BNO055_CHIP_ID_ADDR;
      start_i2c_write(BNO055_I2C_ADDRESS, &i2c_tx_byte, 1, true, I2C_STATE_BNO_CHECK_ID_READ);
      break;

    case I2C_STATE_BNO_CHECK_ID_READ:
      start_i2c_read(BNO055_I2C_ADDRESS, bno_chip_id_buffer, 1, true, I2C_READ_BNO_ID, I2C_STATE_BNO_SET_CONFIG_MODE);
      break;

    case I2C_STATE_BNO_SET_CONFIG_MODE: {
      i2c_tx_buffer[0] = BNO055_OPR_MODE_ADDR;
      i2c_tx_buffer[1] = BNO055_OPR_MODE_CONFIG;
      start_i2c_write(BNO055_I2C_ADDRESS, i2c_tx_buffer, 2, true, I2C_STATE_BNO_RESET);
      break;
    }

    case I2C_STATE_BNO_RESET: {
      i2c_tx_buffer[0] = BNO055_SYS_TRIGGER_ADDR;
      i2c_tx_buffer[1] = 0x20;
      start_i2c_write(BNO055_I2C_ADDRESS, i2c_tx_buffer, 2, true, I2C_STATE_BNO_WAIT_AFTER_RESET);
      i2c_state_started_at_ms = millis();
      break;
    }

    case I2C_STATE_BNO_WAIT_AFTER_RESET:
      if (i2c_state_started_at_ms == 0) {
        i2c_state_started_at_ms = millis();
      }
      if (millis() - i2c_state_started_at_ms >= I2C_BNO_RESET_DELAY_MS) {
        i2c_state = I2C_STATE_BNO_WAIT_ID_SET_REGISTER;
        i2c_state_started_at_ms = 0;
      }
      break;

    case I2C_STATE_BNO_WAIT_ID_SET_REGISTER:
      i2c_tx_byte = BNO055_CHIP_ID_ADDR;
      start_i2c_write(BNO055_I2C_ADDRESS, &i2c_tx_byte, 1, true, I2C_STATE_BNO_WAIT_ID_READ);
      break;

    case I2C_STATE_BNO_WAIT_ID_READ:
      start_i2c_read(BNO055_I2C_ADDRESS, bno_chip_id_buffer, 1, true, I2C_READ_BNO_ID, I2C_STATE_BNO_SET_POWER_MODE);
      break;

    case I2C_STATE_BNO_SET_POWER_MODE: {
      i2c_tx_buffer[0] = BNO055_PWR_MODE_ADDR;
      i2c_tx_buffer[1] = BNO055_POWER_MODE_NORMAL;
      start_i2c_write(BNO055_I2C_ADDRESS, i2c_tx_buffer, 2, true, I2C_STATE_BNO_SET_PAGE);
      break;
    }

    case I2C_STATE_BNO_SET_PAGE: {
      i2c_tx_buffer[0] = BNO055_PAGE_ID_ADDR;
      i2c_tx_buffer[1] = 0x00;
      start_i2c_write(BNO055_I2C_ADDRESS, i2c_tx_buffer, 2, true, I2C_STATE_BNO_SET_SYS_TRIGGER);
      break;
    }

    case I2C_STATE_BNO_SET_SYS_TRIGGER: {
      i2c_tx_buffer[0] = BNO055_SYS_TRIGGER_ADDR;
      i2c_tx_buffer[1] = 0x00;
      start_i2c_write(BNO055_I2C_ADDRESS, i2c_tx_buffer, 2, true, I2C_STATE_BNO_SET_IMUPLUS_MODE);
      break;
    }

    case I2C_STATE_BNO_SET_IMUPLUS_MODE: {
      i2c_tx_buffer[0] = BNO055_OPR_MODE_ADDR;
      i2c_tx_buffer[1] = BNO055_OPR_MODE_NDOF;
      start_i2c_write(BNO055_I2C_ADDRESS, i2c_tx_buffer, 2, true, I2C_STATE_BNO_WAIT_AFTER_OP_MODE);
      i2c_state_started_at_ms = millis();
      break;
    }

    case I2C_STATE_BNO_WAIT_AFTER_OP_MODE:
      if (i2c_state_started_at_ms == 0) {
        i2c_state_started_at_ms = millis();
      }
      if (millis() - i2c_state_started_at_ms >= I2C_BNO_MODE_SWITCH_DELAY_MS) {
        i2c_state = I2C_STATE_WAIT_FOR_REQUEST;
        i2c_state_started_at_ms = 0;
        bno_initialized = true;
      }
      break;

    case I2C_STATE_IR_SET_ANGLE_REGISTER:
#if DEBUG_PROFILING_ENABLED
      DEBUG_PROFILING_START(ir_checkpoint);
#endif
      i2c_tx_byte = IR_ANGLE_AND_DISTANCE_REGISTER;
      start_i2c_write(IR_I2C_ADDRESS, &i2c_tx_byte, 1, true, I2C_STATE_IR_READ_ANGLE);
      break;

    case I2C_STATE_IR_READ_ANGLE:
      start_i2c_read(IR_I2C_ADDRESS, ir_angle_distance_buffer, IR_ANGLE_AND_DISTANCE_BYTES_COUNT, false, I2C_READ_IR_ANGLE_DISTANCE, I2C_STATE_WAIT_FOR_REQUEST /*I2C_STATE_IR_SET_RAW_REGISTER*/);
      break;

    case I2C_STATE_IR_SET_RAW_REGISTER:
      i2c_tx_byte = IR_RAW_REGISTER;
      start_i2c_write(IR_I2C_ADDRESS, &i2c_tx_byte, 1, true, I2C_STATE_IR_READ_RAW);
      break;

    case I2C_STATE_IR_READ_RAW:
      start_i2c_read(
        IR_I2C_ADDRESS,
        ir_raw_buffer,
        IR_RAW_BYTES_COUNT,
        true,
        I2C_READ_IR_RAW,
        I2C_STATE_WAIT_FOR_REQUEST
      );
      if (ir_read_requested) {
        ir_read_requested = false;
      }
      break;

    case I2C_STATE_BNO_SET_EULER_REGISTER:
#if DEBUG_PROFILING_ENABLED
      DEBUG_PROFILING_START(compass_checkpoint);
#endif
      i2c_tx_byte = BNO055_EULER_H_LSB_ADDR;
      start_i2c_write(BNO055_I2C_ADDRESS, &i2c_tx_byte, 1, false, I2C_STATE_BNO_READ_EULER);
      break;

    case I2C_STATE_BNO_READ_EULER:
      start_i2c_read(
        BNO055_I2C_ADDRESS,
        bno_euler_buffer,
        6,
        true,
        I2C_READ_BNO_EULER,
        I2C_STATE_WAIT_FOR_REQUEST
      );
      if (compass_read_requested) {
        compass_read_requested = false;
      }
      break;
  }

#if DEBUG_PROFILING_ENABLED
  DEBUG_PROFILING_RECORD(i2c_start, "I2C Update");
#endif
}


void read_line_sensors() {
#if DEBUG_PROFILING_ENABLED
  unsigned long line_start = 0;
  DEBUG_PROFILING_START(line_start);
#endif

  auto *line = line_data.sensor_line;
  auto *minv = line_data.sensor_line_min;
  auto *maxv = line_data.sensor_line_max;

  for (int i = 0; i < LINE_SENSOR_COUNT; i++) {
    uint16_t v = analogRead(line_sensors_pin_config[i]);
    line[i] = v;

    if (v > maxv[i]) maxv[i] = v;
    if (v < minv[i]) minv[i] = v;
  }

#if DEBUG_PROFILING_ENABLED
  DEBUG_PROFILING_RECORD(line_start, "Line Sensors");
#endif
}

void reset_line_min_max_values() {
  for (int i = 0; i < LINE_SENSOR_COUNT; i++) {
    line_data.sensor_line_max[i] = 0;
    line_data.sensor_line_min[i] = 1023;
  }
}


// ========== Communication Functions ==========
bool parse_motor_command() {
#if DEBUG_PROFILING_ENABLED
  unsigned long cmd_start = 0;
  DEBUG_PROFILING_START(cmd_start);
#endif

  // Expected format: {"a"="±MMMM,±MMMM,±MMMM,±MMMM,K"}
  // MMMM = motor speed (signed, 0 to 255), K = kicker (0 or 1)

  String input = "";
  while (RASPBERRY_SERIAL.available() > 0) {
    input = RASPBERRY_SERIAL.readStringUntil('\n');
  }

  if (input.length() == 0) return false;

  DEBUG_PRINT("RX: ");
  DEBUG_PRINTLN(input.c_str());

  const int EXPECTED_LENGTH = 33;
  if (input.length() != EXPECTED_LENGTH) {
    DEBUG_LOG(DEBUG_ERROR, "Invalid message length: \"" + input + "\"");
    return false;
  }

  if (input[0] != '{' || input[1] != '"' || input[2] != 'a' || input[3] != '"' || input[4] != '=' || input[5] != '"' || input[31] != '"' || input[32] != '}') {
    DEBUG_LOG(DEBUG_ERROR, "Invalid JSON structure: \"" + input + "\"");
    return false;
  }

  String data = input.substring(6, 31);  // "±MMMM,±MMMM,±MMMM,±MMMM,K"

  if (data[5] != ',' || data[11] != ',' || data[17] != ',' || data[23] != ',') {
    DEBUG_LOG(DEBUG_ERROR, "Invalid comma positions: \"" + input + "\"");
    return false;
  }

  for (int i = 0; i < MOTOR_COUNT; i++) {
    char sign = data[i * 6];
    if (sign != '+' && sign != '-') {
      DEBUG_LOG(DEBUG_ERROR, "Invalid sign character: \"" + input + "\"");
      return false;
    }

    for (int j = 1; j <= 4; j++) {
      if (!isdigit(data[i * 6 + j])) {
        DEBUG_LOG(DEBUG_ERROR, "Invalid motor value digit: \"" + input + "\"");
        return false;
      }
    }
  }

  if (!isdigit(data[24])) {
    DEBUG_LOG(DEBUG_ERROR, "Invalid kicker value: \"" + input + "\"");
    return false;
  }

  for (int i = 0; i < MOTOR_COUNT; i++) {
    motors_data.motor_speed[i] = data.substring(i * 6, i * 6 + 5).toInt();
  }

  motors_data.kicker_position = data.substring(24, 25).toInt();
  motors_data.kicker_position = constrain(motors_data.kicker_position, KICKER_IN, KICKER_OUT);

#if DEBUG_PROFILING_ENABLED
  DEBUG_PROFILING_RECORD(cmd_start, "Parse Command");
#endif

  return true;
}

inline void update_sensor_data_struct() {
  sensor_data.compass_data = compass_data;
  sensor_data.ir_data = ir_data;
  sensor_data.motors_data = motors_data;
  sensor_data.line_data = line_data;
}


void build_sensor_message(char* msg) {
#if DEBUG_PROFILING_ENABLED
  unsigned long cmd_start = 0;
  DEBUG_PROFILING_START(cmd_start);
#endif

  memset(msg, 0, RASPBERRY_SERIAL_BUFFER_SIZE);

  msg[0] = '{';
  msg[1] = 0x01; // Message type

  auto write16 = [&](int idx, int16_t value) {
    msg[idx]     = value & 0xFF;
    msg[idx + 1] = (value >> 8) & 0xFF;
  };

  write16(2,  sensor_data.compass_data.heading);
  write16(4,  sensor_data.compass_data.pitch);
  write16(6,  sensor_data.compass_data.roll);

  write16(8,  sensor_data.ir_data.angle);
  write16(10, sensor_data.ir_data.distance);

  uint16_t s_min[12] = {
    (uint16_t)sensor_data.line_data.sensor_line_min[0],
    (uint16_t)sensor_data.line_data.sensor_line_min[1],
    (uint16_t)sensor_data.line_data.sensor_line_min[2],
    (uint16_t)sensor_data.line_data.sensor_line_min[3],
    (uint16_t)sensor_data.line_data.sensor_line_min[4],
    (uint16_t)sensor_data.line_data.sensor_line_min[5],
    (uint16_t)sensor_data.line_data.sensor_line_min[6],
    (uint16_t)sensor_data.line_data.sensor_line_min[7],
    (uint16_t)sensor_data.line_data.sensor_line_min[8],
    (uint16_t)sensor_data.line_data.sensor_line_min[9],
    (uint16_t)sensor_data.line_data.sensor_line_min[10],
    (uint16_t)sensor_data.line_data.sensor_line_min[11]
  };
  uint16_t s_max[12] = {
    (uint16_t)sensor_data.line_data.sensor_line_max[0],
    (uint16_t)sensor_data.line_data.sensor_line_max[1],
    (uint16_t)sensor_data.line_data.sensor_line_max[2],
    (uint16_t)sensor_data.line_data.sensor_line_max[3],
    (uint16_t)sensor_data.line_data.sensor_line_max[4],
    (uint16_t)sensor_data.line_data.sensor_line_max[5],
    (uint16_t)sensor_data.line_data.sensor_line_max[6],
    (uint16_t)sensor_data.line_data.sensor_line_max[7],
    (uint16_t)sensor_data.line_data.sensor_line_max[8],
    (uint16_t)sensor_data.line_data.sensor_line_max[9],
    (uint16_t)sensor_data.line_data.sensor_line_max[10],
    (uint16_t)sensor_data.line_data.sensor_line_max[11]
  };
  msg[11 + 0] |= (uint8_t)(s_min[0] << 0);
  msg[11 + 1] |= (uint8_t)(s_min[0] >> 8);
  msg[11 + 1] |= (uint8_t)(s_max[0] << 2);
  msg[11 + 2] |= (uint8_t)(s_max[0] >> 6);
  msg[11 + 2] |= (uint8_t)(s_min[1] << 4);
  msg[11 + 3] |= (uint8_t)(s_min[1] >> 4);
  msg[11 + 3] |= (uint8_t)(s_max[1] << 6);
  msg[11 + 4] |= (uint8_t)(s_max[1] >> 2);
  msg[11 + 5] |= (uint8_t)(s_min[2] << 0);
  msg[11 + 6] |= (uint8_t)(s_min[2] >> 8);
  msg[11 + 6] |= (uint8_t)(s_max[2] << 2);
  msg[11 + 7] |= (uint8_t)(s_max[2] >> 6);
  msg[11 + 7] |= (uint8_t)(s_min[3] << 4);
  msg[11 + 8] |= (uint8_t)(s_min[3] >> 4);
  msg[11 + 8] |= (uint8_t)(s_max[3] << 6);
  msg[11 + 9] |= (uint8_t)(s_max[3] >> 2);
  msg[11 +10] |= (uint8_t)(s_min[4] << 0);
  msg[11 +11] |= (uint8_t)(s_min[4] >> 8);
  msg[11 +11] |= (uint8_t)(s_max[4] << 2);
  msg[11 +12] |= (uint8_t)(s_max[4] >> 6);
  msg[11 +12] |= (uint8_t)(s_min[5] << 4);
  msg[11 +13] |= (uint8_t)(s_min[5] >> 4);
  msg[11 +13] |= (uint8_t)(s_max[5] << 6);
  msg[11 +14] |= (uint8_t)(s_max[5] >> 2);
  msg[11 +15] |= (uint8_t)(s_min[6] << 0);
  msg[11 +16] |= (uint8_t)(s_min[6] >> 8);
  msg[11 +16] |= (uint8_t)(s_max[6] << 2);
  msg[11 +17] |= (uint8_t)(s_max[6] >> 6);
  msg[11 +17] |= (uint8_t)(s_min[7] << 4);
  msg[11 +18] |= (uint8_t)(s_min[7] >> 4);
  msg[11 +18] |= (uint8_t)(s_max[7] << 6);
  msg[11 +19] |= (uint8_t)(s_max[7] >> 2);
  msg[11 +20] |= (uint8_t)(s_min[8] << 0);
  msg[11 +21] |= (uint8_t)(s_min[8] >> 8);
  msg[11 +21] |= (uint8_t)(s_max[8] << 2);
  msg[11 +22] |= (uint8_t)(s_max[8] >> 6);
  msg[11 +22] |= (uint8_t)(s_min[9] << 4);
  msg[11 +23] |= (uint8_t)(s_min[9] >> 4);
  msg[11 +23] |= (uint8_t)(s_max[9] << 6);
  msg[11 +24] |= (uint8_t)(s_max[9] >> 2);
  msg[11 +25] |= (uint8_t)(s_min[10] << 0);
  msg[11 +26] |= (uint8_t)(s_min[10] >> 8);
  msg[11 +26] |= (uint8_t)(s_max[10] << 2);
  msg[11 +27] |= (uint8_t)(s_max[10] >> 6);
  msg[11 +27] |= (uint8_t)(s_min[11] << 4);
  msg[11 +28] |= (uint8_t)(s_min[11] >> 4);
  msg[11 +28] |= (uint8_t)(s_max[11] << 6);
  msg[11 +29] |= (uint8_t)(s_max[11] >> 2);

  uint8_t checksum = 0;
  for (int i = 0; i < 11 + 36; ++i) {
    checksum += msg[i];
  }
  msg[41] = checksum;
  msg[42] = '}';

#if DEBUG_PROFILING_ENABLED
  DEBUG_PROFILING_RECORD(cmd_start, "Create Sensor Message");
#endif
}

void build_running_state_message(char* msg) {
  memset(msg, 0, RASPBERRY_SERIAL_BUFFER_SIZE);

  msg[0] = '{';
  msg[1] = 0x02;
  msg[2] = is_running | (bt_module_enabled << 1) | (main_switch_enabled << 2) | (module_value << 3);
  msg[3] = '}';
}


inline bool can_send_message_to_rpi(int length) {
  return RASPBERRY_SERIAL.availableForWrite() > length;
}

template<typename T> void send_message_to_rpi(T message) {
#if DEBUG_PROFILING_ENABLED
    unsigned long msg_checkpoint;
    DEBUG_PROFILING_START(msg_checkpoint);
#endif
  RASPBERRY_SERIAL.println(message);
  DEBUG_PRINT("TX: ");
  DEBUG_PRINTLN(message);
#if DEBUG_PROFILING_ENABLED
    DEBUG_PROFILING_RECORD(msg_checkpoint, "Message Send to RPI");
#endif
}



// ========== Setup & Main Loop ==========
void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(MODULE_LED_PIN, OUTPUT);
  pinMode(MODULE_SWITCH_LED_PIN, OUTPUT);
  pinMode(MAIN_SWITCH_LED_PIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  DEBUG_SERIAL.begin(DEBUG_SERIAL_SPEED);
  RASPBERRY_SERIAL.begin(RASPBERRY_SERIAL_SPEED);
  static char extra_buffer[RASPBERRY_SERIAL_BUFFER_SIZE - 39];
  RASPBERRY_SERIAL.addMemoryForWrite(extra_buffer, sizeof(extra_buffer));
  DEBUG_LOG(DEBUG_INFO, "RASPBERRY_SERIAL.availableForWrite(): " + String(RASPBERRY_SERIAL.availableForWrite()));

  DEBUG_PRINTLN("\n=== LNX Infrabot Teensy Controller ===");
  DEBUG_PRINTLN("Initializing...");

  pinMode(MODULE_PIN, INPUT);
  pinMode(MAIN_SWITCH_PIN, INPUT_PULLUP);
  pinMode(MODULE_SWITCH_PIN, INPUT_PULLUP);
  update_running_state();
  DEBUG_PRINTLN("Control switches initialized");

  for (int i = 0; i < MOTOR_COUNT; i++) {
    pinMode(motor_pin_config[i][PWM_INDEX], OUTPUT);
    pinMode(motor_pin_config[i][DIR_INDEX], OUTPUT);
  }
  pinMode(kicker_pin, OUTPUT);
  stop_motors();
  DEBUG_PRINTLN("Motors initialized");

  i2c_master.begin(400000);
  reset_i2c_state_machine();
  compass_data.heading = 0;
  compass_data.pitch = 0;
  compass_data.roll = 0;
  DEBUG_PRINTLN("Async I2C initialized (100 kHz)");

  for (int i = 0; i < LINE_SENSOR_COUNT; i++) {
    pinMode(line_sensors_pin_config[i], INPUT);
  }
  DEBUG_PRINTLN("Line sensors initialized");

  reset_line_min_max_values();

  DEBUG_PRINTLN("=== Initialization complete ===");
  DEBUG_PRINTLN();
}


void target_ups_loop() {
#if DEBUG_PROFILING_ENABLED
  unsigned long loop_start = 0;
  DEBUG_PROFILING_START(loop_start);
#endif

  // Button checks
#if DEBUG_PROFILING_ENABLED
  unsigned long section_start = 0;
  DEBUG_PROFILING_START(section_start);
#endif
  
  if (isButtonPressed(main_switch)) {
    main_switch_enabled = !main_switch_enabled;
    update_running_state();
    build_running_state_message(message_buffer);
    send_message_to_rpi(message_buffer);
    DEBUG_LOG(DEBUG_INFO, main_switch_enabled ? "Manual switch: ON" : "Manual switch: OFF");
  }

  if (isButtonPressed(module_switch)) {
    bt_module_enabled = !bt_module_enabled;
    update_running_state();
    build_running_state_message(message_buffer);
    send_message_to_rpi(message_buffer);
    DEBUG_LOG(DEBUG_INFO, bt_module_enabled ? "Bluetooth: ENABLED" : "Bluetooth: DISABLED");
  }

#if DEBUG_PROFILING_ENABLED
  DEBUG_PROFILING_RECORD(section_start, "Button Checks");
#endif
  // Sensor reading
  request_ir_compass_read();
    
  // Communication
#if DEBUG_PROFILING_ENABLED
  DEBUG_PROFILING_START(section_start);
#endif
  if (can_send_message_to_rpi(SENSOR_DATA_MESSAGE_LENGTH)) {
    update_sensor_data_struct();
    build_sensor_message(message_buffer);
    reset_line_min_max_values();

    send_message_to_rpi(message_buffer);
  }
#if DEBUG_PROFILING_ENABLED
  DEBUG_PROFILING_RECORD(section_start, "Sensors & Communication");
#endif

  // Command parsing
  parse_motor_command();

  // LED and motor updates
#if DEBUG_PROFILING_ENABLED
  DEBUG_PROFILING_START(section_start);
#endif
  
  digitalWrite(MAIN_SWITCH_LED_PIN, main_switch_enabled);
  digitalWrite(MODULE_LED_PIN, module_value);
  digitalWrite(MODULE_SWITCH_LED_PIN, !bt_module_enabled);

  update_running_state();
  digitalWrite(LED_BUILTIN, is_running);

  if (is_running) {
    set_all_motors_speed(motors_data.motor_speed);
    set_kicker_position(motors_data.kicker_position);
    DEBUG_PRINT(">>> RUNNING");
  } else {
    stop_motors();
    DEBUG_PRINT(">>> STOPPED");
  }

#if DEBUG_PROFILING_ENABLED
  DEBUG_PROFILING_RECORD(section_start, "LED & Motors");
#endif

#if DEBUG_PRINTS_ENABLED
  DEBUG_PRINT(" | M_SW=");
  DEBUG_PRINT(main_switch_enabled);
  DEBUG_PRINT(" BT_SW=");
  DEBUG_PRINT(bt_module_enabled);
  DEBUG_PRINT(" BT=");
  DEBUG_PRINT(module_value);
  DEBUG_PRINTLN();

  print_sensor_debug_info();
#endif

#if DEBUG_PROFILING_ENABLED
  DEBUG_PROFILING_RECORD(loop_start, "Target UPS Loop");
#endif
}

void fastest_loop() {
#if DEBUG_PROFILING_ENABLED
  unsigned long fastest_start = 0;
  DEBUG_PROFILING_START(fastest_start);
#endif

  process_i2c_async();
  read_line_sensors();

#if DEBUG_PROFILING_ENABLED
  DEBUG_PROFILING_RECORD(fastest_start, "Fastest Loop");
#endif
}


void loop() {
#if DEBUG_PROFILING_ENABLED
  unsigned long loop_start = 0;
  DEBUG_PROFILING_START(loop_start);
#endif

  unsigned long current_time = micros();

  fastest_loop();

  static unsigned long last_target_ups_loop_time = 0;
  if (current_time - last_target_ups_loop_time >= INTERVAL_US) {
    last_target_ups_loop_time = current_time;
    target_ups_loop();
  }

#if DEBUG_PROFILING_ENABLED
  DEBUG_PROFILING_RECORD(loop_start, "Main Loop");
  DEBUG_PROFILING_PRINT_IF_READY();
#endif
}
