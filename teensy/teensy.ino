#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
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

Button main_switch = {MODULE_SWITCH_PIN, HIGH, HIGH, 0};
Button module_switch = {MAIN_SWITCH_PIN, HIGH, HIGH, 0};


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
    unsigned long sum = 0;
    unsigned long min_val = UINT32_MAX;
    unsigned long max_val = 0;
    unsigned long count = 0;
  };

  const int MAX_PROFILING_POINTS = 20;
  struct ProfilingData {
    const char* names[MAX_PROFILING_POINTS];
    TimingStat stats[MAX_PROFILING_POINTS];
    int count = 0;
  };
  
  ProfilingData profiling_data;
  static unsigned long last_timing_print = 0;
  static unsigned long last_line_reading_time = 0;
  static unsigned long last_message_send_time = 0;
  static unsigned long min_interval_line_readings = 0;
  static unsigned long max_interval_line_readings = 0;
  static unsigned long min_interval_message_sends = 0;
  static unsigned long max_interval_message_sends = 0;

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

  inline void DEBUG_TIME_START(unsigned long& checkpoint) {
    checkpoint = micros();
  }

  inline void DEBUG_TIME_RECORD(unsigned long checkpoint, const char* label) {
    unsigned long elapsed = micros() - checkpoint;
    int idx = get_or_create_profiling_point(label);
    if (idx >= 0) {
      TimingStat& stat = profiling_data.stats[idx];
      stat.sum += elapsed;
      if (elapsed < stat.min_val) stat.min_val = elapsed;
      if (elapsed > stat.max_val) stat.max_val = elapsed;
      stat.count++;
    }
  }

  inline void DEBUG_TIME_MARK_LINE_READING() {
    unsigned long now = micros();
    if (last_line_reading_time > 0) {
      unsigned long interval = now - last_line_reading_time;
      if (interval < min_interval_line_readings) {
        min_interval_line_readings = interval;
      }
      if (interval > max_interval_line_readings) {
        max_interval_line_readings = interval;
      }
    }
    last_line_reading_time = now;
  }

  inline void DEBUG_TIME_MARK_MESSAGE_SEND() {
    unsigned long now = micros();
    if (last_message_send_time > 0) {
      unsigned long interval = now - last_message_send_time;
      if (interval < min_interval_message_sends) {
        min_interval_message_sends = interval;
      }
      if (interval > max_interval_message_sends) {
        max_interval_message_sends = interval;
      }
    }
    last_message_send_time = now;
  }

  void DEBUG_TIME_PRINT_IF_READY() {
    unsigned long current_time = millis();
    if (current_time - last_timing_print >= DEBUG_PROFILING_PRINT_INTERVAL_S * 1000) {
      last_timing_print = current_time;
      
      DEBUG_SERIAL.println();
      DEBUG_SERIAL.println("┌───────────────────────┬─────────┬─────────┬─────────┬───────────┐");
      DEBUG_SERIAL.println("│ Function              │ Avg (µs)│ Min (µs)│ Max (µs)│   Count   │");
      DEBUG_SERIAL.println("├───────────────────────┼─────────┼─────────┼─────────┼───────────┤");
      
      for (int i = 0; i < profiling_data.count; i++) {
        TimingStat& stat = profiling_data.stats[i];
        if (stat.count > 0) {
          DEBUG_SERIAL.print("│ ");
          DEBUG_SERIAL.print(profiling_data.names[i]);
          int name_len = strlen(profiling_data.names[i]);
          for (int j = name_len; j < 21; j++) DEBUG_SERIAL.print(" ");
          
          unsigned long avg = stat.sum / stat.count;
          DEBUG_SERIAL.print(" │ ");
          if (avg < 10) DEBUG_SERIAL.print("      ");
          else if (avg < 100) DEBUG_SERIAL.print("     ");
          else if (avg < 1000) DEBUG_SERIAL.print("    ");
          else if (avg < 10000) DEBUG_SERIAL.print("   ");
          else if (avg < 100000) DEBUG_SERIAL.print("  ");
          DEBUG_SERIAL.print(avg);
          
          DEBUG_SERIAL.print(" │ ");
          if (stat.min_val < 10) DEBUG_SERIAL.print("      ");
          else if (stat.min_val < 100) DEBUG_SERIAL.print("     ");
          else if (stat.min_val < 1000) DEBUG_SERIAL.print("    ");
          else if (stat.min_val < 10000) DEBUG_SERIAL.print("   ");
          else if (stat.min_val < 100000) DEBUG_SERIAL.print("  ");
          DEBUG_SERIAL.print(stat.min_val);
          
          DEBUG_SERIAL.print(" │ ");
          if (stat.max_val < 10) DEBUG_SERIAL.print("      ");
          else if (stat.max_val < 100) DEBUG_SERIAL.print("     ");
          else if (stat.max_val < 1000) DEBUG_SERIAL.print("    ");
          else if (stat.max_val < 10000) DEBUG_SERIAL.print("   ");
          else if (stat.max_val < 100000) DEBUG_SERIAL.print("  ");
          DEBUG_SERIAL.print(stat.max_val);
          
          DEBUG_SERIAL.print(" │ ");
          if (stat.count < 10) DEBUG_SERIAL.print("        ");
          else if (stat.count < 100) DEBUG_SERIAL.print("       ");
          else if (stat.count < 1000) DEBUG_SERIAL.print("      ");
          else if (stat.count < 10000) DEBUG_SERIAL.print("     ");
          else if (stat.count < 100000) DEBUG_SERIAL.print("    ");
          else if (stat.count < 1000000) DEBUG_SERIAL.print("   ");
          else DEBUG_SERIAL.print(" ");
          DEBUG_SERIAL.print(stat.count);
          DEBUG_SERIAL.println(" │");
        }
      }
      
      DEBUG_SERIAL.println("├───────────────────────┼─────────┼─────────┼─────────┼───────────┤");
      DEBUG_SERIAL.print("│ Max interval (line)   │         │ ");
      unsigned long line_interval_min = min_interval_line_readings;
      if (line_interval_min < 10) DEBUG_SERIAL.print("      ");
      else if (line_interval_min < 100) DEBUG_SERIAL.print("     ");
      else if (line_interval_min < 1000) DEBUG_SERIAL.print("    ");
      else if (line_interval_min < 10000) DEBUG_SERIAL.print("   ");
      else if (line_interval_min < 100000) DEBUG_SERIAL.print("  ");
      DEBUG_SERIAL.print(line_interval_min);
      DEBUG_SERIAL.print(" │ ");
      unsigned long line_interval_max = max_interval_line_readings;
      if (line_interval_max < 10) DEBUG_SERIAL.print("      ");
      else if (line_interval_max < 100) DEBUG_SERIAL.print("     ");
      else if (line_interval_max < 1000) DEBUG_SERIAL.print("    ");
      else if (line_interval_max < 10000) DEBUG_SERIAL.print("   ");
      else if (line_interval_max < 100000) DEBUG_SERIAL.print("  ");
      DEBUG_SERIAL.print(line_interval_max);
      DEBUG_SERIAL.println(" │           │");
      
      DEBUG_SERIAL.print("│ Max interval (msg)    │         │ ");
      unsigned long msg_interval_min = min_interval_message_sends;
      if (msg_interval_min < 10) DEBUG_SERIAL.print("      ");
      else if (msg_interval_min < 100) DEBUG_SERIAL.print("     ");
      else if (msg_interval_min < 1000) DEBUG_SERIAL.print("    ");
      else if (msg_interval_min < 10000) DEBUG_SERIAL.print("   ");
      else if (msg_interval_min < 100000) DEBUG_SERIAL.print("  ");
      DEBUG_SERIAL.print(msg_interval_min);
      DEBUG_SERIAL.print(" │ ");
      unsigned long msg_interval_max = max_interval_message_sends;
      if (msg_interval_max < 10) DEBUG_SERIAL.print("      ");
      else if (msg_interval_max < 100) DEBUG_SERIAL.print("     ");
      else if (msg_interval_max < 1000) DEBUG_SERIAL.print("    ");
      else if (msg_interval_max < 10000) DEBUG_SERIAL.print("   ");
      else if (msg_interval_max < 100000) DEBUG_SERIAL.print("  ");
      DEBUG_SERIAL.print(msg_interval_max);
      DEBUG_SERIAL.println(" │           │");
      
      DEBUG_SERIAL.println("└───────────────────────┴─────────┴─────────┴─────────┴───────────┘");
      
      for (int i = 0; i < profiling_data.count; i++) {
        profiling_data.stats[i] = {0, UINT32_MAX, 0, 0};
      }
      max_interval_line_readings = 0;
      max_interval_message_sends = 0;
    }
  }
#else
  struct TimingStat {};
  inline void DEBUG_TIME_START(unsigned long&) {}
  inline void DEBUG_TIME_RECORD(unsigned long, const char*) {}
  inline void DEBUG_TIME_MARK_LINE_READING() {}
  inline void DEBUG_TIME_MARK_MESSAGE_SEND() {}
  void DEBUG_TIME_PRINT_IF_READY() {}
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
#define BNO055_SENSOR_ID 55
Adafruit_BNO055 bno = Adafruit_BNO055(BNO055_SENSOR_ID);
bool bno_initialized = false;

struct CompassData {
  float heading;  // 0-360 degrees
  float pitch;    // -180 to +180 degrees
  float roll;     // -180 to +180 degrees
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
  uint8_t status;
};
IRData ir_data = { IR_NO_SIGNAL, 0, { 0 }, 0 };


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
  module_value = module_switch.state ? (digitalRead(MODULE_PIN) == HIGH) : true;
  is_running = (main_switch.state && module_value);
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
  DEBUG_PRINT("| Status=");
  DEBUG_PRINTLN(sensor_data.ir_data.status);

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
bool bno_initialize() {
  if (bno.begin(OPERATION_MODE_IMUPLUS)) {
    DEBUG_LOG(DEBUG_INFO, "BNO055 initialized successfully");
    bno_initialized = true;
    bno.setExtCrystalUse(true);
    return true;
  } else {
    DEBUG_LOG(DEBUG_WARN, "BNO055 not found");
    bno_initialized = false;
    return false;
  }
}

void read_compass() {
#if DEBUG_PROFILING_ENABLED
  unsigned long compass_start = 0;
  DEBUG_TIME_START(compass_start);
#endif

  if (!bno_initialized) {
    compass_data.heading = 999;
    compass_data.pitch = 999;
    compass_data.roll = 999;
    bno_initialize();
    return;
  }

  sensors_event_t event;
  bno.getEvent(&event);

  compass_data.heading = fmod(event.orientation.x + 360.0, 360.0);
  compass_data.pitch = event.orientation.y;
  compass_data.roll = event.orientation.z;

  if (compass_data.heading == 0 && compass_data.pitch == 0 && compass_data.roll == 0) {
    uint8_t system_status, self_test_result, system_error;
    bno.getSystemStatus(&system_status, &self_test_result, &system_error);

    if (system_error != 0) {
      DEBUG_LOG(DEBUG_ERROR, "BNO055 error detected during read: " + String(system_error));
      bno_initialized = false;
      compass_data.heading = 999;
      compass_data.pitch = 999;
      compass_data.roll = 999;
    }
  }

#if DEBUG_PROFILING_ENABLED
  DEBUG_TIME_RECORD(compass_start, "Compass Read");
#endif
}

void read_ir_sensor() {
#if DEBUG_PROFILING_ENABLED
  unsigned long ir_start = 0;
  DEBUG_TIME_START(ir_start);
#endif

  Wire.beginTransmission(IR_I2C_ADDRESS);
  Wire.write(IR_ANGLE_AND_DISTANCE_REGISTER);
  if (Wire.endTransmission() != 0) {
    DEBUG_LOG(DEBUG_ERROR, "IR sensor communication error");
    ir_data.angle = IR_NO_SIGNAL;
    ir_data.distance = 0;
    return;
  }

  Wire.requestFrom(IR_I2C_ADDRESS, IR_ANGLE_AND_DISTANCE_BYTES_COUNT);
  if (Wire.available() >= IR_ANGLE_AND_DISTANCE_BYTES_COUNT) {
    uint16_t angle = (Wire.read() << 8) | Wire.read();
    uint16_t distance = (Wire.read() << 8) | Wire.read();

    if (angle == IR_NO_SIGNAL_ANGLE && distance == IR_NO_SIGNAL_DISTANCE) {
      ir_data.angle = IR_NO_SIGNAL;
      ir_data.distance = 0;
    } else {
      ir_data.angle = angle;
      ir_data.distance = distance;
    }
  } else {
    ir_data.angle = IR_NO_SIGNAL;
    ir_data.distance = 0;
  }

  Wire.beginTransmission(IR_I2C_ADDRESS);
  Wire.write(IR_RAW_REGISTER);
  Wire.endTransmission();
  Wire.requestFrom(IR_I2C_ADDRESS, IR_RAW_BYTES_COUNT);

  for (int i = 0; i < IR_SENSOR_COUNT; i++) {
    ir_data.sensor_IR[i] = Wire.available() ? Wire.read() : 0;
  }

  ir_data.status = Wire.available() ? Wire.read() : 0;

#if DEBUG_PROFILING_ENABLED
  DEBUG_TIME_RECORD(ir_start, "IR Sensor Read");
#endif
}


void read_line_sensors() {
#if DEBUG_PROFILING_ENABLED
  unsigned long line_start = 0;
  DEBUG_TIME_START(line_start);
  DEBUG_TIME_MARK_LINE_READING();
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
  DEBUG_TIME_RECORD(line_start, "Line Sensors");
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
  DEBUG_TIME_START(cmd_start);
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
  DEBUG_TIME_RECORD(cmd_start, "Parse Command");
#endif

  return true;
}

inline void update_sensor_data_struct() {
#if DEBUG_PROFILING_ENABLED
  unsigned long update_start = 0;
  DEBUG_TIME_START(update_start);
#endif

  sensor_data.compass_data = compass_data;
  sensor_data.ir_data = ir_data;
  sensor_data.motors_data = motors_data;
  sensor_data.line_data = line_data;

#if DEBUG_PROFILING_ENABLED
  DEBUG_TIME_RECORD(update_start, "Update Sensor Struct");
#endif
}


void build_sensor_message(char* msg) {
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
}

void build_running_state_message(char* msg) {
  memset(msg, 0, RASPBERRY_SERIAL_BUFFER_SIZE);

  msg[0] = '{';
  msg[1] = 0x02;
  msg[2] = is_running | (module_switch.state << 1) | (main_switch.state << 2) | (module_value << 3);
  msg[3] = '}';
}


inline bool can_send_message_to_rpi(int length) {
  return RASPBERRY_SERIAL.availableForWrite() > length;
}

template<typename T> void send_message_to_rpi(T message) {
  RASPBERRY_SERIAL.println(message);
  DEBUG_PRINT("TX: ");
  DEBUG_PRINTLN(message);
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

  bno_initialize();

  Wire.begin();
  DEBUG_PRINTLN("IR sensor initialized (MRM-IR-Finder3)");
  Wire.setClock(400000);
  DEBUG_PRINTLN("I2C initialized (400 kHz)");

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
  DEBUG_TIME_START(loop_start);
#endif

  // Button checks
#if DEBUG_PROFILING_ENABLED
  unsigned long section_start = 0;
  DEBUG_TIME_START(section_start);
#endif
  
  if (isButtonPressed(main_switch)) {
    main_switch.state = !main_switch.state;
    update_running_state();
    build_running_state_message(message_buffer);
    send_message_to_rpi(message_buffer);
    DEBUG_LOG(DEBUG_INFO, main_switch.state ? "Manual switch: ON" : "Manual switch: OFF");
  }

  if (isButtonPressed(module_switch)) {
    module_switch.state = !module_switch.state;
    update_running_state();
    build_running_state_message(message_buffer);
    send_message_to_rpi(message_buffer);
    DEBUG_LOG(DEBUG_INFO, module_switch.state ? "Bluetooth: ENABLED" : "Bluetooth: DISABLED");
  }

#if DEBUG_PROFILING_ENABLED
  DEBUG_TIME_RECORD(section_start, "Button Checks");
#endif

  // Sensor reading and communication
#if DEBUG_PROFILING_ENABLED
  DEBUG_TIME_START(section_start);
#endif
  
  if (can_send_message_to_rpi(SENSOR_DATA_MESSAGE_LENGTH)) {
    read_ir_sensor();
    read_compass();
    update_sensor_data_struct();
    build_sensor_message(message_buffer);
    reset_line_min_max_values();

    send_message_to_rpi(message_buffer);
#if DEBUG_PROFILING_ENABLED
    DEBUG_TIME_MARK_MESSAGE_SEND();
#endif
#if DEBUG_LOGS_ENABLED
    messages_sent++;
#endif
  }

#if DEBUG_PROFILING_ENABLED
  DEBUG_TIME_RECORD(section_start, "Sensors+Communication");
#endif

  // Command parsing
#if DEBUG_PROFILING_ENABLED
  DEBUG_TIME_START(section_start);
#endif
  
  parse_motor_command();

#if DEBUG_PROFILING_ENABLED
  DEBUG_TIME_RECORD(section_start, "Parse Command");
#endif

  // LED and motor updates
#if DEBUG_PROFILING_ENABLED
  DEBUG_TIME_START(section_start);
#endif
  
  digitalWrite(MAIN_SWITCH_LED_PIN, main_switch.state);
  digitalWrite(MODULE_LED_PIN, module_value);
  digitalWrite(MODULE_SWITCH_LED_PIN, !module_switch.state);

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
  DEBUG_TIME_RECORD(section_start, "LED & Motors");
#endif

#if DEBUG_PRINTS_ENABLED
  DEBUG_PRINT(" | SW=");
  DEBUG_PRINT(main_switch.state);
  DEBUG_PRINT(" BT_SW=");
  DEBUG_PRINT(module_switch.state);
  DEBUG_PRINT(" BT=");
  DEBUG_PRINT(module_value);
  DEBUG_PRINTLN();

  print_sensor_debug_info();
#endif

#if DEBUG_PROFILING_ENABLED
  DEBUG_TIME_RECORD(loop_start, "Target UPS Loop");
#endif
}

void fastest_loop() {
#if DEBUG_PROFILING_ENABLED
  unsigned long fastest_start = 0;
  DEBUG_TIME_START(fastest_start);
#endif

  read_line_sensors();

#if DEBUG_PROFILING_ENABLED
  DEBUG_TIME_RECORD(fastest_start, "Fastest Loop");
#endif
}


void loop() {
#if DEBUG_PROFILING_ENABLED
  unsigned long loop_start = 0;
  DEBUG_TIME_START(loop_start);
#endif

  unsigned long current_time = micros();

  fastest_loop();

  static unsigned long last_sensor_data_send_time = 0;
  if (current_time - last_sensor_data_send_time >= INTERVAL_US) {
    last_sensor_data_send_time = current_time;
    target_ups_loop();
  }

#if DEBUG_PROFILING_ENABLED
  DEBUG_TIME_RECORD(loop_start, "Main Loop");
  DEBUG_TIME_PRINT_IF_READY();
#endif
}
