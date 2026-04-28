#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <math.h>


// ================================================
// =                Definitions                   =
// ================================================

// ========== Control Switches ==========
#define SWITCH_PIN 31
#define SWITCH_LED_PIN 28
#define MODULE_SWITCH_PIN 32
#define MODULE_SWITCH_LED_PIN 29
#define MODULE_PIN 33
#define MODULE_LED_PIN 30

// Control state variables
volatile bool use_bluetooth_module = true;
volatile bool switch_value = false;  // Manual switch state
volatile bool module_value = true;   // Bluetooth module state
volatile bool run = false;           // Overall run state

// ========== Serial Communication ==========
#define TARGET_MESSAGES_PER_SECOND 120

#define RASPBERRY_SERIAL Serial8
#define RASPBERRY_SERIAL_SPEED 1000000//DATA_STRING_LENGTH * 10 * TARGET_MESSAGES_PER_SECOND  // = 84000

#define DEBUG_PRINTS_ENABLED false
#define DEBUG_LOGS_ENABLED true
#define DEBUG_PERFORMANCE_ENABLED true
#define DEBUG_SERIAL Serial
#define DEBUG_SERIAL_SPEED 38400

// Debug levels
enum DebugLevel {
  DEBUG_INFO,
  DEBUG_WARN,
  DEBUG_ERROR
};

// Enhanced debug system with levels and formatting
template<typename T>
inline void debug_print(const T& msg) {
#if DEBUG_PRINTS_ENABLED
  DEBUG_SERIAL.print(msg);
#endif
}

template<typename T>
inline void debug_println(const T& msg) {
#if DEBUG_PRINTS_ENABLED
  DEBUG_SERIAL.println(msg);
#endif
}

inline void debug_println() {
#if DEBUG_PRINTS_ENABLED
  DEBUG_SERIAL.println();
#endif
}

// Debug with level prefix
template<typename T>
void debug_log(DebugLevel level, const T& msg) {
#if DEBUG_LOGS_ENABLED
  switch (level) {
    case DEBUG_INFO: DEBUG_SERIAL.print("[INFO]  "); break;
    case DEBUG_WARN: DEBUG_SERIAL.print("[WARN]  "); break;
    case DEBUG_ERROR: DEBUG_SERIAL.print("[ERROR] "); break;
  }
  DEBUG_SERIAL.println(msg);
#endif
}

void debug_time_spent(String label = "") {
#if DEBUG_PERFORMANCE_ENABLED
  static unsigned long last_time = 0;
  unsigned long current_time = millis();
  unsigned long time_diff = current_time - last_time;
  debug_log(DEBUG_INFO, label + String(time_diff) + " ms");
  last_time = current_time;
#endif
}


// ========== Motors Configuration ==========
#define MOTOR_COUNT 4
#define PWM_INDEX 0
#define DIR_INDEX 1
#define MOTOR_STOPPED 0
#define KICKER_OUT 1
#define KICKER_IN 0

// Motor pin configuration: [PWM_PIN, DIR_PIN]
const int motor_pin[MOTOR_COUNT][2] = {
  { 6, 5 },   // M1: PWM=6, DIR=5
  { 8, 7 },   // M2: PWM=8, DIR=7
  { 10, 9 },  // M3: PWM=10, DIR=9
  { 12, 11 }  // M4: PWM=12, DIR=11
};
const int kicker_pin = 4;

// Motor state structure
struct MotorsData {
  int16_t motor_speed[MOTOR_COUNT];
  int8_t kicker_position;  // 0=in, 1=out
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
#define IR_NO_SIGNAL 999  // Special value indicating no ball detected
#define IR_NO_SIGNAL_ANGLE 180
#define IR_NO_SIGNAL_DISTANCE 0

struct IRData {
  uint16_t angle;                      // Ball direction (0-359 degrees, or 999 if no signal)
  uint16_t distance;                   // Ball distance in cm
  uint8_t sensor_IR[IR_SENSOR_COUNT];  // Raw values from 12 IR sensors
  uint8_t status;                      // Status byte from sensor
};
IRData ir_data = { IR_NO_SIGNAL, 0, { 0 }, 0 };


// ========== Line Sensors ==========
#define LINE_SENSOR_COUNT 12

// Teensy analog pin mapping for line sensors
// Note: Non-sequential due to Teensy hardware constraints
const int line_pin[LINE_SENSOR_COUNT] = {
  23, 22, 21, 20,  // Sensors 0-3
  17, 16, 15, 14,  // Sensors 4-7
  41, 40, 39, 38   // Sensors 8-11
};

struct LineData {
  int16_t sensor_line[LINE_SENSOR_COUNT];  // Analog values (0-1023)
  int16_t sensor_line_min[LINE_SENSOR_COUNT];  // Analog values (0-1023)
  int16_t sensor_line_max[LINE_SENSOR_COUNT];  // Analog values (0-1023)
};
LineData line_data = { { 0 } };


// ========== Data Collection & Message Format ==========
// Message format: {"a"="HHH,±PPP,±RRR,AAA,DDDD,V1,V2,...V12,S,L1,L2,...L12"}
#define DATA_STRING_LENGTH 141

struct SensorData {
  MotorsData motors_data;
  CompassData compass_data;
  IRData ir_data;
  LineData line_data;
};

SensorData sensor_data;
char tx_message_buffer[DATA_STRING_LENGTH];



// ================================================
// =                 Functions                    =
// ================================================

// ========== Control State Management ==========
void update_running_state() {
  module_value = use_bluetooth_module ? (digitalRead(MODULE_PIN) == HIGH) : true;
  run = (switch_value && module_value);
}

// ========== Debug Functions ==========
void print_sensor_debug_info() {
  // Motor status
  debug_print("Motors: ");
  for (int i = 0; i < MOTOR_COUNT; i++) {
    debug_print("M");
    debug_print(i);
    debug_print("=");
    debug_print(sensor_data.motors_data.motor_speed[i]);
    debug_print(" ");
  }
  debug_print(" | Kicker=");
  debug_println(sensor_data.motors_data.kicker_position);

  // Compass orientation
  debug_print("Compass: H=");
  debug_print(sensor_data.compass_data.heading);
  debug_print(" P=");
  debug_print(sensor_data.compass_data.pitch);
  debug_print(" R=");
  debug_println(sensor_data.compass_data.roll);

  // IR sensor ball detection
  debug_print("IR Ball: Angle=");
  debug_print(sensor_data.ir_data.angle);
  debug_print(" Dist=");
  debug_print(sensor_data.ir_data.distance);
  debug_print("cm | Sensors: ");
  for (int i = 0; i < IR_SENSOR_COUNT; i++) {
    debug_print(sensor_data.ir_data.sensor_IR[i]);
    debug_print(" ");
  }
  debug_print("| Status=");
  debug_println(sensor_data.ir_data.status);

  // Line sensors
  debug_print("Line: Value: ");
  for (int i = 0; i < LINE_SENSOR_COUNT; i++) {
    debug_print(sensor_data.line_data.sensor_line[i]);
    debug_print(" ");
  }
  debug_print("Min: ");
  for (int i = 0; i < LINE_SENSOR_COUNT; i++) {
    debug_print(sensor_data.line_data.sensor_line_min[i]);
    debug_print(" ");
  }
  debug_print("Max: ");
  for (int i = 0; i < LINE_SENSOR_COUNT; i++) {
    debug_print(sensor_data.line_data.sensor_line_max[i]);
    debug_print(" ");
  }
  debug_println();
  debug_println("========================================");
}

// ========== String Formatting Helpers ==========
void format_number_with_sign(float input_number, char final_string[], int width) {
  int number = round(input_number);
  final_string[0] = (number < 0) ? '-' : '+';
  int value = abs(number);

  for (int i = width; i > 0; i--) {
    final_string[i] = (value % 10) + '0';
    value /= 10;
  }
}

void format_number(float input_number, char final_string[], int width) {
  int number = round(input_number);
  int value = abs(number);

  for (int i = width - 1; i >= 0; i--) {
    final_string[i] = (value % 10) + '0';
    value /= 10;
  }
}

inline int append_number_to_string(float number, char* target_string, int position, int number_string_length, bool with_sign) {
  if (with_sign) {
    format_number_with_sign(number, target_string + position, number_string_length);
    position += number_string_length + 1;
  } else {
    format_number(number, target_string + position, number_string_length);
    position += number_string_length;
  }
  target_string[position++] = ',';
  return position;
}

inline int append_char_to_string(char c, char* target_string, int position) {
  target_string[position] = c;
  return position + 1;
}

// ========== Motor Control Functions ==========
void set_motor_speed(int motor_ID, int speed) {
  if (motor_ID < 0 || motor_ID >= MOTOR_COUNT) return;

  int pwm = constrain(abs(speed), 0, 255);
  digitalWrite(motor_pin[motor_ID][DIR_INDEX], (speed >= 0) ? HIGH : LOW);
  analogWrite(motor_pin[motor_ID][PWM_INDEX], pwm);
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
    debug_log(DEBUG_INFO, "BNO055 initialized successfully");
    bno_initialized = true;
    bno.setExtCrystalUse(true);
    return true;
  } else {
    debug_log(DEBUG_WARN, "BNO055 not found");
    bno_initialized = false;
    return false;
  }
}

void read_compass() {
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
      debug_log(DEBUG_ERROR, "BNO055 error detected during read: " + String(system_error));
      bno_initialized = false;
      compass_data.heading = 999;
      compass_data.pitch = 999;
      compass_data.roll = 999;
    }
  }
}


void read_ir_sensor() {
  Wire.beginTransmission(IR_I2C_ADDRESS);
  Wire.write(IR_ANGLE_AND_DISTANCE_REGISTER);
  if (Wire.endTransmission() != 0) {
    debug_log(DEBUG_ERROR, "IR sensor communication error");
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
}


void read_line_sensors() {
  for (int i = 0; i < LINE_SENSOR_COUNT; i++) {
    line_data.sensor_line[i] = analogRead(line_pin[i]);
    if (line_data.sensor_line_max[i] < line_data.sensor_line[i])
      line_data.sensor_line_max[i] = line_data.sensor_line[i];
    if (line_data.sensor_line_min[i] > line_data.sensor_line[i])
      line_data.sensor_line_min[i] = line_data.sensor_line[i];
  }
}

void reset_line_values() {
  for (int i = 0; i < LINE_SENSOR_COUNT; i++) {
    line_data.sensor_line_max[i] = 0;
    line_data.sensor_line_min[i] = 1023;
  }
}


// ========== Communication Functions ==========
bool parse_motor_command() {
  // Expected format: {"a"="±MMMM,±MMMM,±MMMM,±MMMM,K"}
  // MMMM = motor speed (signed, 0 to 255), K = kicker (0 or 1)

  String input = "";
  while (RASPBERRY_SERIAL.available() > 0) {
    input = RASPBERRY_SERIAL.readStringUntil('\n');
  }

  if (input.length() == 0) return false;

  debug_print("RX: ");
  debug_println(input.c_str());

  const int EXPECTED_LENGTH = 33;
  if (input.length() != EXPECTED_LENGTH) {
    debug_log(DEBUG_ERROR, "Invalid message length: \"" + input + "\"");
    return false;
  }

  if (input[0] != '{' || input[1] != '"' || input[2] != 'a' || input[3] != '"' || input[4] != '=' || input[5] != '"' || input[31] != '"' || input[32] != '}') {
    debug_log(DEBUG_ERROR, "Invalid JSON structure: \"" + input + "\"");
    return false;
  }

  String data = input.substring(6, 31);  // "±MMMM,±MMMM,±MMMM,±MMMM,K"

  if (data[5] != ',' || data[11] != ',' || data[17] != ',' || data[23] != ',') {
    debug_log(DEBUG_ERROR, "Invalid comma positions: \"" + input + "\"");
    return false;
  }

  for (int i = 0; i < MOTOR_COUNT; i++) {
    char sign = data[i * 6];
    if (sign != '+' && sign != '-') {
      debug_log(DEBUG_ERROR, "Invalid sign character: \"" + input + "\"");
      return false;
    }

    for (int j = 1; j <= 4; j++) {
      if (!isdigit(data[i * 6 + j])) {
        debug_log(DEBUG_ERROR, "Invalid motor value digit: \"" + input + "\"");
        return false;
      }
    }
  }

  if (!isdigit(data[24])) {
    debug_log(DEBUG_ERROR, "Invalid kicker value: \"" + input + "\"");
    return false;
  }

  for (int i = 0; i < MOTOR_COUNT; i++) {
    motors_data.motor_speed[i] = data.substring(i * 6, i * 6 + 5).toInt();
  }

  motors_data.kicker_position = data.substring(24, 25).toInt();
  motors_data.kicker_position = constrain(motors_data.kicker_position, KICKER_IN, KICKER_OUT);

  return true;
}

inline void update_sensor_data_struct() {
  sensor_data.compass_data = compass_data;
  sensor_data.ir_data = ir_data;
  sensor_data.motors_data = motors_data;
  sensor_data.line_data = line_data;
}


void build_sensor_message() {
  int pos = 0;

  // JSON wrapper start: {"a"="
  pos = append_char_to_string('{', tx_message_buffer, pos);
  pos = append_char_to_string('"', tx_message_buffer, pos);
  pos = append_char_to_string('a', tx_message_buffer, pos);
  pos = append_char_to_string('"', tx_message_buffer, pos);
  pos = append_char_to_string('=', tx_message_buffer, pos);
  pos = append_char_to_string('"', tx_message_buffer, pos);

  // Compass data: HHH,±PPP,±RRR,
  pos = append_number_to_string(sensor_data.compass_data.heading, tx_message_buffer, pos, 3, false);
  pos = append_number_to_string(sensor_data.compass_data.pitch, tx_message_buffer, pos, 3, true);
  pos = append_number_to_string(sensor_data.compass_data.roll, tx_message_buffer, pos, 3, true);

  // IR data: AAA,DDDD,V1,...V12,S,
  pos = append_number_to_string(sensor_data.ir_data.angle, tx_message_buffer, pos, 3, false);
  pos = append_number_to_string(sensor_data.ir_data.distance, tx_message_buffer, pos, 4, false);
  for (int i = 0; i < IR_SENSOR_COUNT; i++) {
    pos = append_number_to_string(sensor_data.ir_data.sensor_IR[i], tx_message_buffer, pos, 3, false);
  }
  pos = append_number_to_string(sensor_data.ir_data.status, tx_message_buffer, pos, 1, false);

  // Line sensor data: L1,...L12
  for (int i = 0; i < LINE_SENSOR_COUNT; i++) {
    pos = append_number_to_string(sensor_data.line_data.sensor_line[i], tx_message_buffer, pos, 4, false);
  }

  // JSON wrapper end: "} (remove trailing comma)
  pos--;
  pos = append_char_to_string('"', tx_message_buffer, pos);
  pos = append_char_to_string('}', tx_message_buffer, pos);
  tx_message_buffer[pos] = '\0';
}

void transmit_sensor_data() {
  RASPBERRY_SERIAL.println(tx_message_buffer);
  debug_print("TX: ");
  debug_println(tx_message_buffer);
}


void transmit_running_state() {
  String switch_message = "{\"b\"=\"";
  switch_message += (run ? 'R' : 'S');
  switch_message += (use_bluetooth_module ? 'B' : 'N');
  switch_message += (switch_value ? 'S' : 's');
  switch_message += (module_value ? 'M' : 'm');
  switch_message += "\"}";

  RASPBERRY_SERIAL.println(switch_message);
}



// ========== Setup & Main Loop ==========
void setup() {
  // Initialize LED pins
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(MODULE_LED_PIN, OUTPUT);
  pinMode(MODULE_SWITCH_LED_PIN, OUTPUT);
  pinMode(SWITCH_LED_PIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  // Initialize serial communication
  DEBUG_SERIAL.begin(DEBUG_SERIAL_SPEED);
  RASPBERRY_SERIAL.begin(RASPBERRY_SERIAL_SPEED);
  delay(100);
  debug_println("\n=== LNX Infrabot Teensy Controller ===");
  debug_println("Initializing...");

  // Initialize control pins and interrupts
  pinMode(MODULE_PIN, INPUT);
  pinMode(SWITCH_PIN, INPUT_PULLUP);
  pinMode(MODULE_SWITCH_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(MODULE_PIN), update_running_state, CHANGE);
  update_running_state();
  debug_println("Control switches initialized");

  // Initialize motor pins
  for (int i = 0; i < MOTOR_COUNT; i++) {
    pinMode(motor_pin[i][PWM_INDEX], OUTPUT);
    pinMode(motor_pin[i][DIR_INDEX], OUTPUT);
  }
  pinMode(kicker_pin, OUTPUT);
  stop_motors();  // Ensure motors start stopped
  debug_println("Motors initialized");

  // Initialize BNO055 compass
  bno_initialize();

  // Initialize I2C for IR sensor
  Wire.begin();
  debug_println("IR sensor initialized (MRM-IR-Finder3)");
  Wire.setClock(400000); // 400 kHz (Fast Mode)
  debug_println("I2C initialized (400 kHz)");

  // Initialize line sensor pins
  for (int i = 0; i < LINE_SENSOR_COUNT; i++) {
    pinMode(line_pin[i], INPUT);
  }
  debug_println("Line sensors initialized");

  reset_line_values();

  debug_println("=== Initialization complete ===");
  debug_println();

  debug_log(DEBUG_INFO, "ASDADASDASDASD: " + String(RASPBERRY_SERIAL.availableForWrite()));
}


void loop() {
  debug_time_spent("start loop: ");

  unsigned long current_time = millis();

  static unsigned long last_switch_time = 0;
  static unsigned long last_module_switch_time = 0;
  static unsigned long last_running_state_transmit_time = 0;

  if (!digitalRead(SWITCH_PIN) && (current_time - last_switch_time > 200)) {
    switch_value = !switch_value;
    update_running_state();
    transmit_running_state();

    last_switch_time = current_time;
    debug_log(DEBUG_INFO, switch_value ? "Manual switch: ON" : "Manual switch: OFF");
  }

  if (!digitalRead(MODULE_SWITCH_PIN) && (current_time - last_module_switch_time > 200)) {
    use_bluetooth_module = !use_bluetooth_module;
    update_running_state();
    transmit_running_state();

    last_module_switch_time = current_time;
    debug_log(DEBUG_INFO, use_bluetooth_module ? "Bluetooth: ENABLED" : "Bluetooth: DISABLED");
  }

  debug_time_spent("after switch checks: ");

  if (last_module_switch_time + 1000 < current_time && last_switch_time + 1000 < current_time && last_running_state_transmit_time + 1000 < current_time) {
    transmit_running_state();
    last_running_state_transmit_time = current_time;
  }

  debug_time_spent("after running state transmit: ");

  read_ir_sensor();
  debug_time_spent("after IR read: ");
  read_compass();
  debug_time_spent("after compass read: ");
  read_line_sensors();
  debug_time_spent("after line reads: ");
  update_sensor_data_struct();

  debug_time_spent("after sensor reads: ");

  // debug_log(DEBUG_INFO, "ASDADASDASDASD: " + String(RASPBERRY_SERIAL.availableForWrite()));

  // if (RASPBERRY_SERIAL.availableForWrite() > 0) {
    build_sensor_message();
    debug_time_spent("after building message: ");
    transmit_sensor_data();
  // }

  // debug_log(DEBUG_INFO, "ASDADASDASDASD: " + String(RASPBERRY_SERIAL.availableForWrite()));

  debug_time_spent("after communication: ");

  #if DEBUG_LOGS_ENABLED
    static int messages_sent = 0;
    static unsigned long last_sent_message_time = 0;

    messages_sent++;

    if (current_time - last_sent_message_time >= 1000) {
      debug_log(DEBUG_INFO, "MSGs/s: " + String(messages_sent));
      last_sent_message_time = current_time;
      messages_sent = 0;
    }
  #endif

  parse_motor_command();

  debug_time_spent("after command parsing: ");

  digitalWrite(SWITCH_LED_PIN, switch_value);
  digitalWrite(MODULE_LED_PIN, module_value);
  digitalWrite(MODULE_SWITCH_LED_PIN, !use_bluetooth_module);

  run = (switch_value && module_value);
  digitalWrite(LED_BUILTIN, run);
  
  debug_time_spent("after LED updates: ");

  if (run) {
    set_all_motors_speed(motors_data.motor_speed);
    set_kicker_position(motors_data.kicker_position);
    debug_print(">>> RUNNING");
  } else {
    stop_motors();
    debug_print(">>> STOPPED");
  }

#if DEBUG_PRINTS_ENABLED
  debug_print(" | SW=");
  debug_print(switch_value);
  debug_print(" BT_SW=");
  debug_print(use_bluetooth_module);
  debug_print(" BT=");
  debug_print(module_value);
  debug_println();

  print_sensor_debug_info();
#endif

  //delay(1);
}
