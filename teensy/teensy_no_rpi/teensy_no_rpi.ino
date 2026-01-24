#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <algorithm>
#include <math.h>
#include <string.h>


// ================================================
// =            In-game variabiles                =
// ================================================

// Speed
struct Speeds {
  int16_t full;
  int16_t main;
  int16_t turn;
  int16_t adjust_turn;
};
Speeds speed = {
  255, // full 255
  70, // main 200
  40, // turn 100
  40, // adjust turn 70
};

// Compass
float opponent_goal_angle = 0; // real opponent goal angle
float target_angle = 180; // opponent goal adjusted to this angle 
float goal_alignment_offset = target_angle - opponent_goal_angle; // centering opponents goal to 180
float heading_tolerance = 15; // tolerated missrotation
float heading_adjust_zone = 60; // slower rotation in here

// IR seeker
float IR_zones[8][3] = { // bottom value, top value, add value (move = IR + add)
  {  0,  60,  +0},
  { 60,  90, +10},
  { 90, 120, +20},
  {120, 180, +30},
  {180, 240, -30},
  {240, 270, -20},
  {270, 300, -10},
  {300, 360,  -0},
};

// ================================================
// =                Definitions                   =
// ================================================

// START/STOP
#define USE_BLUETOOTH_MODULE 1

#define SWITCH_PIN 33
#define MODULE_PIN 32
#define SWITCH_LED_PIN 30
#define MODULE_LED_PIN 31


volatile bool switch_value = false; // starting whith false value
volatile bool module_value = true;  // if off, on first moment gets 0

// Serials
#define RASPBERRY_SERIAL Serial8
#define RASPBERRY_SERIAL_SPEED 38400

#define DEBUG false
#define DEBUG_SERIAL Serial
#define DEBUG_SERIAL_SPEED 38400

void debug_println(const char *msg) {
  if (DEBUG) {
    DEBUG_SERIAL.println(msg);
  }
}

void debug_println(const float msg) {
  if (DEBUG) {
    DEBUG_SERIAL.println(msg);
  }
}

void debug_print(const char *msg) {
  if (DEBUG) {
    DEBUG_SERIAL.print(msg);
  }
}

void debug_print(const float msg) {
  if (DEBUG) {
    DEBUG_SERIAL.print(msg);
  }
}


// Motors
#define MOTOR_COUNT 4
const int motor_pin[MOTOR_COUNT][2] = {
    {6, 5},   // M1PWM, M1DIR
    {8, 7},   // M2PWM, M2DIR
    {10, 9},  // M3PWM, M3DIR
    {12, 11}  // M4PWM, M4DIR
};
const int kicker_pin = 4;  // ON/OFF

int16_t motor_speed[MOTOR_COUNT] = {0, 0, 0, 0};
int8_t kicker_position = 0; // 1 = out (kick) 0 = in

struct motors {
  int16_t motor_speed[MOTOR_COUNT];
  int8_t kicker_position;
};

motors motors_data;


// BNO05
Adafruit_BNO055 bno = Adafruit_BNO055(55);
bool bno_initialized = false;

struct CompassData {
  float heading;
  float relative_heading;
  float pitch;
  float roll;
};
CompassData compass_data;


// MRM-IR_data-Finder3
#define IR_I2C_ADDRESS 0x10
#define IR_ANGLE_AND_DISTANCE_REGISTER 0x00
#define IR_ANGLE_AND_DISTANCE_BYTES_COUNT 4
#define IR_RAW_REGISTER 0x01
#define IR_RAW_BYTES_COUNT 12
#define IR_SENSOR_COUNT 12
#define IR_NO_SIGNAL 999

struct IRData {
  uint16_t angle;                     // DIR
  uint16_t distance;                  // CM
  uint8_t sensor_IR[IR_SENSOR_COUNT]; // RAW values from sensors
  uint8_t status;                     // 13th byte: status/flag
};
IRData IR_data;


// Line sensor
#define LINE_SENSOR_COUNT 16

//const int line_pin[LINE_SENSOR_COUNT] = { 14, 15, 16, 17, 20, 21, 22, 23, 24, 25, 26, 27, 38, 39, 40, 41 };
// pin mapping on teensy does not allow more place-eficient
const int line_pin[LINE_SENSOR_COUNT] = { 23, 22, 21, 20, 24, 25, 17, 16, 15, 14, 26, 27, 41, 40, 39, 38 };



struct LineData {
  int16_t sensor_line[LINE_SENSOR_COUNT];
};
LineData line_data;


// Data collection
#define BNO_DATA_STRING_LENGTH ((4 + 1) * 3)                                                        // -xxx,+yyy,+zzz,
#define IR_SENSOR_DATA_STRING_LENGTH ((3 + 1) + (4 + 1) + (3 + 1) * IR_SENSOR_COUNT + (1 + 1) - 1) // aaa,dddd,12*[vvv,]s,
#define LINE_SENSOR_DATA_STRING_LENGTH ((4 + 1) * LINE_SENSOR_COUNT)                                      // 12*[cccc,]
#define ALL_DATA_LENGTH (BNO_DATA_STRING_LENGTH + IR_SENSOR_DATA_STRING_LENGTH + LINE_SENSOR_DATA_STRING_LENGTH - 1)
// -xxx,+yyy,+zzz,aaa,dddd,12*[vvv,]s,12*[cccc,] - ','
#define DATA_STRING_LENGTH (1 + 5 + ALL_DATA_LENGTH + 1 + 1 + 1) // {"a"="..."}\0
// "a"="+xxx,+yyy,+zzz,aaa,dddd,12*[vvv,]s,12*[cccc,] - ','"\0

struct AllData {
  motors motors_data;
  CompassData compass_data;
  IRData IR_data;
  LineData line_data;
};

AllData all_data;

char message_data[DATA_STRING_LENGTH];



// ================================================
// =                 Functions                    =
// ================================================

// START/STOP
void update_running_state() {
  if (USE_BLUETOOTH_MODULE) {
    module_value = (digitalRead(MODULE_PIN) == HIGH);
  }
  else {
    module_value = (true);
  }
}

// Debug
void print_debug_data(int clear) {
  debug_print("Motors: ");
  for (int i = 0; i < MOTOR_COUNT; i++) {
    debug_print(i);
    debug_print(":");
    debug_print(all_data.motors_data.motor_speed[i]);
    debug_print("   ");
  }
  debug_print("Kicker:");
  debug_println(all_data.motors_data.kicker_position);
  
  debug_print("Heading: ");
  debug_print(all_data.compass_data.heading);
  debug_print("\t\tPitch: ");
  debug_print(all_data.compass_data.pitch);
  debug_print("\tRoll: ");
  debug_println(all_data.compass_data.roll);
  
  debug_print("IR_data_DIR: ");
  debug_print(all_data.IR_data.angle);
  debug_print("   IR_data_Dist: ");
  debug_print(all_data.IR_data.distance);
  debug_print("   ");
  for (int i = 0; i < IR_SENSOR_COUNT; i++) {
    debug_print(i);
    debug_print(":");
    debug_print(all_data.IR_data.sensor_IR[i]);
    debug_print("   ");
  }
  debug_print("Status:  ");
  debug_println(all_data.IR_data.status);
  
  debug_print("Line:  ");
  for (int i = 0; i < LINE_SENSOR_COUNT; i++) {
    debug_print(i);
    debug_print(":");
    debug_print(all_data.line_data.sensor_line[i]);
    debug_print("   ");
  }
  debug_println("");
  debug_println("-------------------------------------------------------------------------");
}

// Helpers
void format_number_with_sign(float input_number, char final_string[], int width) {
  int number = round(input_number);
  if (number < 0) {
    final_string[0] = '-';
  } else {
    final_string[0] = '+';
  }
  int value = abs(number);

  for (int i = width; i > 0; i--) {
    final_string[i] = (value % 10) + '0';
    value /= 10;
  }
  final_string[width + 1] = ',';
  final_string[width + 2] = '\0';
}

void format_number(float input_number, char final_string[], int width) {
  int number = round(input_number);
  int value = abs(number);

  for (int i = width - 1; i >= 0; i--) {
    final_string[i] = (value % 10) + '0';
    value /= 10;
  }
  final_string[width] = ',';
  final_string[width + 1] = '\0';
}

int append_number_to_string(float number, char *target_string, int position, int number_string_length, int sign) {
  char num_str[number_string_length + 3];
  if (sign == 1) {
    format_number_with_sign(number, num_str, number_string_length);
  } else {
    format_number(number, num_str, number_string_length);
  }
  int len = strlen(num_str);
  memcpy(target_string + position, num_str, len);
  return position + len;
}

int append_char_to_string(char c, char *target_string, int position) {
  target_string[position] = c;
  return position + 1;
}

// Main functions

void set_motor_speed(int motor_ID, int speed) {
  int pwm = abs(speed);
  if (speed < 0) {
    digitalWrite(motor_pin[motor_ID][1], LOW);
  } else {
    digitalWrite(motor_pin[motor_ID][1], HIGH);
  }
  analogWrite(motor_pin[motor_ID][0], pwm);
}

void set_all_motors_speed(int16_t m_speed[MOTOR_COUNT]) {
  for (int i = 0; i < MOTOR_COUNT; i++) {
    set_motor_speed(i, m_speed[i]);
    motors_data.motor_speed[i] = m_speed[i];
  }
}


void calculate_motor_speeds(float theta_deg, int16_t speed) {
  // Konverzia uhla na radiány
  float theta = fmod(theta_deg + 90, 360) * PI / 180.0f;

  int16_t motor_speed[MOTOR_COUNT];

  // Rozklad rýchlosti
  float vx = speed * cosf(theta);  // dopredu/dozadu
  float vy = speed * sinf(theta);  // dolava/doprava

  // X-omni konfigurácia
  motor_speed[0] = roundf( vx - vy);  // M0 = front right
  motor_speed[1] = roundf(-vx - vy);  // M1 = back right
  motor_speed[2] = roundf(-vx + vy);  // M2 = back left
  motor_speed[3] = roundf( vx + vy);  // M3 = front left

  // Normalizácia
  int16_t max_val = 0;
  for (int i = 0; i < MOTOR_COUNT; i++) {
    int16_t abs_val = abs(motor_speed[i]);
    if (abs_val > max_val) max_val = abs_val;
  }

  if (max_val > speed) {
    for (int i = 0; i < MOTOR_COUNT; i++) {
      motors_data.motor_speed[i] = motor_speed[i] * speed / max_val;
    }
  }
  else {
    for (int i = 0; i < MOTOR_COUNT; i++) {
      motors_data.motor_speed[i] = motor_speed[i];
    }
  }
}

void move(int16_t dir, int16_t speed) {
  calculate_motor_speeds(dir, speed);
  set_all_motors_speed(motors_data.motor_speed);
}

void turn_right(int16_t speed) {
  for (int i = 0; i < MOTOR_COUNT; i++) {
    motors_data.motor_speed[i] = speed;
  }
  set_all_motors_speed(motors_data.motor_speed);
}

void turn_left(int16_t speed) {
  for (int i = 0; i < MOTOR_COUNT; i++) {
    motors_data.motor_speed[i] = -speed;
  }
  set_all_motors_speed(motors_data.motor_speed);
}

void set_kicker_position(int8_t position) {
  digitalWrite(kicker_pin, position);
  motors_data.kicker_position = position;
}

void stop_motors() {
  for (int i = 0; i < MOTOR_COUNT; i++) {
    set_motor_speed(i, 0);
    motors_data.motor_speed[i] = 0;
  }
  digitalWrite(kicker_pin, 0);
  motors_data.kicker_position = 0;
}

void read_compass() {
  if (!bno_initialized) {
    return;
  }
  sensors_event_t event;
  bno.getEvent(&event);

  compass_data.heading = event.orientation.x;
  compass_data.relative_heading = fmod(compass_data.heading + goal_alignment_offset + 360.0, 360.0);
  compass_data.pitch = event.orientation.y;
  compass_data.roll = event.orientation.z;
}


void read_IR_sensor() {
  Wire.beginTransmission(IR_I2C_ADDRESS);
  Wire.write(IR_ANGLE_AND_DISTANCE_REGISTER);
  Wire.endTransmission();
  Wire.requestFrom(IR_I2C_ADDRESS, IR_ANGLE_AND_DISTANCE_BYTES_COUNT);

  if (Wire.available() >= IR_ANGLE_AND_DISTANCE_BYTES_COUNT) {
    uint16_t angle = Wire.read() << 8;
    angle |= Wire.read();
    uint16_t distance = Wire.read() << 8;
    distance |= Wire.read();
    
    if (angle == 180 && distance == 0) {
      IR_data.angle = IR_NO_SIGNAL;
      IR_data.distance = 0;
    } else {
      IR_data.angle = angle;
      IR_data.distance = distance;
    }
  } else {
    IR_data.angle = IR_NO_SIGNAL;
    IR_data.distance = 0;
    debug_println("No IR_data signal detected");
  }

  Wire.beginTransmission(IR_I2C_ADDRESS);
  Wire.write(IR_RAW_REGISTER);
  Wire.endTransmission();
  Wire.requestFrom(IR_I2C_ADDRESS, IR_RAW_BYTES_COUNT);

  for (int i = 0; i < IR_SENSOR_COUNT; i++) {
    if (Wire.available()) {
      IR_data.sensor_IR[i] = Wire.read();
    } else {
      IR_data.sensor_IR[i] = 0;
    }
  }
  
  if (Wire.available()) {
    IR_data.status = Wire.read();
  } else {
    IR_data.status = 0;
  }
}


void read_line_sensor() {
  for (int i = 0; i < LINE_SENSOR_COUNT; i++) {
    line_data.sensor_line[i] = analogRead(line_pin[i]);
  }
}


void recieve_data() {
  // String input = DEBUG_SERIAL.readStringUntil('\n');
  String input = RASPBERRY_SERIAL.readStringUntil('\n');
  
  if (input.length() != 33 || input[0] != '{' || input[1] != '"' || input[2] != 'a' || input[3] != '"' || input[4] != '=' || input[5] != '"' || input[31] != '"' || input[32] != '}') {
    debug_println("Invalid string!");
    return;
  }

  String data = input.substring(6, 31);

  if (data[5] != ',' || data[11] != ',' || data[17] != ','|| data[30] != ',') {
    debug_println("Wrong comma structure!");
    return;
  }
  
  for (int i = 0; i <= 18; i += 6) {
    if (data[i] != '+' && data[i] != '-') {
      debug_println("Wrong sign!");
      return;
    }
  }
  
  for (int i = 0; i < MOTOR_COUNT; i++) {
    for (int j = 1; j < 5; j++) {
      if (data[i * 6 + j] < '0' || data[i * 6 + j] > '9') {
        debug_print("Wrong number at:");
        debug_println(i * 6 + j);
        return;
      }
    }
  }
  if (data[31] < '0' || data[31] > '9') {
    debug_print("Wrong number at:");
    debug_println(31);
    return;
  }

  int values[MOTOR_COUNT + 1];
  values[0] = data.substring(0, 5).toInt();
  values[1] = data.substring(6, 11).toInt();
  values[2] = data.substring(12, 17).toInt();
  values[3] = data.substring(18, 23).toInt();
  values[4] = data.substring(24, 25).toInt();

  debug_print("Default string: ");
  debug_println(input.c_str());
  debug_println("Extracted values:");
  // for (int i = 0; i < 4; i++) {
  //   debug_println(values[i]);
  // }
  for (int i = 0; i < MOTOR_COUNT; i++) {
    motors_data.motor_speed[i] = values[i];
  }
  motors_data.kicker_position = values[4];
}

void update_all_data() {
  all_data.compass_data = compass_data;
  all_data.IR_data = IR_data;
  all_data.motors_data = motors_data;
  all_data.line_data = line_data;
}


void create_raspberry_message() {
  for (int i = 0; i < DATA_STRING_LENGTH; i++) {
    message_data[i] = '#';
  }

  int position = 0;
  // message start
  position = append_char_to_string('{', message_data, position);
  position = append_char_to_string('"', message_data, position);
  position = append_char_to_string('a', message_data, position);
  position = append_char_to_string('"', message_data, position);
  position = append_char_to_string('=', message_data, position);
  position = append_char_to_string('"', message_data, position);
  // compass
  position = append_number_to_string(all_data.compass_data.heading, message_data, position, 3, 0);
  position = append_number_to_string(all_data.compass_data.pitch, message_data, position, 3, 1);
  position = append_number_to_string(all_data.compass_data.roll, message_data, position, 3, 1);
  // IR sensor
  position = append_number_to_string(all_data.IR_data.angle, message_data, position, 3, 0);
  position = append_number_to_string(all_data.IR_data.distance, message_data, position, 4, 0);
  for (int i = 0; i < IR_SENSOR_COUNT; i++) {
    position = append_number_to_string(all_data.IR_data.sensor_IR[i], message_data, position, 3, 0);
  }
  position = append_number_to_string(all_data.IR_data.status, message_data, position, 1, 0);
  // Line sensor
  for (int i = 0; i < LINE_SENSOR_COUNT; i++) {
    position = append_number_to_string(all_data.line_data.sensor_line[i], message_data, position, 4, 0);
  }
  // message end
  position = position - 1; // delete last ','
  position = append_char_to_string('"', message_data, position);
  position = append_char_to_string('}', message_data, position);
  position = append_char_to_string('\0', message_data, position);
}

void send_raspberry_message() {
  RASPBERRY_SERIAL.println(message_data);
  debug_println(message_data);
}



void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(MODULE_LED_PIN, OUTPUT);
  pinMode(SWITCH_LED_PIN, OUTPUT);

  DEBUG_SERIAL.begin(DEBUG_SERIAL_SPEED);
  RASPBERRY_SERIAL.begin(RASPBERRY_SERIAL_SPEED);

  pinMode(MODULE_PIN, INPUT);
  pinMode(SWITCH_PIN, INPUT_PULLUP);
  // Attach interrupts to both pins
  attachInterrupt(digitalPinToInterrupt(MODULE_PIN), update_running_state, CHANGE);
  //attachInterrupt(digitalPinToInterrupt(SWITCH_PIN), update_running_state, CHANGE); // used as button, not switch

  // Motors
  for (int i = 0; i < 4; i++) {
    pinMode(motor_pin[i][0], OUTPUT);
    pinMode(motor_pin[i][1], OUTPUT);
    analogWrite(motor_pin[i][0], motor_speed[i]);
  }
  pinMode(kicker_pin, OUTPUT);

  // Compass
  if (bno.begin()) {
    bno_initialized = true;
    delay(1000);
    bno.setExtCrystalUse(true);
  } else {
    debug_println("Failed to initialize BNO055!");
    compass_data.heading = 0;
    compass_data.pitch = 0;
    compass_data.roll = 0;
  }

  // IR_data finder
  Wire.begin();
  debug_println("MRM-IR_data-Finder3 dual read start");

  // Line sensor
  for (int i = 0; i < LINE_SENSOR_COUNT; i++) {
    pinMode(line_pin[i], INPUT);
  }
}

// ================================================
// =                  Play                        =
// ================================================

void play() {
  if (target_angle - heading_tolerance <= compass_data.relative_heading && compass_data.relative_heading <= target_angle + heading_tolerance) {
    Serial.print("Rotation: --OK--");
    go_to_ball();
  }
  else if (target_angle - heading_adjust_zone < compass_data.relative_heading && compass_data.relative_heading < target_angle - heading_tolerance) {
    Serial.print("Rotation: Adjust");
    turn_right(speed.adjust_turn);
  }
  else if (target_angle + heading_tolerance < compass_data.relative_heading && compass_data.relative_heading < target_angle + heading_adjust_zone) {
    Serial.print("Rotation: Adjust");
    turn_left(speed.adjust_turn);
  }
  else if (compass_data.relative_heading < target_angle - heading_adjust_zone) {
    Serial.print("Rotation: Wrong!");
    turn_right(speed.turn);
  }
  else if (target_angle + heading_adjust_zone < compass_data.relative_heading) {
    Serial.print("Rotation: Wrong!");
    turn_left(speed.turn);
  }
  else Serial.println("Rotation: ERROR!");
}

void go_to_ball() {
  if (IR_data.angle == IR_NO_SIGNAL) {
    stop_motors();
    Serial.println(IR_NO_SIGNAL);
  }
  //move(IR_data.angle, speed.main);
  for (int i = 0; i < 8; i++) {
    if (IR_zones[i][0] < IR_data.angle && IR_data.angle < IR_zones[i][1]) {
      move(IR_data.angle + IR_zones[i][2], speed.main);
      Serial.print("   Zone: ");
      Serial.print(i);
      Serial.print("  IR_VALUE: ");
      Serial.println(IR_data.angle);
    }
  }
}

void loop() {
  if (!digitalRead(SWITCH_PIN)) {
    switch_value = !switch_value;
    delay(100);
  }

  read_compass();
  read_IR_sensor();
  read_line_sensor();
  update_all_data();

  if (DEBUG) { print_debug_data(0); }

  // Printout states and turn on LEDs
  Serial.print("SWITCH: ");
  Serial.print(switch_value);
  Serial.print("    MODULE: ");
  Serial.print(module_value);
  (switch_value) ? digitalWrite(SWITCH_LED_PIN, HIGH) : digitalWrite(SWITCH_LED_PIN, LOW);
  (module_value) ? digitalWrite(MODULE_LED_PIN, HIGH) : digitalWrite(MODULE_LED_PIN, LOW);
  (module_value && switch_value) ? digitalWrite(LED_BUILTIN, HIGH) : digitalWrite(LED_BUILTIN, LOW);
  switch_value = 1; //------------------------------------------------------------------------------------------------------------------------------------------------------------
  if (module_value && switch_value) {
    Serial.println("        MOVE!");
    play();
  }
  else {
    stop_motors();
    Serial.println("        STOP!");
  }
  delay(10);
}
