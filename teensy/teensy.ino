#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <math.h>
#include <algorithm>
#include <string.h>





//---------------------------Motory---------------------------
#define Motor_amount 4
const int motor_pin[Motor_amount][2] = {
  {5, 4}, // M1PWM, M1DIR
  {33, 33}, // M2PWM, M2DIR
  {33, 33}, // M3PWM, M3DIR
  {33, 33} // M4PWM, M4DIR
};

int16_t motor_speed[Motor_amount] = {0, 0, 0, 0};

struct motors {
  int16_t motor_speed[Motor_amount];
};

motors motors_data;
//-------------------------BNO05------------------
Adafruit_BNO055 bno = Adafruit_BNO055(55);

struct CompassData {
  float heading;
  float pitch;
  float roll;
};
CompassData compass_data;
//-------------------------MRM-IR_data-Finder3-------------------
#define I2C_ADDRESS 0x10                   // Adress MRM‑IR‑Finder3
#define REGISTER_ANGLE_AND_DISTANCE 0x00   // vIR_datatual register pre dir and dist
#define REGISTER_RAW 0x01                  // vIR_datatual register for RAW data
#define DATA_COUNT_ANGLE 4                 // 4 bytes dir + dist
#define DATA_COUNT_RAW 12                  // 12 bytes for RAW data mode
#define IR_SENSOR_AMOUNT 12           // sensor amount

#define NO_SIGNAL 999                     // no signal from mrm / no ball found

struct IRData {
  uint16_t angle;       // DIR
  uint16_t distance;    // CM
  uint8_t sensor_IR[IR_SENSOR_AMOUNT]; // RAW values from sensors
  uint8_t status;       // 13. byte = status / flag
};
IRData IR_data;
//Line sensor
#define LINE_SENSOR_AMOUNT 16

const int line_pin[LINE_SENSOR_AMOUNT] = {
  14, 15, 16, 17, 20, 21, 22, 23, 24, 25, 26, 27, 38, 39, 40, 41
};

struct LineData {
  int16_t sensor_line[LINE_SENSOR_AMOUNT];
};
LineData line_data;

// Data collection and raspberry -------------------
#define RASPBERRY_SERIAL_SPEED 38400

#define BNO_DATA_LENGHT ( (4 +1)*3 ) // -xxx,+yyy,+zzz,
#define IR_SENSOR_DATA_LENGHT ( (3 + 1) + (4 + 1) + (3 + 1)*IR_SENSOR_AMOUNT + (1 + 1) - 1 ) // aaa,dddd,12*[vvv,]s,
#define LINE_SENSOR_DATA ( (4 + 1)*LINE_SENSOR_AMOUNT ) // 12*[cccc,]
#define ALL_DATA_LENGHT (BNO_DATA_LENGHT + IR_SENSOR_DATA_LENGHT + LINE_SENSOR_DATA - 1)
  // -xxx,+yyy,+zzz,aaa,dddd,12*[vvv,]s,12*[cccc,] - ','
#define DATA_STRING_LENGHT (1 + 5 + ALL_DATA_LENGHT + 1 + 1 + 1) // {"a"="..."}\0
  // "a"="+xxx,+yyy,+zzz,aaa,dddd,12*[vvv,]s,12*[cccc,] - ','"\0


struct AllData {
  motors motors_data;
  CompassData compass_data;
  IRData IR_data;
  LineData line_data;
};

AllData all_data;

char message_data[DATA_STRING_LENGHT];

//---------------------------Functions--------------------------------------
int TEST = 0;// if 1 print all values by serial


// Motors
void turn_on_motor(int motor_ID, int speed) {
  int pwm = abs(speed);
  if(speed < 0){
      digitalWrite(motor_pin[motor_ID][1], LOW);
  } else {
      digitalWrite(motor_pin[motor_ID][1], HIGH);
  }
  analogWrite(motor_pin[motor_ID][0], pwm);
}

void motors_on(int16_t m_speed[Motor_amount]) {

  for (int i = 0; i < Motor_amount; i++) {
    turn_on_motor(i, m_speed[i]);
      motors_data.motor_speed[i] = m_speed[i];
  }
}


// Compass
void read_compass() {
  //static float angles[3]; // X = Heading, Y = Pitch, Z = Roll
  sensors_event_t event;
  bno.getEvent(&event);

  compass_data.heading = event.orientation.x; // Heading
  compass_data.pitch = event.orientation.y; // Pitch
  compass_data.roll = event.orientation.z; // Roll
}

//  IR_data
void read_IR() {
  
  Wire.beginTransmission(I2C_ADDRESS);
  Wire.write(REGISTER_ANGLE_AND_DISTANCE);
  Wire.endTransmission();
  Wire.requestFrom(I2C_ADDRESS, DATA_COUNT_ANGLE);

  if (Wire.available() >= DATA_COUNT_ANGLE) {
    uint16_t angle = Wire.read() << 8;
    angle |= Wire.read();
    uint16_t distance = Wire.read() << 8;
    distance |= Wire.read();
    // save
    if (angle == 180 && distance == 0) {
      IR_data.angle = NO_SIGNAL; // default „no signal“
      IR_data.distance = 0;
    } else {
      IR_data.angle = angle;
      IR_data.distance = distance;
    }
  } else {
      IR_data.angle = NO_SIGNAL; // default „no signal“
      IR_data.distance = 0;
      Serial.println("No IR_data signal detected");
  }

  // ---  12 RAW bytes ---
  Wire.beginTransmission(I2C_ADDRESS);
  Wire.write(REGISTER_RAW);
  Wire.endTransmission();
  Wire.requestFrom(I2C_ADDRESS, DATA_COUNT_RAW);

  for (int i = 0; i < IR_SENSOR_AMOUNT; i++) {
      if (Wire.available()) {
          IR_data.sensor_IR[i] = Wire.read();
      } else {
          IR_data.sensor_IR[i] = 0;
      }
  }
  // --- last byte = status ---
  if (Wire.available()) {
      IR_data.status = Wire.read();
  } else {
      IR_data.status = 0;
  }
}


// Line sensor
void read_line() {
  for (int i = 0; i < LINE_SENSOR_AMOUNT; i++) {
    line_data.sensor_line[i] = analogRead(line_pin[i]);
  }
}

// Data collection & Raspberry comunication-----------------------------------------------------------------------
void recieve_data() {
  //String input = Serial.readStringUntil('\n'); // reading line
  String input = Serial8.readStringUntil('\n'); // reading line
  // main lenght and format chcek
  if (input.length() != 31 || input[0] != '{' || input[1] != '"' || input[2] != 'a' || input[3] != '"' || input[4] != '=' || input[5] != '"'
                  || input[29] != '"' || input[30] != '}' ) {
    Serial.println("Invalid string!");
    return;
  }
  String data = input.substring(6, 29); // deleting start end ending of string
  // comma check
  if (data[5] != ',' || data[11] != ',' || data[17] != ',') {
    Serial.println("Wrong comma structure!");
    return;
  }
  // sign chcek
  for (int i = 0; i <= 18; i += 6) { 
    if (data[i] != '+' && data[i] != '-') {
      Serial.println("Wrong sign!");
      return;
    }
  }
  // numbers check
  for (int i = 0; i < 4; i++) {
    for (int j = 1; j < 5; j++) {
      if (data[i*6 + j] < '0' || data[i*6 + j] > '9') {
        Serial.print("Wrong number at:");
        Serial.println(i*6 + j);
        return;
      }
    }
  }
  int values[4];
  values[0] = data.substring(0, 5).toInt();
  values[1] = data.substring(6, 11).toInt();
  values[2] = data.substring(12, 17).toInt();
  values[3] = data.substring(18, 23).toInt();

  Serial.print("Default string: ");
  Serial.println(input);
  Serial.println("Extracted values:");
  /*for (int i = 0; i < 4; i++) {
    Serial.println(values[i]);
  }*/
  for (int i = 0; i < 4; i++) {
    motors_data.motor_speed[i] = values[i];
  }
}



void Save_data() {
  all_data.compass_data = compass_data;
  all_data.IR_data      = IR_data;
  all_data.motors_data  = motors_data;
  all_data.line_data    = line_data;
}

void Print_data(int clear) {
  // Motors
  Serial.print("Motors: ");
  for (int i = 0; i < Motor_amount; i++) {
    Serial.print(i);
    Serial.print(":");
    Serial.print(all_data.motors_data.motor_speed[i]);
    Serial.print("   ");
  }
  Serial.println();
  // Compass
  Serial.print("Heading: ");
  Serial.print(all_data.compass_data.heading);
  Serial.print("\t\tPitch: ");
  Serial.print(all_data.compass_data.pitch);
  Serial.print("\tRoll: ");
  Serial.println(all_data.compass_data.roll);
  // IR
  Serial.print("IR_data_DIR: ");
  Serial.print(all_data.IR_data.angle);
  Serial.print("   IR_data_Dist: ");
  Serial.print(all_data.IR_data.distance);
  Serial.print("   ");
  for (int i = 0; i < IR_SENSOR_AMOUNT; i++) {
    Serial.print(i);
    Serial.print(":");
    Serial.print(all_data.IR_data.sensor_IR[i]);
    Serial.print("   ");
  }
  Serial.print("Status:  ");
  Serial.println(all_data.IR_data.status);
  // Line
  Serial.print("Line:  ");
  for (int i = 0; i < LINE_SENSOR_AMOUNT; i++) {
    Serial.print(i);
    Serial.print(":");
    Serial.print(all_data.line_data.sensor_line[i]);
    Serial.print("   ");
  }
  Serial.println("");
  // END LINE
  Serial.println("-------------------------------------------------------------------------");
}

// Getting ready comunicat string---------------------
void format_number_w_sign(float input_number, char final_string[], int width) {
  int number = round(input_number);
  if (number < 0) {
    final_string[0] = '-';
  }
  else {
    final_string[0] = '+';
  }
  int value = abs(number);
  int final_string_size = width + 2;

  for (int i = width; i > 0; i--) {
    final_string[i] = (value % 10) + '0';
    value /= 10;
  }
  final_string[width + 1] = ',';  // message split
  final_string[width + 2] = '\0'; // string ending
}

void format_number_wn_sign(float input_number, char final_string[], int width) {
  int number = round(input_number);
  int value = abs(number);
  int final_string_size = width + 1;

  for (int i = width - 1; i >= 0; i--) {
    final_string[i] = (value % 10) + '0';
    value /= 10;
  }
  final_string[width] = ',';  // message split
  final_string[width + 1] = '\0'; // string ending
}

int append_number_to_string(float number, char* target_string, int position, int number_size, int sign) {
  char hold_number[10]; // buffer pre jedno číslo
  if (sign == 1) { // formating number
    format_number_w_sign(number, hold_number, number_size);
  }
  else {
    format_number_wn_sign(number, hold_number, number_size);
  }
  int len = strlen(hold_number);                         // final lenght
  memcpy(target_string + position, hold_number, len);    // copy to main string
  return position + len;                                 // next position
}
// similar than number, but append just one char
int append_char_to_string(char c, char* target_string, int position) {
    target_string[position] = c;
    return position + 1;
}

void creat_comunication_string() { // formating messsage data into easy-sendable form
  for (int i = 0; i < DATA_STRING_LENGHT; i ++) {
    message_data[i] = '#';
  }
  int position = 0;
  // message start
  position = append_char_to_string('{',                                   message_data, position);
  position = append_char_to_string('"',                                   message_data, position);
  position = append_char_to_string('a',                                   message_data, position);
  position = append_char_to_string('"',                                   message_data, position);
  position = append_char_to_string('=',                                   message_data, position);
  position = append_char_to_string('"',                                   message_data, position);
  // compass
  position = append_number_to_string(all_data.compass_data.heading,       message_data, position, 3, 0);
  position = append_number_to_string(all_data.compass_data.pitch,         message_data, position, 3, 1);
  position = append_number_to_string(all_data.compass_data.roll,          message_data, position, 3, 1);
  // IR sensor
  position = append_number_to_string(all_data.IR_data.angle,              message_data, position, 3, 0);
  position = append_number_to_string(all_data.IR_data.distance,           message_data, position, 4, 0);
  for (int i = 0; i < IR_SENSOR_AMOUNT; i++) {
    position = append_number_to_string(all_data.IR_data.sensor_IR[i],     message_data, position, 3, 0);
  }
  position = append_number_to_string(all_data.IR_data.status,             message_data, position, 1, 0);
  // Line sensor
  for (int i = 0; i < LINE_SENSOR_AMOUNT; i++) {
    position = append_number_to_string(all_data.line_data.sensor_line[i], message_data, position, 4, 0);
  }
  // message end
  position = position - 1; // delete last ','
  position = append_char_to_string('"',                                   message_data, position);
  position = append_char_to_string('}',                                   message_data, position);
  position = append_char_to_string('\0',                                  message_data, position);
}

void send_message() {
  Serial8.println(message_data);
  Serial.println(message_data);
}

//--------------------------------------SETUP------------------------------------------
void setup() {
  // put your setup code here, to run once:
  Serial.begin(38400);
  pinMode(LED_BUILTIN, OUTPUT);

  Serial8.begin(RASPBERRY_SERIAL_SPEED);

  // Motors
  for(int i = 0; i < 4; i++){
    pinMode(motor_pin[i][0], OUTPUT);
    pinMode(motor_pin[i][1], OUTPUT);
    analogWrite(motor_pin[i][0], motor_speed[i]);
  }

  //Compass
  if (!bno.begin()) {
    Serial.println("Nepodarilo sa inicializovat BNO055!");
    while (1);
  }
  delay(1000);
  bno.setExtCrystalUse(true);

  //IR_data finder
  Wire.begin();
  Serial.println("MRM-IR_data-Finder3 dual read start");

  //Line sensor
  for (int i = 0; i < LINE_SENSOR_AMOUNT; i++) {
    pinMode(line_pin[i], INPUT);
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  /*
  // FIRST TEST
  Serial.println("Hello world! OFF");
  delay(500);
  digitalWrite(LED_BUILTIN, HIGH);
  Serial.println("Hello world! ON");
  delay(500);
  digitalWrite(LED_BUILTIN, LOW);
  */


  
  // Compass
  read_compass();
  // IR_data
  read_IR();
  // Line
  read_line();

  Save_data();
  Print_data(0);
  creat_comunication_string();
  send_message();

  // Send data

  // Recieve data
  if (/*Serial.available()*/Serial8.available() > 0) {
    recieve_data();
  }

  // Motors
  motors_on(motors_data.motor_speed);

  delay(50);
}
