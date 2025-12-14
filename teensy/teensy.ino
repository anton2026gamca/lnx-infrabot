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

#define BNO_DATA_LENGHT ( (4 +1)*3 ) // -xxx,+yyy,+zzz,
#define IR_SENSOR_DATA_LENGHT ( (3 + 1) + (4 + 1) + (3 + 1)*IR_SENSOR_AMOUNT + (1 + 1) - 1 ) // aaa,dddd,12*[vvv,]s,
#define LINE_SENSOR_DATA ( (4 + 1)*LINE_SENSOR_AMOUNT ) // 12*[cccc,]
#define ALL_DATA_LENGHT (BNO_DATA_LENGHT + IR_SENSOR_DATA_LENGHT + LINE_SENSOR_DATA - 1)
  // -xxx,+yyy,+zzz,aaa,dddd,12*[vvv,]s,12*[cccc,] - ','
#define DATA_STRING_LENGHT (5 + ALL_DATA_LENGHT + 1 + 1) // "a"="..."\0
  // "a"="+xxx,+yyy,+zzz,aaa,dddd,12*[vvv,]s,12*[cccc,] - ','"\0


struct AllData {
  motors motors_data;
  CompassData compass_data;
  IRData IR_data;
  LineData line_data;
};

AllData all_data;

char send_data[DATA_STRING_LENGHT];

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
  if (sign == 1) {
    format_number_w_sign(number, hold_number, number_size); // formating number
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

void creat_comunication_string() {
  for (int i = 0; i < DATA_STRING_LENGHT; i ++) {
    send_data[i] = '#';
  }
  
  int position = 0;
  position = append_char_to_string('"',                                   send_data, position);
  position = append_char_to_string('a',                                   send_data, position);
  position = append_char_to_string('"',                                   send_data, position);
  position = append_char_to_string('=',                                   send_data, position);
  position = append_char_to_string('"',                                   send_data, position);
  
  position = append_number_to_string(all_data.compass_data.heading,       send_data, position, 3, 0);
  position = append_number_to_string(all_data.compass_data.pitch,         send_data, position, 3, 1);
  position = append_number_to_string(all_data.compass_data.roll,          send_data, position, 3, 1);

  position = append_number_to_string(all_data.IR_data.angle,              send_data, position, 3, 0);
  position = append_number_to_string(all_data.IR_data.distance,           send_data, position, 4, 0);
  for (int i = 0; i < IR_SENSOR_AMOUNT; i++) {
    position = append_number_to_string(all_data.IR_data.sensor_IR[i],     send_data, position, 3, 0);
  }
  position = append_number_to_string(all_data.IR_data.status,             send_data, position, 1, 0);

  for (int i = 0; i < LINE_SENSOR_AMOUNT; i++) {
    position = append_number_to_string(all_data.line_data.sensor_line[i], send_data, position, 4, 0);
  }
  
  position = position - 1; // delete last ','
  position = append_char_to_string('"',                                   send_data, position);
  position = append_char_to_string('\0',                                  send_data, position);
}


//--------------------------------------SETUP------------------------------------------
void setup() {
  // put your setup code here, to run once:
  Serial.begin(38400);
  pinMode(LED_BUILTIN, OUTPUT);

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

  // Send data

  // Recieve data

  // Motors
  motors_on(motors_data.motor_speed);


  creat_comunication_string();
  Serial.println(send_data);
  

  delay(100);
}
