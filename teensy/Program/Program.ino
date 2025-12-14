#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <math.h>
#include <algorithm>





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

struct compass {
  float heading;
  float pitch;
  float roll;
};
compass compass_data;
//-------------------------MRM-IR_data-Finder3-------------------
#define I2C_ADDRESS 0x10                   // Adress MRM‑IR‑Finder3
#define REGISTER_ANGLE_AND_DISTANCE 0x00   // vIR_datatual register pre dir and dist
#define REGISTER_RAW 0x01                  // vIR_datatual register for RAW data
#define DATA_COUNT_ANGLE 4                 // 4 bytes dir + dist
#define DATA_COUNT_RAW 12                  // 12 bytes for RAW data mode
#define IR_data_sensor_amount 12           // sensor amount

#define NO_SIGNAL 1000                     // no signal from mrm / no ball found

struct IR {
  uint16_t angle;       // DIR
  uint16_t distance;    // CM
  uint8_t sensor_IR[IR_data_sensor_amount]; // RAW values from sensors
  uint8_t status;       // 13. byte = status / flag
};
IR IR_data;
//Line sensor
#define Line_sensor_amount 16

const int line_pin[Line_sensor_amount] = {
  14, 15, 16, 17, 20, 21, 22, 23, 24, 25, 26, 27, 38, 39, 40, 41
};

struct line {
  int16_t sensor_line[Line_sensor_amount];
};
line line_data;

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
      motors_data.motor_speed[i];
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

  for (int i = 0; i < IR_data_sensor_amount; i++) {
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
  for (int i = 0; i < Line_sensor_amount; i++) {
    line_data.sensor_line[i] = analogRead(line_pin[i]);
  }
}

// Data collection & Raspberry comunication-----------------------------------------------------------------------

struct all {
  motors motors_data;
  compass compass_data;
  IR IR_data;
  line line_data;
};

all all_data;

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
  for (int i = 0; i < IR_data_sensor_amount; i++) {
    Serial.print(i);
    Serial.print(":");
    Serial.print(all_data.IR_data.sensor_IR[i]);
    Serial.print("   ");
  }
  Serial.print("Status:  ");
  Serial.println(all_data.IR_data.status);

  // Line
  Serial.print("Line:  ");
  for (int i = 0; i < Line_sensor_amount; i++) {
    Serial.print(i);
    Serial.print(":");
    Serial.print(all_data.line_data.sensor_line[i]);
    Serial.print("   ");
  }
  Serial.println("");
  

  // END LINE
  Serial.println("-------------------------------------------------------------------------");
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
  for (int i = 0; i < Line_sensor_amount; i++) {
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


  // Motors
  motors_on(motors_data.motor_speed);
  // Compass
  read_compass();
  // IR_data
  read_IR();
  // Line
  read_line();

  Save_data();
  Print_data(0);


  

  delay(100);
}
