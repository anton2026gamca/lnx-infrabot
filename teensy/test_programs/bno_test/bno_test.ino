#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

Adafruit_BNO055 bno = Adafruit_BNO055(55);

unsigned long previousMillis = 0;
const unsigned long interval = 100; // 0,1 sekundy = 100 ms
int eventCount = 0;

float accelThreshold = 1.5; // prah pre "výstup"

void setup() {
  Serial.begin(115200); // rýchlejší sériový výstup

  if(!bno.begin()) {
    Serial.println("BNO055 nenajdeny!");
    while(1);
  }

  bno.setExtCrystalUse(true);
  Serial.println("Spusteny program - pocet udalosti kazdych 0.1s");
}

void loop() {
  unsigned long currentMillis = millis();

  // Rýchle čítanie akcelerácie
  sensors_event_t event;
  bno.getEvent(&event);

  // Ak prekročí prah -> počítame ako udalosť
  if(abs(event.acceleration.x) > accelThreshold ||
     abs(event.acceleration.y) > accelThreshold ||
     abs(event.acceleration.z) > accelThreshold) {
    eventCount++;
  }

  // Každých 0,1 s vypíš počet a resetuj
  if(currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    Serial.print("Udalosti za poslednych 0.1s: ");
    Serial.println(eventCount);
    eventCount = 0;
  }
}

/*#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

Adafruit_BNO055 bno = Adafruit_BNO055(55);

unsigned long previousMillis = 0;
const unsigned long interval = 1000; // 1 sekunda
int eventCount = 0;

float accelThreshold = 1.5; // prah pre "výstup"

void setup() {
  Serial.begin(115200); // rýchlejší sériový výstup

  if(!bno.begin()) {
    Serial.println("BNO055 nenajdeny!");
    while(1);
  }

  bno.setExtCrystalUse(true);
  Serial.println("Spusteny program na meranie max. udalosti za sekundu");
}

void loop() {
  unsigned long currentMillis = millis();

  // Rýchle čítanie akcelerácie bez akéhokoľvek delay
  sensors_event_t event;
  bno.getEvent(&event);

  // Ak prekročí prah -> počítame ako udalosť
  if(abs(event.acceleration.x) > accelThreshold ||
     abs(event.acceleration.y) > accelThreshold ||
     abs(event.acceleration.z) > accelThreshold) {
    eventCount++;
  }

  // Každú sekundu vypíš výsledok a resetuj
  if(currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    Serial.print("Počet udalostí za poslednú sekundu: ");
    Serial.println(eventCount);
    eventCount = 0;
  }
}*/