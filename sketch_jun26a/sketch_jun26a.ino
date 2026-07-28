// ESP32 + two DRV8825 stepper drivers + HC-SR04

#include "MotorControl.h"
#include "UltrasonicSensor.h"

void setup() {
  Serial.begin(115200);

  setupMotors();
  //setupUltrasonicSensor();

  Serial.println("Stepper motors + HC-SR04 distance sensor");
  Serial.println("Starting measurements...");
}

void loop() {
  // printDistance();
  // delay(250);

  runBothMotors();
}
