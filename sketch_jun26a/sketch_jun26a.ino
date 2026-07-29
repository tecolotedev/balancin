// ESP32 + two DRV8825 stepper drivers + HC-SR04 + MPU6050/MPU6500

#include "I2CDiagnostic.h"
#include "MPU6050Sensor.h"
#include "MotorControl.h"
#include "UltrasonicSensor.h"

// Change this to 0 to return to the normal project.
#define MPU6050_DIAGNOSTIC_MODE 0

void setup() {
  Serial.begin(115200);

#if MPU6050_DIAGNOSTIC_MODE
  setupI2CDiagnostic();
#else
  setupMotors();
  // setupUltrasonicSensor();
  setupMPU6050Sensor();

  Serial.println("Stepper motors + HC-SR04 + MPU6050/MPU6500");
#endif
}

void loop() {
#if MPU6050_DIAGNOSTIC_MODE
  scanI2CBus();
  delay(5000);
#else
  // printDistance();

  const float accelerationXG = printMPU6050Data();

  // Positive X acceleration moves one way; zero or negative moves the other.
  if (!isnan(accelerationXG)) {
    moveBothMotors(accelerationXG > 0.0f ? HIGH : LOW, 20);
  }



#endif
}
