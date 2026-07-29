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
  const float accelerationXGRounded =
      fabsf(accelerationXG) <= 0.1f ? 0.0f : roundf(accelerationXG * 100.0f) / 100.0f;

  // Positive X acceleration moves one way; zero or negative moves the other.
  // Do not move the motors when the rounded value is zero.
  if (!isnan(accelerationXGRounded) && accelerationXGRounded != 0.0f) {
    Serial.print("Accel X:");
    Serial.println(accelerationXGRounded, 3);
    moveBothMotors(accelerationXGRounded > 0.0f ? HIGH : LOW, 2);
  }

  delay(10);


#endif
}
