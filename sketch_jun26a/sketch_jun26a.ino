// ESP32 + two DRV8825 stepper drivers + HC-SR04 + MPU6050/MPU6500

#include "I2CDiagnostic.h"
#include "MPU6050Sensor.h"
#include "MotorControl.h"

// Change this to 0 to return to the normal project.
#define MPU6050_DIAGNOSTIC_MODE 0

void setup() {
  Serial.begin(115200);
  setupMotors();
  setupMPU6050Sensor();
  Serial.println("Stepper motors  + MPU6050/MPU6500");
}

void loop() {
  const MPU6050Data data = readMPU6050();

  if (isnan(data.accelerationXG)) {
    Serial.println("Could not read IMU data.");
    delay(100);
    return;
  }

  const float accelerationXG = data.accelerationXG;
  const float accelerationXGRounded =
      fabsf(accelerationXG) <= 0.1f ? 0.0f : roundf(accelerationXG * 100.0f) / 100.0f;

  // Positive X acceleration moves one way; zero or negative moves the other.
  // Do not move the motors when the rounded value is zero.
  if (!isnan(accelerationXGRounded) && accelerationXGRounded != 0.0f) {
    Serial.print("Accel X:");
    Serial.println(accelerationXGRounded, 3);
    rotateBothMotors(accelerationXGRounded > 0.0f ? HIGH : LOW, 2, 9990);
  }

  delay(10);


}
