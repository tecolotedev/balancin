#pragma once

struct MPU6050Data {
  float accelerationXG;
  float accelerationYG;
  float accelerationZG;

  float gyroXDegreesPerSecond;
  float gyroYDegreesPerSecond;
  float gyroZDegreesPerSecond;

  float temperatureC;

  // Roll and pitch are stabilized with accelerometer + gyroscope data.
  // Yaw is relative only because these sensors do not include a magnetometer.
  float rollDegrees;
  float pitchDegrees;
  float yawDegrees;
};

// Auto-detects an MPU6050 or MPU6500 at 0x68/0x69 on GPIO 21/27.
// The existing function name is retained to avoid breaking the main sketch.
bool setupMPU6050Sensor();

// Reads acceleration, angular velocity, temperature, and orientation angles.
// accelerationXG is NAN if the sensor read fails.
MPU6050Data readMPU6050();
