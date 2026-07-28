#pragma once

struct MPU6050Data {
  float accelerationXG;
  float accelerationYG;
  float accelerationZG;

  float gyroXDegreesPerSecond;
  float gyroYDegreesPerSecond;
  float gyroZDegreesPerSecond;

  float temperatureC;
};

// Auto-detects an MPU6050 or MPU6500 at 0x68/0x69 on GPIO 21/27.
// The existing function name is retained to avoid breaking the main sketch.
bool setupMPU6050Sensor();

// Reads acceleration, angular velocity, and model-corrected temperature.
bool readMPU6050(MPU6050Data& data);

// Basic example that prints one sensor reading to the Serial Monitor.
void printMPU6050Data();
