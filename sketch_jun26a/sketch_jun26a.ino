// ESP32 + two DRV8825 stepper drivers + HC-SR04 + MPU6050/MPU6500

#include "I2CDiagnostic.h"
#include "MPU6050Sensor.h"
#include "MotorControl.h"

// Change this to 0 to return to the normal project.
#define MPU6050_DIAGNOSTIC_MODE 0

namespace {

// PID starting values. Tune these on the real mechanism, beginning with KP.
constexpr float PID_KP = 0.80f;
constexpr float PID_KI = 0.05f;
constexpr float PID_KD = 0.10f;
constexpr float PID_INTEGRAL_LIMIT = 0.50f;
constexpr float PID_OUTPUT_LIMIT = 1.0f;
constexpr float ACCELERATION_DEAD_BAND_G = 0.1f;

constexpr unsigned int FAST_LOW_DELAY_MICROSECONDS = 2500;
constexpr unsigned int SLOW_LOW_DELAY_MICROSECONDS = 15000;

float previousError = 0.0f;
float integralError = 0.0f;
uint32_t previousPidUpdateMicros = 0;
bool pidInitialized = false;

float clampFloat(float value, float minimum, float maximum) {
  return constrain(value, minimum, maximum);
}

}  // namespace

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
  const uint32_t nowMicros = micros();

  if (!pidInitialized) {
    previousError = accelerationXG;
    previousPidUpdateMicros = nowMicros;
    pidInitialized = true;
    return;
  }

  const float elapsedSeconds =
      (nowMicros - previousPidUpdateMicros) / 1000000.0f;
  if (elapsedSeconds <= 0.0f) {
    return;
  }

  // The target is 0 g, so the accelerometer value is the signed error.
  const float error = accelerationXG;
  const float proportional = PID_KP * error;

  if (fabsf(error) < ACCELERATION_DEAD_BAND_G) {
    integralError = 0.0f;
  } else {
    integralError = clampFloat(integralError + error * elapsedSeconds,
                               -PID_INTEGRAL_LIMIT, PID_INTEGRAL_LIMIT);
  }

  const float integral = PID_KI * integralError;
  const float derivative = PID_KD * (error - previousError) / elapsedSeconds;
  const float pidOutput = clampFloat(proportional + integral + derivative,
                                     -PID_OUTPUT_LIMIT, PID_OUTPUT_LIMIT);

  previousError = error;
  previousPidUpdateMicros = nowMicros;

  if (fabsf(pidOutput) >= ACCELERATION_DEAD_BAND_G) {
    // If the platform moves farther from level, swap HIGH and LOW here.
    const int direction = pidOutput > 0.0f ? HIGH : LOW;
    const float correctionStrength = fabsf(pidOutput) / PID_OUTPUT_LIMIT;
    const unsigned int delayLow = static_cast<unsigned int>(
        SLOW_LOW_DELAY_MICROSECONDS - correctionStrength *
        (SLOW_LOW_DELAY_MICROSECONDS - FAST_LOW_DELAY_MICROSECONDS));
    
    int steps_per_correction = 0;

    if(fabsf(error) < 0.2){
      steps_per_correction = 2;
    }else if (fabsf(error) < 0.4){
      steps_per_correction = 4;
    }else if (fabsf(error) < 0.6){
      steps_per_correction = 8;
    }else{
      steps_per_correction = 20;
    }
    
    
    rotateBothMotors(direction, steps_per_correction, delayLow);
  }

  delay(50);
}
