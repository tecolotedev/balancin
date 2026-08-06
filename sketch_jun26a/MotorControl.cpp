#include <Arduino.h>

#include "MotorControl.h"


// Each ESP32 output connects to the matching input on both DRV8825 boards.
constexpr int MOTOR_STEP_PIN = 22;
constexpr int MOTOR_DIR_PIN = 23;
constexpr int MOTOR_ENABLE_PIN = 25;

// DRV8825 EN is active LOW.
constexpr int DRIVER_ENABLED = LOW;
constexpr int DRIVER_DISABLED = HIGH;

void rotateBothMotors(
  int direction,
  unsigned int steps,
  unsigned int lowDelayMicroseconds) {
  if (steps == 0) {
    return;
  }


  // Enable both motors
  digitalWrite(MOTOR_ENABLE_PIN, DRIVER_ENABLED);

  // Both drivers receive the same direction command.
  digitalWrite(MOTOR_DIR_PIN, direction);
  delayMicroseconds(5);

  for (int step = 0; step < steps; step++) {
    digitalWrite(MOTOR_STEP_PIN, HIGH);
    delayMicroseconds(10);  // Higher values produce a slower speed.

    digitalWrite(MOTOR_STEP_PIN, LOW);
    delayMicroseconds(lowDelayMicroseconds);
  }

  // Disable both motors
  digitalWrite(MOTOR_ENABLE_PIN, DRIVER_DISABLED);
}


void setupMotors() {
  pinMode(MOTOR_STEP_PIN, OUTPUT);
  pinMode(MOTOR_DIR_PIN, OUTPUT);
  pinMode(MOTOR_ENABLE_PIN, OUTPUT);

  digitalWrite(MOTOR_STEP_PIN, LOW);

  // Start with both drivers disabled.
  digitalWrite(MOTOR_ENABLE_PIN, DRIVER_DISABLED);
}
