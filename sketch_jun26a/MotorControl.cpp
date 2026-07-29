#include <Arduino.h>

#include "MotorControl.h"

namespace {

constexpr int MOTOR1_STEP_PIN = 22;
constexpr int MOTOR1_DIR_PIN = 23;
constexpr int MOTOR1_ENABLE_PIN = 25;

constexpr int MOTOR2_STEP_PIN = 18;
constexpr int MOTOR2_DIR_PIN = 19;
constexpr int MOTOR2_ENABLE_PIN = 26;

// DRV8825 EN is active LOW.
constexpr int DRIVER_ENABLED = LOW;
constexpr int DRIVER_DISABLED = HIGH;

void rotateBothMotors(int direction, int steps) {
  if (steps <= 0) {
    return;
  }

  // Both drivers receive the same direction command.
  digitalWrite(MOTOR1_DIR_PIN, direction);
  digitalWrite(MOTOR2_DIR_PIN, direction);
  delayMicroseconds(5);

  for (int step = 0; step < steps; step++) {
    digitalWrite(MOTOR1_STEP_PIN, HIGH);
    digitalWrite(MOTOR2_STEP_PIN, HIGH);
    delayMicroseconds(10);  // Higher values produce a slower speed.

    digitalWrite(MOTOR1_STEP_PIN, LOW);
    digitalWrite(MOTOR2_STEP_PIN, LOW);
    delayMicroseconds(5990);
  }
}

}  // namespace

void setupMotors() {
  pinMode(MOTOR1_STEP_PIN, OUTPUT);
  pinMode(MOTOR1_DIR_PIN, OUTPUT);
  pinMode(MOTOR1_ENABLE_PIN, OUTPUT);

  pinMode(MOTOR2_STEP_PIN, OUTPUT);
  pinMode(MOTOR2_DIR_PIN, OUTPUT);
  pinMode(MOTOR2_ENABLE_PIN, OUTPUT);

  digitalWrite(MOTOR1_STEP_PIN, LOW);
  digitalWrite(MOTOR2_STEP_PIN, LOW);

  // Start with both drivers disabled.
  digitalWrite(MOTOR1_ENABLE_PIN, DRIVER_DISABLED);
  digitalWrite(MOTOR2_ENABLE_PIN, DRIVER_DISABLED);
}

void moveBothMotors(int direction, int steps) {
  digitalWrite(MOTOR1_ENABLE_PIN, DRIVER_ENABLED);
  digitalWrite(MOTOR2_ENABLE_PIN, DRIVER_ENABLED);
  delay(10);

  rotateBothMotors(direction, steps);

  digitalWrite(MOTOR1_ENABLE_PIN, DRIVER_DISABLED);
  digitalWrite(MOTOR2_ENABLE_PIN, DRIVER_DISABLED);
  delay(10);
}

void runBothMotors() {
  moveBothMotors(HIGH, 2000);
  delay(100);

  moveBothMotors(LOW, 2000);
  delay(100);
}
