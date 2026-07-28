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

// Change this value to match the selected microstepping mode.
constexpr int STEPS_PER_REVOLUTION = 200;

void rotateBothMotors(int direction, int revolutions) {
  if (revolutions <= 0) {
    return;
  }

  // Both drivers receive the same direction command.
  digitalWrite(MOTOR1_DIR_PIN, direction);
  digitalWrite(MOTOR2_DIR_PIN, direction);
  delayMicroseconds(5);

  for (int revolution = 0; revolution < revolutions; revolution++) {
    for (int step = 0; step < STEPS_PER_REVOLUTION; step++) {
      digitalWrite(MOTOR1_STEP_PIN, HIGH);
      digitalWrite(MOTOR2_STEP_PIN, HIGH);
      delayMicroseconds(10);  // Higher values produce a slower speed.

      digitalWrite(MOTOR1_STEP_PIN, LOW);
      digitalWrite(MOTOR2_STEP_PIN, LOW);
      delayMicroseconds(5990);
    }
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

void runBothMotors() {
  digitalWrite(MOTOR1_ENABLE_PIN, DRIVER_ENABLED);
  digitalWrite(MOTOR2_ENABLE_PIN, DRIVER_ENABLED);
  delay(10);

  rotateBothMotors(HIGH, 10);
  delay(100);

  rotateBothMotors(LOW, 10);
  delay(100);

  digitalWrite(MOTOR1_ENABLE_PIN, DRIVER_DISABLED);
  digitalWrite(MOTOR2_ENABLE_PIN, DRIVER_DISABLED);
  delay(10);
}
