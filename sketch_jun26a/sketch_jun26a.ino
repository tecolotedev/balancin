// Stepper Motor Control (Forward & Reverse)
// Using ESP32 + DRV8825 + NEMA 17

#define MOTOR1_STEP_PIN 22
#define MOTOR1_DIR_PIN 23
#define MOTOR1_ENABLE_PIN 25

#define MOTOR2_STEP_PIN 18
#define MOTOR2_DIR_PIN 19
#define MOTOR2_ENABLE_PIN 26

// DRV8825 EN is active LOW.
const int DRIVER_ENABLED = LOW;
const int DRIVER_DISABLED = HIGH;

// Steps per revolution (depends on your microstepping setup)
// Example: 200 steps/rev * 16 microsteps = 3200
const int stepsPerRevolution = 800;

void setup() {
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

void loop() {
  runMotor(MOTOR1_STEP_PIN, MOTOR1_DIR_PIN, MOTOR1_ENABLE_PIN,
           MOTOR2_ENABLE_PIN);
  runMotor(MOTOR2_STEP_PIN, MOTOR2_DIR_PIN, MOTOR2_ENABLE_PIN,
           MOTOR1_ENABLE_PIN);
}

void runMotor(int stepPin, int dirPin, int enablePin,
              int otherEnablePin) {
  // Disable the other driver before enabling this one.
  digitalWrite(otherEnablePin, DRIVER_DISABLED);
  delay(10);
  digitalWrite(enablePin, DRIVER_ENABLED);
  delay(10);

  digitalWrite(dirPin, HIGH);
  rotateOneRevolution(stepPin);
  delay(500);

  digitalWrite(dirPin, LOW);
  rotateOneRevolution(stepPin);
  delay(500);

  digitalWrite(enablePin, DRIVER_DISABLED);
  delay(10);
}

void rotateOneRevolution(int stepPin) {
  for (int i = 0; i < stepsPerRevolution; i++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(1600);  // Adjust for speed (higher = slower)
    digitalWrite(stepPin, LOW);
    delayMicroseconds(100);
  }
}
