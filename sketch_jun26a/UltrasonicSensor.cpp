#include <Arduino.h>

#include "UltrasonicSensor.h"

namespace {

// Keep these wires on GPIO 32/33. GPIO 18/19 are used by motor 2.
constexpr int ULTRASONIC_TRIG_PIN = 32;
constexpr int ULTRASONIC_ECHO_PIN = 33;

constexpr unsigned long ULTRASONIC_TIMEOUT_US = 30000;
constexpr float SOUND_SPEED_CM_PER_US = 0.0343;
constexpr float MINIMUM_RELIABLE_DISTANCE_CM = 6.0;
constexpr int ULTRASONIC_SAMPLES = 5;

unsigned long readEchoDurationUs() {
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);
  delayMicroseconds(5);

  if (digitalRead(ULTRASONIC_ECHO_PIN) == HIGH) {
    Serial.println("ECHO is stuck HIGH. Check wiring/voltage divider.");
    return 0;
  }

  digitalWrite(ULTRASONIC_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);

  return pulseIn(ULTRASONIC_ECHO_PIN, HIGH, ULTRASONIC_TIMEOUT_US);
}

}  // namespace

void setupUltrasonicSensor() {
  pinMode(ULTRASONIC_TRIG_PIN, OUTPUT);
  pinMode(ULTRASONIC_ECHO_PIN, INPUT_PULLDOWN);
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);
}

float readDistanceCm() {
  const unsigned long duration = readEchoDurationUs();

  if (duration == 0) {
    return -1.0;
  }

  return (duration * SOUND_SPEED_CM_PER_US) / 2.0;
}

float readStableDistanceCm() {
  float distances[ULTRASONIC_SAMPLES];
  int validSamples = 0;

  Serial.print("Raw durations us:");

  for (int sample = 0; sample < ULTRASONIC_SAMPLES; sample++) {
    const unsigned long duration = readEchoDurationUs();
    Serial.print(" ");
    Serial.print(duration);

    if (duration > 0) {
      const float distanceCm = (duration * SOUND_SPEED_CM_PER_US) / 2.0;

      if (distanceCm >= MINIMUM_RELIABLE_DISTANCE_CM) {
        distances[validSamples] = distanceCm;
        validSamples++;
      }
    }

    delay(60);
  }

  Serial.print(" | ");

  if (validSamples == 0) {
    return -1.0;
  }

  for (int i = 0; i < validSamples - 1; i++) {
    for (int j = i + 1; j < validSamples; j++) {
      if (distances[j] < distances[i]) {
        const float temp = distances[i];
        distances[i] = distances[j];
        distances[j] = temp;
      }
    }
  }

  return distances[validSamples / 2];
}

void printDistance() {
  const float distanceCm = readStableDistanceCm();

  if (distanceCm < 0.0) {
    Serial.println("No reliable object detected");
    return;
  }

  Serial.print("Distance: ");
  Serial.print(distanceCm, 1);
  Serial.println(" cm");
}
