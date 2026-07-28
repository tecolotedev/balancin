#include <Arduino.h>
#include <Wire.h>

#include "I2CDiagnostic.h"

namespace {

constexpr int I2C_SDA_PIN = 21;
constexpr int I2C_SCL_PIN = 27;
constexpr uint32_t I2C_FREQUENCY_HZ = 100000;
constexpr uint16_t I2C_TIMEOUT_MS = 100;

constexpr uint8_t MPU6050_ADDRESS_LOW = 0x68;
constexpr uint8_t MPU6050_ADDRESS_HIGH = 0x69;
constexpr uint8_t MPU6050_WHO_AM_I_REGISTER = 0x75;
constexpr uint8_t MPU6050_EXPECTED_ID = 0x68;
constexpr uint8_t MPU6500_EXPECTED_ID = 0x70;

bool diagnosticReady = false;

void printWhoAmI(uint8_t address) {
  Wire.beginTransmission(address);
  Wire.write(MPU6050_WHO_AM_I_REGISTER);

  const uint8_t registerError = Wire.endTransmission(true);
  if (registerError != 0) {
    Serial.printf("  WHO_AM_I register-select error: %u\n", registerError);
    return;
  }

  const size_t received =
      Wire.requestFrom(address, static_cast<size_t>(1), true);

  if (received != 1) {
    Serial.printf("  WHO_AM_I returned %u bytes; expected 1\n",
                  static_cast<unsigned>(received));
    return;
  }

  const uint8_t deviceId = Wire.read();
  const char* identity = "unexpected device ID";
  if (deviceId == MPU6050_EXPECTED_ID) {
    identity = "MPU6050 ID matches";
  } else if (deviceId == MPU6500_EXPECTED_ID) {
    identity = "MPU6500 ID matches";
  }

  Serial.printf("  WHO_AM_I = 0x%02X (%s)\n",
                static_cast<unsigned>(deviceId), identity);
}

}  // namespace

void setupI2CDiagnostic() {
  delay(1000);

  diagnosticReady =
      Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, I2C_FREQUENCY_HZ);
  Wire.setTimeOut(I2C_TIMEOUT_MS);

  Serial.println();
  Serial.println("=== MPU6050/MPU6500 I2C diagnostic mode ===");
  Serial.printf("SDA: GPIO%d, SCL: GPIO%d\n", I2C_SDA_PIN, I2C_SCL_PIN);
  Serial.printf("Wire.begin: %s\n", diagnosticReady ? "OK" : "FAILED");
  Serial.printf("I2C clock: %lu Hz\n",
                static_cast<unsigned long>(Wire.getClock()));
  Serial.println("Errors: 2=address NACK, 4=other error, 5=timeout");
  Serial.println("Set MPU6050_DIAGNOSTIC_MODE to 0 to restore normal mode.");
}

void scanI2CBus() {
  if (!diagnosticReady) {
    Serial.println("Cannot scan because Wire.begin failed.");
    return;
  }

  Serial.println();
  Serial.println("Scanning I2C addresses...");
  Serial.printf("Idle levels: SDA=%d, SCL=%d\n",
                digitalRead(I2C_SDA_PIN), digitalRead(I2C_SCL_PIN));

  int devicesFound = 0;

  for (uint8_t address = 1; address < 0x7F; address++) {
    Wire.beginTransmission(address);
    const uint8_t error = Wire.endTransmission(true);

    if (error == 0) {
      Serial.printf("FOUND device at 0x%02X\n",
                    static_cast<unsigned>(address));
      devicesFound++;

      if (address == MPU6050_ADDRESS_LOW ||
          address == MPU6050_ADDRESS_HIGH) {
        printWhoAmI(address);
      }
    } else if (error != 2) {
      Serial.printf("Address 0x%02X returned error %u\n",
                    static_cast<unsigned>(address),
                    static_cast<unsigned>(error));
    }
  }

  if (devicesFound == 0) {
    Serial.println("No I2C devices found.");
  } else {
    Serial.printf("Scan complete: %d device(s) found.\n", devicesFound);
  }
}
