#include <Arduino.h>
#include <Wire.h>

#include "MPU6050Sensor.h"

namespace {

// GPIO 22 cannot be used for SCL because motor 1 already uses it for STEP.
constexpr int IMU_SDA_PIN = 21;
constexpr int IMU_SCL_PIN = 27;
constexpr uint32_t IMU_I2C_FREQUENCY_HZ = 100000;
constexpr uint16_t IMU_I2C_TIMEOUT_MS = 100;

constexpr uint8_t IMU_ADDRESS_LOW = 0x68;
constexpr uint8_t IMU_ADDRESS_HIGH = 0x69;
constexpr uint8_t MPU6050_DEVICE_ID = 0x68;
constexpr uint8_t MPU6500_DEVICE_ID = 0x70;

constexpr uint8_t REGISTER_ACCEL_XOUT_H = 0x3B;
constexpr uint8_t REGISTER_GYRO_CONFIG = 0x1B;
constexpr uint8_t REGISTER_ACCEL_CONFIG = 0x1C;
constexpr uint8_t REGISTER_PWR_MGMT_1 = 0x6B;
constexpr uint8_t REGISTER_WHO_AM_I = 0x75;

// Scale factors for +/-2 g and +/-250 degrees/second.
constexpr float ACCEL_SCALE = 16384.0f;
constexpr float GYRO_SCALE = 131.0f;

enum class SensorModel {
  UNKNOWN,
  MPU6050,
  MPU6500,
};

bool sensorReady = false;
uint8_t sensorAddress = IMU_ADDRESS_LOW;
SensorModel sensorModel = SensorModel::UNKNOWN;

const char* sensorModelName() {
  switch (sensorModel) {
    case SensorModel::MPU6050:
      return "MPU6050";
    case SensorModel::MPU6500:
      return "MPU6500";
    default:
      return "unknown IMU";
  }
}

bool writeRegister(uint8_t registerAddress, uint8_t value) {
  Wire.beginTransmission(sensorAddress);
  Wire.write(registerAddress);
  Wire.write(value);
  return Wire.endTransmission(true) == 0;
}

bool readRegister(uint8_t address, uint8_t registerAddress, uint8_t& value) {
  Wire.beginTransmission(address);
  Wire.write(registerAddress);

  if (Wire.endTransmission(true) != 0) {
    return false;
  }

  if (Wire.requestFrom(address, static_cast<size_t>(1), true) != 1) {
    return false;
  }

  value = Wire.read();
  return true;
}

int16_t readSignedWord() {
  const uint16_t highByte = Wire.read();
  const uint16_t lowByte = Wire.read();
  return static_cast<int16_t>((highByte << 8) | lowByte);
}

bool detectSupportedSensor() {
  constexpr uint8_t ADDRESSES[] = {IMU_ADDRESS_LOW, IMU_ADDRESS_HIGH};

  for (const uint8_t address : ADDRESSES) {
    uint8_t deviceId = 0;

    if (!readRegister(address, REGISTER_WHO_AM_I, deviceId)) {
      continue;
    }

    Serial.printf("IMU at 0x%02X reports WHO_AM_I=0x%02X\n",
                  static_cast<unsigned>(address),
                  static_cast<unsigned>(deviceId));

    if (deviceId == MPU6050_DEVICE_ID) {
      sensorAddress = address;
      sensorModel = SensorModel::MPU6050;
      return true;
    }

    if (deviceId == MPU6500_DEVICE_ID) {
      sensorAddress = address;
      sensorModel = SensorModel::MPU6500;
      return true;
    }
  }

  return false;
}

float convertTemperatureToCelsius(int16_t rawTemperature) {
  if (sensorModel == SensorModel::MPU6500) {
    return (rawTemperature / 333.87f) + 21.0f;
  }

  return (rawTemperature / 340.0f) + 36.53f;
}

}  // namespace

bool setupMPU6050Sensor() {
  sensorReady = false;
  sensorModel = SensorModel::UNKNOWN;

  if (!Wire.begin(IMU_SDA_PIN, IMU_SCL_PIN, IMU_I2C_FREQUENCY_HZ)) {
    Serial.println("Could not start the I2C bus.");
    return false;
  }

  Wire.setTimeOut(IMU_I2C_TIMEOUT_MS);
  delay(100);

  if (!detectSupportedSensor()) {
    Serial.println("No supported MPU6050 or MPU6500 sensor found.");
    return false;
  }

  // Wake the sensor and select the default internal clock.
  if (!writeRegister(REGISTER_PWR_MGMT_1, 0x00)) {
    Serial.printf("Could not wake the %s.\n", sensorModelName());
    return false;
  }

  // Select +/-2 g accelerometer and +/-250 degrees/second gyroscope ranges.
  if (!writeRegister(REGISTER_ACCEL_CONFIG, 0x00) ||
      !writeRegister(REGISTER_GYRO_CONFIG, 0x00)) {
    Serial.printf("Could not configure the %s.\n", sensorModelName());
    return false;
  }

  delay(100);
  sensorReady = true;
  Serial.printf("%s ready at address 0x%02X.\n",
                sensorModelName(), static_cast<unsigned>(sensorAddress));
  return true;
}

bool readMPU6050(MPU6050Data& data) {
  if (!sensorReady) {
    return false;
  }

  Wire.beginTransmission(sensorAddress);
  Wire.write(REGISTER_ACCEL_XOUT_H);

  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  constexpr size_t BYTES_TO_READ = 14;
  if (Wire.requestFrom(sensorAddress, BYTES_TO_READ, true) != BYTES_TO_READ) {
    return false;
  }

  const int16_t rawAccelerationX = readSignedWord();
  const int16_t rawAccelerationY = readSignedWord();
  const int16_t rawAccelerationZ = readSignedWord();
  const int16_t rawTemperature = readSignedWord();
  const int16_t rawGyroX = readSignedWord();
  const int16_t rawGyroY = readSignedWord();
  const int16_t rawGyroZ = readSignedWord();

  data.accelerationXG = rawAccelerationX / ACCEL_SCALE;
  data.accelerationYG = rawAccelerationY / ACCEL_SCALE;
  data.accelerationZG = rawAccelerationZ / ACCEL_SCALE;

  data.gyroXDegreesPerSecond = rawGyroX / GYRO_SCALE;
  data.gyroYDegreesPerSecond = rawGyroY / GYRO_SCALE;
  data.gyroZDegreesPerSecond = rawGyroZ / GYRO_SCALE;

  data.temperatureC = convertTemperatureToCelsius(rawTemperature);
  return true;
}

void printMPU6050Data() {
  MPU6050Data data;

  if (!readMPU6050(data)) {
    Serial.println("Could not read IMU data.");
    return;
  }

  Serial.print("Accel [g] X:");
  Serial.print(data.accelerationXG, 3);
  Serial.print(" Y:");
  Serial.print(data.accelerationYG, 3);
  Serial.print(" Z:");
  Serial.print(data.accelerationZG, 3);

  Serial.print(" | Gyro [deg/s] X:");
  Serial.print(data.gyroXDegreesPerSecond, 2);
  Serial.print(" Y:");
  Serial.print(data.gyroYDegreesPerSecond, 2);
  Serial.print(" Z:");
  Serial.print(data.gyroZDegreesPerSecond, 2);

  Serial.print(" | Temp:");
  Serial.print(data.temperatureC, 1);
  Serial.println(" C");
}
