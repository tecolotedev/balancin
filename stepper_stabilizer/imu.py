"""MPU6050/MPU6500 data conversion and orientation filtering."""

from dataclasses import dataclass
from enum import Enum
import math
import struct
from typing import Optional, Sequence


ACCEL_SCALE = 16_384.0
GYRO_SCALE = 131.0
ORIENTATION_TIME_CONSTANT_SECONDS = 0.5
MAX_ORIENTATION_INTERVAL_SECONDS = 0.25


class SensorModel(Enum):
    MPU6050 = "MPU6050"
    MPU6500 = "MPU6500"

    @classmethod
    def from_device_id(cls, device_id: int) -> Optional["SensorModel"]:
        if device_id == 0x68:
            return cls.MPU6050
        if device_id == 0x70:
            return cls.MPU6500
        return None


@dataclass
class ImuData:
    acceleration_x_g: float = 0.0
    acceleration_y_g: float = 0.0
    acceleration_z_g: float = 0.0
    gyro_x_degrees_per_second: float = 0.0
    gyro_y_degrees_per_second: float = 0.0
    gyro_z_degrees_per_second: float = 0.0
    temperature_c: float = 0.0
    roll_degrees: float = 0.0
    pitch_degrees: float = 0.0
    yaw_degrees: float = 0.0


def decode_measurement(
    model: SensorModel, register_bytes: Sequence[int]
) -> ImuData:
    """Decode the 14-byte measurement block beginning at register 0x3B."""

    if len(register_bytes) != 14:
        raise ValueError(
            f"expected 14 IMU measurement bytes, got {len(register_bytes)}"
        )

    raw_values = struct.unpack(">7h", bytes(register_bytes))
    (
        raw_acceleration_x,
        raw_acceleration_y,
        raw_acceleration_z,
        raw_temperature,
        raw_gyro_x,
        raw_gyro_y,
        raw_gyro_z,
    ) = raw_values

    if model is SensorModel.MPU6500:
        temperature_c = raw_temperature / 333.87 + 21.0
    else:
        temperature_c = raw_temperature / 340.0 + 36.53

    return ImuData(
        acceleration_x_g=raw_acceleration_x / ACCEL_SCALE,
        acceleration_y_g=raw_acceleration_y / ACCEL_SCALE,
        acceleration_z_g=raw_acceleration_z / ACCEL_SCALE,
        gyro_x_degrees_per_second=raw_gyro_x / GYRO_SCALE,
        gyro_y_degrees_per_second=raw_gyro_y / GYRO_SCALE,
        gyro_z_degrees_per_second=raw_gyro_z / GYRO_SCALE,
        temperature_c=temperature_c,
    )


class OrientationFilter:
    """Complementary-filter roll/pitch and relative gyroscope yaw."""

    def __init__(self) -> None:
        self._initialized = False
        self._roll_degrees = 0.0
        self._pitch_degrees = 0.0
        self._yaw_degrees = 0.0

    def update(
        self, data: ImuData, elapsed_seconds: Optional[float]
    ) -> None:
        accelerometer_roll = math.degrees(
            math.atan2(data.acceleration_y_g, data.acceleration_z_g)
        )
        accelerometer_pitch = math.degrees(
            math.atan2(
                -data.acceleration_x_g,
                math.sqrt(
                    data.acceleration_y_g**2
                    + data.acceleration_z_g**2
                ),
            )
        )

        interval_is_valid = (
            elapsed_seconds is not None
            and math.isfinite(elapsed_seconds)
            and 0 < elapsed_seconds <= MAX_ORIENTATION_INTERVAL_SECONDS
        )

        if not self._initialized or not interval_is_valid:
            self._reset_from_accelerometer(
                accelerometer_roll, accelerometer_pitch
            )
        else:
            assert elapsed_seconds is not None
            gyro_roll = (
                self._roll_degrees
                + data.gyro_x_degrees_per_second * elapsed_seconds
            )
            gyro_pitch = (
                self._pitch_degrees
                + data.gyro_y_degrees_per_second * elapsed_seconds
            )
            filter_weight = ORIENTATION_TIME_CONSTANT_SECONDS / (
                ORIENTATION_TIME_CONSTANT_SECONDS + elapsed_seconds
            )

            self._roll_degrees = (
                filter_weight * gyro_roll
                + (1.0 - filter_weight) * accelerometer_roll
            )
            self._pitch_degrees = (
                filter_weight * gyro_pitch
                + (1.0 - filter_weight) * accelerometer_pitch
            )
            self._yaw_degrees = _wrap_angle(
                self._yaw_degrees
                + data.gyro_z_degrees_per_second * elapsed_seconds
            )

        data.roll_degrees = self._roll_degrees
        data.pitch_degrees = self._pitch_degrees
        data.yaw_degrees = self._yaw_degrees

    def _reset_from_accelerometer(self, roll: float, pitch: float) -> None:
        self._roll_degrees = roll
        self._pitch_degrees = pitch
        self._yaw_degrees = 0.0
        self._initialized = True


def _wrap_angle(angle_degrees: float) -> float:
    while angle_degrees > 180.0:
        angle_degrees -= 360.0
    while angle_degrees <= -180.0:
        angle_degrees += 360.0
    return angle_degrees

