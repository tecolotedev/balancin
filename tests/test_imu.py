import struct
import unittest

from stepper_stabilizer.imu import (
    ImuData,
    OrientationFilter,
    SensorModel,
    decode_measurement,
)


class ImuTests(unittest.TestCase):
    def test_detects_supported_device_ids(self) -> None:
        self.assertIs(
            SensorModel.from_device_id(0x68), SensorModel.MPU6050
        )
        self.assertIs(
            SensorModel.from_device_id(0x70), SensorModel.MPU6500
        )
        self.assertIsNone(SensorModel.from_device_id(0x00))

    def test_decodes_signed_acceleration_and_gyro(self) -> None:
        raw = struct.pack(
            ">7h", 16_384, -16_384, 8_192, 0, 131, -262, 65
        )
        data = decode_measurement(SensorModel.MPU6050, raw)

        self.assertAlmostEqual(data.acceleration_x_g, 1.0)
        self.assertAlmostEqual(data.acceleration_y_g, -1.0)
        self.assertAlmostEqual(data.acceleration_z_g, 0.5)
        self.assertAlmostEqual(data.gyro_x_degrees_per_second, 1.0)
        self.assertAlmostEqual(data.gyro_y_degrees_per_second, -2.0)
        self.assertAlmostEqual(
            data.gyro_z_degrees_per_second, 65.0 / 131.0
        )
        self.assertAlmostEqual(data.temperature_c, 36.53)

    def test_rejects_incomplete_measurement(self) -> None:
        with self.assertRaises(ValueError):
            decode_measurement(SensorModel.MPU6050, b"\x00" * 13)

    def test_initializes_level_orientation_from_gravity(self) -> None:
        data = ImuData(acceleration_z_g=1.0)
        OrientationFilter().update(data, None)

        self.assertAlmostEqual(data.roll_degrees, 0.0)
        self.assertAlmostEqual(data.pitch_degrees, 0.0)
        self.assertAlmostEqual(data.yaw_degrees, 0.0)

    def test_resets_relative_yaw_after_long_sampling_gap(self) -> None:
        orientation = OrientationFilter()
        data = ImuData(
            acceleration_z_g=1.0,
            gyro_z_degrees_per_second=90.0,
        )

        orientation.update(data, None)
        orientation.update(data, 0.1)
        self.assertAlmostEqual(data.yaw_degrees, 9.0)

        orientation.update(data, 0.3)
        self.assertAlmostEqual(data.yaw_degrees, 0.0)


if __name__ == "__main__":
    unittest.main()

