use std::f32::consts::PI;
use std::time::Duration;

const ACCEL_SCALE: f32 = 16_384.0;
const GYRO_SCALE: f32 = 131.0;
const ORIENTATION_TIME_CONSTANT_SECONDS: f32 = 0.5;
const MAX_ORIENTATION_INTERVAL: Duration = Duration::from_millis(250);

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SensorModel {
    Mpu6050,
    Mpu6500,
}

impl SensorModel {
    pub fn from_device_id(device_id: u8) -> Option<Self> {
        match device_id {
            0x68 => Some(Self::Mpu6050),
            0x70 => Some(Self::Mpu6500),
            _ => None,
        }
    }

    pub fn name(self) -> &'static str {
        match self {
            Self::Mpu6050 => "MPU6050",
            Self::Mpu6500 => "MPU6500",
        }
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct ImuData {
    pub acceleration_x_g: f32,
    pub acceleration_y_g: f32,
    pub acceleration_z_g: f32,
    pub gyro_x_degrees_per_second: f32,
    pub gyro_y_degrees_per_second: f32,
    pub gyro_z_degrees_per_second: f32,
    pub temperature_c: f32,
    pub roll_degrees: f32,
    pub pitch_degrees: f32,
    pub yaw_degrees: f32,
}

impl ImuData {
    /// Decodes the 14-byte measurement block beginning at register 0x3B.
    pub fn from_register_bytes(model: SensorModel, bytes: [u8; 14]) -> Self {
        let word = |offset| i16::from_be_bytes([bytes[offset], bytes[offset + 1]]);

        let raw_acceleration_x = word(0);
        let raw_acceleration_y = word(2);
        let raw_acceleration_z = word(4);
        let raw_temperature = word(6);
        let raw_gyro_x = word(8);
        let raw_gyro_y = word(10);
        let raw_gyro_z = word(12);

        let temperature_c = match model {
            SensorModel::Mpu6050 => raw_temperature as f32 / 340.0 + 36.53,
            SensorModel::Mpu6500 => raw_temperature as f32 / 333.87 + 21.0,
        };

        Self {
            acceleration_x_g: raw_acceleration_x as f32 / ACCEL_SCALE,
            acceleration_y_g: raw_acceleration_y as f32 / ACCEL_SCALE,
            acceleration_z_g: raw_acceleration_z as f32 / ACCEL_SCALE,
            gyro_x_degrees_per_second: raw_gyro_x as f32 / GYRO_SCALE,
            gyro_y_degrees_per_second: raw_gyro_y as f32 / GYRO_SCALE,
            gyro_z_degrees_per_second: raw_gyro_z as f32 / GYRO_SCALE,
            temperature_c,
            ..Self::default()
        }
    }
}

#[derive(Debug, Default)]
pub struct OrientationFilter {
    initialized: bool,
    roll_degrees: f32,
    pitch_degrees: f32,
    yaw_degrees: f32,
}

impl OrientationFilter {
    /// Adds complementary-filter roll, pitch, and relative yaw to a sample.
    pub fn update(&mut self, data: &mut ImuData, elapsed: Option<Duration>) {
        let radians_to_degrees = 180.0 / PI;
        let accelerometer_roll_degrees =
            data.acceleration_y_g.atan2(data.acceleration_z_g) * radians_to_degrees;
        let accelerometer_pitch_degrees = (-data.acceleration_x_g)
            .atan2((data.acceleration_y_g.powi(2) + data.acceleration_z_g.powi(2)).sqrt())
            * radians_to_degrees;

        if !self.initialized {
            self.reset_from_accelerometer(accelerometer_roll_degrees, accelerometer_pitch_degrees);
        } else if let Some(elapsed) =
            elapsed.filter(|value| !value.is_zero() && *value <= MAX_ORIENTATION_INTERVAL)
        {
            let elapsed_seconds = elapsed.as_secs_f32();
            let gyro_roll_degrees =
                self.roll_degrees + data.gyro_x_degrees_per_second * elapsed_seconds;
            let gyro_pitch_degrees =
                self.pitch_degrees + data.gyro_y_degrees_per_second * elapsed_seconds;
            let filter_weight = ORIENTATION_TIME_CONSTANT_SECONDS
                / (ORIENTATION_TIME_CONSTANT_SECONDS + elapsed_seconds);

            self.roll_degrees = filter_weight * gyro_roll_degrees
                + (1.0 - filter_weight) * accelerometer_roll_degrees;
            self.pitch_degrees = filter_weight * gyro_pitch_degrees
                + (1.0 - filter_weight) * accelerometer_pitch_degrees;
            self.yaw_degrees =
                wrap_angle(self.yaw_degrees + data.gyro_z_degrees_per_second * elapsed_seconds);
        } else {
            self.reset_from_accelerometer(accelerometer_roll_degrees, accelerometer_pitch_degrees);
        }

        data.roll_degrees = self.roll_degrees;
        data.pitch_degrees = self.pitch_degrees;
        data.yaw_degrees = self.yaw_degrees;
    }

    fn reset_from_accelerometer(&mut self, roll: f32, pitch: f32) {
        self.roll_degrees = roll;
        self.pitch_degrees = pitch;
        self.yaw_degrees = 0.0;
        self.initialized = true;
    }
}

fn wrap_angle(mut angle_degrees: f32) -> f32 {
    while angle_degrees > 180.0 {
        angle_degrees -= 360.0;
    }
    while angle_degrees <= -180.0 {
        angle_degrees += 360.0;
    }
    angle_degrees
}

#[cfg(test)]
mod tests {
    use super::*;

    fn raw_bytes(words: [i16; 7]) -> [u8; 14] {
        let mut bytes = [0; 14];
        for (index, word) in words.iter().enumerate() {
            let encoded = word.to_be_bytes();
            bytes[index * 2] = encoded[0];
            bytes[index * 2 + 1] = encoded[1];
        }
        bytes
    }

    fn assert_near(actual: f32, expected: f32) {
        assert!(
            (actual - expected).abs() < 0.001,
            "expected {expected}, got {actual}"
        );
    }

    #[test]
    fn detects_supported_device_ids() {
        assert_eq!(
            SensorModel::from_device_id(0x68),
            Some(SensorModel::Mpu6050)
        );
        assert_eq!(
            SensorModel::from_device_id(0x70),
            Some(SensorModel::Mpu6500)
        );
        assert_eq!(SensorModel::from_device_id(0x00), None);
    }

    #[test]
    fn decodes_signed_acceleration_and_gyro_values() {
        let bytes = raw_bytes([16_384, -16_384, 8_192, 0, 131, -262, 65]);
        let data = ImuData::from_register_bytes(SensorModel::Mpu6050, bytes);

        assert_near(data.acceleration_x_g, 1.0);
        assert_near(data.acceleration_y_g, -1.0);
        assert_near(data.acceleration_z_g, 0.5);
        assert_near(data.gyro_x_degrees_per_second, 1.0);
        assert_near(data.gyro_y_degrees_per_second, -2.0);
        assert_near(data.gyro_z_degrees_per_second, 65.0 / 131.0);
        assert_near(data.temperature_c, 36.53);
    }

    #[test]
    fn initializes_level_orientation_from_gravity() {
        let mut data = ImuData {
            acceleration_z_g: 1.0,
            ..ImuData::default()
        };
        OrientationFilter::default().update(&mut data, None);

        assert_near(data.roll_degrees, 0.0);
        assert_near(data.pitch_degrees, 0.0);
        assert_near(data.yaw_degrees, 0.0);
    }

    #[test]
    fn resets_relative_yaw_after_a_long_sampling_gap() {
        let mut filter = OrientationFilter::default();
        let mut data = ImuData {
            acceleration_z_g: 1.0,
            gyro_z_degrees_per_second: 90.0,
            ..ImuData::default()
        };

        filter.update(&mut data, None);
        filter.update(&mut data, Some(Duration::from_millis(100)));
        assert_near(data.yaw_degrees, 9.0);

        filter.update(&mut data, Some(Duration::from_millis(300)));
        assert_near(data.yaw_degrees, 0.0);
    }
}
