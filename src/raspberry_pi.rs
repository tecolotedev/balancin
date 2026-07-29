use std::error::Error;
use std::io;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::thread;
use std::time::{Duration, Instant};

use rppal::gpio::{Gpio, OutputPin};
use rppal::i2c::I2c;

use stepper_stabilizer::controller::{Correction, Direction, PidConfig, PidController};
use stepper_stabilizer::imu::{ImuData, OrientationFilter, SensorModel};

type Result<T> = std::result::Result<T, Box<dyn Error>>;

const MOTOR_1_STEP_BCM: u8 = 17;
const MOTOR_1_DIR_BCM: u8 = 27;
const MOTOR_1_ENABLE_BCM: u8 = 22;
const MOTOR_2_STEP_BCM: u8 = 23;
const MOTOR_2_DIR_BCM: u8 = 24;
const MOTOR_2_ENABLE_BCM: u8 = 25;

const IMU_ADDRESS_LOW: u16 = 0x68;
const IMU_ADDRESS_HIGH: u16 = 0x69;
const REGISTER_ACCEL_XOUT_H: u8 = 0x3b;
const REGISTER_GYRO_CONFIG: u8 = 0x1b;
const REGISTER_ACCEL_CONFIG: u8 = 0x1c;
const REGISTER_PWR_MGMT_1: u8 = 0x6b;
const REGISTER_WHO_AM_I: u8 = 0x75;

const CONTROL_LOOP_DELAY: Duration = Duration::from_millis(50);
const DRIVER_DIRECTION_SETUP: Duration = Duration::from_micros(5);
const DRIVER_STEP_HIGH: Duration = Duration::from_micros(10);

pub fn run() -> Result<()> {
    match std::env::args().nth(1).as_deref() {
        Some("--diagnose") => diagnose_imu(),
        Some("--help" | "-h") => {
            print_help();
            Ok(())
        }
        Some(argument) => Err(io::Error::new(
            io::ErrorKind::InvalidInput,
            format!("unknown argument `{argument}`; use --help"),
        )
        .into()),
        None => stabilize(),
    }
}

fn print_help() {
    println!(
        "Usage: stepper-stabilizer [--diagnose]\n\n\
         With no arguments, runs the stabilization loop.\n\
         --diagnose  probe 0x68/0x69 and print the detected IMU\n\
         --help      show this help"
    );
}

fn stabilize() -> Result<()> {
    let running = Arc::new(AtomicBool::new(true));
    let signal_running = Arc::clone(&running);
    ctrlc::set_handler(move || {
        signal_running.store(false, Ordering::SeqCst);
    })?;

    let mut motors = Motors::new()?;
    let mut sensor = MpuSensor::new()?;
    let mut controller = PidController::new(PidConfig::default());
    let mut last_pid_update = Instant::now();
    let mut consecutive_read_errors = 0_u32;

    println!(
        "{} ready at 0x{:02X}; starting stabilization (Ctrl-C to stop)",
        sensor.model.name(),
        sensor.address
    );

    while running.load(Ordering::SeqCst) {
        match sensor.read() {
            Ok(data) => {
                consecutive_read_errors = 0;
                let now = Instant::now();
                let elapsed = now.duration_since(last_pid_update);
                last_pid_update = now;

                if let Some(correction) = controller.update(data.acceleration_x_g, elapsed) {
                    motors.rotate_both(correction);
                }
            }
            Err(error) => {
                consecutive_read_errors += 1;
                eprintln!("IMU read failed ({consecutive_read_errors}): {error}");

                if consecutive_read_errors >= 10 {
                    return Err(io::Error::other(
                        "stopping after 10 consecutive IMU read failures",
                    )
                    .into());
                }
            }
        }

        thread::sleep(CONTROL_LOOP_DELAY);
    }

    motors.disable();
    println!("stopped; both motor drivers are disabled");
    Ok(())
}

fn diagnose_imu() -> Result<()> {
    println!("Probing I2C bus 1 for an MPU6050/MPU6500...");
    let sensor = MpuSensor::new()?;
    println!(
        "found {} at address 0x{:02X}",
        sensor.model.name(),
        sensor.address
    );
    Ok(())
}

struct Motors {
    step_1: OutputPin,
    direction_1: OutputPin,
    enable_1: OutputPin,
    step_2: OutputPin,
    direction_2: OutputPin,
    enable_2: OutputPin,
}

impl Motors {
    fn new() -> Result<Self> {
        let gpio = Gpio::new()?;

        // EN is active-low. Configure it first and retain the disabled HIGH
        // state after drop as an additional safety measure.
        let mut enable_1 = gpio.get(MOTOR_1_ENABLE_BCM)?.into_output_high();
        let mut enable_2 = gpio.get(MOTOR_2_ENABLE_BCM)?.into_output_high();
        enable_1.set_reset_on_drop(false);
        enable_2.set_reset_on_drop(false);

        Ok(Self {
            step_1: gpio.get(MOTOR_1_STEP_BCM)?.into_output_low(),
            direction_1: gpio.get(MOTOR_1_DIR_BCM)?.into_output_low(),
            enable_1,
            step_2: gpio.get(MOTOR_2_STEP_BCM)?.into_output_low(),
            direction_2: gpio.get(MOTOR_2_DIR_BCM)?.into_output_low(),
            enable_2,
        })
    }

    fn rotate_both(&mut self, correction: Correction) {
        if correction.steps == 0 {
            return;
        }

        match correction.direction {
            Direction::Forward => {
                self.direction_1.set_high();
                self.direction_2.set_high();
            }
            Direction::Reverse => {
                self.direction_1.set_low();
                self.direction_2.set_low();
            }
        }
        thread::sleep(DRIVER_DIRECTION_SETUP);

        self.enable_1.set_low();
        self.enable_2.set_low();

        for _ in 0..correction.steps {
            self.step_1.set_high();
            self.step_2.set_high();
            thread::sleep(DRIVER_STEP_HIGH);
            self.step_1.set_low();
            self.step_2.set_low();
            thread::sleep(correction.low_delay);
        }

        self.disable();
    }

    fn disable(&mut self) {
        self.step_1.set_low();
        self.step_2.set_low();
        self.enable_1.set_high();
        self.enable_2.set_high();
    }
}

impl Drop for Motors {
    fn drop(&mut self) {
        self.disable();
    }
}

struct MpuSensor {
    i2c: I2c,
    address: u16,
    model: SensorModel,
    orientation: OrientationFilter,
    last_sample: Option<Instant>,
}

impl MpuSensor {
    fn new() -> Result<Self> {
        let mut i2c = I2c::new()?;
        i2c.set_timeout(100)?;

        let (address, model) = Self::detect(&mut i2c)?.ok_or_else(|| {
            io::Error::new(
                io::ErrorKind::NotFound,
                "no MPU6050/MPU6500 responded at 0x68 or 0x69",
            )
        })?;
        i2c.set_slave_address(address)?;

        Self::write_register(&mut i2c, REGISTER_PWR_MGMT_1, 0x00)?;
        Self::write_register(&mut i2c, REGISTER_ACCEL_CONFIG, 0x00)?;
        Self::write_register(&mut i2c, REGISTER_GYRO_CONFIG, 0x00)?;
        thread::sleep(Duration::from_millis(100));

        Ok(Self {
            i2c,
            address,
            model,
            orientation: OrientationFilter::default(),
            last_sample: None,
        })
    }

    fn detect(i2c: &mut I2c) -> Result<Option<(u16, SensorModel)>> {
        for address in [IMU_ADDRESS_LOW, IMU_ADDRESS_HIGH] {
            i2c.set_slave_address(address)?;
            let mut device_id = [0_u8; 1];
            if i2c
                .write_read(&[REGISTER_WHO_AM_I], &mut device_id)
                .is_err()
            {
                continue;
            }

            println!(
                "I2C address 0x{address:02X} reports WHO_AM_I=0x{:02X}",
                device_id[0]
            );
            if let Some(model) = SensorModel::from_device_id(device_id[0]) {
                return Ok(Some((address, model)));
            }
        }

        Ok(None)
    }

    fn write_register(i2c: &mut I2c, register: u8, value: u8) -> Result<()> {
        let written = i2c.write(&[register, value])?;
        if written != 2 {
            return Err(io::Error::new(
                io::ErrorKind::WriteZero,
                format!("wrote {written} bytes to register 0x{register:02X}; expected 2"),
            )
            .into());
        }
        Ok(())
    }

    fn read(&mut self) -> Result<ImuData> {
        let mut bytes = [0_u8; 14];
        self.i2c.write_read(&[REGISTER_ACCEL_XOUT_H], &mut bytes)?;

        let now = Instant::now();
        let elapsed = self
            .last_sample
            .map(|last_sample| now.duration_since(last_sample));
        self.last_sample = Some(now);

        let mut data = ImuData::from_register_bytes(self.model, bytes);
        self.orientation.update(&mut data, elapsed);
        Ok(data)
    }
}
