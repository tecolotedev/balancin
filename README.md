# Raspberry Pi stepper stabilizer

This is the Raspberry Pi/Rust migration of the original ESP32 Arduino sketch.
It drives two DRV8825 stepper drivers together and uses the X-axis acceleration
from an MPU6050 or MPU6500 as the error input to the original PID algorithm.

The old Arduino implementation remains in [`sketch_jun26a/`](sketch_jun26a/)
as a reference. The Rust application in `src/` is the active implementation.

## Hardware

- Raspberry Pi with a 40-pin header and current Raspberry Pi OS
- 2 × DRV8825 carrier boards
- 2 × stepper motors
- MPU6050 (for example, GY-521) or MPU6500
- A separate motor power supply sized for the motors
- At least 47 µF (100 µF recommended) electrolytic capacitor at each DRV8825
  between VMOT and GND
- 2 × 10 kΩ resistors for pull-ups on the DRV8825 EN inputs

### Raspberry Pi pin map

All GPIO numbers in the program and this table use **BCM numbering**, not
physical header numbers.

| Function | BCM GPIO | Physical pin | Connect to |
|---|---:|---:|---|
| I²C SDA | 2 | 3 | IMU SDA |
| I²C SCL | 3 | 5 | IMU SCL |
| Motor 1 STEP | 17 | 11 | DRV8825 #1 STEP |
| Motor 1 DIR | 27 | 13 | DRV8825 #1 DIR |
| Motor 1 EN | 22 | 15 | DRV8825 #1 EN |
| Motor 2 STEP | 23 | 16 | DRV8825 #2 STEP |
| Motor 2 DIR | 24 | 18 | DRV8825 #2 DIR |
| Motor 2 EN | 25 | 22 | DRV8825 #2 EN |
| 3.3 V | — | 1 or 17 | IMU VCC; DRV8825 RESET/SLEEP |
| Ground | — | 6, 9, 14, etc. | IMU, both drivers, motor PSU ground |

Wire the IMU AD0 pin to ground to select address `0x68`, or to 3.3 V to select
`0x69`. The program detects either address and identifies an MPU6050
(`WHO_AM_I=0x68`) or MPU6500 (`WHO_AM_I=0x70`).

For each DRV8825:

1. Join RESET and SLEEP and pull them up to 3.3 V.
2. Add a 10 kΩ resistor from EN to 3.3 V. EN is active-low, so this keeps the
   driver disabled while the Pi boots or its GPIO is unconfigured.
3. Ground M0, M1, and M2 for full-step mode, or wire them for the desired
   microstep setting. The program's step counts assume the original setup.
4. Connect VMOT only to the external motor supply, with the bulk capacitor
   close to the carrier. Never power a motor from the Raspberry Pi.
5. Connect the motor supply ground, both driver grounds, and Pi ground
   together. Do not connect 5 V to a Pi GPIO.

Both DIR outputs currently receive the same level, matching the ESP32 sketch.
If the mechanism requires one motor to rotate in the opposite physical
direction, reverse one motor coil pair or invert that driver's direction in
`Motors::rotate_both` in `src/raspberry_pi.rs`.

Before connecting the motors, set each DRV8825 current limit for its motor.

## Raspberry Pi setup

Install a current 64-bit Raspberry Pi OS, then enable I²C:

```sh
sudo raspi-config
```

Choose **Interface Options → I2C → Enable**, then reboot. Confirm that the IMU
is visible:

```sh
sudo apt update
sudo apt install -y i2c-tools
i2cdetect -y 1
```

The table should show `68` or `69`.

Install Rust on the Pi using the standard `rustup` installer, clone/copy this
directory to `~/stepper-stabilizer`, and build:

```sh
cd ~/stepper-stabilizer
cargo build --release
```

The project uses
[`rppal`](https://docs.rs/rppal/0.22.1/rppal/) for Raspberry Pi GPIO and I²C.
It supports Raspberry Pi models through Pi 5 and requires a recent Raspberry
Pi OS.

If GPIO or I²C reports a permission error, add the current user to both groups,
then log out and back in:

```sh
sudo usermod -aG gpio,i2c "$USER"
```

## Run

First test only the sensor, with both drivers kept disabled by their EN
pull-ups:

```sh
./target/release/stepper-stabilizer --diagnose
```

Then run the control loop:

```sh
./target/release/stepper-stabilizer
```

Press Ctrl-C to stop. SIGINT and SIGTERM are handled so both EN pins are driven
high before exit. The application also stops after 10 consecutive IMU read
failures instead of continuing with stale data.

To run at login as a user service:

```sh
mkdir -p ~/.config/systemd/user
cp deploy/stepper-stabilizer.service ~/.config/systemd/user/
systemctl --user daemon-reload
systemctl --user enable --now stepper-stabilizer
journalctl --user -u stepper-stabilizer -f
```

The supplied service assumes the project is at `~/stepper-stabilizer`.

## Tune and test safely

The initial PID values and correction step bands are unchanged from the ESP32
version. They are in `PidConfig::default()` in `src/controller.rs`:

- `Kp = 0.80`
- `Ki = 0.05`
- `Kd = 0.10`
- acceleration dead band = `0.10 g`
- correction bands = 2, 4, 8, or 20 steps
- STEP low delay = 2.5–15 ms

Test with the mechanism unloaded or raised off the floor. Begin with a low
DRV8825 current limit and keep a physical power cutoff within reach. If a tilt
makes the mechanism move farther from its target, swap the `Forward` and
`Reverse` GPIO levels in `Motors::rotate_both` before tuning PID gains.

Run the controller and IMU calculation tests on any development computer:

```sh
cargo test
cargo clippy --all-targets -- -D warnings
```

The hardware executable deliberately prints a platform message on non-Linux
hosts; actual GPIO/I²C access must be tested on the Raspberry Pi.

