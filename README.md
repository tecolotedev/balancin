# Raspberry Pi 5 stepper stabilizer

This is the Python 3 migration of the original ESP32 Arduino project. It drives
two DRV8825 stepper drivers together and uses X-axis acceleration from an
MPU6050 or MPU6500 as the error input to the original PID controller.

The active Raspberry Pi implementation is in `stepper_stabilizer/`. The
original Arduino sketch remains in `sketch_jun26a/` as a reference.

## Hardware

- Raspberry Pi 5 with current 64-bit Raspberry Pi OS
- 2 × DRV8825 carrier boards
- 2 × stepper motors
- MPU6050 (such as a GY-521 board) or MPU6500
- Separate motor power supply sized for the motors
- At least 47 µF (100 µF recommended) electrolytic capacitor at each DRV8825,
  placed between VMOT and GND near the carrier
- 1 × 10 kΩ resistor for a pull-up on the shared DRV8825 EN signal

### Raspberry Pi pin map

The program and this table use **BCM GPIO numbering**, not physical header
numbers.

| Function | BCM GPIO | Physical pin | Connect to |
|---|---:|---:|---|
| I²C SDA | 2 | 3 | IMU SDA |
| I²C SCL | 3 | 5 | IMU SCL |
| Shared STEP | 5 | 29 | STEP on both DRV8825 boards |
| Shared DIR | 27 | 13 | DIR on both DRV8825 boards |
| Shared EN | 22 | 15 | EN on both DRV8825 boards |
| 3.3 V | — | 1 or 17 | IMU VCC; DRV8825 RESET/SLEEP |
| Ground | — | 6, 9, 14, etc. | IMU, both drivers, motor PSU ground |

Wire IMU AD0 to ground for address `0x68`, or to 3.3 V for `0x69`. The program
detects either address and identifies an MPU6050 (`WHO_AM_I=0x68`) or MPU6500
(`WHO_AM_I=0x70`).

For each DRV8825:

1. Join RESET and SLEEP and pull them up to 3.3 V.
2. Join both EN inputs and add a 10 kΩ resistor from this shared signal to
   3.3 V. EN is active-low; the resistor keeps both drivers disabled while the
   Pi boots or after the program releases GPIO.
3. Ground M0, M1, and M2 for full-step mode, or set the desired microstepping.
   The correction step counts are inherited from the original setup.
4. Connect VMOT only to the external motor supply, with the bulk capacitor
   close to the carrier. Never power a motor from the Raspberry Pi.
5. Connect the motor-supply ground, both driver grounds, and Pi ground
   together. Never connect 5 V to a Pi GPIO.

Connect BCM GPIO 5 to both STEP inputs, BCM GPIO 27 to both DIR inputs, and BCM
GPIO 22 to both EN inputs. Do not leave the former Motor 2 GPIO outputs attached
to these shared signals; two GPIO outputs must never be wired together.

Both drivers receive the same DIR level. If the
mechanism requires opposite physical motor rotation, reverse one motor coil
pair or change the direction handling in
`stepper_stabilizer/hardware.py`.

Set the current limit on each DRV8825 for its motor before connecting motor
power.

## Raspberry Pi OS setup

Install the Python 3 GPIO/I²C packages. The program uses `lgpio` directly on
Raspberry Pi 5:

```sh
sudo apt update
sudo apt install -y python3-lgpio python3-smbus \
  python3-venv i2c-tools
```

Enable I²C:

```sh
sudo raspi-config
```

Choose **Interface Options → I2C → Enable**.

Make sure your account can access GPIO and I²C, then reboot so the I²C
interface and new group membership are active:

```sh
sudo usermod -aG gpio,i2c "$USER"
sudo reboot
```

After reboot, confirm that `/dev/i2c-1` exists and scan it. `i2cdetect` verifies
the bus; it does not enable I²C. The sensor should appear at `68` or `69`:

```sh
ls -l /dev/i2c-1
i2cdetect -y 1
```

## Diagnose and run

Copy this project to `~/stepper-stabilizer`, then start with the sensor-only
diagnostic. Keep the motor supply off during this first test:

```sh
cd ~/stepper-stabilizer
python3 -m stepper_stabilizer --diagnose
```

Run the control loop:

```sh
python3 -m stepper_stabilizer
```

To test the motors without using the IMU or PID, raise or unload the mechanism
and run:

```sh
STEPPER_GPIOCHIP=4 python3 -m stepper_stabilizer --motor-test
```

The test moves both motors 200 steps forward, pauses for one second, then moves
them 200 steps in reverse. Press Ctrl-C to stop; the drivers are disabled when
the test exits.

Press Ctrl-C to stop. SIGINT and SIGTERM are handled so the shared EN signal is
driven high before exit, disabling both drivers. The program stops after 10
consecutive IMU read errors instead of continuing with stale data.

The program finds the Pi 5 header chip by its `pinctrl-rp1` label, so it works
whether the kernel exposes it as `/dev/gpiochip0` or `/dev/gpiochip4`. If
automatic detection is unavailable in a container, select the number reported
for `pinctrl-rp1` by `gpiodetect`:

```sh
STEPPER_GPIOCHIP=0 python3 -m stepper_stabilizer
```

If GPIO cannot be opened, confirm the selected device and account access:

```sh
gpiodetect
ls -l /dev/gpiochip*
id
```

The account must have read/write access to the selected device. After adding an
account to the `gpio` group, log out completely and log back in before testing.

The project can optionally be installed in a virtual environment created with
access to Raspberry Pi OS packages:

```sh
python3 -m venv --system-site-packages .venv
.venv/bin/python -m pip install -e .
.venv/bin/stepper-stabilizer --diagnose
```

## Start automatically

The included user service assumes the project is at
`~/stepper-stabilizer`:

```sh
mkdir -p ~/.config/systemd/user
cp deploy/stepper-stabilizer.service ~/.config/systemd/user/
systemctl --user daemon-reload
systemctl --user enable --now stepper-stabilizer
journalctl --user -u stepper-stabilizer -f
```

Stop and disable it with:

```sh
systemctl --user disable --now stepper-stabilizer
```

## Tune and test safely

The initial values in `stepper_stabilizer/controller.py` match the ESP32
project:

- `Kp = 0.80`
- `Ki = 0.05`
- `Kd = 0.10`
- acceleration dead band = `0.10 g`
- correction bands = 2, 4, 8, or 20 steps
- STEP low delay = 2.5–15 ms

Test unloaded or with the mechanism raised safely. Begin with a low DRV8825
current limit and keep a physical power cutoff within reach. If a tilt makes
the mechanism move farther away from the target, reverse the logical direction
in `Motors.rotate_both` before tuning PID values.

Python on Raspberry Pi OS is not a hard real-time controller. The inherited
2.5–15 ms STEP interval is slow enough for this implementation, but a
microcontroller-based pulse generator is preferable if future changes require
high or tightly timed step rates.

Run the logic and simulated hardware tests on the Pi or any development
computer:

```sh
python3 -m unittest discover -s tests -v
```
