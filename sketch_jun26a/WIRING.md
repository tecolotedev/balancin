# Legacy ESP32 MPU6050 / MPU6500 wiring

> This describes the archived ESP32 sketch only. For the active Raspberry Pi 5
> wiring and Python 3 instructions, see [`../README.md`](../README.md).

The MPU6050 or MPU6500 uses a separate I2C pin pair because GPIO 22 is already
the STEP output for motor 1.

```text
       ESP32                         MPU6050 / GY-521
  ┌─────────────┐                  ┌─────────────────┐
  │         3V3 ├──────────────────┤ VCC             │
  │         GND ├──────────────────┤ GND             │
  │ GPIO 21 SDA ├──────────────────┤ SDA             │
  │ GPIO 27 SCL ├──────────────────┤ SCL             │
  │         GND ├──────────────────┤ AD0             │
  └─────────────┘                  │ INT  (not used)  │
                                   │ XDA  (not used)  │
                                   │ XCL  (not used)  │
                                   └─────────────────┘
```

| ESP32 | MPU6050 | Purpose |
|---|---|---|
| 3V3 | VCC | Sensor power |
| GND | GND | Common ground |
| GPIO 21 | SDA | I2C data |
| GPIO 27 | SCL | I2C clock |
| GND | AD0 | Selects I2C address `0x68` |

Use 3.3 V for the sensor and I2C signals. Leave `INT`, `XDA`, and `XCL`
disconnected for this basic example.

## Shared motor-driver signals

Use only three ESP32 outputs for both DRV8825 boards:

| ESP32 | DRV8825 #1 | DRV8825 #2 |
|---|---|---|
| GPIO 22 | STEP | STEP |
| GPIO 23 | DIR | DIR |
| GPIO 25 | EN | EN |

Add a 10 kΩ pull-up from the shared EN signal to 3.3 V. The old Motor 2 pins
(GPIO 18, GPIO 19, and GPIO 26) are no longer used. Do not connect them to the
shared signals because two GPIO outputs must never be tied together.
