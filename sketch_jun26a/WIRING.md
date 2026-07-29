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
