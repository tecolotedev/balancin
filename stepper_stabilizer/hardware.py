"""Raspberry Pi 5 GPIO and I²C adapters."""

from contextlib import AbstractContextManager
import time
from typing import Any, Callable, List, Optional, Protocol

from .controller import Correction, Direction
from .imu import ImuData, OrientationFilter, SensorModel, decode_measurement


# BCM GPIO numbering (not physical header pin numbers).
MOTOR_1_STEP_BCM = 17
MOTOR_1_DIR_BCM = 27
MOTOR_1_ENABLE_BCM = 22
MOTOR_2_STEP_BCM = 23
MOTOR_2_DIR_BCM = 24
MOTOR_2_ENABLE_BCM = 25

I2C_BUS = 1
IMU_ADDRESSES = (0x68, 0x69)
REGISTER_ACCEL_XOUT_H = 0x3B
REGISTER_GYRO_CONFIG = 0x1B
REGISTER_ACCEL_CONFIG = 0x1C
REGISTER_PWR_MGMT_1 = 0x6B
REGISTER_WHO_AM_I = 0x75

DRIVER_DIRECTION_SETUP_SECONDS = 0.000_005
DRIVER_STEP_HIGH_SECONDS = 0.000_010


class _Output(Protocol):
    def on(self) -> None: ...
    def off(self) -> None: ...
    def close(self) -> None: ...


OutputFactory = Callable[..., _Output]


class Motors(AbstractContextManager["Motors"]):
    """Own and safely operate both DRV8825 GPIO groups."""

    def __init__(
        self,
        output_factory: Optional[OutputFactory] = None,
        sleeper: Callable[[float], None] = time.sleep,
    ) -> None:
        if output_factory is None:
            try:
                from gpiozero import DigitalOutputDevice
            except ImportError as error:
                raise RuntimeError(
                    "GPIO Zero is not installed; on Raspberry Pi OS run "
                    "`sudo apt install python3-gpiozero python3-lgpio`"
                ) from error
            output_factory = DigitalOutputDevice

        self._sleep = sleeper
        self._closed = False
        created: List[_Output] = []

        def create(pin: int, initial_value: bool) -> _Output:
            output = output_factory(
                pin,
                active_high=True,
                initial_value=initial_value,
            )
            created.append(output)
            return output

        try:
            # DRV8825 EN is active-low. Claim EN first in the disabled HIGH
            # state so no step input can energize a motor during startup.
            self._enable_1 = create(MOTOR_1_ENABLE_BCM, True)
            self._enable_2 = create(MOTOR_2_ENABLE_BCM, True)
            self._step_1 = create(MOTOR_1_STEP_BCM, False)
            self._direction_1 = create(MOTOR_1_DIR_BCM, False)
            self._step_2 = create(MOTOR_2_STEP_BCM, False)
            self._direction_2 = create(MOTOR_2_DIR_BCM, False)
        except Exception:
            for output in reversed(created):
                output.close()
            raise

    def rotate_both(self, correction: Correction) -> None:
        if self._closed:
            raise RuntimeError("motor GPIO has already been closed")
        if correction.steps <= 0:
            return

        if correction.direction is Direction.FORWARD:
            self._direction_1.on()
            self._direction_2.on()
        else:
            self._direction_1.off()
            self._direction_2.off()
        self._sleep(DRIVER_DIRECTION_SETUP_SECONDS)

        try:
            self._enable_1.off()
            self._enable_2.off()
            for _ in range(correction.steps):
                self._step_1.on()
                self._step_2.on()
                self._sleep(DRIVER_STEP_HIGH_SECONDS)
                self._step_1.off()
                self._step_2.off()
                self._sleep(correction.low_delay_seconds)
        finally:
            self.disable()

    def disable(self) -> None:
        if self._closed:
            return

        first_error: Optional[Exception] = None
        # Disable both drivers before changing STEP. Attempt every output even
        # if one GPIO operation fails.
        for action in (
            self._enable_1.on,
            self._enable_2.on,
            self._step_1.off,
            self._step_2.off,
        ):
            try:
                action()
            except Exception as error:
                if first_error is None:
                    first_error = error

        if first_error is not None:
            raise first_error

    def close(self) -> None:
        if self._closed:
            return

        first_error: Optional[Exception] = None
        try:
            self.disable()
        except Exception as error:
            first_error = error

        for output in (
            self._direction_2,
            self._step_2,
            self._direction_1,
            self._step_1,
            self._enable_2,
            self._enable_1,
        ):
            try:
                output.close()
            except Exception as error:
                if first_error is None:
                    first_error = error
        self._closed = True

        if first_error is not None:
            raise first_error

    def __exit__(self, *args: object) -> None:
        self.close()


class MpuSensor(AbstractContextManager["MpuSensor"]):
    """Auto-detect and read an MPU6050 or MPU6500 on I²C bus 1."""

    def __init__(
        self,
        bus_number: int = I2C_BUS,
        bus_factory: Optional[Callable[[int], Any]] = None,
        sleeper: Callable[[float], None] = time.sleep,
    ) -> None:
        self._sleep = sleeper
        self._bus = (
            bus_factory(bus_number)
            if bus_factory is not None
            else _open_smbus(bus_number)
        )
        self._orientation = OrientationFilter()
        self._last_sample_time: Optional[float] = None
        self._closed = False

        try:
            detected = self._detect()
            if detected is None:
                raise RuntimeError(
                    "no MPU6050/MPU6500 responded at I2C address "
                    "0x68 or 0x69"
                )
            self.address, self.model = detected

            self._bus.write_byte_data(
                self.address, REGISTER_PWR_MGMT_1, 0x00
            )
            self._bus.write_byte_data(
                self.address, REGISTER_ACCEL_CONFIG, 0x00
            )
            self._bus.write_byte_data(
                self.address, REGISTER_GYRO_CONFIG, 0x00
            )
            self._sleep(0.1)
        except Exception:
            self._bus.close()
            self._closed = True
            raise

    def _detect(self) -> Optional[tuple[int, SensorModel]]:
        for address in IMU_ADDRESSES:
            try:
                device_id = self._bus.read_byte_data(
                    address, REGISTER_WHO_AM_I
                )
            except OSError:
                continue

            print(
                f"I2C address 0x{address:02X} reports "
                f"WHO_AM_I=0x{device_id:02X}"
            )
            model = SensorModel.from_device_id(device_id)
            if model is not None:
                return address, model
        return None

    def read(self) -> ImuData:
        if self._closed:
            raise RuntimeError("I2C sensor has already been closed")

        register_bytes = self._bus.read_i2c_block_data(
            self.address,
            REGISTER_ACCEL_XOUT_H,
            14,
        )
        now = time.monotonic()
        elapsed = (
            None
            if self._last_sample_time is None
            else now - self._last_sample_time
        )
        self._last_sample_time = now

        data = decode_measurement(self.model, register_bytes)
        self._orientation.update(data, elapsed)
        return data

    def close(self) -> None:
        if self._closed:
            return
        self._bus.close()
        self._closed = True

    def __exit__(self, *args: object) -> None:
        self.close()


def _open_smbus(bus_number: int) -> Any:
    try:
        from smbus2 import SMBus
    except ImportError:
        try:
            from smbus import SMBus
        except ImportError as error:
            raise RuntimeError(
                "SMBus support is not installed; on Raspberry Pi OS run "
                "`sudo apt install python3-smbus`"
            ) from error
    return SMBus(bus_number)
