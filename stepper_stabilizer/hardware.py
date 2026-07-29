"""Raspberry Pi 5 GPIO and I²C adapters."""

from contextlib import AbstractContextManager
import os
from pathlib import Path
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
GPIOCHIP_OVERRIDE_ENV = "STEPPER_GPIOCHIP"
RP1_GPIOCHIP_LABEL = "pinctrl-rp1"


class _Output(Protocol):
    def on(self) -> None: ...
    def off(self) -> None: ...
    def close(self) -> None: ...


OutputFactory = Callable[..., _Output]


def _gpiochip_number(path: Path) -> Optional[int]:
    name = path.name
    if not name.startswith("gpiochip"):
        return None
    try:
        return int(name.removeprefix("gpiochip"))
    except ValueError:
        return None


def _find_header_gpiochip(
    lgpio_module: Any,
    device_root: Path = Path("/dev"),
) -> tuple[int, Path]:
    """Locate the RP1 gpiochip used by the Raspberry Pi 5 header."""

    override = os.environ.get(GPIOCHIP_OVERRIDE_ENV)
    if override is not None:
        value = override.removeprefix("gpiochip")
        try:
            number = int(value)
        except ValueError as error:
            raise RuntimeError(
                f"{GPIOCHIP_OVERRIDE_ENV} must be a number such as 0, "
                f"not {override!r}"
            ) from error
        if number < 0:
            raise RuntimeError(
                f"{GPIOCHIP_OVERRIDE_ENV} must be zero or greater, "
                f"not {override!r}"
            )
        return number, device_root / f"gpiochip{number}"

    gpiochips = []
    for device in device_root.glob("gpiochip*"):
        number = _gpiochip_number(device)
        if number is None:
            continue
        gpiochips.append((number, device))
    gpiochips.sort()

    if not gpiochips:
        raise RuntimeError(
            "the Raspberry Pi GPIO character device was not found; "
            "no /dev/gpiochip devices are available. Run `gpiodetect` "
            "and verify this is running on the Pi host, not in an "
            "unconfigured container"
        )

    detected = []
    first_error: Optional[RuntimeError] = None
    for number, device in gpiochips:
        try:
            _verify_gpiochip_access(device)
            handle = lgpio_module.gpiochip_open(number)
        except Exception as error:
            if first_error is None:
                first_error = RuntimeError(
                    f"could not inspect {device}: {error}"
                )
            continue
        try:
            info = lgpio_module.gpio_get_chip_info(handle)
        finally:
            lgpio_module.gpiochip_close(handle)

        # lgpio returns [status, number-of-lines, device-name, label].
        label = info[-1]
        if isinstance(label, bytes):
            label = label.decode(errors="replace")
        label = str(label)
        detected.append(f"{device.name} [{label}]")
        if label == RP1_GPIOCHIP_LABEL:
            return number, device

    if not detected and first_error is not None:
        raise first_error

    detail = ", ".join(detected)
    raise RuntimeError(
        f"no /dev/gpiochip device has the {RP1_GPIOCHIP_LABEL!r} label; "
        f"detected: {detail}. Run `gpiodetect` and set "
        f"{GPIOCHIP_OVERRIDE_ENV} to the RP1 gpiochip number"
    )


def _verify_gpiochip_access(device: Path) -> None:
    try:
        descriptor = os.open(
            device,
            os.O_RDWR | getattr(os, "O_CLOEXEC", 0),
        )
    except PermissionError as error:
        raise RuntimeError(
            f"permission denied opening {device}; add this account to the "
            "gpio group with `sudo usermod -aG gpio \"$USER\"`, then log "
            "out completely and log back in"
        ) from error
    except FileNotFoundError as error:
        raise RuntimeError(
            f"{device} does not exist; run `gpiodetect` and set "
            f"{GPIOCHIP_OVERRIDE_ENV} to the RP1 gpiochip number"
        ) from error
    except OSError as error:
        raise RuntimeError(
            f"the operating system could not open {device}: {error}"
        ) from error
    else:
        os.close(descriptor)


class _LgpioChip:
    """Small direct lgpio adapter that avoids GPIO Zero chip guessing."""

    def __init__(self) -> None:
        try:
            import lgpio
        except ImportError as error:
            raise RuntimeError(
                "lgpio is not installed; on Raspberry Pi OS run "
                "`sudo apt install python3-lgpio`"
            ) from error

        self._lgpio = lgpio
        self._closed = False
        self.number, self.device = _find_header_gpiochip(lgpio)
        _verify_gpiochip_access(self.device)

        try:
            self._handle = lgpio.gpiochip_open(self.number)
        except Exception as error:
            raise RuntimeError(
                f"lgpio could not open {self.device}: {error}; run "
                f"`gpiodetect` and, if {RP1_GPIOCHIP_LABEL} has a "
                "different number, set "
                f"{GPIOCHIP_OVERRIDE_ENV}=<that-number>"
            ) from error

    def output(
        self,
        pin: int,
        *,
        active_high: bool,
        initial_value: bool,
    ) -> "_LgpioOutput":
        return _LgpioOutput(
            self,
            pin,
            active_high=active_high,
            initial_value=initial_value,
        )

    def close(self) -> None:
        if self._closed:
            return
        self._lgpio.gpiochip_close(self._handle)
        self._closed = True


class _LgpioOutput:
    def __init__(
        self,
        chip: _LgpioChip,
        pin: int,
        *,
        active_high: bool,
        initial_value: bool,
    ) -> None:
        self._chip = chip
        self._pin = pin
        self._active_high = active_high
        self._closed = False
        initial_level = initial_value == active_high
        try:
            chip._lgpio.gpio_claim_output(
                chip._handle,
                pin,
                int(initial_level),
            )
        except Exception as error:
            raise RuntimeError(
                f"could not claim BCM GPIO {pin} on {chip.device}: "
                f"{error}; stop any other program using this pin and run "
                f"`sudo lsof {chip.device}` to identify it"
            ) from error

    def on(self) -> None:
        self._write(self._active_high)

    def off(self) -> None:
        self._write(not self._active_high)

    def _write(self, level: bool) -> None:
        if self._closed:
            raise RuntimeError(f"BCM GPIO {self._pin} is already closed")
        self._chip._lgpio.gpio_write(
            self._chip._handle,
            self._pin,
            int(level),
        )

    def close(self) -> None:
        if self._closed:
            return
        self._chip._lgpio.gpio_free(
            self._chip._handle,
            self._pin,
        )
        self._closed = True


class Motors(AbstractContextManager["Motors"]):
    """Own and safely operate both DRV8825 GPIO groups."""

    def __init__(
        self,
        output_factory: Optional[OutputFactory] = None,
        sleeper: Callable[[float], None] = time.sleep,
    ) -> None:
        gpio_chip: Optional[_LgpioChip] = None
        if output_factory is None:
            gpio_chip = _LgpioChip()
            output_factory = gpio_chip.output

        self._sleep = sleeper
        self._closed = False
        self._gpio_chip = gpio_chip
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
                try:
                    output.close()
                except Exception:
                    pass
            if self._gpio_chip is not None:
                try:
                    self._gpio_chip.close()
                except Exception:
                    pass
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
        if self._gpio_chip is not None:
            try:
                self._gpio_chip.close()
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
