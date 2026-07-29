from pathlib import Path
import sys
from tempfile import TemporaryDirectory
import unittest
from unittest.mock import patch

from stepper_stabilizer.controller import Correction, Direction
from stepper_stabilizer.hardware import (
    DRIVER_DIRECTION_SETUP_SECONDS,
    DRIVER_STEP_HIGH_SECONDS,
    MOTOR_1_ENABLE_BCM,
    MOTOR_2_ENABLE_BCM,
    Motors,
    _LgpioChip,
    _find_header_gpiochip,
)


class FakeOutput:
    def __init__(
        self,
        pin: int,
        *,
        active_high: bool,
        initial_value: bool,
    ) -> None:
        self.pin = pin
        self.active_high = active_high
        self.value = initial_value
        self.closed = False
        self.fail_next_off = False

    def on(self) -> None:
        self.value = True

    def off(self) -> None:
        if self.fail_next_off:
            self.fail_next_off = False
            raise OSError("simulated GPIO failure")
        self.value = False

    def close(self) -> None:
        self.closed = True


class FakeLgpio:
    def __init__(self, labels=None) -> None:
        self.calls = []
        self.labels = labels or {}

    def gpiochip_open(self, number: int) -> int:
        self.calls.append(("open", number))
        return 42 + number

    def gpiochip_close(self, handle: int) -> None:
        self.calls.append(("close_chip", handle))

    def gpio_get_chip_info(self, handle: int):
        number = handle - 42
        self.calls.append(("chip_info", handle))
        return [
            0,
            54,
            f"gpiochip{number}",
            self.labels.get(number, "internal-gpio"),
        ]

    def gpio_claim_output(
        self,
        handle: int,
        pin: int,
        level: int,
    ) -> None:
        self.calls.append(("claim", handle, pin, level))

    def gpio_write(self, handle: int, pin: int, level: int) -> None:
        self.calls.append(("write", handle, pin, level))

    def gpio_free(self, handle: int, pin: int) -> None:
        self.calls.append(("free", handle, pin))


class LgpioAdapterTests(unittest.TestCase):
    def test_finds_rp1_label_instead_of_assuming_chip_number(self) -> None:
        with TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            devices = root / "dev"
            devices.mkdir()
            (devices / "gpiochip0").touch()
            (devices / "gpiochip4").touch()
            fake_lgpio = FakeLgpio(labels={4: "pinctrl-rp1"})

            with patch(
                "stepper_stabilizer.hardware._verify_gpiochip_access"
            ):
                self.assertEqual(
                    _find_header_gpiochip(fake_lgpio, devices),
                    (4, devices / "gpiochip4"),
                )

    def test_direct_adapter_claims_writes_and_releases_pin(self) -> None:
        fake_lgpio = FakeLgpio()
        with (
            patch.dict(sys.modules, {"lgpio": fake_lgpio}),
            patch(
                "stepper_stabilizer.hardware._find_header_gpiochip",
                return_value=(0, Path("/dev/gpiochip0")),
            ),
            patch("stepper_stabilizer.hardware._verify_gpiochip_access"),
        ):
            chip = _LgpioChip()
            output = chip.output(
                17,
                active_high=True,
                initial_value=False,
            )
            output.on()
            output.off()
            output.close()
            chip.close()

        self.assertEqual(
            fake_lgpio.calls,
            [
                ("open", 0),
                ("claim", 42, 17, 0),
                ("write", 42, 17, 1),
                ("write", 42, 17, 0),
                ("free", 42, 17),
                ("close_chip", 42),
            ],
        )


class MotorTests(unittest.TestCase):
    def test_drivers_start_and_finish_disabled(self) -> None:
        outputs = {}
        sleeps = []

        def factory(pin: int, **kwargs: bool) -> FakeOutput:
            output = FakeOutput(pin, **kwargs)
            outputs[pin] = output
            return output

        motors = Motors(output_factory=factory, sleeper=sleeps.append)
        self.assertTrue(outputs[MOTOR_1_ENABLE_BCM].value)
        self.assertTrue(outputs[MOTOR_2_ENABLE_BCM].value)

        motors.rotate_both(
            Correction(
                direction=Direction.FORWARD,
                steps=2,
                low_delay_seconds=0.005,
            )
        )

        self.assertTrue(outputs[MOTOR_1_ENABLE_BCM].value)
        self.assertTrue(outputs[MOTOR_2_ENABLE_BCM].value)
        self.assertEqual(
            sleeps,
            [
                DRIVER_DIRECTION_SETUP_SECONDS,
                DRIVER_STEP_HIGH_SECONDS,
                0.005,
                DRIVER_STEP_HIGH_SECONDS,
                0.005,
            ],
        )

        motors.close()
        self.assertTrue(all(output.closed for output in outputs.values()))

    def test_partial_enable_failure_disables_both_drivers(self) -> None:
        outputs = {}

        def factory(pin: int, **kwargs: bool) -> FakeOutput:
            output = FakeOutput(pin, **kwargs)
            outputs[pin] = output
            return output

        motors = Motors(output_factory=factory, sleeper=lambda _: None)
        outputs[MOTOR_2_ENABLE_BCM].fail_next_off = True

        with self.assertRaises(OSError):
            motors.rotate_both(
                Correction(
                    direction=Direction.FORWARD,
                    steps=1,
                    low_delay_seconds=0.005,
                )
            )

        self.assertTrue(outputs[MOTOR_1_ENABLE_BCM].value)
        self.assertTrue(outputs[MOTOR_2_ENABLE_BCM].value)
        motors.close()


if __name__ == "__main__":
    unittest.main()
