import unittest

from stepper_stabilizer.controller import Correction, Direction
from stepper_stabilizer.hardware import (
    DRIVER_DIRECTION_SETUP_SECONDS,
    DRIVER_STEP_HIGH_SECONDS,
    MOTOR_1_ENABLE_BCM,
    MOTOR_2_ENABLE_BCM,
    Motors,
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
