import unittest
from unittest.mock import patch

from stepper_stabilizer import app
from stepper_stabilizer.controller import Direction


class FakeMotors:
    def __init__(self) -> None:
        self.corrections = []
        self.closed = False

    def __enter__(self) -> "FakeMotors":
        return self

    def __exit__(self, *args: object) -> None:
        self.closed = True

    def rotate_both(self, correction) -> None:
        self.corrections.append(correction)


class MotorTestTests(unittest.TestCase):
    def test_motor_test_moves_forward_then_reverse_and_closes(self) -> None:
        motors = FakeMotors()
        with (
            patch.object(app, "Motors", return_value=motors),
            patch.object(app.time, "sleep") as sleep,
        ):
            app.run_motor_test()

        self.assertTrue(motors.closed)
        self.assertEqual(len(motors.corrections), 2)
        self.assertEqual(
            [correction.direction for correction in motors.corrections],
            [Direction.FORWARD, Direction.REVERSE],
        )
        self.assertEqual(
            [correction.steps for correction in motors.corrections],
            [4000, 4000],
        )
        self.assertEqual(
            [
                correction.low_delay_seconds
                for correction in motors.corrections
            ],
            [0.015, 0.015],
        )
        sleep.assert_called_once_with(app.MOTOR_TEST_PAUSE_SECONDS)

    def test_motor_test_flag_dispatches_without_starting_stabilizer(self) -> None:
        with (
            patch.object(app, "run_motor_test") as motor_test,
            patch.object(app, "stabilize") as stabilize,
        ):
            result = app.main(["--motor-test"])

        self.assertEqual(result, 0)
        motor_test.assert_called_once_with()
        stabilize.assert_not_called()


if __name__ == "__main__":
    unittest.main()
