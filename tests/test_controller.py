import math
import unittest

from stepper_stabilizer.controller import (
    Direction,
    PidConfig,
    PidController,
)


SAMPLE_TIME = 0.050


def initialized_controller(initial_error: float) -> PidController:
    controller = PidController()
    result = controller.update(initial_error, SAMPLE_TIME)
    if result is not None:
        raise AssertionError("the first sample must not move the motors")
    return controller


class PidControllerTests(unittest.TestCase):
    def test_ignores_invalid_samples(self) -> None:
        controller = PidController()
        self.assertIsNone(controller.update(math.nan, SAMPLE_TIME))
        self.assertIsNone(controller.update(math.inf, SAMPLE_TIME))
        self.assertIsNone(controller.update(0.5, SAMPLE_TIME))

    def test_does_not_move_inside_dead_band(self) -> None:
        controller = initialized_controller(0.05)
        self.assertIsNone(controller.update(0.05, SAMPLE_TIME))

    def test_errors_select_opposite_directions(self) -> None:
        positive = initialized_controller(0.3)
        negative = initialized_controller(-0.3)
        self.assertEqual(
            positive.update(0.3, SAMPLE_TIME).direction,
            Direction.FORWARD,
        )
        self.assertEqual(
            negative.update(-0.3, SAMPLE_TIME).direction,
            Direction.REVERSE,
        )

    def test_preserves_original_step_bands(self) -> None:
        for error, expected_steps in (
            (0.19, 2),
            (0.20, 4),
            (0.39, 4),
            (0.40, 8),
            (0.59, 8),
            (0.60, 20),
        ):
            with self.subTest(error=error):
                controller = initialized_controller(error)
                correction = controller.update(error, SAMPLE_TIME)
                self.assertIsNotNone(correction)
                self.assertEqual(correction.steps, expected_steps)

    def test_clamps_full_output_to_fastest_delay(self) -> None:
        controller = initialized_controller(0.0)
        correction = controller.update(10.0, SAMPLE_TIME)
        self.assertIsNotNone(correction)
        self.assertAlmostEqual(
            correction.low_delay_seconds,
            PidConfig().fast_low_delay_seconds,
        )

    def test_zero_elapsed_time_does_not_change_history(self) -> None:
        controller = initialized_controller(0.2)
        self.assertIsNone(controller.update(0.8, 0.0))
        correction = controller.update(0.2, SAMPLE_TIME)
        self.assertIsNotNone(correction)
        self.assertEqual(correction.steps, 4)


if __name__ == "__main__":
    unittest.main()

