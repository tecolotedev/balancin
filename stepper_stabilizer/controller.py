"""PID control logic, kept independent from Raspberry Pi hardware."""

from dataclasses import dataclass
from enum import Enum
import math
from typing import Optional


class Direction(Enum):
    """Logical level sent to both DRV8825 DIR inputs."""

    REVERSE = 0
    FORWARD = 1


@dataclass(frozen=True)
class Correction:
    """One step burst requested by a PID update."""

    direction: Direction
    steps: int
    low_delay_seconds: float


@dataclass(frozen=True)
class PidConfig:
    """PID values and movement limits carried over from the ESP32 sketch."""

    kp: float = 0.80
    ki: float = 0.05
    kd: float = 0.10
    integral_limit: float = 0.50
    output_limit: float = 1.0
    dead_band_g: float = 0.15
    fast_low_delay_seconds: float = 0.0025
    slow_low_delay_seconds: float = 0.015

    def __post_init__(self) -> None:
        if self.integral_limit < 0:
            raise ValueError("integral_limit must not be negative")
        if self.output_limit <= 0:
            raise ValueError("output_limit must be positive")
        if self.dead_band_g < 0:
            raise ValueError("dead_band_g must not be negative")
        if not 0 < self.fast_low_delay_seconds <= self.slow_low_delay_seconds:
            raise ValueError(
                "step delays must satisfy 0 < fast_low_delay <= slow_low_delay"
            )


class PidController:
    """Convert signed X-axis acceleration into synchronized motor corrections."""

    def __init__(self, config: Optional[PidConfig] = None) -> None:
        self.config = config or PidConfig()
        self._previous_error: Optional[float] = None
        self._integral_error = 0.0

    def update(
        self, acceleration_x_g: float, elapsed_seconds: float
    ) -> Optional[Correction]:
        """Process one sample.

        The target is 0 g, so ``acceleration_x_g`` is the signed error. The
        first valid sample initializes the derivative term without moving.
        """

        if not math.isfinite(acceleration_x_g):
            return None

        if self._previous_error is None:
            self._previous_error = acceleration_x_g
            return None

        if not math.isfinite(elapsed_seconds) or elapsed_seconds <= 0:
            return None

        error = acceleration_x_g
        proportional = self.config.kp * error

        if abs(error) < self.config.dead_band_g:
            self._integral_error = 0.0
        else:
            self._integral_error = _clamp(
                self._integral_error + error * elapsed_seconds,
                -self.config.integral_limit,
                self.config.integral_limit,
            )

        integral = self.config.ki * self._integral_error
        derivative = (
            self.config.kd
            * (error - self._previous_error)
            / elapsed_seconds
        )
        output = _clamp(
            proportional + integral + derivative,
            -self.config.output_limit,
            self.config.output_limit,
        )

        self._previous_error = error

        if abs(output) < self.config.dead_band_g:
            return None

        direction = Direction.FORWARD if output > 0 else Direction.REVERSE
        strength = _clamp(
            abs(output) / self.config.output_limit,
            0.0,
            1.0,
        )
        low_delay = (
            self.config.slow_low_delay_seconds
            - strength
            * (
                self.config.slow_low_delay_seconds
                - self.config.fast_low_delay_seconds
            )
        )

        return Correction(
            direction=direction,
            steps=_steps_for_error(error),
            low_delay_seconds=low_delay,
        )


def _steps_for_error(error: float) -> int:
    absolute_error = abs(error)
    if absolute_error < 0.2:
        return 2
    if absolute_error < 0.4:
        return 4
    if absolute_error < 0.6:
        return 8
    return 20


def _clamp(value: float, minimum: float, maximum: float) -> float:
    return max(minimum, min(value, maximum))

