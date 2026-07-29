use std::time::Duration;

/// Direction level sent to both DRV8825 DIR inputs.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Direction {
    Reverse,
    Forward,
}

/// A motor movement requested by one PID update.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Correction {
    pub direction: Direction,
    pub steps: u32,
    pub low_delay: Duration,
}

/// PID gains and movement limits carried over from the ESP32 sketch.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PidConfig {
    pub kp: f32,
    pub ki: f32,
    pub kd: f32,
    pub integral_limit: f32,
    pub output_limit: f32,
    pub dead_band_g: f32,
    pub fast_low_delay: Duration,
    pub slow_low_delay: Duration,
}

impl Default for PidConfig {
    fn default() -> Self {
        Self {
            kp: 0.80,
            ki: 0.05,
            kd: 0.10,
            integral_limit: 0.50,
            output_limit: 1.0,
            dead_band_g: 0.10,
            fast_low_delay: Duration::from_micros(2_500),
            slow_low_delay: Duration::from_micros(15_000),
        }
    }
}

/// Stateful PID controller that converts acceleration error into step bursts.
#[derive(Debug)]
pub struct PidController {
    config: PidConfig,
    previous_error: Option<f32>,
    integral_error: f32,
}

impl PidController {
    pub fn new(config: PidConfig) -> Self {
        Self {
            config,
            previous_error: None,
            integral_error: 0.0,
        }
    }

    /// Updates the controller.
    ///
    /// The target is 0 g, so `acceleration_x_g` is also the signed error.
    /// The first valid sample initializes the derivative term without moving.
    pub fn update(&mut self, acceleration_x_g: f32, elapsed: Duration) -> Option<Correction> {
        if !acceleration_x_g.is_finite() {
            return None;
        }

        let Some(previous_error) = self.previous_error else {
            self.previous_error = Some(acceleration_x_g);
            return None;
        };

        let elapsed_seconds = elapsed.as_secs_f32();
        if !elapsed_seconds.is_finite() || elapsed_seconds <= 0.0 {
            return None;
        }

        let error = acceleration_x_g;
        let proportional = self.config.kp * error;

        if error.abs() < self.config.dead_band_g {
            self.integral_error = 0.0;
        } else {
            self.integral_error = (self.integral_error + error * elapsed_seconds)
                .clamp(-self.config.integral_limit, self.config.integral_limit);
        }

        let integral = self.config.ki * self.integral_error;
        let derivative = self.config.kd * (error - previous_error) / elapsed_seconds;
        let output = (proportional + integral + derivative)
            .clamp(-self.config.output_limit, self.config.output_limit);

        self.previous_error = Some(error);

        if output.abs() < self.config.dead_band_g {
            return None;
        }

        let direction = if output > 0.0 {
            Direction::Forward
        } else {
            Direction::Reverse
        };
        let strength = (output.abs() / self.config.output_limit).clamp(0.0, 1.0);
        let fast_us = self.config.fast_low_delay.as_micros() as f32;
        let slow_us = self.config.slow_low_delay.as_micros() as f32;
        let low_delay_us = slow_us - strength * (slow_us - fast_us);

        Some(Correction {
            direction,
            steps: steps_for_error(error),
            low_delay: Duration::from_micros(low_delay_us as u64),
        })
    }
}

fn steps_for_error(error: f32) -> u32 {
    match error.abs() {
        value if value < 0.2 => 2,
        value if value < 0.4 => 4,
        value if value < 0.6 => 8,
        _ => 20,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const SAMPLE_TIME: Duration = Duration::from_millis(50);

    fn initialized_controller(initial_error: f32) -> PidController {
        let mut controller = PidController::new(PidConfig::default());
        assert_eq!(
            controller.update(initial_error, SAMPLE_TIME),
            None,
            "the first sample must only initialize the derivative term"
        );
        controller
    }

    #[test]
    fn ignores_invalid_samples() {
        let mut controller = PidController::new(PidConfig::default());
        assert_eq!(controller.update(f32::NAN, SAMPLE_TIME), None);
        assert_eq!(controller.update(f32::INFINITY, SAMPLE_TIME), None);
        assert_eq!(controller.update(0.5, SAMPLE_TIME), None);
    }

    #[test]
    fn does_not_move_inside_dead_band() {
        let mut controller = initialized_controller(0.05);
        assert_eq!(controller.update(0.05, SAMPLE_TIME), None);
    }

    #[test]
    fn positive_and_negative_errors_select_opposite_directions() {
        let mut positive = initialized_controller(0.3);
        let mut negative = initialized_controller(-0.3);

        assert_eq!(
            positive.update(0.3, SAMPLE_TIME).unwrap().direction,
            Direction::Forward
        );
        assert_eq!(
            negative.update(-0.3, SAMPLE_TIME).unwrap().direction,
            Direction::Reverse
        );
    }

    #[test]
    fn preserves_the_original_step_bands() {
        for (error, expected_steps) in [
            (0.19, 2),
            (0.20, 4),
            (0.39, 4),
            (0.40, 8),
            (0.59, 8),
            (0.60, 20),
        ] {
            let mut controller = initialized_controller(error);
            assert_eq!(
                controller.update(error, SAMPLE_TIME).unwrap().steps,
                expected_steps,
                "unexpected step count for {error} g"
            );
        }
    }

    #[test]
    fn clamps_full_output_to_fastest_delay() {
        let mut controller = initialized_controller(0.0);
        let correction = controller.update(10.0, SAMPLE_TIME).unwrap();
        assert_eq!(correction.low_delay, Duration::from_micros(2_500));
    }

    #[test]
    fn zero_elapsed_time_does_not_change_controller_history() {
        let mut controller = initialized_controller(0.2);
        assert_eq!(controller.update(0.8, Duration::ZERO), None);

        let correction = controller.update(0.2, SAMPLE_TIME).unwrap();
        assert_eq!(correction.steps, 4);
    }
}
