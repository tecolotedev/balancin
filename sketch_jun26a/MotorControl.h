#pragma once

// Configures both DRV8825 drivers and leaves them disabled.
void setupMotors();

// Enables both motors, moves them in the selected direction for `steps` pulses,
// then disables the drivers. HIGH and LOW select opposite directions.
// `lowDelayMicroseconds` controls the duration of each LOW STEP pulse.
void rotateBothMotors(int direction, unsigned int steps,
                      unsigned int lowDelayMicroseconds);
