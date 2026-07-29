#pragma once

// Configures both DRV8825 drivers and leaves them disabled.
void setupMotors();

// Enables both motors, moves them in the selected direction for `steps` pulses,
// then disables the drivers. HIGH and LOW select opposite directions.
void moveBothMotors(int direction, int steps);

// Moves both motors together 2,000 steps forward and 2,000 steps backward.
void runBothMotors();
