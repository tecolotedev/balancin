#pragma once

// Configures both DRV8825 drivers and leaves them disabled.
void setupMotors();

// Moves both motors together one revolution forward and one backward.
void runBothMotors();
