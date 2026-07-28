#pragma once

// Starts I2C on GPIO 21/27 at 100 kHz and prints the bus configuration.
void setupI2CDiagnostic();

// Scans every I2C address and reads WHO_AM_I from 0x68 or 0x69.
void scanI2CBus();
