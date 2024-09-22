| Supported Targets | ESP32 | ESP32-C2 | ESP32-C3 | ESP32-C5 | ESP32-C6 | ESP32-H2 | ESP32-P4 | ESP32-S2 | ESP32-S3 |
| ----------------- | ----- | -------- | -------- | -------- | -------- | -------- | -------- | -------- | -------- |

# TMC2209 ESP-IDF Library

This library provides a convenient interface for controlling the TMC2209 stepper motor driver using the ESP-IDF framework on ESP32 microcontrollers. It leverages the advanced features of the TMC2209 to achieve smooth, quiet, and efficient motor control.

## Features

* **StealthChop2™:** Ultra-quiet motor operation, ideal for noise-sensitive applications.
* **SpreadCycle™:** Highly dynamic motor control with excellent resonance dampening.
* **MicroPlyer™:** Smooth 256 microsteps interpolation for precise movements.
* **StallGuard4™:** Sensorless load and stall detection for enhanced safety and diagnostics.
* **CoolStep™:** Load-adaptive current control for energy savings and reduced heat generation.
* **UART Interface:**  Configure and monitor the driver using a simple UART connection.
* **STEP/DIR Interface:**  Compatible with traditional step/direction motor control signals.
* **Internal Pulse Generator:**  Standalone motion control without external STEP pulses.

## Installation

1. Clone this repository into your ESP-IDF project's `components` directory.
2. Add `tmc2209` to the `COMPONENT_SRCS` list in your `CMakeLists.txt`.

## Usage

1. Include the `tmc2209.h` header file in your code.
2. Create a `TMC2209_Driver` struct and populate its `settings` field with your desired configuration.
3. Call the `setup_driver` function to initialize the driver and apply the settings.
4. Use the provided functions (e.g., `rotate`, `move_at_velocity`) to control the motor.

## Example

```c
#include "tmc2209.h"

void app_main() {
    TMC2209_Driver motorDriver;
    motorDriver.uart_num = UART_NUM_1;
    motorDriver.dir_pin = 6;
    motorDriver.step_pin = 7;

    // Configure driver settings (adjust as needed)
    motorDriver.settings.rms_current = 500; 
    motorDriver.settings.microsteps = 16;
    motorDriver.settings.stealthchop.enabled = true;
    // ... other settings

    // Setup the driver
    esp_err_t err = setup_driver(&motorDriver);
    if (err != ESP_OK) {
        // Handle error
    }

    // Control the motor
    rotate(&motorDriver, 90.0, 500); // Rotate 90 degrees at max 500 steps/s
}
