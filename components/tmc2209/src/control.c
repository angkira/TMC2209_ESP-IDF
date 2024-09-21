#include "control.h"
#include "register.h"
#include "driver.dto.h"

#include "driver/gpio.h"

#include "esp_log.h"

void rotate_motor(TMC2209_Driver *driver, int32_t steps, uint32_t speed)
{
  // Set DIR pin based on the sign of steps
  gpio_set_level(driver->dir_pin, steps >= 0 ? 0 : 1);

  set_target_velocity(driver, speed);

  // Get the current tick count
  TickType_t xLastWakeTime = xTaskGetTickCount();
  // Calculate delay between steps (in microseconds)
  uint32_t delay_us = 1000000 / speed / driver->settings.microsteps;

  // Ensure delay is at least one tick
  if (delay_us < portTICK_PERIOD_MS)
  {
    delay_us = portTICK_PERIOD_MS; // Minimum delay of one tick
  }

  uint32_t microsteps = abs(steps) * driver->settings.microsteps;

  // Loop for the absolute value of steps
  for (int32_t i = 0; i < microsteps; i++)
  {
    // Toggle STEP pin
    gpio_set_level(driver->step_pin, 1);
    esp_rom_delay_us(10); // Short pulse, adjust if needed
    gpio_set_level(driver->step_pin, 0);

    // Introduce delay
    vTaskDelayUntil(&xLastWakeTime, delay_us / portTICK_PERIOD_MS);
  }

  // Switch back to UART control mode (VACTUAL = 0)
  writeRegister(driver, REG_VACTUAL, 0);
}

void enable_driver(TMC2209_Driver *driver)
{
  writeRegister(driver, REG_GCONF, 0x0000000F);
}

void disable_driver(TMC2209_Driver *driver)
{
  writeRegister(driver, REG_GCONF, 0x0000000C);
}

// ... (Previous code from tmc2209.c)

void moveAtVelocity(TMC2209_Driver *driver, int32_t velocity)
{
  writeRegister(driver, REG_VMAX, abs(velocity));
  writeRegister(driver, REG_XDIRECT, (velocity >= 0) ? 0 : 0xFFFFFFFF); // Direction
}

// Function to set the motor's target velocity in microsteps per second
void set_target_velocity(TMC2209_Driver *driver, int32_t velocity)
{
  // The datasheet mentions that VACTUAL allows moving the motor by UART control
  // It gives the motor velocity in +-(2^23)-1 [usteps / t]
  // 0: Normal operation. Driver reacts to STEP input.
  // /=0: Motor moves with the velocity given by VACTUAL
  // The motor direction is controlled by the sign of VACTUAL

  // With internal oscillator:
  // VACTUAL[2209] = 0.715Hz / v[Hz]

  // Convert velocity (usteps/s) to VACTUAL
  int32_t vactual = (int32_t)((0.715f / (float)velocity) * (1 << 23));

  // Ensure VACTUAL is within the valid range
  if (vactual > ((1 << 23) - 1))
  {
    vactual = (1 << 23) - 1;
  }
  else if (vactual < -((1 << 23) - 1))
  {
    vactual = -((1 << 23) - 1);
  }

  writeRegister(driver, REG_VACTUAL, vactual);
}
