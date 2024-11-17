#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_task_wdt.h"

#include "control.h"
#include "register.h"
#include "setup.h"
#include "driver.dto.h"

static const char *TAG = "control";

void rotate_by_steps(TMC2209_Driver *driver, int32_t steps, uint32_t speed_steps_per_second)
{
  // Get the current task handle
  TaskHandle_t task_handle = xTaskGetCurrentTaskHandle();
  bool wdt_registered = false;

  // Initialize the watchdog timer with a longer timeout
  esp_task_wdt_config_t wdt_config = {
      .timeout_ms = 10000,  // 10 seconds timeout
      .idle_core_mask = 0,  // No idle cores
      .trigger_panic = false // Trigger panic on timeout
  };

  esp_task_wdt_reconfigure(&wdt_config);

  // Register the current task with the watchdog timer
  esp_err_t err = esp_task_wdt_add(task_handle);
  if (err == ESP_ERR_INVALID_STATE)
  {
    ESP_LOGW(TAG, "Task already registered with WDT");
    wdt_registered = true;
  }
  else if (err == ESP_OK)
  {
    wdt_registered = true;
  }
  else
  {
    ESP_LOGE(TAG, "Failed to register task with WDT: %s", esp_err_to_name(err));
    return;
  }

  // Calculate delay between steps (in microseconds)
  uint32_t delay_us = 1000000 / speed_steps_per_second / driver->settings.microsteps;

  // Ensure delay is at least one tick
  if (delay_us < portTICK_PERIOD_MS)
  {
    delay_us = portTICK_PERIOD_MS; // Minimum delay of one tick
  }

  uint32_t microsteps = abs(steps) * driver->settings.microsteps;

  ESP_LOGI(TAG, "Rotating motor by %" PRId32 " steps at %lu steps per second", steps, speed_steps_per_second);
  ESP_LOGI(TAG, "Delay between steps: %lu", delay_us);

  // Set DIR pin based on the sign of steps
  gpio_set_level(driver->dir_pin, steps >= 0 ? 0 : 1);

  for (int32_t i = 0; i < microsteps; i++)
  {
    // Toggle STEP pin
    gpio_set_level(driver->step_pin, 1);

    esp_rom_delay_us(delay_us);

    gpio_set_level(driver->step_pin, 0);

    esp_rom_delay_us(delay_us);

    // Reset the watchdog timer
    if (i % 100 == 0) // Adjust the value as needed
    {
      esp_task_wdt_reset();
    }
  }

  // Unregister the current task from the watchdog timer if it was registered
  if (wdt_registered)
  {
    err = esp_task_wdt_delete(task_handle);
    if (err != ESP_OK)
    {
      ESP_LOGE(TAG, "Failed to unregister task with WDT: %s", esp_err_to_name(err));
    }
  }

  // Restore the default watchdog timer configuration
  esp_task_wdt_config_t default_wdt_config = {
      .timeout_ms = CONFIG_ESP_TASK_WDT_TIMEOUT_S * 1000, // Default timeout from config
      .idle_core_mask = 0,                                // No idle cores
      .trigger_panic = true                               // Trigger panic on timeout
  };
  
  esp_task_wdt_reconfigure(&default_wdt_config);
}

void rotate_by_angle(TMC2209_Driver *driver, float angle, uint32_t speed_rpm)
{
  // Calculate the number of steps required to move the motor by the specified angle
  int32_t steps = (int32_t)(driver->settings.full_steps_per_rev * angle / 360.0f);

  int32_t speed = (int32_t)(driver->settings.full_steps_per_rev * speed_rpm / 60.0f);

  // Rotate the motor by the calculated number of steps
  rotate_by_steps(driver, steps, speed);
}

void enable_driver(TMC2209_Driver *driver)
{
  writeRegister(driver, REG_GCONF, 0x0000000F);
}

void disable_driver(TMC2209_Driver *driver)
{
  writeRegister(driver, REG_GCONF, 0x0000000C);
}

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
