#include "tmc2209.h"
#include "esp_log.h"

static const char *TAG = "main";

#define TMC2209_ADDRESS 0x00

void app_main()
{
  TMC2209_Driver motorDriver;
  motorDriver.uart_num = UART_NUM_1;
  motorDriver.dir_pin = 6;
  motorDriver.step_pin = 7;

  // Initialize driver settings
  motorDriver.settings.driver_address = TMC2209_ADDRESS;
  motorDriver.settings.rms_current = 800;
  motorDriver.settings.microsteps = 64;

  TMC2209_SpreadCycleConfig spreadCycleConfig = {
      .enabled = true,
      .slow_decay_time = 5,
      .blank_time = 2,
      .hysteresis_start = 0,
      .hysteresis_end = 0
  };

  TMC2209_StealthchopConfig stealthchopConfig = {
      .enabled = false,
  };

  motorDriver.settings.spreadcycle = spreadCycleConfig;
  motorDriver.settings.stealthchop = stealthchopConfig;

  // Setup the driver
  esp_err_t err = setup_driver(&motorDriver);
  if (err != ESP_OK)
  {
    ESP_LOGE(TAG, "Error setting up TMC2209 driver: %s", esp_err_to_name(err));
    return;
  }

  // // Enable the driver
  enable_driver(&motorDriver);
  ESP_LOGI(TAG, "TMC2209 driver enabled");

  // Basic motor control sequence
  ESP_LOGI(TAG, "Starting motor movement...");
  // moveAtVelocity(&motorDriver, 10000);

  rotate_motor(&motorDriver, 200, 1000000, CHOPPER_MODE_SPREADCYCLE);

  // vTaskDelay(20000 / portTICK_PERIOD_MS);

  // moveAtVelocity(&motorDriver, 0);

  // Disable the driver when finished
  disable_driver(&motorDriver);
  ESP_LOGI(TAG, "TMC2209 driver disabled");
}
